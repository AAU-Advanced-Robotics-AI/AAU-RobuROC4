#!/usr/bin/env bash
# run_pgo_batch.sh
#
# Runs the GTSAM pose-graph optimizer (lidar_gps_pgo) offline on every raw bag
# in a folder and produces, for each, a PlotJuggler-replayable <bag>_pgo rosbag
# in the pipeline ENU frame (topic /odometry/pgo/global), with the matching
# <bag>_ekf comparison topics copied in so one PlotJuggler session shows PGO vs
# EKF vs GPS vs LIO.
#
# Per bag:
#   1. offline_pgo.launch.py  — re-run FAST-LIO live + pgo_node, one sim clock,
#                               auto /pgo/save -> <bag>_pgo_artifacts/
#                               (optimized_poses_tum.txt + georeference.txt + ...)
#   2. pgo_to_bag.py          — <bag>_pgo_artifacts (+ <bag>_ekf) -> <bag>_pgo bag
#
# Requires FAST-LIO output which no recorded bag contains, so this always re-runs
# FAST-LIO (the bottleneck).  Needs three workspaces sourced (see below).
#
# Usage:
#   ./run_pgo_batch.sh <folder> [rate] [skip_existing]
#
#   folder         Folder containing raw bag directories (required)
#   rate           FAST-LIO replay rate (default 1.0 — lower to 0.5 if FAST-LIO
#                  drops scans; watch for "drop message" warnings)
#   skip_existing  true (default) = skip bags that already have a _pgo output
#                  false          = delete existing _pgo output and reprocess

set -eo pipefail

FOLDER="${1:-}"
RATE="${2:-1.0}"
SKIP_EXISTING="${3:-true}"

if [[ -z "$FOLDER" ]]; then
    echo "Usage: $0 <folder> [rate] [skip_existing]" >&2
    exit 1
fi
if [[ ! -d "$FOLDER" ]]; then
    echo "ERROR: Folder not found: $FOLDER" >&2
    exit 1
fi

# Topics that must be present in a raw bag before processing.
REQUIRED_TOPICS=(
    "/livox/lidar"
    "/livox/imu"
    "/ublox_gps_node/navpvt"
    "/tf_static"
)

SCRIPT_DIR="$(dirname "$(realpath "$0")")"
PKG_DIR="$(realpath "$SCRIPT_DIR/..")"

# ── Source ROS2 + all three workspaces (relaxed -u for ROS setup scripts) ─────
# fast_lio + livox_ros_driver2 (lio_ws), ublox_msgs (rtk_ws), this ws (AAU-RobuROC4).
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
[[ -f "$HOME/lio_ws/install/setup.bash" ]] && source "$HOME/lio_ws/install/setup.bash"
# shellcheck disable=SC1091
[[ -f "$HOME/rtk_ws/install/setup.bash" ]] && source "$HOME/rtk_ws/install/setup.bash"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../../install/setup.bash"
set -u

# ── Middleware: FastDDS, not CycloneDDS ──────────────────────────────────────
# CycloneDDS drops the large /livox/lidar CustomMsgs (~138 KB @ 10 Hz) during
# 1.0x replay on this localhost/WSL2 setup, starving FAST-LIO -> divergence
# (0.5x barely keeps up).  FastDDS (rmw_fastrtps_cpp, the ROS 2 default) delivers
# them reliably at 1.0x: verified GPS/odom alignment rms 0.35 m, same as 0.5x.
# Scoped to this batch's child processes only — your ~/.bashrc cyclone setup for
# the online/RTK stack is untouched.  Do NOT set a custom CYCLONEDDS_URI (it
# conflicts with ROS_LOCALHOST_ONLY and wedges the ros2 daemon).
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# ── Kill stragglers from any previous/interrupted run before starting, and on
#    exit, so a stopped batch never leaves a bag-player pumping a second
#    lidar/imu/clock stream (which makes FAST-LIO diverge on the next launch).
#    Patterns live in this file, not on the command line, so pkill -f cannot
#    match this script's own `bash run_pgo_batch.sh` invocation.
cleanup_stragglers() {
    pkill -9 -f 'fastlio_mapping'          2>/dev/null || true
    pkill -9 -f 'lidar_gps_pgo/lib.*pgo_node' 2>/dev/null || true
    pkill -9 -f 'offline_pgo.launch.py'    2>/dev/null || true
    pkill -9 -f 'bag play .*_pgo|ros2 bag play' 2>/dev/null || true
}
trap cleanup_stragglers EXIT INT TERM
cleanup_stragglers   # clean slate at start

# ── Helpers ──────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'
info()  { echo -e "${GREEN}[pgo_batch]${NC} $*"; }
warn()  { echo -e "${YELLOW}[pgo_batch]${NC} $*"; }
error() { echo -e "${RED}[pgo_batch]${NC} $*" >&2; }

check_topics() {
    local bag="$1" bag_info missing=() empty=()
    bag_info="$(ros2 bag info "$bag" 2>/dev/null)" || { error "  Could not read bag info for $bag"; return 1; }
    for topic in "${REQUIRED_TOPICS[@]}"; do
        if ! echo "$bag_info" | grep -q "Topic: $topic "; then
            missing+=("$topic")
        elif echo "$bag_info" | grep -q "Topic: $topic.*Count: 0 "; then
            empty+=("$topic")
        fi
    done
    local ok=0
    if [[ ${#missing[@]} -gt 0 ]]; then warn "  Missing required topics:"; for t in "${missing[@]}"; do warn "    • $t"; done; ok=1; fi
    if [[ ${#empty[@]} -gt 0 ]]; then warn "  Topics present but empty:"; for t in "${empty[@]}"; do warn "    • $t"; done; ok=1; fi
    return $ok
}

# ── Collect raw bags (exclude derived outputs) ───────────────────────────────
mapfile -t RAW_BAGS < <(
    find "$FOLDER" -maxdepth 1 -type d \
        ! -name '*_fastlio' ! -name '*_odometry' ! -name '*_ekf' \
        ! -name '*_pgo' ! -name '*_pgo_artifacts' \
        ! -name '*_outbound' ! -name '*_return' \
        ! -path "$FOLDER" | sort
)

TOTAL=${#RAW_BAGS[@]}
if [[ $TOTAL -eq 0 ]]; then error "No raw bags found in: $FOLDER"; exit 1; fi

info "Found $TOTAL bag(s) in: $FOLDER"
info "  FAST-LIO rate : ${RATE}x"
info "  Skip existing : ${SKIP_EXISTING}"
info "  Middleware    : ${RMW_IMPLEMENTATION} (localhost=${ROS_LOCALHOST_ONLY:-unset})"
echo ""

FAILED=()

for i in "${!RAW_BAGS[@]}"; do
    bag="${RAW_BAGS[$i]}"
    name="$(basename "$bag")"
    artifacts="${bag}_pgo_artifacts"
    pgo_bag="${bag}_pgo"
    ekf_bag="${bag}_ekf"
    idx=$(( i + 1 ))

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    info "[$idx/$TOTAL] $name"

    if [[ -d "$pgo_bag" ]]; then
        if [[ "$SKIP_EXISTING" == "true" ]]; then
            warn "  Already exists — skipping: $(basename "$pgo_bag")"; continue
        else
            warn "  Deleting existing outputs for $name"
            rm -rf "$pgo_bag" "$artifacts"
        fi
    fi

    info "  Checking required topics ..."
    if ! check_topics "$bag"; then
        error "  Topic check FAILED — skipping $name"; FAILED+=("$name (missing topics)"); continue
    fi
    info "  All required topics present."

    # ── 1. PGO (FAST-LIO + pgo_node) ─────────────────────────────────────────
    rm -rf "$artifacts"
    info "  Running PGO (FAST-LIO + GTSAM) ..."
    if ros2 launch lidar_gps_pgo offline_pgo.launch.py raw_bag:="$bag" rate:="$RATE"; then
        info "  PGO done → $(basename "$artifacts")"
    else
        error "  PGO FAILED for $name"; FAILED+=("$name (pgo)"); continue
    fi
    if [[ ! -f "$artifacts/optimized_poses_tum.txt" ]]; then
        error "  PGO produced no optimized_poses_tum.txt for $name"; FAILED+=("$name (no artifacts)"); continue
    fi

    # ── 2. Convert optimizer artifacts → ENU rosbag (+ _ekf passthrough) ─────
    info "  Building PlotJuggler bag → $(basename "$pgo_bag") ..."
    ekf_arg=(); [[ -d "$ekf_bag" ]] && ekf_arg=(--ekf "$ekf_bag")
    if python3 "$SCRIPT_DIR/pgo_to_bag.py" "$artifacts" "$pgo_bag" "${ekf_arg[@]}"; then
        info "  Wrote $(basename "$pgo_bag")"
    else
        error "  pgo_to_bag FAILED for $name"; FAILED+=("$name (to_bag)"); continue
    fi
    echo ""
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [[ ${#FAILED[@]} -eq 0 ]]; then
    info "All $TOTAL bag(s) processed successfully."
else
    warn "${#FAILED[@]} bag(s) had failures:"
    for f in "${FAILED[@]}"; do error "  • $f"; done
    exit 1
fi
