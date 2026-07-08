#!/usr/bin/env bash
# reprocess_bags.sh
#
# Full pipeline for every _fastlio bag in ad_hoc_tests and reference_paths:
#
#   1. Topic check  — verify the _fastlio bag contains all required topics.
#   2. Stage 2      — offline_odometry.launch.py → _odometry bag.
#                     lio_to_enu performs online Kabsch alignment during replay.
#   3. Stage 3      — offline_ekf.launch.py on _odometry → _ekf bag.
#
# Existing _odometry and _ekf bags are deleted before each run so outputs are
# always fresh.
#
# Usage:
#   ./reprocess_bags.sh [rate_s2 [rate_s3 [skip_existing]]]
#
#   rate_s2        Bag playback rate for Stage 2 (default 20.0)
#   rate_s3        Bag playback rate for Stage 3 (default 15.0)
#   skip_existing  true (default) = skip bags whose _ekf output already exists
#                  false          = delete existing _odometry/_ekf and reprocess

set -eo pipefail

RATE_S2="${1:-20.0}"
RATE_S3="${2:-15.0}"
SKIP_EXISTING="${3:-true}"

FOLDERS=(
    # "$HOME/RobuROC_ROSbags/ad_hoc_tests"
    # "$HOME/RobuROC_ROSbags/reference_paths"
    "$HOME/rosbags/test_day_06_09"
)

# Bags recorded live (already contain /Odometry from FAST-LIO2 — no Stage 1
# needed).  Each entry is a bag directory path; they are processed directly as
# Stage-2 inputs alongside the _fastlio bags above.
RAW_BAG_FOLDERS=(
    # "$HOME/data/rosbags"
)

# Topics that must be present in a _fastlio bag before processing.
REQUIRED_TOPICS=(
    "/Odometry"
    "/ublox_gps_node/fix"
    "/ublox_gps_node/fix_velocity"
    "/tf_static"
)

SCRIPT_DIR="$(dirname "$(realpath "$0")")" 

# ── Source ROS2 + workspace (relaxed -u for ROS setup scripts) ───────────────
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../../../install/setup.bash"
set -u

# ── Helpers ──────────────────────────────────────────────────────────────────
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'

info()  { echo -e "${GREEN}[reprocess]${NC} $*"; }
warn()  { echo -e "${YELLOW}[reprocess]${NC} $*"; }
error() { echo -e "${RED}[reprocess]${NC} $*" >&2; }

# check_topics <bag_path>
# Prints missing/empty topics and returns 1 if validation fails, 0 if all ok.
check_topics() {
    local bag="$1"
    local bag_info
    local missing=() empty=()
    bag_info="$(ros2 bag info "$bag" 2>/dev/null)" || {
        error "  Could not read bag info for $bag"
        return 1
    }
    for topic in "${REQUIRED_TOPICS[@]}"; do
        if ! echo "$bag_info" | grep -q "Topic: $topic "; then
            missing+=("$topic")
        elif echo "$bag_info" | grep -q "Topic: $topic.*Count: 0 "; then
            empty+=("$topic")
        fi
    done
    local ok=0
    if [[ ${#missing[@]} -gt 0 ]]; then
        warn "  Missing required topics:"
        for t in "${missing[@]}"; do warn "    • $t"; done
        ok=1
    fi
    if [[ ${#empty[@]} -gt 0 ]]; then
        warn "  Topics present but empty (0 messages):"
        for t in "${empty[@]}"; do warn "    • $t"; done
        ok=1
    fi
    return $ok
}

# ── Collect all _fastlio bags across both folders ────────────────────────────
mapfile -t FASTLIO_BAGS < <(
    for folder in "${FOLDERS[@]}"; do
        if [[ ! -d "$folder" ]]; then
            warn "Folder not found, skipping: $folder"
            continue
        fi
        find "$folder" -maxdepth 1 -type d -name '*_fastlio' | sort
    done
    # Raw live bags (contain /Odometry directly — skip Stage 1).
    # Exclude bags that are already derived outputs (_fastlio, _odometry, _ekf).
    for folder in "${RAW_BAG_FOLDERS[@]}"; do
        if [[ ! -d "$folder" ]]; then
            warn "Raw-bag folder not found, skipping: $folder"
            continue
        fi
        find "$folder" -maxdepth 1 -type d \
            ! -name '*_fastlio' \
            ! -name '*_odometry' \
            ! -name '*_ekf' | sort
    done
)

TOTAL=${#FASTLIO_BAGS[@]}
if [[ $TOTAL -eq 0 ]]; then
    error "No _fastlio bags found in any of the specified folders."
    exit 1
fi

info "Found $TOTAL _fastlio bags."
info "  Stage-2 rate  : ${RATE_S2}x"
info "  Stage-3 rate  : ${RATE_S3}x"
info "  Skip existing : ${SKIP_EXISTING}"
echo ""

FAILED=()

for i in "${!FASTLIO_BAGS[@]}"; do
    fastlio_bag="${FASTLIO_BAGS[$i]}"
    base="${fastlio_bag%_fastlio}"
    name="$(basename "$base")"
    odometry_bag="${base}_odometry"
    ekf_bag="${base}_ekf"
    idx=$(( i + 1 ))

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    info "[$idx/$TOTAL] $name"

    # ── Skip or delete existing outputs ──────────────────────────────────────
    if [[ -d "$ekf_bag" ]]; then
        if [[ "$SKIP_EXISTING" == "true" ]]; then
            warn "  Already exists — skipping: $(basename "$ekf_bag")"
            continue
        else
            warn "  Deleting existing outputs for $name"
            for stale in "$odometry_bag" "$ekf_bag"; do
                if [[ -d "$stale" ]]; then
                    warn "    Deleting: $(basename "$stale")"
                    rm -rf "$stale"
                fi
            done
        fi
    fi

    # ── 1. Topic check ────────────────────────────────────────────────────────
    info "  Checking required topics in $(basename "$fastlio_bag") ..."
    if ! check_topics "$fastlio_bag"; then
        error "  Topic check FAILED — skipping $name"
        FAILED+=("$name (missing topics)")
        continue
    fi
    info "  All required topics present."

    # ── 2. Stage 2: odometry ─────────────────────────────────────────────────
    info "  Stage 2 (odometry) ..."
    if ros2 launch roburoc_localization offline_odometry.launch.py \
            bag:="$fastlio_bag" rate:="$RATE_S2"; then
        info "  Stage 2 done → $(basename "$odometry_bag")"
    else
        error "  Stage 2 FAILED for $name — skipping EKF"
        FAILED+=("$name (stage 2)")
        continue
    fi

    if [[ ! -d "$odometry_bag" ]]; then
        error "  Stage 2 produced no output bag — skipping EKF"
        FAILED+=("$name (stage 2 no output)")
        continue
    fi

    # ── 3. Stage 3: EKF ──────────────────────────────────────────────────────
    info "  Stage 3 (EKF) on $(basename "$odometry_bag") ..."
    if ros2 launch roburoc_localization offline_ekf.launch.py \
            bag:="$odometry_bag" rate:="$RATE_S3"; then
        info "  Stage 3 done → $(basename "$ekf_bag")"
    else
        error "  Stage 3 FAILED for $name"
        FAILED+=("$name (stage 3)")
    fi

    echo ""
done

# ── Final report ─────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [[ ${#FAILED[@]} -eq 0 ]]; then
    info "All $TOTAL datasets processed successfully."
else
    warn "${#FAILED[@]} dataset(s) had failures:"
    for f in "${FAILED[@]}"; do
        error "  • $f"
    done
    exit 1
fi
