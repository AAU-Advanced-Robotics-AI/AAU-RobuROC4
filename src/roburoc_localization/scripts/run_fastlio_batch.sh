#!/usr/bin/env bash
# run_fastlio_batch.sh
#
# Runs offline_fastlio.launch.py (Stage 1) on every raw bag in a chosen folder.
# Raw bags are any subdirectories that are NOT already derived outputs
# (_fastlio, _odometry, _ekf).
#
# Existing _fastlio output bags are deleted before each run so outputs are
# always fresh.
#
# Usage:
#   ./run_fastlio_batch.sh <folder> [rate] [skip_existing]
#
#   folder         Path to the folder containing raw bag directories (required)
#   rate           Bag playback rate for FAST-LIO (default 1.0 — lower to 0.5 if
#                  FAST-LIO drops messages)
#   skip_existing  true (default) = skip bags that already have a _fastlio output
#                  false          = delete existing _fastlio output and reprocess

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
    "/ublox_gps_node/fix"
    "/ublox_gps_node/fix_velocity"
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

info()  { echo -e "${GREEN}[fastlio_batch]${NC} $*"; }
warn()  { echo -e "${YELLOW}[fastlio_batch]${NC} $*"; }
error() { echo -e "${RED}[fastlio_batch]${NC} $*" >&2; }

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

# ── Collect raw bags (exclude derived outputs) ───────────────────────────────
mapfile -t RAW_BAGS < <(
    find "$FOLDER" -maxdepth 1 -type d \
        ! -name '*_fastlio' \
        ! -name '*_odometry' \
        ! -name '*_ekf' \
        ! -path "$FOLDER" | sort
)

TOTAL=${#RAW_BAGS[@]}
if [[ $TOTAL -eq 0 ]]; then
    error "No raw bags found in: $FOLDER"
    exit 1
fi

info "Found $TOTAL bag(s) in: $FOLDER"
info "  FAST-LIO rate: ${RATE}x"
info "  Skip existing:  ${SKIP_EXISTING}"
echo ""

FAILED=()

for i in "${!RAW_BAGS[@]}"; do
    bag="${RAW_BAGS[$i]}"
    name="$(basename "$bag")"
    fastlio_bag="${bag}_fastlio"
    idx=$(( i + 1 ))

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    info "[$idx/$TOTAL] $name"

    # ── Skip or delete existing output ──────────────────────────────────────────
    if [[ -d "$fastlio_bag" ]]; then
        if [[ "$SKIP_EXISTING" == "true" ]]; then
            warn "  Already exists — skipping: $(basename "$fastlio_bag")"
            continue
        else
            warn "  Deleting existing output: $(basename "$fastlio_bag")"
            rm -rf "$fastlio_bag"
        fi
    fi

    # ── Topic check ───────────────────────────────────────────────────────────
    info "  Checking required topics ..."
    if ! check_topics "$bag"; then
        error "  Topic check FAILED — skipping $name"
        FAILED+=("$name (missing topics)")
        continue
    fi
    info "  All required topics present."

    # ── Stage 1: FAST-LIO ─────────────────────────────────────────────────────
    info "  Running FAST-LIO ..."
    if ros2 launch roburoc_localization offline_fastlio.launch.py \
            bag:="$bag" rate:="$RATE"; then
        info "  FAST-LIO done → $(basename "$fastlio_bag")"
    else
        error "  FAST-LIO FAILED for $name"
        FAILED+=("$name (fastlio)")
        continue
    fi

    if [[ ! -d "$fastlio_bag" ]]; then
        error "  FAST-LIO produced no output bag for $name"
        FAILED+=("$name (no output)")
        continue
    fi

    echo ""
done

# ── Final report ─────────────────────────────────────────────────────────────
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [[ ${#FAILED[@]} -eq 0 ]]; then
    info "All $TOTAL bag(s) processed successfully."
else
    warn "${#FAILED[@]} bag(s) had failures:"
    for f in "${FAILED[@]}"; do
        error "  • $f"
    done
    exit 1
fi
