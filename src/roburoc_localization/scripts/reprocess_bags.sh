#!/usr/bin/env bash
# reprocess_bags.sh
#
# Full pipeline for every _fastlio bag in ad_hoc_tests and reference_paths:
#
#   1. Topic check  — verify the _fastlio bag contains all required topics.
#   2. Stage 2      — offline_odometry.launch.py → _odometry bag.
#   3. Kabsch align — kabsch_align.py --write-aligned → _odometry_aligned bag.
#                     Prints rotation/translation residuals for extrinsic cal.
#   4. Stage 3      — offline_ekf.launch.py on _odometry_aligned → _ekf bag.
#
# Existing _odometry, _odometry_aligned, and _ekf bags are deleted before each
# run so outputs are always fresh.
#
# Usage:
#   ./reprocess_bags.sh [rate_s2 [rate_s3]]
#
#   rate_s2  Bag playback rate for Stage 2 (default 20.0)
#   rate_s3  Bag playback rate for Stage 3 (default 15.0)

set -eo pipefail

RATE_S2="${1:-20.0}"
RATE_S3="${2:-15.0}"

FOLDERS=(
    "$HOME/RobuROC_ROSbags/ad_hoc_tests"
    "$HOME/RobuROC_ROSbags/reference_paths"
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
)

TOTAL=${#FASTLIO_BAGS[@]}
if [[ $TOTAL -eq 0 ]]; then
    error "No _fastlio bags found in any of the specified folders."
    exit 1
fi

info "Found $TOTAL _fastlio bags."
info "  Stage-2 rate : ${RATE_S2}x"
info "  Stage-3 rate : ${RATE_S3}x"
echo ""

FAILED=()
declare -A KABSCH_RESULTS  # name → summary line for final report

for i in "${!FASTLIO_BAGS[@]}"; do
    fastlio_bag="${FASTLIO_BAGS[$i]}"
    base="${fastlio_bag%_fastlio}"
    name="$(basename "$base")"
    odometry_bag="${base}_odometry"
    aligned_bag="${odometry_bag}_aligned"
    ekf_bag="${base}_ekf"
    idx=$(( i + 1 ))

    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    info "[$idx/$TOTAL] $name"

    # ── 1. Topic check ────────────────────────────────────────────────────────
    info "  Checking required topics in $(basename "$fastlio_bag") ..."
    if ! check_topics "$fastlio_bag"; then
        error "  Topic check FAILED — skipping $name"
        FAILED+=("$name (missing topics)")
        continue
    fi
    info "  All required topics present."

    # Delete stale derived bags
    for stale in "$odometry_bag" "$aligned_bag" "$ekf_bag"; do
        if [[ -d "$stale" ]]; then
            warn "  Deleting stale: $(basename "$stale")"
            rm -rf "$stale"
        fi
    done

    # ── 2. Stage 2: odometry ─────────────────────────────────────────────────
    info "  Stage 2 (odometry) ..."
    if ros2 launch roburoc_localization offline_odometry.launch.py \
            bag:="$fastlio_bag" rate:="$RATE_S2"; then
        info "  Stage 2 done → $(basename "$odometry_bag")"
    else
        error "  Stage 2 FAILED for $name — skipping Kabsch + EKF"
        FAILED+=("$name (stage 2)")
        continue
    fi

    if [[ ! -d "$odometry_bag" ]]; then
        error "  Stage 2 produced no output bag — skipping Kabsch + EKF"
        FAILED+=("$name (stage 2 no output)")
        continue
    fi

    # ── 3. Kabsch alignment ───────────────────────────────────────────────────
    info "  Kabsch alignment (GPS ↔ LIO) ..."
    kabsch_log="$(mktemp)"
    ekf_input_bag="$aligned_bag"   # default; overridden to odometry_bag on failure
    if python3 "$SCRIPT_DIR/kabsch_align.py" \
            "$odometry_bag" \
            --write-aligned \
            2>&1 | tee "$kabsch_log"; then

        # Extract summary lines for the final report
        theta_line="$(grep -m1 'Rotation θ'    "$kabsch_log" || true)"
        trans_line="$(grep -m1 'Translation t'  "$kabsch_log" || true)"
        rms_line="$(  grep -m1 'Fit RMS'        "$kabsch_log" || true)"
        KABSCH_RESULTS["$name"]="${theta_line}  |  ${trans_line}  |  ${rms_line}"
        info "  Kabsch done → $(basename "$aligned_bag")"
    else
        warn "  Kabsch could not align $name (see above) — running EKF on unaligned odometry bag"
        KABSCH_RESULTS["$name"]="  (no alignment — Kabsch failed)"
        ekf_input_bag="$odometry_bag"
    fi
    rm -f "$kabsch_log"

    if [[ ! -d "$ekf_input_bag" ]]; then
        error "  EKF input bag missing: $(basename "$ekf_input_bag") — skipping EKF"
        FAILED+=("$name (no ekf input)")
        continue
    fi

    # ── 4. Stage 3: EKF ──────────────────────────────────────────────────────
    info "  Stage 3 (EKF) on $(basename "$ekf_input_bag") ..."
    if ros2 launch roburoc_localization offline_ekf.launch.py \
            bag:="$ekf_input_bag" rate:="$RATE_S3"; then
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
echo " Kabsch alignment summary (GPS ↔ LIO residuals per dataset)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
# Sort by dataset name for reproducible output
while IFS= read -r name; do
    echo "  $name"
    echo "    ${KABSCH_RESULTS[$name]}"
done < <(printf '%s\n' "${!KABSCH_RESULTS[@]}" | sort)

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
