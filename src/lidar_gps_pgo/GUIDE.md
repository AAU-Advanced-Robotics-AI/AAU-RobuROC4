# lidar_gps_pgo — how it works (current)

A **C++ / GTSAM pose-graph optimizer** that fuses FAST-LIO2 LiDAR-inertial odometry,
RTK GPS (u-blox ZED-F9P), and ScanContext loop closures into a single globally
consistent, ENU-referenced trajectory. It is an alternative to the `robot_localization`
dual-EKF stack in `roburoc_localization`.

> **The path we use is 100% C++.** `pgo_node` (ROS 2 wrapper) + `pgo_backend` (ROS-free
> GTSAM core) do all the estimation. The only Python is *offline glue* — the launch file,
> the batch driver, and the bag/plot converters. There is no Python estimator.
>
> This guide reflects the code **after** the GPS-fusion redesign. It supersedes the
> RTK-float "drifting-bias" description in `README.md` (that model was removed).

---

## 1. Components

| file | role |
|---|---|
| `src/pgo_node.cpp` | ROS 2 node: subscriptions, TF/extrinsics, GPS ingestion, publishers, `~/save` service |
| `src/pgo_backend.cpp` / `include/.../pgo_backend.hpp` | **ROS-free core**: keyframes, iSAM2 graph, GPS factors, ENU↔map alignment, loop commit, save |
| `include/.../gps_factors.hpp` | custom GTSAM factor `LeverArmGPSFactor` (`h(T)=t+R·lever`, analytic Jacobians) |
| `src/scan_context.cpp` | ScanContext descriptor + retrieval for loop-closure candidates |
| `src/geo_utils.cpp` | WGS84 geodetic↔ENU, and `fitYawTranslation2D` (the Kabsch 2-D yaw+translation fit) |
| `test/test_backend.cpp` | ROS-free self-tests (geodesy, Jacobians, ScanContext, a synthetic full run) |
| `launch/offline_pgo.launch.py` | offline driver: FAST-LIO + `pgo_node` + bag replay, one sim clock, auto-save |
| `config/pgo_offline.yaml` | parameters for offline replay |
| `roburoc_localization/scripts/run_pgo_batch.sh` | batch every raw bag in a folder |
| `roburoc_localization/scripts/pgo_to_bag.py` | optimizer artifacts → PlotJuggler `_pgo` rosbag (ENU) |

---

## 2. The factor graph

Keyframes are inserted every ~1 m / 15° of FAST-LIO motion. Each keyframe is a pose
`X(i)` in the **map** frame (which coincides with FAST-LIO's odom origin at boot, so
`map→odom` starts at identity).

```
prior → X(0) --LIO Between--> X(1) --LIO Between--> X(2) ...     keyframe chain
                |                    |                            LeverArmGPSFactor
                |  (per keyframe, if a GPS fix associates)        (fixed x1 / float x25σ /
                |                                                  none weak), Huber-robust
                `----- robust Between loop closures (ScanContext + GICP) -----'
```

- **Odometry factors** — relative FAST-LIO pose between consecutive keyframes.
- **GPS factors** — absolute antenna position, applied *inside* `LeverArmGPSFactor` with the
  lever arm so antenna offset + vehicle attitude stay consistent (see §4).
- **Loop closures** — ScanContext (20×60) proposes candidates; PCL **GICP** against a
  ±12-keyframe submap verifies them (gated by fitness/RMSE); accepted as Cauchy-robust
  `BetweenFactor`s. GICP runs in a background thread outside the backend mutex.
- Optimizer: **iSAM2**, incremental.

---

## 3. Offline data flow

No recorded bag contains FAST-LIO output, so FAST-LIO is **re-run live** and everything
runs on one simulation clock (`use_sim_time:=true`, bag `--clock`):

```
raw bag ──(--clock)──> FAST-LIO (roburoc_localization/fast_lio.launch.py)
   │                      ├─ /Odometry          (camera_init→body, relative LIO)
   │                      └─ /cloud_registered  (world-frame scan)
   ├─ /ublox_gps_node/fix      (NavSatFix: position + covariance)
   ├─ /ublox_gps_node/navpvt   (RTK fixed/float/none carrier-solution flag)
   └──────────────────> pgo_node ──> optimized trajectory + map, map→odom TF,
                                       /pgo/odometry /pgo/pose /pgo/path /pgo/map
```

`pgo_node` synchronizes `/Odometry` **with** `/cloud_registered` (message_filters
ApproximateTime, **RELIABLE** QoS — best-effort silently drops the large clouds) before
making a keyframe. It consumes FAST-LIO's raw *relative* odometry — not the Kabsch-aligned
`/odometry/lio/global` — and does its own ENU alignment.

---

## 4. GPS handling (the redesigned part)

The dataset is mostly RTK-float (~13 % fixed, ~12 % float, ~74 % none per run), so GPS
fusion is built around float rather than assuming plentiful fixed.

**Ingestion — `gps_input_mode: fused`** (`pgo_node.cpp`)
- Position + covariance from **`/ublox_gps_node/fix`** (NavSatFix, dense ~4 Hz).
- RTK class from the **paired `/ublox_gps_node/navpvt`** carrier-solution flag: the latest
  flag is cached (`cbNavPvtFlag`) and applied to the next `/fix` (`cbNavSatFix`). Fixes are
  stamped with node time (= bag time under sim time) so they align with keyframe times.

**Per-class covariance scaling** (`addGpsFactors`) — one `LeverArmGPSFactor` per fix:
`σ = reported_σ · √scale`, floored, z de-weighted, Huber-robust.

| class | scale (variance) | floor | notes |
|---|---|---|---|
| **fixed** | `gps_cov_scale_fixed` = 1 | `fixed_sigma_floor` 0.02 m | cm-level anchor; dropped if reported σ > `fixed_sigma_cap` (0.30 m) |
| **float** | `gps_cov_scale_float` = 25 (≈ 5× σ) | `float_sigma_floor` 0.30 m | locally consistent, biased → weak absolute anchor |
| **none** | (unscaled) | `nonrtk_sigma` 3.0 m | weak backstop; only if `use_nonrtk: true`, else discarded |

*(The former RTK-float drifting-bias chain — `BiasedLeverArmGPSFactor` + `B(m)` random walk —
is gone. Those params remain in the config, unused.)*

**ENU↔map alignment** (`maybeAlign`, Kabsch `fitYawTranslation2D`) — this is what makes
float-only runs work, and the two rules that matter:
1. **Homogeneous set only**: fixed-only if ≥ `align_min_pairs` (10) keyframes carry a fixed
   fix, else **float-only**. *Never mix fixed+float* — float's slowly-varying bias vs.
   fixed breaks a single rigid yaw+translation fit (this is exactly what left float-heavy
   runs stuck at metre-level RMS and produced **zero** GPS factors).
2. **Class-dependent accept threshold**: `align_max_rms` (1.0 m) for fixed, but
   `align_max_rms_float` (2.5 m) for float — float carries ~1–2 m of drift over tens of
   metres, which cm-level thresholds reject.

**GPS factors are gated on alignment**: fixes are cached until alignment succeeds, then all
are added and every later fix is added immediately. If alignment never succeeds, the run has
LIO + loop closures only (no ENU anchor).

Results across the `test_day_05_28` set with this design (previously most bags got 0 GPS):

| bag | keyframes | loops | fixed | float | none | align RMS |
|---|---|---|---|---|---|---|
| 161115 | 569 | 24 | 87 | 406 | 76 | 0.35 m (fixed) |
| 162109 | 571 | 25 | 0 | 508 | 63 | 1.28 m (float) |
| 162957 | 572 | 25 | 0 | 512 | 60 | 1.23 m (float) |
| 163851 | 568 | 23 | 0 | 492 | 76 | 1.24 m (float) |
| 165026 | 580 | 25 | 0 | 503 | 77 | 1.27 m (float) |

---

## 5. Frames & extrinsics

- **map** ≡ FAST-LIO odom origin at the first keyframe; `map→odom` TF starts at identity and
  stays smooth. GPS is a *constraint*, not the frame definition.
- The **ENU↔map** transform (yaw + translation, 4-DoF) is estimated online and exported to
  `georeference.txt` (`enu_to_map_yaw_rad`, `enu_to_map_t_xy`, `enu_to_map_z_offset`).
- Antenna lever arm (`livox_frame→gps_frame`) and output frame (`livox_frame→base_link`) are
  read from TF at startup, else from the fallback params. Offline TF lacks these, so
  `pgo_offline.yaml` sets `antenna_lever` / `t_body_base` / `q_body_base` computed from
  `roburoc_localization/config/sensor_mount.yaml`.

---

## 6. Build & run

**Dependencies:** GTSAM 4.1 (borglab PPA), `ublox_msgs` (`~/rtk_ws`), `fast_lio` +
`livox_ros_driver2` (`~/lio_ws`), PCL/Eigen. Build:

```bash
source /opt/ros/humble/setup.bash && source ~/rtk_ws/install/setup.bash
cd ~/AAU-RobuROC4 && colcon build --packages-select lidar_gps_pgo \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**Environment (important — see §8):**

```bash
source /opt/ros/humble/setup.bash ~/lio_ws/install/setup.bash \
       ~/rtk_ws/install/setup.bash ~/AAU-RobuROC4/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp          # NOT cyclone (large-msg drops)
# one-time (persist in /etc/sysctl.d): sudo sysctl -w net.core.rmem_max=2147483647
```

**One bag:**
```bash
ros2 launch lidar_gps_pgo offline_pgo.launch.py \
    raw_bag:=~/data/rosbags/test_day_05_28/roburoc_lio_20260528_161115 rate:=1.0
```

**All bags in a folder** (sources workspaces, sets FastDDS, cleans stragglers):
```bash
~/AAU-RobuROC4/src/roburoc_localization/scripts/run_pgo_batch.sh \
    ~/data/rosbags/test_day_05_28 1.0 false      # <folder> <rate> <skip_existing>
```

---

## 7. Outputs & comparison

Per bag, `offline_pgo.launch.py` auto-calls `/pgo/save` → `<bag>_pgo_artifacts/`:
`optimized_poses_tum.txt` (evo-compatible), `graph.g2o`, `map.pcd`, `georeference.txt`.

`pgo_to_bag.py` (run by the batch) turns those into `<bag>_pgo` — a **PlotJuggler
rosbag** with the final loop-closed trajectory transformed into the pipeline ENU frame as
`/odometry/pgo/global`, plus the matching `_ekf` bag's `/odometry/filtered/global`,
`/odometry/gps`, `/odometry/lio/global` copied in. Open one bag → overlay PGO vs EKF vs GPS
vs LIO. (If a run never aligned, the topic is written in the raw map frame `map_pgo` with a
warning.) `test_day_05_28/compare_pgo_ekf.py` scores CTE vs `ground_truth_reference.npy`.

---

## 8. Environment gotchas (hard-won)

- **Use FastDDS, not CycloneDDS, for offline replay.** Cyclone drops the large
  `/livox/lidar` CustomMsgs at 1.0× on localhost → FAST-LIO starves → diverges ("No
  Effective Points", metre-scale jumps). FastDDS delivers them; scoped to the batch via
  `RMW_IMPLEMENTATION` so your online/RTK cyclone setup is untouched.
- **`net.core.rmem_max`** must be raised (2 GB) for large-message DDS; the Linux default
  (208 KB) throttles delivery. It resets on WSL2 restart unless in `/etc/sysctl.d`.
- **Do NOT set a custom `CYCLONEDDS_URI`** — it conflicts with `ROS_LOCALHOST_ONLY=1` and
  wedges the ROS daemon (`ros2 topic list` hangs).
- `ros2 bag play`'s process name is `"ros2 bag play …"`, not `rosbag2_player` — kill it with
  `pkill -f 'ros2 bag play'`. Killing a `ros2 launch` orphans its children; verify with `ps`.

---

## 9. Key tuning knobs (`config/pgo_offline.yaml`)

| param | default | effect |
|---|---|---|
| `gps_cov_scale_float` | 25 | float de-weighting (variance ×); ↑ trusts float less |
| `align_max_rms_float` | 2.5 m | float alignment accept threshold; ↑ if float bags won't align |
| `use_nonrtk` | true | keep `none` fixes as weak 3 m backstops vs. discard |
| `keyframe_dist` / `_angle_deg` | 1.0 m / 15° | keyframe density |
| `sc_dist_threshold` | 0.13 | ↑ → more loop candidates (GICP still gates) |
| `icp_min_fitness` / `icp_max_rmse` | 0.45 / 0.40 | loop acceptance gates |
| `odom_sigma_*` vs GPS scales | — | how hard GPS bends the LIO chain |
```
