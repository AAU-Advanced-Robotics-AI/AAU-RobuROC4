# lidar_gps_pgo

C++ GTSAM pose-graph optimization (PGO) backend for an outdoor robot running
**FAST-LIO2** (Livox Mid-360) and **RTK GPS** (ublox ZED-F9P) on
**ROS 2 Humble**.

```
prior -> X(0) --LIO Between--> X(1) --LIO Between--> X(2) ...  keyframe chain
           | LeverArmGPSFactor              (RTK fixed)
           | BiasedLeverArmGPSFactor---B(m) (RTK float; B = drifting bias)
           `---- robust BetweenFactor loop closures (ScanContext + ICP) ----'
                              B(m) --random walk--> B(m+1) ...
```

* **Odometry factors** — relative FAST-LIO2 poses between keyframes
  (1 m / 15° spacing).
* **RTK fixed** — lever-arm GPS factors (`h(T) = t + R·l`, analytic
  Jacobians, Huber-robust), sigma from NavPVT `hAcc`/`vAcc` floored at 2 cm.
* **RTK float** — *not* treated as inflated white noise. Field data shows the
  float error is a large, slowly drifting bias (e.g. 2.2 m drift in 5 min at
  ~0.1 m reported covariance); white-noise factors would drag the whole
  trajectory toward that bias. Instead each contiguous float stretch gets a
  chain of `Point3` bias states: a magnitude prior
  (`float_bias_prior_sigma`, 2 m) on the first node, random-walk links
  (`float_bias_rw_sigma`·√dt, 0.1 m/√s) along the chain, and a small
  measurement sigma (`float_meas_sigma`, 0.15 m). Float data then mostly
  constrains local shape plus a weak absolute bound, and the optimizer
  estimates the bias explicitly.
* **Loop closures** — ScanContext (20×60) on keyframe clouds, verified by
  PCL GICP against a ±12-keyframe submap with the ScanContext yaw as initial
  guess, gated by inlier fitness/RMSE, added as Cauchy-robust
  BetweenFactors. Runs in a background thread; ICP never blocks odometry.
* Optimizer: **iSAM2**, incremental.

## GPS quality classification

`gps_input_mode: navpvt` (default) reads `ublox_msgs/NavPVT` and uses the
carrier solution bits of `flags` (0x40 = float, 0x80 = fixed), plus
`fixType>=3` and the gnssFixOK bit — the reliable way on a ZED-F9P.
NavPVT has no header stamp, so messages are stamped with node time (equals
bag time under `use_sim_time`). `gps_input_mode: navsatfix` is a fallback
that classifies by covariance thresholds (`fixed_max_std`, `float_max_std`);
note your driver's covariance can be wildly optimistic in float mode.

## Frames and extrinsics

* FAST-LIO2's `/Odometry` is the **livox_frame** w.r.t. its start pose; that
  child frame is taken as the graph's body frame automatically.
* The **map frame coincides with the LIO odom frame at startup**, so the
  published `map -> odom` TF starts at identity and stays smooth. GPS is a
  set of constraints, not the frame definition; the ENU↔map transform (yaw +
  translation, estimated online after `align_min_travel` of RTK-fixed
  travel) is exported in `georeference.txt`.
* Antenna lever arm (**livox_frame → gps_frame**) and the output frame
  (**livox_frame → base_link**) are looked up from TF once at startup;
  params `antenna_lever` / `t_body_base` / `q_body_base` are fallbacks.
  The lever arm is applied *inside* the GPS factors with analytic Jacobians,
  so antenna offset and vehicle attitude stay consistent.

## Build

```bash
sudo apt install libpcl-dev libeigen3-dev
# GTSAM 4.1.x/4.2.0 (Humble-era). Either the borglab PPA:
sudo add-apt-repository ppa:borglab/gtsam-release-4.1 && sudo apt install libgtsam-dev
# ...or build 4.2.0 from source with default options.
# ublox_msgs from your ublox driver workspace (KumarRobotics/ublox, ros2 branch)

cd ~/ros2_ws/src && ln -s /path/to/lidar_gps_pgo .
cd ~/ros2_ws && colcon build --packages-select lidar_gps_pgo
```

The custom factors use the classic GTSAM 4.1/4.2.0 API
(`boost::optional` Jacobians). For GTSAM ≥ 4.2.1 with the new API, switch
the two `evaluateError` signatures in `include/lidar_gps_pgo/gps_factors.hpp`
to `OptionalMatrixType` (5-line change).

## Test on a bag

```bash
ros2 launch lidar_gps_pgo pgo.launch.py use_sim_time:=true
ros2 bag play my_field_run --clock

# rviz2, fixed frame "map": /pgo/path, /pgo/odometry, /pgo/loop_markers
ros2 service call /pgo/save std_srvs/srv/Trigger
```

`save` writes to `save_directory` (default `~/pgo_output`):
`optimized_poses_tum.txt` (evo-compatible), `graph.g2o`, `map.pcd`,
`georeference.txt` (datum + ENU→map transform → map coords back to lat/lon).

## Topics

| dir | topic | type | note |
|---|---|---|---|
| in | `/Odometry` | nav_msgs/Odometry | FAST-LIO2 |
| in | `/cloud_registered` | sensor_msgs/PointCloud2 | odom frame (default); or `/cloud_registered_body` with `cloud_frame_mode: body` |
| in | `/navpvt` | ublox_msgs/NavPVT | RTK quality + position |
| out | `/pgo/odometry` | Odometry | corrected `base_link`, full LIO rate |
| out | `/pgo/pose`, `/pgo/path` | Odometry / Path | keyframe rate |
| out | `/pgo/map` | PointCloud2 | on save / `map_pub_period` |
| out | `/pgo/loop_markers` | Marker | LINE_LIST |
| out | TF `map -> <odom frame>` | | odom frame read from `/Odometry` |

`/Laser_map` is not used: the backend aggregates its own map from keyframe
clouds so that loop closures and GPS corrections re-deform it consistently.

## Parameters to check first (`config/pgo.yaml`)

* TF must contain `livox_frame -> gps_frame` and `livox_frame -> base_link`
  (or set the fallback params).
* `sc_lidar_height` — Mid-360 height above ground.
* `float_bias_rw_sigma` — from your data: ~2.2 m drift over 300 s ≈
  0.13 m/√s; default 0.10. Raise it if float sessions show faster drift.
* `sc_dist_threshold` (0.13) — raise toward 0.2 to find more loop
  candidates (GICP still gates them), lower to be stricter.
* `icp_min_fitness` / `icp_max_rmse` — loop acceptance gates.
* `odom_sigma_*` vs GPS sigmas balance how much RTK bends the LIO chain.

## Design notes / limits

* Everything latency-critical is C++: keyframe insertion + iSAM2 update a
  few ms; ScanContext retrieval ~ms; GICP verification runs in a background
  thread outside the backend mutex (three-phase task/verify/commit API).
* Float bias chains add one 3-DoF variable per float keyframe — negligible.
* Alignment is yaw+translation (4-DoF), valid because FAST-LIO
  gravity-aligns its odom frame. If only float data exists, alignment waits
  for `align_min_travel_float` (40 m) since the bias can tilt a short fit.
* Doppler velocity from the F9P is currently unused; adding
  ENU velocity factors on keyframe pairs would be a natural extension.

## Core tests (no ROS required)

`test/test_backend.cpp` builds with `-DBUILD_TESTING=ON` (or standalone
against gtsam+PCL) and runs: geodetic/ENU round-trip, 2D alignment fit,
**numeric vs analytic Jacobian checks for both custom factors**, ScanContext
yaw-sign convention, and a full synthetic run — 80 m square, yaw-drifting
odometry, RTK-fixed legs, an RTK-float leg whose simulated bias drifts to
~2.1 m at 0.1 m reported sigma, a GPS outage on the final leg, and a
revisit. Asserts: alignment yaw error < 2°, ≥ 1 accepted loop closure, and
endpoint error < 1 m (raw odometry drifts ~15 m).
