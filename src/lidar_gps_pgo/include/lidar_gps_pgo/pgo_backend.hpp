// ROS-free pose-graph back-end (GTSAM / iSAM2 + PCL for loop verification).
//
// Graph:
//   prior ->X(0) --odom Between--> X(1) --> ... keyframe chain (FAST-LIO2)
//            |LeverArmGPSFactor (RTK fixed)
//            |BiasedLeverArmGPSFactor--B(m)  (RTK float; B = drifting bias,
//            |                               random-walk chained + priored)
//            `-- robust BetweenFactor loop closures (ScanContext + ICP)
//
// The map frame coincides with the FAST-LIO odom frame at the first keyframe
// (prior anchors X(0) at the first odometry pose), so T_map_odom starts at
// identity and stays smooth. ENU measurements are mapped into the map frame
// with an online-estimated yaw+translation (both frames are gravity aligned).
//
// RTK float error model: measurements carry a large, slowly drifting bias
// with little white noise (observed: metres of drift at ~0.1 m reported
// covariance). Each contiguous float stretch gets a chain of Point3 bias
// variables: prior sigma float_bias_prior_sigma on the first, random-walk
// BetweenFactors with sigma float_bias_rw_sigma*sqrt(dt) along the chain,
// and a small white-noise sigma float_meas_sigma on the factor itself. Float
// data therefore constrains mostly *shape* (relative motion) plus a weak
// absolute position bound, instead of dragging the graph toward the bias.
//
// Thread-safety: not thread-safe; callers hold a mutex. Loop closure is
// split so ICP runs outside the lock:
//   nextLoopTask() [lock] -> verifyLoop() [no lock] -> commitLoop() [lock].
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <deque>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "lidar_gps_pgo/geo_utils.hpp"
#include "lidar_gps_pgo/scan_context.hpp"

namespace lidar_gps_pgo {

using CloudT = pcl::PointCloud<pcl::PointXYZ>;

enum class GpsQuality { kFixed, kFloat, kOther };

struct BackendConfig {
  // keyframing
  double keyframe_dist = 1.0;         // [m]
  double keyframe_angle_deg = 15.0;   // [deg]
  double keyframe_voxel = 0.25;       // [m]
  double cloud_min_range = 0.5;       // [m]
  double cloud_max_range = 100.0;     // [m] (0 = no cap)
  // factor noise
  double prior_sigma_rot = 0.05;      // [rad]
  double prior_sigma_trans = 0.10;    // [m]
  double odom_sigma_rot = 0.01;       // [rad] per keyframe step
  double odom_sigma_trans = 0.05;     // [m]
  // GPS
  bool use_gps = true;
  bool use_float = true;
  bool use_nonrtk = false;
  double fixed_sigma_floor = 0.02;    // [m]
  double fixed_sigma_cap = 0.30;      // [m] ignore "fixed" worse than this
  double float_sigma_floor = 0.30;    // [m] floor for scaled float sigma
  // Per-quality covariance scaling of the NavSatFix reported covariance. The
  // factor sigma is  reported_sigma * sqrt(scale)  (scale multiplies variance).
  // Fixed keeps its reported cm-level cov; float is de-weighted (locally
  // consistent but biased), non-RTK relies on nonrtk_sigma instead.
  double gps_cov_scale_fixed = 1.0;
  double gps_cov_scale_float = 25.0;  // 25x variance ~= 5x sigma
  // ---- legacy drifting-bias float model (unused: replaced by cov scaling) ---
  double float_meas_sigma = 0.15;
  double float_bias_prior_sigma = 2.0;
  double float_bias_rw_sigma = 0.10;
  double float_bias_session_gap = 30.0;
  double nonrtk_sigma = 3.0;          // [m]
  double gps_z_sigma_scale = 2.0;
  bool gps_robust = true;             // Huber on fixed/nonrtk factors
  Eigen::Vector3d antenna_lever = Eigen::Vector3d::Zero();  // body frame [m]
  double gps_time_tol = 0.20;         // [s]
  double gps_max_interp_gap = 0.6;    // [s]
  double gps_assoc_timeout = 5.0;     // [s]
  // ENU <-> map alignment
  double align_min_travel = 10.0;     // [m] (fixed-quality pairs)
  double align_min_travel_float = 40.0;  // [m] if only float available
  int align_min_pairs = 10;
  double align_max_rms = 1.0;         // [m] accept threshold for fixed alignment
  double align_max_rms_float = 2.5;   // [m] looser threshold for float-only fit
  // loop closure
  bool enable_loop_closure = true;
  ScanContextParams sc;
  int loop_min_query_gap = 5;         // [keyframes]
  std::string icp_method = "gicp";    // gicp | point_to_point
  double icp_voxel = 0.4;             // [m]
  int submap_size = 12;               // +- keyframes around candidate
  double icp_max_corr = 2.0;          // [m]
  double icp_min_fitness = 0.45;      // inlier ratio to accept
  double icp_max_rmse = 0.40;         // [m]
  double loop_sigma_rot = 0.05;       // [rad]
  double loop_sigma_trans = 0.20;     // [m]
  double loop_robust_k = 1.0;         // Cauchy
  // outputs
  double map_voxel = 0.30;            // [m]
};

struct GpsMeasurement {
  Eigen::Vector3d enu;
  Eigen::Vector3d sigma;   // reported std dev per ENU axis
  GpsQuality quality;
};

struct Keyframe {
  int idx;
  double t;
  gtsam::Pose3 odom_pose;            // LIO odom frame
  CloudT::Ptr cloud;                 // body frame, downsampled
  std::optional<GpsMeasurement> gps;
};

struct LoopTask {
  int query, candidate;
  double yaw, sc_dist;
  CloudT::Ptr src;   // query keyframe cloud (body frame)
  CloudT::Ptr tgt;   // submap in candidate keyframe frame
};

struct LoopResult {
  int query, candidate;
  gtsam::Pose3 t_candidate_query;
  double fitness, rmse;
};

class PGOBackend {
 public:
  using LogFn = std::function<void(const std::string&)>;
  explicit PGOBackend(const BackendConfig& cfg, LogFn log = nullptr);

  // --- odometry / keyframes ---
  bool needsKeyframe(const gtsam::Pose3& t_odom_body) const;
  // cloud: body-frame points. now: current time (for GPS assoc timeouts;
  // pass the latest odometry stamp so bag replay works).
  int addKeyframe(double t, const gtsam::Pose3& t_odom_body,
                  const CloudT::ConstPtr& cloud_body, double now);

  // --- GPS ---
  // Update the antenna lever arm (e.g. once TF becomes available). Safe to
  // call any time before the first GPS factor is added; afterwards it only
  // affects new factors.
  void setAntennaLever(const Eigen::Vector3d& l) { cfg_.antenna_lever = l; }
  void addGpsFix(double t, double lat_deg, double lon_deg, double alt,
                 const Eigen::Vector3d& sigma_enu, GpsQuality quality,
                 double now);

  // --- loop closure (see threading note above) ---
  std::optional<LoopTask> nextLoopTask(int max_tries = 5);
  std::optional<LoopResult> verifyLoop(const LoopTask& task) const;
  void commitLoop(const LoopResult& r);

  // --- outputs ---
  const gtsam::Pose3& mapToOdom() const { return t_map_odom_; }
  const std::vector<Keyframe>& keyframes() const { return keyframes_; }
  const std::vector<gtsam::Pose3>& optimizedPoses() const { return opt_poses_; }
  const std::vector<std::pair<int, int>>& loops() const { return loops_; }
  bool hasAlignment() const { return alignment_.has_value(); }
  const std::optional<Alignment2D>& alignment() const { return alignment_; }
  const std::map<std::string, int>& gpsFactorCounts() const {
    return n_gps_factors_;
  }
  CloudT::Ptr buildMap(double voxel = -1.0) const;
  std::vector<std::string> save(const std::string& outdir) const;

 private:
  struct BufferedFix {
    double t;
    Eigen::Vector3d enu;
    Eigen::Vector3d sigma;
    GpsQuality quality;
  };

  void refresh();
  void associatePendingGps(double now);
  std::optional<GpsMeasurement> interpolateGps(double t) const;
  void attachGps(int idx, const GpsMeasurement& m);
  void maybeAlign();
  void addGpsFactors(const std::vector<int>& idxs);
  Eigen::Vector3d enuToMap(const Eigen::Vector3d& enu) const;

  BackendConfig cfg_;
  LogFn log_;

  gtsam::ISAM2 isam_;
  gtsam::NonlinearFactorGraph g2o_graph_;  // priors + betweens for export
  gtsam::SharedNoiseModel prior_noise_, odom_noise_, loop_noise_,
      loop_noise_plain_;

  GeoConverter geo_;
  ScanContextManager sc_;

  std::vector<Keyframe> keyframes_;
  std::vector<gtsam::Pose3> opt_poses_;
  gtsam::Pose3 t_map_odom_;
  std::optional<Alignment2D> alignment_;
  std::vector<std::pair<int, int>> loops_;
  std::map<std::string, int> n_gps_factors_;

  gtsam::Values est_;
  std::deque<BufferedFix> gps_buf_;
  std::vector<int> pending_gps_;      // keyframes waiting for a fix
  std::vector<int> unfactored_gps_;   // fixes cached pre-alignment
  std::deque<int> sc_queue_;
  int last_loop_query_ = -1000000;
  int align_fail_count_ = 0;
  // float bias chain state
  int n_bias_ = 0;
  int last_bias_key_ = -1;
  double last_float_t_ = -1e18;
};

}  // namespace lidar_gps_pgo
