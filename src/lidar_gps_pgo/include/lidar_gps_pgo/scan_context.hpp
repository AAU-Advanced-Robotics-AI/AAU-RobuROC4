// Scan Context place recognition (Kim & Kim, IROS 2018), Eigen implementation.
//
// Descriptors are built from keyframe clouds in the *body* frame (pose
// independent). Retrieval: brute-force ring-key (rotation invariant) top-K,
// then full descriptor cosine distance minimised over column (yaw) shifts.
//
// Yaw convention (verified by unit test): a detection (c, yaw) means
// T_candidate_query.rotation() ~= Rz(yaw), i.e. rotating query-frame points
// by Rz(yaw) roughly aligns them with the candidate frame -> ICP init.
#pragma once

#include <Eigen/Core>
#include <optional>
#include <vector>

namespace lidar_gps_pgo {

struct ScanContextParams {
  int num_ring = 20;
  int num_sector = 60;
  double max_radius = 80.0;
  double min_range = 1.0;
  double lidar_height = 1.0;   // sensor height above ground [m]
  int num_candidates = 10;
  double dist_threshold = 0.13;
  int exclude_recent = 50;     // keyframes
};

struct ScanContextDetection {
  int candidate;
  double yaw;        // [rad]
  double distance;   // descriptor distance
};

class ScanContextManager {
 public:
  explicit ScanContextManager(const ScanContextParams& p = {}) : p_(p) {}

  // points: Nx3 body-frame points of the new keyframe (index = current size).
  void add(const Eigen::Matrix<double, Eigen::Dynamic, 3>& points);

  std::optional<ScanContextDetection> detect(int query_idx) const;

  size_t size() const { return descs_.size(); }

 private:
  static double wrap2pi(double a);
  Eigen::MatrixXd makeDescriptor(
      const Eigen::Matrix<double, Eigen::Dynamic, 3>& pts) const;
  // returns {distance, shift}
  std::pair<double, int> descriptorDistance(const Eigen::MatrixXd& q,
                                            const Eigen::MatrixXd& c) const;

  ScanContextParams p_;
  std::vector<Eigen::MatrixXd> descs_;
  std::vector<Eigen::VectorXd> ring_keys_;
};

}  // namespace lidar_gps_pgo
