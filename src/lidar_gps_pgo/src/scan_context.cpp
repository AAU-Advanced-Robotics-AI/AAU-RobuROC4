#include "lidar_gps_pgo/scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace lidar_gps_pgo {

void ScanContextManager::add(
    const Eigen::Matrix<double, Eigen::Dynamic, 3>& points) {
  Eigen::MatrixXd d = makeDescriptor(points);
  ring_keys_.push_back(d.rowwise().mean());
  descs_.push_back(std::move(d));
}

Eigen::MatrixXd ScanContextManager::makeDescriptor(
    const Eigen::Matrix<double, Eigen::Dynamic, 3>& pts) const {
  Eigen::MatrixXd sc = Eigen::MatrixXd::Zero(p_.num_ring, p_.num_sector);
  for (Eigen::Index i = 0; i < pts.rows(); ++i) {
    const double x = pts(i, 0), y = pts(i, 1);
    const double r = std::hypot(x, y);
    if (r <= p_.min_range || r >= p_.max_radius) continue;
    int ring = std::min(static_cast<int>(r / p_.max_radius * p_.num_ring),
                        p_.num_ring - 1);
    int sector = static_cast<int>((std::atan2(y, x) + M_PI) / (2.0 * M_PI) *
                                  p_.num_sector);
    sector = std::clamp(sector, 0, p_.num_sector - 1);
    const double z = std::max(pts(i, 2) + p_.lidar_height, 0.0);
    sc(ring, sector) = std::max(sc(ring, sector), z);
  }
  return sc;
}

std::optional<ScanContextDetection> ScanContextManager::detect(
    int query_idx) const {
  const int pool_end = query_idx - p_.exclude_recent;
  if (pool_end < 1 || query_idx >= static_cast<int>(descs_.size()))
    return std::nullopt;
  const Eigen::MatrixXd& q_desc = descs_[query_idx];
  const Eigen::VectorXd& q_key = ring_keys_[query_idx];
  if (q_key.norm() < 1e-9) return std::nullopt;

  // ring-key top-K (brute force; 20-D x few-thousand keys is negligible)
  std::vector<std::pair<double, int>> key_d(pool_end);
  for (int i = 0; i < pool_end; ++i)
    key_d[i] = {(ring_keys_[i] - q_key).squaredNorm(), i};
  const int k = std::min(p_.num_candidates, pool_end);
  std::partial_sort(key_d.begin(), key_d.begin() + k, key_d.end());

  int best_c = -1, best_shift = 0;
  double best_d = std::numeric_limits<double>::infinity();
  for (int j = 0; j < k; ++j) {
    const int c = key_d[j].second;
    if (descs_[c].norm() < 1e-9) continue;
    auto [d, s] = descriptorDistance(q_desc, descs_[c]);
    if (d < best_d) { best_d = d; best_c = c; best_shift = s; }
  }
  if (best_c < 0 || best_d > p_.dist_threshold) return std::nullopt;
  const double yaw =
      wrap2pi(2.0 * M_PI * static_cast<double>(best_shift) / p_.num_sector);
  return ScanContextDetection{best_c, yaw, best_d};
}

double ScanContextManager::wrap2pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

std::pair<double, int> ScanContextManager::descriptorDistance(
    const Eigen::MatrixXd& q, const Eigen::MatrixXd& c) const {
  const int ns = p_.num_sector;
  const Eigen::VectorXd norm_c = c.colwise().norm();
  const Eigen::VectorXd norm_q = q.colwise().norm();
  double best_d = std::numeric_limits<double>::infinity();
  int best_s = 0;
  for (int s = 0; s < ns; ++s) {
    double sum = 0.0;
    int cnt = 0;
    for (int col = 0; col < ns; ++col) {
      const int qc = (col - s + ns) % ns;  // column of rolled-by-s query
      const double den = norm_q(qc) * norm_c(col);
      if (den < 1e-9) continue;
      sum += q.col(qc).dot(c.col(col)) / den;
      ++cnt;
    }
    if (cnt == 0) continue;
    const double d = 1.0 - sum / static_cast<double>(cnt);
    if (d < best_d) { best_d = d; best_s = s; }
  }
  return {best_d, best_s};
}

}  // namespace lidar_gps_pgo
