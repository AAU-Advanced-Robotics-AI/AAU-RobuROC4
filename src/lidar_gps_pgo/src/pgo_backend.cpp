#include "lidar_gps_pgo/pgo_backend.hpp"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

#include "lidar_gps_pgo/gps_factors.hpp"

namespace lidar_gps_pgo {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::X;

namespace {

CloudT::Ptr voxelDownsample(const CloudT::ConstPtr& in, double voxel) {
  auto out = std::make_shared<CloudT>();
  if (voxel <= 0.0) { *out = *in; return out; }
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setLeafSize(voxel, voxel, voxel);
  vg.setInputCloud(in);
  vg.filter(*out);
  return out;
}

CloudT::Ptr rangeFilter(const CloudT::ConstPtr& in, double min_r,
                        double max_r) {
  auto out = std::make_shared<CloudT>();
  out->reserve(in->size());
  for (const auto& p : *in) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
      continue;
    const double r = std::hypot(p.x, p.y);
    if (min_r > 0.0 && r < min_r) continue;
    if (max_r > 0.0 && r > max_r) continue;
    out->push_back(p);
  }
  return out;
}

Eigen::Matrix<double, Eigen::Dynamic, 3> toEigen(const CloudT& c) {
  Eigen::Matrix<double, Eigen::Dynamic, 3> m(c.size(), 3);
  for (size_t i = 0; i < c.size(); ++i)
    m.row(i) << c[i].x, c[i].y, c[i].z;
  return m;
}

const char* qualityName(GpsQuality q) {
  switch (q) {
    case GpsQuality::kFixed: return "fixed";
    case GpsQuality::kFloat: return "float";
    default: return "other";
  }
}

}  // namespace

PGOBackend::PGOBackend(const BackendConfig& cfg, LogFn log)
    : cfg_(cfg),
      log_(log ? std::move(log) : [](const std::string&) {}),
      sc_(cfg.sc) {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = 0.1;
  params.relinearizeSkip = 1;
  isam_ = gtsam::ISAM2(params);

  auto sig6 = [](double r, double t) {
    return gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << r, r, r, t, t, t).finished());
  };
  prior_noise_ = sig6(cfg_.prior_sigma_rot, cfg_.prior_sigma_trans);
  odom_noise_ = sig6(cfg_.odom_sigma_rot, cfg_.odom_sigma_trans);
  loop_noise_plain_ = sig6(cfg_.loop_sigma_rot, cfg_.loop_sigma_trans);
  loop_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(cfg_.loop_robust_k),
      loop_noise_plain_);

  n_gps_factors_ = {{"fixed", 0}, {"float", 0}, {"other", 0}};
}

// ------------------------------------------------------------------ keyframes
bool PGOBackend::needsKeyframe(const gtsam::Pose3& t_odom_body) const {
  if (keyframes_.empty()) return true;
  const gtsam::Pose3 d = keyframes_.back().odom_pose.between(t_odom_body);
  const double dt = d.translation().norm();
  const double dr = gtsam::Rot3::Logmap(d.rotation()).norm();
  return dt > cfg_.keyframe_dist ||
         dr > cfg_.keyframe_angle_deg * M_PI / 180.0;
}

int PGOBackend::addKeyframe(double t, const gtsam::Pose3& t_odom_body,
                            const CloudT::ConstPtr& cloud_body, double now) {
  const int i = static_cast<int>(keyframes_.size());
  CloudT::Ptr cloud = voxelDownsample(
      rangeFilter(cloud_body, cfg_.cloud_min_range, cfg_.cloud_max_range),
      cfg_.keyframe_voxel);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  if (i == 0) {
    // Anchor the map frame at the LIO odom frame's first pose.
    gtsam::PriorFactor<gtsam::Pose3> f(X(0), t_odom_body, prior_noise_);
    graph.add(f);
    g2o_graph_.add(f);
    values.insert(X(0), t_odom_body);
  } else {
    const gtsam::Pose3 rel =
        keyframes_.back().odom_pose.between(t_odom_body);
    gtsam::BetweenFactor<gtsam::Pose3> f(X(i - 1), X(i), rel, odom_noise_);
    graph.add(f);
    g2o_graph_.add(f);
    values.insert(X(i), opt_poses_.back().compose(rel));
  }
  keyframes_.push_back(Keyframe{i, t, t_odom_body, cloud, std::nullopt});
  isam_.update(graph, values);
  refresh();

  sc_.add(toEigen(*cloud));
  if (cfg_.enable_loop_closure) sc_queue_.push_back(i);
  if (cfg_.use_gps) {
    pending_gps_.push_back(i);
    associatePendingGps(now);
    maybeAlign();
  }
  return i;
}

// ------------------------------------------------------------------------ GPS
void PGOBackend::addGpsFix(double t, double lat, double lon, double alt,
                           const Eigen::Vector3d& sigma_enu,
                           GpsQuality quality, double now) {
  if (!cfg_.use_gps) return;
  if (quality == GpsQuality::kFloat && !cfg_.use_float) return;
  if (quality == GpsQuality::kOther && !cfg_.use_nonrtk) return;
  if (quality == GpsQuality::kFixed &&
      std::max(sigma_enu.x(), sigma_enu.y()) > cfg_.fixed_sigma_cap)
    return;  // implausible "fixed"
  if (!geo_.hasDatum()) {
    geo_.setDatum(lat, lon, alt);
    char buf[128];
    std::snprintf(buf, sizeof(buf), "GPS datum set: %.7f %.7f %.2f", lat, lon,
                  alt);
    log_(buf);
  }
  BufferedFix fix{t, geo_.toEnu(lat, lon, alt), sigma_enu.cwiseMax(0.0),
                  quality};
  if (gps_buf_.empty() || t >= gps_buf_.back().t) {
    gps_buf_.push_back(std::move(fix));
  } else {
    auto it = std::lower_bound(
        gps_buf_.begin(), gps_buf_.end(), t,
        [](const BufferedFix& b, double tt) { return b.t < tt; });
    gps_buf_.insert(it, std::move(fix));
  }
  while (!gps_buf_.empty() && gps_buf_.front().t < t - 600.0)
    gps_buf_.pop_front();
  associatePendingGps(now);
  maybeAlign();
}

void PGOBackend::associatePendingGps(double now) {
  if (pending_gps_.empty()) return;
  std::vector<int> keep;
  const double latest = gps_buf_.empty() ? -1e18 : gps_buf_.back().t;
  for (int i : pending_gps_) {
    const double tk = keyframes_[i].t;
    if (latest < tk + cfg_.gps_time_tol) {
      if (now - tk < cfg_.gps_assoc_timeout) keep.push_back(i);
      continue;  // a bracketing fix may still arrive (or timed out: drop)
    }
    auto m = interpolateGps(tk);
    if (m) attachGps(i, *m);
  }
  pending_gps_ = std::move(keep);
}

std::optional<GpsMeasurement> PGOBackend::interpolateGps(double t) const {
  if (gps_buf_.empty()) return std::nullopt;
  auto it = std::lower_bound(
      gps_buf_.begin(), gps_buf_.end(), t,
      [](const BufferedFix& b, double tt) { return b.t < tt; });
  // bracketing pair -> linear interpolation
  if (it != gps_buf_.begin() && it != gps_buf_.end()) {
    const BufferedFix& b0 = *(it - 1);
    const BufferedFix& b1 = *it;
    if (b1.t - b0.t <= cfg_.gps_max_interp_gap && b0.quality == b1.quality) {
      const double a = (t - b0.t) / std::max(b1.t - b0.t, 1e-9);
      return GpsMeasurement{(1 - a) * b0.enu + a * b1.enu,
                            b0.sigma.cwiseMax(b1.sigma), b0.quality};
    }
  }
  // nearest within tolerance (never across a quality change via interp)
  const BufferedFix* best = nullptr;
  double best_dt = cfg_.gps_time_tol;
  for (auto jt : {it == gps_buf_.begin() ? gps_buf_.end() : it - 1, it}) {
    if (jt == gps_buf_.end()) continue;
    const double dt = std::abs(jt->t - t);
    if (dt <= best_dt) { best_dt = dt; best = &*jt; }
  }
  if (!best) return std::nullopt;
  return GpsMeasurement{best->enu, best->sigma, best->quality};
}

void PGOBackend::attachGps(int idx, const GpsMeasurement& m) {
  keyframes_[idx].gps = m;
  if (alignment_) {
    addGpsFactors({idx});
  } else {
    unfactored_gps_.push_back(idx);
  }
}

void PGOBackend::maybeAlign() {
  if (alignment_ || !cfg_.use_gps) return;
  std::vector<int> fixed, floatv;
  for (const auto& kf : keyframes_) {
    if (!kf.gps) continue;
    if (kf.gps->quality == GpsQuality::kFixed) fixed.push_back(kf.idx);
    else if (kf.gps->quality == GpsQuality::kFloat) floatv.push_back(kf.idx);
  }
  // Align on a HOMOGENEOUS set: fixed-only if enough fixed keyframes, else
  // float-only (float is locally consistent). Never mix fixed+float — the
  // float bias relative to fixed breaks the single rigid yaw+translation fit
  // (this is what left float-heavy runs stuck at metre-level RMS -> no GPS).
  const bool use_fixed = static_cast<int>(fixed.size()) >= cfg_.align_min_pairs;
  const std::vector<int>& sel = use_fixed ? fixed : floatv;
  const char* kind = use_fixed ? "fixed" : "float";
  const double min_travel =
      use_fixed ? cfg_.align_min_travel : cfg_.align_min_travel_float;
  if (static_cast<int>(sel.size()) < cfg_.align_min_pairs) return;

  std::vector<Eigen::Vector3d> enu, map_ant;
  for (int i : sel) {
    enu.push_back(keyframes_[i].gps->enu);
    map_ant.push_back(opt_poses_[i].transformFrom(cfg_.antenna_lever));
  }
  double travel = 0.0;
  for (const auto& p : map_ant)
    travel = std::max(travel, (p.head<2>() - map_ant.front().head<2>()).norm());
  if (travel < min_travel) return;

  // Float carries a slowly-drifting bias, so a rigid fit over tens of metres
  // has larger residual than cm-level fixed; use a looser accept threshold.
  const double max_rms =
      use_fixed ? cfg_.align_max_rms : cfg_.align_max_rms_float;
  auto a = fitYawTranslation2D(enu, map_ant);
  if (!a) return;
  if (a->rms > max_rms) {
    if (++align_fail_count_ % 20 == 1) {
      char buf[160];
      std::snprintf(buf, sizeof(buf),
                    "GPS alignment rms %.2f m > %.2f m; waiting for more data "
                    "(%zu %s pairs, %.1f m)",
                    a->rms, max_rms, sel.size(), kind, travel);
      log_(buf);
    }
    return;
  }
  alignment_ = *a;
  char buf[160];
  std::snprintf(buf, sizeof(buf),
                "GPS/odom alignment: yaw %.2f deg, rms %.3f m (%zu %s pairs, "
                "%.1f m)",
                a->yaw * 180.0 / M_PI, a->rms, sel.size(), kind, travel);
  log_(buf);
  std::vector<int> pending;
  std::swap(pending, unfactored_gps_);
  std::sort(pending.begin(), pending.end());
  addGpsFactors(pending);
}

Eigen::Vector3d PGOBackend::enuToMap(const Eigen::Vector3d& enu) const {
  const auto& a = *alignment_;
  const Eigen::Vector2d xy = a.R * enu.head<2>() + a.t;
  return {xy.x(), xy.y(), enu.z() + a.z_offset};
}

void PGOBackend::addGpsFactors(const std::vector<int>& idxs) {
  if (idxs.empty() || !alignment_) return;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  for (int i : idxs) {
    const auto& m = *keyframes_[i].gps;
    const gtsam::Point3 p_ant_map = enuToMap(m.enu);

    // Unified absolute GPS factor with per-quality covariance scaling:
    // sigma = reported_sigma * sqrt(cov_scale), floored, then robustified.
    // Fixed ~ cm, float de-weighted (~5x sigma), non-RTK falls back to
    // nonrtk_sigma. (Replaces the former RTK-float drifting-bias chain.)
    Eigen::Vector3d s = m.sigma;
    double floor = cfg_.nonrtk_sigma;
    if (m.quality == GpsQuality::kFixed) {
      s *= std::sqrt(cfg_.gps_cov_scale_fixed);
      floor = cfg_.fixed_sigma_floor;
    } else if (m.quality == GpsQuality::kFloat) {
      s *= std::sqrt(cfg_.gps_cov_scale_float);
      floor = cfg_.float_sigma_floor;
    }
    s = s.cwiseMax(floor);
    s.z() *= cfg_.gps_z_sigma_scale;
    gtsam::SharedNoiseModel noise = gtsam::noiseModel::Diagonal::Sigmas(s);
    if (cfg_.gps_robust) {
      noise = gtsam::noiseModel::Robust::Create(
          gtsam::noiseModel::mEstimator::Huber::Create(1.345), noise);
    }
    graph.add(LeverArmGPSFactor(X(i), p_ant_map, cfg_.antenna_lever, noise));
    ++n_gps_factors_[qualityName(m.quality)];
  }
  try {
    isam_.update(graph, values);
    if (idxs.size() > 3)
      for (int k = 0; k < 3; ++k) isam_.update();
  } catch (const std::exception& e) {
    log_(std::string("iSAM2 update failed on GPS batch: ") + e.what());
  }
  refresh();
}

// --------------------------------------------------------------- loop closure
std::optional<LoopTask> PGOBackend::nextLoopTask(int max_tries) {
  if (!cfg_.enable_loop_closure) { sc_queue_.clear(); return std::nullopt; }
  int tries = 0;
  while (!sc_queue_.empty() && tries < max_tries) {
    const int q = sc_queue_.front();
    sc_queue_.pop_front();
    ++tries;
    if (q - last_loop_query_ < cfg_.loop_min_query_gap) continue;
    auto det = sc_.detect(q);
    if (!det) continue;
    const int cand = det->candidate;
    if (keyframes_[q].cloud->size() < 100) continue;

    // target submap in the candidate's local frame (current estimates)
    const int lo = std::max(0, cand - cfg_.submap_size);
    const int hi = std::min({static_cast<int>(keyframes_.size()) - 1,
                             cand + cfg_.submap_size, q - 1});
    const gtsam::Pose3& t_c = opt_poses_[cand];
    auto tgt = std::make_shared<CloudT>();
    for (int j = lo; j <= hi; ++j) {
      const Eigen::Matrix4d t_cj = t_c.between(opt_poses_[j]).matrix();
      CloudT tmp;
      pcl::transformPointCloud(*keyframes_[j].cloud, tmp, t_cj.cast<float>());
      *tgt += tmp;
    }
    if (tgt->size() < 300) continue;
    auto src = std::make_shared<CloudT>(*keyframes_[q].cloud);
    return LoopTask{q, cand, det->yaw, det->distance, src, tgt};
  }
  return std::nullopt;
}

std::optional<LoopResult> PGOBackend::verifyLoop(const LoopTask& task) const {
  CloudT::Ptr src = voxelDownsample(task.src, cfg_.icp_voxel);
  CloudT::Ptr tgt = voxelDownsample(task.tgt, cfg_.icp_voxel);
  if (src->size() < 100 || tgt->size() < 300) return std::nullopt;

  Eigen::Matrix4f init = Eigen::Matrix4f::Identity();
  init.topLeftCorner<3, 3>() =
      Eigen::AngleAxisf(static_cast<float>(task.yaw),
                        Eigen::Vector3f::UnitZ()).toRotationMatrix();

  CloudT aligned;
  Eigen::Matrix4f final_tf;
  if (cfg_.icp_method == "gicp") {
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(cfg_.icp_max_corr);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(aligned, init);
    if (!icp.hasConverged()) return std::nullopt;
    final_tf = icp.getFinalTransformation();
  } else {
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaxCorrespondenceDistance(cfg_.icp_max_corr);
    icp.setMaximumIterations(60);
    icp.setTransformationEpsilon(1e-6);
    icp.setInputSource(src);
    icp.setInputTarget(tgt);
    icp.align(aligned, init);
    if (!icp.hasConverged()) return std::nullopt;
    final_tf = icp.getFinalTransformation();
  }

  // inlier fitness / rmse at a fine threshold
  const double fine_corr =
      std::max(2.0 * cfg_.icp_voxel, cfg_.icp_max_corr / 3.0);
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(tgt);
  std::vector<int> nn(1);
  std::vector<float> d2(1);
  size_t inliers = 0;
  double se = 0.0;
  for (const auto& p : aligned) {
    if (tree.nearestKSearch(p, 1, nn, d2) > 0 &&
        d2[0] <= fine_corr * fine_corr) {
      ++inliers;
      se += d2[0];
    }
  }
  const double fitness = static_cast<double>(inliers) / aligned.size();
  const double rmse = inliers ? std::sqrt(se / inliers) : 1e9;
  if (fitness < cfg_.icp_min_fitness || rmse > cfg_.icp_max_rmse) {
    char buf[160];
    std::snprintf(buf, sizeof(buf),
                  "Loop %d->%d rejected (sc %.3f, fitness %.2f, rmse %.2f)",
                  task.candidate, task.query, task.sc_dist, fitness, rmse);
    log_(buf);
    return std::nullopt;
  }
  return LoopResult{task.query, task.candidate,
                    gtsam::Pose3(final_tf.cast<double>()), fitness, rmse};
}

void PGOBackend::commitLoop(const LoopResult& r) {
  gtsam::NonlinearFactorGraph graph;
  graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(r.candidate), X(r.query), r.t_candidate_query, loop_noise_));
  // g2o export can't serialise robust kernels -> plain-noise copy
  g2o_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
      X(r.candidate), X(r.query), r.t_candidate_query, loop_noise_plain_));
  try {
    isam_.update(graph, gtsam::Values());
    for (int k = 0; k < 4; ++k) isam_.update();
  } catch (const std::exception& e) {
    log_(std::string("iSAM2 update failed on loop: ") + e.what());
  }
  last_loop_query_ = r.query;
  loops_.emplace_back(r.candidate, r.query);
  refresh();
  char buf[128];
  std::snprintf(buf, sizeof(buf),
                "Loop closure added: %d -> %d (fitness %.2f, rmse %.2f m)",
                r.candidate, r.query, r.fitness, r.rmse);
  log_(buf);
}

// -------------------------------------------------------------------- outputs
void PGOBackend::refresh() {
  est_ = isam_.calculateEstimate();
  const size_t n = keyframes_.size();
  opt_poses_.resize(n);
  for (size_t i = 0; i < n; ++i)
    opt_poses_[i] = est_.at<gtsam::Pose3>(X(i));
  if (n) {
    t_map_odom_ =
        opt_poses_.back() * keyframes_.back().odom_pose.inverse();
  }
}

CloudT::Ptr PGOBackend::buildMap(double voxel) const {
  if (voxel < 0.0) voxel = cfg_.map_voxel;
  auto all = std::make_shared<CloudT>();
  for (size_t i = 0; i < keyframes_.size(); ++i) {
    if (keyframes_[i].cloud->empty()) continue;
    CloudT tmp;
    pcl::transformPointCloud(*keyframes_[i].cloud, tmp,
                             opt_poses_[i].matrix().cast<float>());
    *all += tmp;
  }
  if (all->empty()) return all;
  return voxelDownsample(all, voxel);
}

std::vector<std::string> PGOBackend::save(const std::string& outdir) const {
  namespace fs = std::filesystem;
  fs::create_directories(outdir);
  std::vector<std::string> files;

  {
    const std::string path = outdir + "/optimized_poses_tum.txt";
    std::ofstream f(path);
    f.setf(std::ios::fixed);
    for (size_t i = 0; i < keyframes_.size(); ++i) {
      const auto t = opt_poses_[i].translation();
      const auto q = opt_poses_[i].rotation().toQuaternion();
      f.precision(9);
      f << keyframes_[i].t << ' ';
      f.precision(6);
      f << t.x() << ' ' << t.y() << ' ' << t.z() << ' ';
      f.precision(9);
      f << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w() << '\n';
    }
    files.push_back(path);
  }
  {
    const std::string path = outdir + "/graph.g2o";
    gtsam::Values poses_only;
    for (size_t i = 0; i < keyframes_.size(); ++i)
      poses_only.insert(X(i), opt_poses_[i]);
    gtsam::writeG2o(g2o_graph_, poses_only, path);
    files.push_back(path);
  }
  {
    auto map = buildMap();
    if (!map->empty()) {
      const std::string path = outdir + "/map.pcd";
      pcl::io::savePCDFileBinary(path, *map);
      files.push_back(path);
    }
  }
  {
    const std::string path = outdir + "/georeference.txt";
    std::ofstream f(path);
    f << "# lidar_gps_pgo export\n";
    f << "keyframes: " << keyframes_.size() << '\n';
    f << "loop_closures: " << loops_.size() << '\n';
    for (const auto& [k, v] : n_gps_factors_)
      f << "gps_factors_" << k << ": " << v << '\n';
    f << "float_bias_nodes: " << n_bias_ << '\n';
    if (geo_.hasDatum()) {
      const auto d = geo_.datum();
      f.setf(std::ios::fixed);
      f.precision(9);
      f << "datum_lat_lon_alt: " << d.x() << ' ' << d.y() << ' ';
      f.precision(4);
      f << d.z() << '\n';
    }
    if (alignment_) {
      f.precision(6);
      f << "enu_to_map_yaw_rad: " << alignment_->yaw << '\n';
      f.precision(4);
      f << "enu_to_map_t_xy: " << alignment_->t.x() << ' '
        << alignment_->t.y() << '\n';
      f << "enu_to_map_z_offset: " << alignment_->z_offset << '\n';
      f << "alignment_rms_m: " << alignment_->rms << '\n';
    }
    files.push_back(path);
  }
  return files;
}

}  // namespace lidar_gps_pgo
