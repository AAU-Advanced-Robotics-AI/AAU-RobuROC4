// Standalone backend tests (no ROS). Build & run:
//   see README "Core tests" section, or the CMake target test_backend.
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include <Eigen/Geometry>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>

#include "lidar_gps_pgo/geo_utils.hpp"
#include "lidar_gps_pgo/gps_factors.hpp"
#include "lidar_gps_pgo/pgo_backend.hpp"
#include "lidar_gps_pgo/scan_context.hpp"

using namespace lidar_gps_pgo;
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::X;

#define CHECK(cond)                                                        \
  do {                                                                     \
    if (!(cond)) {                                                         \
      std::fprintf(stderr, "CHECK failed at %s:%d: %s\n", __FILE__,        \
                   __LINE__, #cond);                                       \
      std::exit(1);                                                        \
    }                                                                      \
  } while (0)

static std::mt19937 rng(7);
static double urand(double a, double b) {
  return std::uniform_real_distribution<double>(a, b)(rng);
}
static double nrand(double s) {
  return std::normal_distribution<double>(0.0, s)(rng);
}

// ------------------------------------------------------------------- helpers
static std::vector<Eigen::Vector2d> makeLandmarks(int n, double extent) {
  std::vector<Eigen::Vector2d> lm(n);
  for (auto& p : lm) p = {urand(-extent, extent), urand(-extent, extent)};
  return lm;
}

// Simulated scan: vertical "poles" at landmark xy, body frame.
static CloudT::Ptr cloudAt(const gtsam::Pose3& pose,
                           const std::vector<Eigen::Vector2d>& lm,
                           double radius = 80.0) {
  auto out = std::make_shared<CloudT>();
  const Eigen::Vector3d t = pose.translation();
  const Eigen::Matrix3d Rt = pose.rotation().matrix().transpose();
  for (const auto& c : lm) {
    const double d = (c - t.head<2>()).norm();
    if (d >= radius || d <= 2.0) continue;
    for (int k = 0; k < 10; ++k) {
      const double z = 0.2 + 3.8 * k / 9.0;
      Eigen::Vector3d pw(c.x(), c.y(), z);
      Eigen::Vector3d pb = Rt * (pw - t);
      out->push_back(pcl::PointXYZ(pb.x() + nrand(0.01),
                                   pb.y() + nrand(0.01),
                                   pb.z() + nrand(0.01)));
    }
  }
  return out;
}

// --------------------------------------------------------------------- tests
static void testGeo() {
  GeoConverter g;
  g.setDatum(42.0, -83.5, 250.0);
  const Eigen::Vector3d enu(120.0, -40.0, 3.0);
  const Eigen::Vector3d lla = g.enuToGeodetic(enu);
  const Eigen::Vector3d back = g.toEnu(lla.x(), lla.y(), lla.z());
  CHECK((back - enu).norm() < 1e-3);
  CHECK(g.toEnu(42.0, -83.5, 250.0).norm() < 1e-6);
  std::puts("[ok] geodetic <-> ENU roundtrip");
}

static void testAlignmentFit() {
  const double ang = 25.0 * M_PI / 180.0;
  Eigen::Rotation2Dd R(ang);
  std::vector<Eigen::Vector3d> src, dst;
  for (int i = 0; i < 30; ++i) {
    Eigen::Vector2d s(urand(-50, 50), urand(-50, 50));
    Eigen::Vector2d d = R * s + Eigen::Vector2d(5.0, -3.0) +
                        Eigen::Vector2d(nrand(0.02), nrand(0.02));
    src.push_back({s.x(), s.y(), 1.0});
    dst.push_back({d.x(), d.y(), 3.5});
  }
  auto a = fitYawTranslation2D(src, dst);
  CHECK(a.has_value());
  CHECK(a->rms < 0.05);
  CHECK(std::abs(wrapAngle(a->yaw - ang)) < 1e-2);
  CHECK(std::abs(a->z_offset - 2.5) < 1e-6);
  std::puts("[ok] 2D yaw+translation fit");
}

static void testFactorJacobians() {
  const gtsam::Pose3 pose(gtsam::Rot3::Ypr(0.7, 0.1, -0.2),
                          gtsam::Point3(1.0, -2.0, 0.5));
  const gtsam::Point3 lever(0.3, -0.1, 0.5);
  const gtsam::Point3 bias(0.4, -0.6, 0.1);
  const gtsam::Point3 meas(1.5, -2.2, 1.1);
  auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);

  {
    LeverArmGPSFactor f(X(0), meas, lever, noise);
    gtsam::Matrix H;
    f.evaluateError(pose, H);
    auto err = [&](const gtsam::Pose3& p) {
      return gtsam::Vector(f.evaluateError(p));
    };
    gtsam::Matrix Hn(3, 6);
    const double eps = 1e-6;
    for (int j = 0; j < 6; ++j) {
      gtsam::Vector6 d = gtsam::Vector6::Zero();
      d(j) = eps;
      Hn.col(j) = (err(pose.retract(d)) - err(pose.retract(-d))) / (2 * eps);
    }
    CHECK((H - Hn).cwiseAbs().maxCoeff() < 1e-5);
  }
  {
    BiasedLeverArmGPSFactor f(X(0), B(0), meas, lever, noise);
    gtsam::Matrix H1, H2;
    f.evaluateError(pose, bias, H1, H2);
    auto err = [&](const gtsam::Pose3& p, const gtsam::Point3& b) {
      return gtsam::Vector(f.evaluateError(p, b));
    };
    gtsam::Matrix H1n(3, 6), H2n(3, 3);
    const double eps = 1e-6;
    for (int j = 0; j < 6; ++j) {
      gtsam::Vector6 d = gtsam::Vector6::Zero();
      d(j) = eps;
      H1n.col(j) =
          (err(pose.retract(d), bias) - err(pose.retract(-d), bias)) /
          (2 * eps);
    }
    for (int j = 0; j < 3; ++j) {
      gtsam::Vector3 d = gtsam::Vector3::Zero();
      d(j) = eps;
      H2n.col(j) = (err(pose, bias + d) - err(pose, bias - d)) / (2 * eps);
    }
    CHECK((H1 - H1n).cwiseAbs().maxCoeff() < 1e-5);
    CHECK((H2 - H2n).cwiseAbs().maxCoeff() < 1e-8);
  }
  std::puts("[ok] LeverArmGPSFactor / BiasedLeverArmGPSFactor Jacobians");
}

static void testScanContextYawSign() {
  auto lm = makeLandmarks(400, 160.0);
  const double psi = 40.0 * M_PI / 180.0;
  ScanContextParams p;
  p.exclude_recent = 1;
  p.dist_threshold = 0.5;
  p.num_candidates = 4;
  ScanContextManager sc(p);

  auto toE = [](const CloudT& c) {
    Eigen::Matrix<double, Eigen::Dynamic, 3> m(c.size(), 3);
    for (size_t i = 0; i < c.size(); ++i) m.row(i) << c[i].x, c[i].y, c[i].z;
    return m;
  };
  sc.add(toE(*cloudAt(gtsam::Pose3(), lm)));
  for (int i = 0; i < 3; ++i) {
    gtsam::Pose3 far(gtsam::Rot3(),
                     gtsam::Point3(urand(120, 150), urand(120, 150), 0));
    sc.add(toE(*cloudAt(far, lm)));
  }
  sc.add(toE(*cloudAt(
      gtsam::Pose3(gtsam::Rot3::Rz(psi), gtsam::Point3(0, 0, 0)), lm)));
  auto det = sc.detect(4);
  CHECK(det.has_value());
  CHECK(det->candidate == 0);
  const double err = std::abs(wrapAngle(det->yaw - psi));
  std::printf("[ok] scancontext: cand=%d yaw=%.1f deg (true 40.0) dist=%.3f\n",
              det->candidate, det->yaw * 180.0 / M_PI, det->distance);
  CHECK(err < 9.0 * M_PI / 180.0);  // < 1.5 sector widths
}

static void testFullBackend() {
  auto lm = makeLandmarks(500, 170.0);
  const int side = 80;
  const double enu_yaw = 30.0 * M_PI / 180.0;  // world -> ENU rotation
  const Eigen::Matrix3d r_ew =
      Eigen::AngleAxisd(enu_yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d ant(0.3, 0.0, 0.5);

  // ground truth square + 15 m re-entry of the first leg
  std::vector<gtsam::Pose3> gt;
  std::vector<int> legs;
  double x = 0, y = 0, hdg = 0;
  for (int leg = 0; leg < 4; ++leg) {
    for (int s = 0; s < side; ++s) {
      gt.emplace_back(gtsam::Rot3::Rz(hdg), gtsam::Point3(x, y, 0));
      legs.push_back(leg);
      x += std::cos(hdg);
      y += std::sin(hdg);
    }
    hdg += M_PI / 2.0;
  }
  for (int s = 0; s < 15; ++s) {
    gt.emplace_back(gtsam::Rot3::Rz(0.0), gtsam::Point3(x, y, 0));
    legs.push_back(4);
    x += 1.0;
  }

  // drifting odometry: constant yaw-rate bias + noise
  const double yaw_bias = 0.06 * M_PI / 180.0;
  std::vector<gtsam::Pose3> odo{gt[0]};
  for (size_t k = 1; k < gt.size(); ++k) {
    const gtsam::Pose3 rel = gt[k - 1].between(gt[k]);
    const gtsam::Pose3 err(
        gtsam::Rot3::Rz(yaw_bias + nrand(0.001)),
        gtsam::Point3(nrand(0.01), nrand(0.01), nrand(0.01)));
    odo.push_back(odo.back() * rel * err);
  }
  const double raw_end_err =
      (odo.back().translation() - gt.back().translation()).norm();
  CHECK(raw_end_err > 3.0);

  GeoConverter geo;
  geo.setDatum(42.30, -83.70, 250.0);

  BackendConfig cfg;
  cfg.keyframe_dist = 0.8;
  cfg.antenna_lever = ant;
  cfg.align_min_travel = 15.0;
  cfg.sc.dist_threshold = 0.25;
  cfg.sc.exclude_recent = 60;
  cfg.loop_min_query_gap = 3;
  cfg.icp_method = "point_to_point";  // poles: GICP covariances degenerate
  cfg.icp_voxel = 0.2;
  cfg.icp_min_fitness = 0.4;
  cfg.icp_max_rmse = 0.5;
  // float bias model roughly matching the observed drift magnitude
  cfg.float_meas_sigma = 0.12;
  cfg.float_bias_prior_sigma = 3.0;
  cfg.float_bias_rw_sigma = 0.10;
  PGOBackend b(cfg, [](const std::string& m) {
    std::printf("   backend: %s\n", m.c_str());
  });

  // float-leg bias: slow random walk reaching O(1.5 m)
  Eigen::Vector3d float_bias(0.0, 0.0, 0.0);

  std::vector<gtsam::Pose3> kf_gt;
  int loops_tried = 0;
  for (size_t k = 0; k < gt.size(); ++k) {
    const double t = static_cast<double>(k);
    // two fixes bracketing the keyframe time (legs 3/4 = outage)
    if (legs[k] <= 2) {
      const bool is_float = (legs[k] == 1);
      if (is_float)  // slow drift reaching ~2 m over the leg (cf. field data:
                     // 2.2 m drift at ~0.1 m reported covariance)
        float_bias += Eigen::Vector3d(0.022 + nrand(0.008),
                                      -0.015 + nrand(0.008), nrand(0.004));
      const double white = is_float ? 0.03 : 0.02;  // little HF noise
      const double rep_sigma = is_float ? 0.10 : 0.02;  // reported (too small!)
      for (double dt : {-0.1, 0.1}) {
        const size_t kk = std::min(k + (dt > 0 ? 1 : 0), gt.size() - 1);
        const double a = dt > 0 ? dt : 0.0;
        const Eigen::Vector3d p0 = gt[k].transformFrom(gtsam::Point3(ant));
        const Eigen::Vector3d p1 = gt[kk].transformFrom(gtsam::Point3(ant));
        Eigen::Vector3d p = (1.0 - a) * p0 + a * p1;
        Eigen::Vector3d enu = r_ew * p;
        if (is_float) enu += float_bias;
        enu += Eigen::Vector3d(nrand(white), nrand(white), nrand(white));
        const Eigen::Vector3d lla = geo.enuToGeodetic(enu);
        b.addGpsFix(t + dt, lla.x(), lla.y(), lla.z(),
                    Eigen::Vector3d::Constant(rep_sigma),
                    is_float ? GpsQuality::kFloat : GpsQuality::kFixed, t);
      }
    }
    if (b.needsKeyframe(odo[k])) {
      b.addKeyframe(t, odo[k], cloudAt(gt[k], lm), t);
      kf_gt.push_back(gt[k]);
      if (auto task = b.nextLoopTask()) {
        ++loops_tried;
        if (auto res = b.verifyLoop(*task)) b.commitLoop(*res);
      }
    }
  }

  CHECK(b.hasAlignment());
  const double yaw_err = std::abs(wrapAngle(b.alignment()->yaw + enu_yaw));
  std::printf("[ok] alignment yaw %.2f deg (expected -30.00), rms %.3f m\n",
              b.alignment()->yaw * 180.0 / M_PI, b.alignment()->rms);
  CHECK(yaw_err < 2.0 * M_PI / 180.0);
  CHECK(b.gpsFactorCounts().at("fixed") > 50);
  CHECK(b.gpsFactorCounts().at("float") > 10);
  CHECK(!b.loops().empty());

  double mean_err = 0.0;
  for (size_t i = 0; i < kf_gt.size(); ++i)
    mean_err += (b.optimizedPoses()[i].translation() -
                 kf_gt[i].translation()).norm();
  mean_err /= kf_gt.size();
  const double end_err = (b.optimizedPoses().back().translation() -
                          kf_gt.back().translation()).norm();
  std::printf(
      "[ok] %zu keyframes, %zu loops (%d tried), fixed=%d float=%d\n",
      b.keyframes().size(), b.loops().size(), loops_tried,
      b.gpsFactorCounts().at("fixed"), b.gpsFactorCounts().at("float"));
  std::printf("[ok] ATE mean %.3f m, endpoint %.3f m (raw odom %.2f m, "
              "float bias reached %.2f m)\n",
              mean_err, end_err, raw_end_err, float_bias.norm());
  CHECK(float_bias.norm() > 1.5);  // the drift was actually significant
  CHECK(end_err < 1.0);
  CHECK(mean_err < 1.0);

  const std::string outdir = "/tmp/pgo_test_out";
  auto files = b.save(outdir);
  CHECK(files.size() >= 4);
  for (const auto& f : files) CHECK(std::ifstream(f).good());
  std::puts("[ok] save() wrote all outputs");
}

int main() {
  testGeo();
  testAlignmentFit();
  testFactorJacobians();
  testScanContextYawSign();
  testFullBackend();
  std::puts("\nALL TESTS PASSED");
  return 0;
}
