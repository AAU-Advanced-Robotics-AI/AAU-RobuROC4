// Custom GTSAM factors for GNSS measurements with an antenna lever arm.
//
//  LeverArmGPSFactor          h(T)    = t + R*l          (RTK fixed)
//  BiasedLeverArmGPSFactor    h(T, b) = t + R*l + b      (RTK float)
//
// Written against the classic GTSAM 4.1.x / 4.2.0 factor API
// (boost::optional Jacobians), which is what ROS 2 Humble-era installs ship.
//
// where T = (R, t) is the body pose in the map frame, l the antenna position
// in the body frame and b a slowly drifting position bias (modelled per float
// "session" as a random-walk chain of Point3 variables). This captures the
// dominant RTK-float error mode: a large, slowly varying offset with very
// little high-frequency noise, which an inflated-white-noise GPSFactor gets
// badly wrong (it would drag the whole trajectory toward the biased fixes).
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace lidar_gps_pgo {

class LeverArmGPSFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  LeverArmGPSFactor(gtsam::Key pose_key, const gtsam::Point3& measured,
                    const gtsam::Point3& lever_arm,
                    const gtsam::SharedNoiseModel& model)
      : NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        measured_(measured),
        lever_(lever_arm) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose,
      boost::optional<gtsam::Matrix&> H = boost::none) const override {
    const gtsam::Matrix3 R = pose.rotation().matrix();
    if (H) {
      H->resize(3, 6);
      H->leftCols(3) = -R * gtsam::skewSymmetric(lever_);
      H->rightCols(3) = R;
    }
    return pose.translation() + R * lever_ - measured_;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new LeverArmGPSFactor(*this));
  }

 private:
  gtsam::Point3 measured_;
  gtsam::Point3 lever_;
};

class BiasedLeverArmGPSFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Point3> {
 public:
  BiasedLeverArmGPSFactor(gtsam::Key pose_key, gtsam::Key bias_key,
                          const gtsam::Point3& measured,
                          const gtsam::Point3& lever_arm,
                          const gtsam::SharedNoiseModel& model)
      : NoiseModelFactor2<gtsam::Pose3, gtsam::Point3>(model, pose_key,
                                                       bias_key),
        measured_(measured),
        lever_(lever_arm) {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& pose, const gtsam::Point3& bias,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const override {
    const gtsam::Matrix3 R = pose.rotation().matrix();
    if (H1) {
      H1->resize(3, 6);
      H1->leftCols(3) = -R * gtsam::skewSymmetric(lever_);
      H1->rightCols(3) = R;
    }
    if (H2) *H2 = gtsam::Matrix3::Identity();
    return pose.translation() + R * lever_ + bias - measured_;
  }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(
        new BiasedLeverArmGPSFactor(*this));
  }

 private:
  gtsam::Point3 measured_;
  gtsam::Point3 lever_;
};

}  // namespace lidar_gps_pgo
