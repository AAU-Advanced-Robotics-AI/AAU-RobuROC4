// Geodetic <-> local ENU conversion and 2D (yaw + translation) alignment.
// ROS-free, header + small cpp, unit-tested standalone.
#pragma once

#include <Eigen/Core>
#include <optional>

namespace lidar_gps_pgo {

Eigen::Vector3d geodeticToEcef(double lat_deg, double lon_deg, double alt);
Eigen::Vector3d ecefToGeodetic(const Eigen::Vector3d& ecef);  // lat, lon, alt

// Local ENU frame anchored at a datum fix.
class GeoConverter {
 public:
  bool hasDatum() const { return has_datum_; }
  void setDatum(double lat_deg, double lon_deg, double alt);
  Eigen::Vector3d datum() const { return datum_; }  // lat, lon, alt
  Eigen::Vector3d toEnu(double lat_deg, double lon_deg, double alt) const;
  Eigen::Vector3d enuToGeodetic(const Eigen::Vector3d& enu) const;

 private:
  bool has_datum_ = false;
  Eigen::Vector3d datum_{0, 0, 0};
  Eigen::Vector3d datum_ecef_{0, 0, 0};
  Eigen::Matrix3d r_enu_ecef_ = Eigen::Matrix3d::Identity();
};

struct Alignment2D {
  Eigen::Matrix2d R;   // ENU -> map (xy)
  Eigen::Vector2d t;
  double z_offset;     // map_z = enu_z + z_offset
  double yaw;          // atan2(R10, R00)
  double rms;
};

// Least-squares rigid fit dst ~= R*src + t on the xy plane; z handled as a
// mean offset. Returns nullopt for degenerate input (<2 points).
std::optional<Alignment2D> fitYawTranslation2D(
    const std::vector<Eigen::Vector3d>& src_enu,
    const std::vector<Eigen::Vector3d>& dst_map);

inline double wrapAngle(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}

}  // namespace lidar_gps_pgo
