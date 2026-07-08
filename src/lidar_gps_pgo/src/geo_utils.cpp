#include "lidar_gps_pgo/geo_utils.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

namespace lidar_gps_pgo {

namespace {
constexpr double kA = 6378137.0;            // WGS84 semi-major axis
constexpr double kE2 = 6.69437999014e-3;    // first eccentricity squared
}  // namespace

Eigen::Vector3d geodeticToEcef(double lat_deg, double lon_deg, double alt) {
  const double lat = lat_deg * M_PI / 180.0;
  const double lon = lon_deg * M_PI / 180.0;
  const double sl = std::sin(lat), cl = std::cos(lat);
  const double n = kA / std::sqrt(1.0 - kE2 * sl * sl);
  return {(n + alt) * cl * std::cos(lon), (n + alt) * cl * std::sin(lon),
          (n * (1.0 - kE2) + alt) * sl};
}

Eigen::Vector3d ecefToGeodetic(const Eigen::Vector3d& e) {
  // Bowring's closed-form approximation (sub-mm for terrestrial points).
  const double b = kA * std::sqrt(1.0 - kE2);
  const double ep2 = (kA * kA - b * b) / (b * b);
  const double p = std::hypot(e.x(), e.y());
  const double th = std::atan2(kA * e.z(), b * p);
  const double lon = std::atan2(e.y(), e.x());
  const double lat =
      std::atan2(e.z() + ep2 * b * std::pow(std::sin(th), 3),
                 p - kE2 * kA * std::pow(std::cos(th), 3));
  const double n = kA / std::sqrt(1.0 - kE2 * std::sin(lat) * std::sin(lat));
  const double alt = p / std::cos(lat) - n;
  return {lat * 180.0 / M_PI, lon * 180.0 / M_PI, alt};
}

void GeoConverter::setDatum(double lat_deg, double lon_deg, double alt) {
  datum_ = {lat_deg, lon_deg, alt};
  datum_ecef_ = geodeticToEcef(lat_deg, lon_deg, alt);
  const double lam = lon_deg * M_PI / 180.0;
  const double phi = lat_deg * M_PI / 180.0;
  const double sl = std::sin(lam), cl = std::cos(lam);
  const double sp = std::sin(phi), cp = std::cos(phi);
  r_enu_ecef_ << -sl, cl, 0.0,
                 -sp * cl, -sp * sl, cp,
                  cp * cl,  cp * sl, sp;
  has_datum_ = true;
}

Eigen::Vector3d GeoConverter::toEnu(double lat_deg, double lon_deg,
                                    double alt) const {
  return r_enu_ecef_ * (geodeticToEcef(lat_deg, lon_deg, alt) - datum_ecef_);
}

Eigen::Vector3d GeoConverter::enuToGeodetic(const Eigen::Vector3d& enu) const {
  return ecefToGeodetic(datum_ecef_ + r_enu_ecef_.transpose() * enu);
}

std::optional<Alignment2D> fitYawTranslation2D(
    const std::vector<Eigen::Vector3d>& src_enu,
    const std::vector<Eigen::Vector3d>& dst_map) {
  const size_t n = src_enu.size();
  if (n < 2 || dst_map.size() != n) return std::nullopt;
  Eigen::Matrix2Xd src(2, n), dst(2, n);
  double z_off = 0.0;
  for (size_t i = 0; i < n; ++i) {
    src.col(i) = src_enu[i].head<2>();
    dst.col(i) = dst_map[i].head<2>();
    z_off += dst_map[i].z() - src_enu[i].z();
  }
  z_off /= static_cast<double>(n);

  const Eigen::Matrix3d T = Eigen::umeyama(src, dst, /*with_scaling=*/false);
  Alignment2D a;
  a.R = T.topLeftCorner<2, 2>();
  a.t = T.topRightCorner<2, 1>();
  a.z_offset = z_off;
  a.yaw = std::atan2(a.R(1, 0), a.R(0, 0));
  double se = 0.0;
  for (size_t i = 0; i < n; ++i)
    se += (dst.col(i) - (a.R * src.col(i) + a.t)).squaredNorm();
  a.rms = std::sqrt(se / static_cast<double>(n));
  return a;
}

}  // namespace lidar_gps_pgo
