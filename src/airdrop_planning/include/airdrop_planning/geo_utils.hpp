// geo_utils.hpp
// Konversi kecil lat/lon (WGS-84) <-> NED lokal dengan pendekatan
// equirectangular di sekitar datum. Cukup akurat untuk skala misi
// KRTI (< 5 km) dan konsisten dengan flat-earth assumption di paper.

#ifndef AIRDROP_PLANNING__GEO_UTILS_HPP_
#define AIRDROP_PLANNING__GEO_UTILS_HPP_

#include <cmath>

namespace airdrop_planning
{

struct GeoPoint
{
  double lat_deg{0.0};
  double lon_deg{0.0};
};

struct Ned2D
{
  double north{0.0};
  double east{0.0};
};

class LocalFrame
{
public:
  LocalFrame() = default;

  void setDatum(double lat_deg, double lon_deg)
  {
    datum_ = {lat_deg, lon_deg};
    cos_lat0_ = std::cos(deg2rad(lat_deg));
    has_datum_ = true;
  }

  bool hasDatum() const {return has_datum_;}
  GeoPoint datum() const {return datum_;}

  Ned2D toNed(const GeoPoint & g) const
  {
    Ned2D n;
    n.north = deg2rad(g.lat_deg - datum_.lat_deg) * kEarthRadius;
    n.east = deg2rad(g.lon_deg - datum_.lon_deg) * kEarthRadius * cos_lat0_;
    return n;
  }

  GeoPoint toGeo(const Ned2D & n) const
  {
    GeoPoint g;
    g.lat_deg = datum_.lat_deg + rad2deg(n.north / kEarthRadius);
    g.lon_deg = datum_.lon_deg + rad2deg(n.east / (kEarthRadius * cos_lat0_));
    return g;
  }

  static double deg2rad(double d) {return d * M_PI / 180.0;}
  static double rad2deg(double r) {return r * 180.0 / M_PI;}

private:
  static constexpr double kEarthRadius = 6378137.0;  // WGS-84 semi-major [m]
  GeoPoint datum_{};
  double cos_lat0_{1.0};
  bool has_datum_{false};
};

inline double wrapPi(double a)
{
  while (a > M_PI) {a -= 2.0 * M_PI;}
  while (a < -M_PI) {a += 2.0 * M_PI;}
  return a;
}

}  // namespace airdrop_planning

#endif  // AIRDROP_PLANNING__GEO_UTILS_HPP_
