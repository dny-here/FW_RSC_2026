// ballistic_model.hpp
// Model balistik payload jatuh bebas dengan drag + angin.
// Referensi: Mathisen et al., "Autonomous Ballistic Airdrop of Objects
// from a Small Fixed-Wing UAV" — persamaan (3), (4) dan wind profile
// power law persamaan (5) (Touma 1977, p = 1/7).
//
// Frame: NED lokal. x = North, y = East, z = Down.
// Ketinggian h diukur AGL sehingga integrasi berhenti saat h <= 0.

#ifndef AIRDROP_PLANNING__BALLISTIC_MODEL_HPP_
#define AIRDROP_PLANNING__BALLISTIC_MODEL_HPP_

#include <array>
#include <cmath>

namespace airdrop_planning
{

struct BallisticParams
{
  double mass_kg{0.5};          // payload KRTI: kayu 500 g
  double drag_coeff{0.8};       // C_D — WAJIB diidentifikasi lewat drop test
  double ref_area_m2{0.01};     // A, luas frontal payload [m^2]
  double air_density{1.225};    // rho [kg/m^3]
  double gravity{9.80665};      // g [m/s^2]
  double wind_power_exp{1.0 / 7.0};  // p pada persamaan (5)
  double dt{0.005};             // timestep integrasi [s]
};

struct BallisticResult
{
  double offset_north_m{0.0};   // x : perpindahan horizontal payload selama jatuh
  double offset_east_m{0.0};    // y
  double fall_time_s{0.0};
  double impact_speed_ms{0.0};
};

class BallisticModel
{
public:
  explicit BallisticModel(const BallisticParams & p)
  : p_(p) {}

  // Simulasikan jatuhnya payload.
  //  release_alt_agl : ketinggian pelepasan di atas tanah [m]
  //  v0_ned          : kecepatan GROUND payload saat lepas (== ground velocity
  //                    wahana saat release) [m/s], (vn, ve, vd)
  //  wind_ned_at_alt : estimasi angin (arah KE mana udara bergerak) pada
  //                    ketinggian release [m/s], (wn, we); wd dianggap 0
  // Angin diskalakan terhadap ketinggian dengan power law (5).
  BallisticResult simulate(
    double release_alt_agl,
    const std::array<double, 3> & v0_ned,
    const std::array<double, 2> & wind_ned_at_alt) const
  {
    // state = [x, y, h, vx, vy, vz_down]
    std::array<double, 6> s{0.0, 0.0, release_alt_agl,
      v0_ned[0], v0_ned[1], v0_ned[2]};

    BallisticResult out;
    const double k = p_.drag_coeff * p_.air_density * p_.ref_area_m2 /
      (2.0 * p_.mass_kg);

    double t = 0.0;
    while (s[2] > 0.0 && t < kMaxFallTime) {
      s = rk4Step(s, k, wind_ned_at_alt, release_alt_agl);
      t += p_.dt;
    }

    out.offset_north_m = s[0];
    out.offset_east_m = s[1];
    out.fall_time_s = t;
    out.impact_speed_ms = std::sqrt(s[3] * s[3] + s[4] * s[4] + s[5] * s[5]);
    return out;
  }

private:
  static constexpr double kMaxFallTime = 60.0;  // guard divergensi

  // Wind profile power law (5): w(h) = w_ref * (h / h_ref)^p
  std::array<double, 2> windAt(
    double h, const std::array<double, 2> & w_ref, double h_ref) const
  {
    if (h <= 0.0 || h_ref <= 0.0) {return {0.0, 0.0};}
    const double scale = std::pow(h / h_ref, p_.wind_power_exp);
    return {w_ref[0] * scale, w_ref[1] * scale};
  }

  std::array<double, 6> deriv(
    const std::array<double, 6> & s, double k,
    const std::array<double, 2> & w_ref, double h_ref) const
  {
    const auto w = windAt(s[2], w_ref, h_ref);
    const double rvx = s[3] - w[0];
    const double rvy = s[4] - w[1];
    const double rvz = s[5];               // angin vertikal = 0
    const double vr = std::sqrt(rvx * rvx + rvy * rvy + rvz * rvz);  // (4)

    // (3): a = -k * (v - w) * Vr, plus gravitasi pada sumbu down.
    return {
      s[3], s[4], -s[5],                   // dh/dt = -vz_down
      -k * rvx * vr,
      -k * rvy * vr,
      p_.gravity - k * rvz * vr
    };
  }

  std::array<double, 6> rk4Step(
    const std::array<double, 6> & s, double k,
    const std::array<double, 2> & w_ref, double h_ref) const
  {
    const double dt = p_.dt;
    auto add = [](const std::array<double, 6> & a,
        const std::array<double, 6> & b, double f) {
        std::array<double, 6> r;
        for (size_t i = 0; i < 6; ++i) {r[i] = a[i] + b[i] * f;}
        return r;
      };

    const auto k1 = deriv(s, k, w_ref, h_ref);
    const auto k2 = deriv(add(s, k1, dt / 2.0), k, w_ref, h_ref);
    const auto k3 = deriv(add(s, k2, dt / 2.0), k, w_ref, h_ref);
    const auto k4 = deriv(add(s, k3, dt), k, w_ref, h_ref);

    std::array<double, 6> out;
    for (size_t i = 0; i < 6; ++i) {
      out[i] = s[i] + dt / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    return out;
  }

  BallisticParams p_;
};

}  // namespace airdrop_planning

#endif  // AIRDROP_PLANNING__BALLISTIC_MODEL_HPP_
