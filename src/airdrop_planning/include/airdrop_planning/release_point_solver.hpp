// release_point_solver.hpp
// Implementasi "Algorithm 1: Compute Release Point" (prec_drop, Sec. 2.3)
// + konstruksi truncated Dubins approach path (Fig. 5):
//
//   1. Simulasikan persamaan balistik (3) dari ketinggian release ke tanah
//      -> dapat perpindahan (x, y).
//   2. releasePoint = target digeser (-x, -y) pada arah (north, east).
//   3. Arah release = MELAWAN arah angin. Jika angin ~ 0, pakai arah
//      yang memberi jalur terbang terpendek dari posisi wahana.
//   4. Buat garis melalui release point searah angin; titik p pada jarak d
//      di sisi downwind; lingkaran loiter radius r tangen di p dengan
//      pusat s (loiter clockwise).

#ifndef AIRDROP_PLANNING__RELEASE_POINT_SOLVER_HPP_
#define AIRDROP_PLANNING__RELEASE_POINT_SOLVER_HPP_

#include <array>
#include <cmath>

#include "airdrop_planning/ballistic_model.hpp"
#include "airdrop_planning/geo_utils.hpp"

namespace airdrop_planning
{

struct ApproachParams
{
  double release_alt_agl{100.0};     // [m]
  double release_airspeed{18.0};    // [m/s] bisa ganti
  double approach_distance{300.0};  // d  [m]
  double loiter_radius{80.0};       // r  [m]
  double min_wind_for_heading{1.0}; // [m/s] di bawah ini arah angin diabaikan
};

struct ApproachPlanNed
{
  Ned2D release_point;
  Ned2D entry_point;      // p
  Ned2D loiter_center;    // s
  double approach_heading{0.0};   // [rad] dari North, CW positif (arah terbang)
  double fall_time{0.0};
  double offset_north{0.0};
  double offset_east{0.0};
};

class ReleasePointSolver
{
public:
  ReleasePointSolver(const BallisticParams & bp, const ApproachParams & ap)
  : model_(bp), ap_(ap) {}

  // ---------- perencanaan awal ----------
  // target_ned : posisi target di frame lokal
  // uav_ned    : posisi wahana saat ini (untuk kasus angin ~ 0)
  // wind_ned   : vektor angin (KE mana udara bergerak), komponen (n, e)
  ApproachPlanNed solve(
    const Ned2D & target_ned,
    const Ned2D & uav_ned,
    const std::array<double, 2> & wind_ned) const
  {
    // --- arah approach: melawan angin ---
    const double wind_speed = std::hypot(wind_ned[0], wind_ned[1]);
    double heading;
    if (wind_speed >= ap_.min_wind_for_heading) {
      // Terbang melawan angin: arah terbang = kebalikan arah gerak udara.
      heading = std::atan2(-wind_ned[1], -wind_ned[0]);
    } else {
      // Angin lemah: jalur terpendek — arah dari wahana ke target.
      heading = std::atan2(target_ned.east - uav_ned.east,
          target_ned.north - uav_ned.north);
    }

    // Prediksi ground velocity saat rilis = airspeed*heading + angin.
    const std::array<double, 3> v0{
      ap_.release_airspeed * std::cos(heading) + wind_ned[0],
      ap_.release_airspeed * std::sin(heading) + wind_ned[1],
      0.0};

    return buildPlan(target_ned, heading, v0, wind_ned);
  }

  // ---------- replan di titik p ----------
  // heading         : arah garis approach semula — TIDAK berubah.
  // ground_speed    : ground speed horizontal aktual wahana, SETELAH
  //                   dikurangi konstanta reduksi (paper: 2 m/s bila motor
  //                   dimatikan sebelum rilis; ~0 bila motor tetap on).
  // wind_ned        : angin yang dipakai saat perencanaan awal — sesuai
  //                   paper, angin TIDAK di-update lagi setelah titik p.
  ApproachPlanNed solveAlongHeading(
    const Ned2D & target_ned,
    double heading,
    double ground_speed,
    const std::array<double, 2> & wind_ned) const
  {
    const std::array<double, 3> v0{
      ground_speed * std::cos(heading),
      ground_speed * std::sin(heading),
      0.0};
    return buildPlan(target_ned, heading, v0, wind_ned);
  }

  const ApproachParams & params() const {return ap_;}

private:
  // Algorithm 1 + geometri Fig. 5 untuk heading & v0 yang sudah ditentukan.
  ApproachPlanNed buildPlan(
    const Ned2D & target_ned,
    double heading,
    const std::array<double, 3> & v0,
    const std::array<double, 2> & wind_ned) const
  {
    ApproachPlanNed plan;
    plan.approach_heading = heading;

    // --- Algorithm 1: simulasi balistik lalu geser target (-x, -y) ---
    const auto bal = model_.simulate(ap_.release_alt_agl, v0, wind_ned);
    plan.fall_time = bal.fall_time_s;
    plan.offset_north = bal.offset_north_m;
    plan.offset_east = bal.offset_east_m;

    plan.release_point.north = target_ned.north - bal.offset_north_m;
    plan.release_point.east = target_ned.east - bal.offset_east_m;

    // --- truncated Dubins (Fig. 5) ---
    // Titik p: sejauh d dari release point PADA ARAH DATANGNYA wahana
    // (downwind), yaitu kebalikan arah terbang.
    const double hn = std::cos(heading);
    const double he = std::sin(heading);
    plan.entry_point.north = plan.release_point.north - ap_.approach_distance * hn;
    plan.entry_point.east = plan.release_point.east - ap_.approach_distance * he;

    // Pusat loiter s: tangen di p, loiter clockwise (belok kanan),
    // sehingga pusat berada di sisi KANAN arah terbang.
    // Unit "kanan" dari heading psi (N->E, CW): (-sin psi, cos psi).
    plan.loiter_center.north = plan.entry_point.north - ap_.loiter_radius * std::sin(heading);
    plan.loiter_center.east = plan.entry_point.east + ap_.loiter_radius * std::cos(heading);

    return plan;
  }

  BallisticModel model_;
  ApproachParams ap_;
};

}  // namespace airdrop_planning

#endif  // AIRDROP_PLANNING__RELEASE_POINT_SOLVER_HPP_
