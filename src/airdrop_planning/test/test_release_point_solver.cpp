// test_release_point_solver.cpp
// Mengunci geometri truncated Dubins (prec_drop Fig. 5) dan — yang terpenting —
// invarian yang membuat drop bisa terpicu sama sekali: syarat capture di titik p
// HANYA terpenuhi pada orbit yang berpusat di loiter center s.
//
// Regresi yang dijaga: mission_bridge pernah mengirim entry point p sebagai
// tujuan GUIDED, sehingga ArduPlane mengorbit p. Pada orbit itu syarat capture
// tidak pernah terpenuhi dan FSM parkir selamanya di STATE_LOITER tanpa error.

#include <array>
#include <cmath>

#include "gtest/gtest.h"

#include "airdrop_planning/release_point_solver.hpp"

using airdrop_planning::ApproachParams;
using airdrop_planning::ApproachPlanNed;
using airdrop_planning::BallisticParams;
using airdrop_planning::CaptureGate;
using airdrop_planning::Ned2D;
using airdrop_planning::ReleasePointSolver;
using airdrop_planning::entryCaptured;
using airdrop_planning::wrapPi;

namespace
{

ApproachParams defaultApproach()
{
  ApproachParams ap;
  ap.release_alt_agl = 100.0;
  ap.release_airspeed = 18.0;
  ap.approach_distance = 300.0;
  ap.loiter_radius = 80.0;
  ap.min_wind_for_heading = 1.0;
  return ap;
}

BallisticParams defaultBallistic()
{
  BallisticParams bp;
  bp.mass_kg = 0.5;
  bp.drag_coeff = 0.8;
  bp.ref_area_m2 = 0.01;
  bp.dt = 0.005;
  return bp;
}

// Menerbangkan satu putaran penuh orbit CW radius r di sekitar `center`, dan
// melaporkan apakah ADA satu titik pun di orbit itu yang memenuhi syarat
// capture. Course di setiap titik diambil tangensial (arah gerak sebenarnya
// pada orbit), bukan diasumsikan.
bool orbitEverCaptures(
  const ApproachPlanNed & plan, const Ned2D & center, double r,
  const CaptureGate & gate, int samples = 3600)
{
  for (int i = 0; i < samples; ++i) {
    const double theta = 2.0 * M_PI * i / samples;
    const Ned2D pos{center.north + r * std::cos(theta),
      center.east + r * std::sin(theta)};
    // Orbit clockwise (N->E->S->W) = theta menaik; arah gerak = d/dtheta.
    const double course = std::atan2(std::cos(theta), -std::sin(theta));
    if (entryCaptured(plan, pos, course, gate)) {return true;}
  }
  return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// Geometri dasar Fig. 5
// ---------------------------------------------------------------------------

TEST(ReleasePointSolver, EntryPointBerjarakDDariReleasePointDiSisiDownwind)
{
  const auto ap = defaultApproach();
  ReleasePointSolver solver(defaultBallistic(), ap);

  // Angin 6 m/s bertiup KE arah timur -> approach terbang ke barat.
  const auto plan = solver.solve(Ned2D{0.0, 0.0}, Ned2D{-500.0, 0.0}, {0.0, 6.0});

  const double d = std::hypot(
    plan.entry_point.north - plan.release_point.north,
    plan.entry_point.east - plan.release_point.east);
  EXPECT_NEAR(d, ap.approach_distance, 1e-6);

  // p harus di BELAKANG release point relatif arah terbang (sisi downwind):
  // proyeksi (p - release) pada heading bernilai negatif.
  const double along =
    (plan.entry_point.north - plan.release_point.north) * std::cos(plan.approach_heading) +
    (plan.entry_point.east - plan.release_point.east) * std::sin(plan.approach_heading);
  EXPECT_LT(along, 0.0);
}

TEST(ReleasePointSolver, ApproachHeadingMelawanArahAngin)
{
  ReleasePointSolver solver(defaultBallistic(), defaultApproach());
  // Angin bergerak ke timur (+E) -> wahana harus terbang ke barat (-E).
  const auto plan = solver.solve(Ned2D{0.0, 0.0}, Ned2D{-500.0, 0.0}, {0.0, 6.0});
  EXPECT_NEAR(std::abs(wrapPi(plan.approach_heading)), M_PI / 2.0, 1e-9);
  EXPECT_LT(std::sin(plan.approach_heading), 0.0);  // komponen East negatif
}

TEST(ReleasePointSolver, ReleasePointDigeserBerlawananDriftBalistik)
{
  ReleasePointSolver solver(defaultBallistic(), defaultApproach());
  const Ned2D target{120.0, -45.0};
  const auto plan = solver.solve(target, Ned2D{-500.0, 0.0}, {0.0, 6.0});

  // Algorithm 1: releasePoint = target digeser (-x, -y).
  EXPECT_NEAR(plan.release_point.north, target.north - plan.offset_north, 1e-9);
  EXPECT_NEAR(plan.release_point.east, target.east - plan.offset_east, 1e-9);
  EXPECT_GT(plan.fall_time, 0.0);
}

TEST(ReleasePointSolver, LoiterCenterBerjarakRDariEntryPoint)
{
  const auto ap = defaultApproach();
  ReleasePointSolver solver(defaultBallistic(), ap);
  const auto plan = solver.solve(Ned2D{0.0, 0.0}, Ned2D{-500.0, 0.0}, {0.0, 6.0});

  const double r = std::hypot(
    plan.loiter_center.north - plan.entry_point.north,
    plan.loiter_center.east - plan.entry_point.east);
  EXPECT_NEAR(r, ap.loiter_radius, 1e-6);
}

// ---------------------------------------------------------------------------
// Invarian singgung — inti dari seluruh slice
// ---------------------------------------------------------------------------

TEST(ReleasePointSolver, ArahTangensialCwDiPSamaDenganApproachHeading)
{
  ReleasePointSolver solver(defaultBallistic(), defaultApproach());

  // Diuji untuk banyak arah angin supaya bukan kebetulan satu kuadran.
  for (int deg = 0; deg < 360; deg += 15) {
    const double a = deg * M_PI / 180.0;
    const auto plan = solver.solve(
      Ned2D{0.0, 0.0}, Ned2D{-500.0, 0.0},
      {6.0 * std::cos(a), 6.0 * std::sin(a)});

    // Sudut p dilihat dari pusat orbit s.
    const double theta = std::atan2(
      plan.entry_point.east - plan.loiter_center.east,
      plan.entry_point.north - plan.loiter_center.north);
    // Arah gerak CW di sudut itu.
    const double tangent_cw = std::atan2(std::cos(theta), -std::sin(theta));

    EXPECT_NEAR(std::abs(wrapPi(tangent_cw - plan.approach_heading)), 0.0, 1e-9)
      << "arah angin " << deg << " deg";
  }
}

TEST(ReleasePointSolver, OrbitDiSekitarLoiterCenterMemenuhiSyaratCapture)
{
  const auto ap = defaultApproach();
  ReleasePointSolver solver(defaultBallistic(), ap);
  const CaptureGate gate{25.0, 0.35};

  for (int deg = 0; deg < 360; deg += 15) {
    const double a = deg * M_PI / 180.0;
    const auto plan = solver.solve(
      Ned2D{0.0, 0.0}, Ned2D{-500.0, 0.0},
      {6.0 * std::cos(a), 6.0 * std::sin(a)});

    EXPECT_TRUE(orbitEverCaptures(plan, plan.loiter_center, ap.loiter_radius, gate))
      << "arah angin " << deg << " deg";
  }
}

// Regresi eksplisit atas bug SITL: mengorbit p membuat drop mustahil terpicu.
TEST(ReleasePointSolver, OrbitDiSekitarEntryPointTidakPernahMemenuhiSyaratCapture)
{
  const auto ap = defaultApproach();
  ReleasePointSolver solver(defaultBallistic(), ap);
  const CaptureGate gate{25.0, 0.35};

  for (int deg = 0; deg < 360; deg += 15) {
    const double a = deg * M_PI / 180.0;
    const auto plan = solver.solve(
      Ned2D{0.0, 0.0}, Ned2D{-500.0, 0.0},
      {6.0 * std::cos(a), 6.0 * std::sin(a)});

    EXPECT_FALSE(orbitEverCaptures(plan, plan.entry_point, ap.loiter_radius, gate))
      << "arah angin " << deg << " deg";
  }
}

// Radius orbit yang diterbangkan (WP_LOITER_RAD) harus sama dengan
// approach.loiter_radius. Kalau melenceng cukup jauh, orbit tidak lagi
// menyinggung p dan capture hilang lagi — kopling ini nyata, bukan gaya.
TEST(ReleasePointSolver, RadiusOrbitTidakCocokMenghilangkanCapture)
{
  const auto ap = defaultApproach();
  ReleasePointSolver solver(defaultBallistic(), ap);
  const CaptureGate gate{25.0, 0.35};
  const auto plan = solver.solve(Ned2D{0.0, 0.0}, Ned2D{-500.0, 0.0}, {0.0, 6.0});

  EXPECT_TRUE(orbitEverCaptures(plan, plan.loiter_center, ap.loiter_radius, gate));
  // WP_LOITER_RAD dua kali lipat -> orbit lewat jauh dari p.
  EXPECT_FALSE(orbitEverCaptures(plan, plan.loiter_center, 2.0 * ap.loiter_radius, gate));
}

// ---------------------------------------------------------------------------
// Replan di titik p (paper: heading DIKUNCI, angin TIDAK di-update)
// ---------------------------------------------------------------------------

TEST(ReleasePointSolver, SolveAlongHeadingMempertahankanHeading)
{
  ReleasePointSolver solver(defaultBallistic(), defaultApproach());
  const auto plan = solver.solve(Ned2D{0.0, 0.0}, Ned2D{-500.0, 0.0}, {0.0, 6.0});

  const auto replan = solver.solveAlongHeading(
    Ned2D{0.0, 0.0}, plan.approach_heading, 22.0, {0.0, 6.0});

  EXPECT_NEAR(replan.approach_heading, plan.approach_heading, 1e-12);
  // Ground speed lebih tinggi -> payload melayang lebih jauh -> release point
  // bergeser lebih ke belakang sepanjang heading.
  EXPECT_GT(std::hypot(replan.offset_north, replan.offset_east),
    std::hypot(plan.offset_north, plan.offset_east));
}

TEST(ReleasePointSolver, AnginLemahMemakaiArahDariWahanaKeTarget)
{
  ReleasePointSolver solver(defaultBallistic(), defaultApproach());
  // 0.2 m/s < min_wind_for_heading -> abaikan angin, ambil jalur terpendek.
  const Ned2D uav{-500.0, 0.0};
  const auto plan = solver.solve(Ned2D{0.0, 0.0}, uav, {0.2, 0.0});
  EXPECT_NEAR(plan.approach_heading, 0.0, 1e-9);  // wahana di selatan -> terbang ke utara
}
