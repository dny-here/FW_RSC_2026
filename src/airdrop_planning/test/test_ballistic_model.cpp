// Unit test model balistik & solver release point.
// Jalankan: colcon test --packages-select airdrop_planning

#include <gtest/gtest.h>
#include <cmath>

#include "airdrop_planning/ballistic_model.hpp"
#include "airdrop_planning/release_point_solver.hpp"

using airdrop_planning::BallisticModel;
using airdrop_planning::BallisticParams;
using airdrop_planning::ReleasePointSolver;
using airdrop_planning::ApproachParams;
using airdrop_planning::Ned2D;

TEST(BallisticModel, VacuumFreefallMatchesAnalytic)
{
  BallisticParams p;
  p.drag_coeff = 0.0;   // tanpa drag -> solusi analitik
  BallisticModel m(p);

  const double h = 50.0;
  const double v = 18.0;
  auto r = m.simulate(h, {v, 0.0, 0.0}, {0.0, 0.0});

  const double t_expected = std::sqrt(2.0 * h / p.gravity);   // ~3.19 s
  EXPECT_NEAR(r.fall_time_s, t_expected, 0.02);
  EXPECT_NEAR(r.offset_north_m, v * t_expected, 0.5);
  EXPECT_NEAR(r.offset_east_m, 0.0, 1e-6);
}

TEST(BallisticModel, DragReducesDownrangeTravel)
{
  BallisticParams no_drag; no_drag.drag_coeff = 0.0;
  BallisticParams drag;    drag.drag_coeff = 1.0; drag.ref_area_m2 = 0.02;

  auto r0 = BallisticModel(no_drag).simulate(50.0, {18.0, 0.0, 0.0}, {0.0, 0.0});
  auto r1 = BallisticModel(drag).simulate(50.0, {18.0, 0.0, 0.0}, {0.0, 0.0});

  EXPECT_LT(r1.offset_north_m, r0.offset_north_m);
  EXPECT_GT(r1.fall_time_s, 0.0);
}

TEST(BallisticModel, TailwindPushesObjectDownwind)
{
  BallisticParams p; p.drag_coeff = 1.0; p.ref_area_m2 = 0.02;
  BallisticModel m(p);

  auto no_wind = m.simulate(50.0, {18.0, 0.0, 0.0}, {0.0, 0.0});
  auto east_wind = m.simulate(50.0, {18.0, 0.0, 0.0}, {0.0, 5.0});

  EXPECT_GT(east_wind.offset_east_m, no_wind.offset_east_m);
}

TEST(ReleasePointSolver, ReleaseIsUpstreamOfTargetAgainstWind)
{
  BallisticParams bp; bp.drag_coeff = 0.8; bp.ref_area_m2 = 0.01;
  ApproachParams ap;

  ReleasePointSolver solver(bp, ap);

  // Angin bertiup KE arah timur (+E) 5 m/s -> wahana approach ke barat.
  Ned2D target{0.0, 0.0};
  Ned2D uav{-1000.0, 0.0};
  auto plan = solver.solve(target, uav, {0.0, 5.0});

  // Heading approach = melawan angin = menuju barat (-E) => sin(h) < 0
  EXPECT_LT(std::sin(plan.approach_heading), -0.9);

  // Wahana bergerak ke barat dgn ground speed (18 - 5) tapi ditiup...
  // ground vel = Va*heading + wind = (-18 + 5) arah timur = -13 E.
  // Payload melayang ke barat selama jatuh => offset_east negatif
  // => release point berada DI TIMUR target (digeser -offset).
  EXPECT_GT(plan.release_point.east, target.east);

  // Entry point p harus berada di belakang release point (downwind, timur).
  EXPECT_GT(plan.entry_point.east, plan.release_point.east);

  // Jarak p -> release = approach distance.
  const double d = std::hypot(
    plan.entry_point.north - plan.release_point.north,
    plan.entry_point.east - plan.release_point.east);
  EXPECT_NEAR(d, ap.approach_distance, 1e-6);

  // Loiter center berjarak r dari p (tangen).
  const double rs = std::hypot(
    plan.loiter_center.north - plan.entry_point.north,
    plan.loiter_center.east - plan.entry_point.east);
  EXPECT_NEAR(rs, ap.loiter_radius, 1e-6);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
