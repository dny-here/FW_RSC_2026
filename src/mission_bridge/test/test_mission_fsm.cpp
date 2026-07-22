#include <gtest/gtest.h>
#include "mission_bridge/mission_fsm.hpp"

using namespace mission_bridge;

static FsmInput idle_in(double t) { return FsmInput{false, false, false, false, t}; }

TEST(MissionFsm, StartsInTransit)
{
  MissionFsm fsm(FsmConfig{20.0});
  EXPECT_EQ(fsm.phase(), Phase::TRANSIT_TO_DZ);
}

TEST(MissionFsm, LoiterReachedAuthorizesAndPulsesOnce)
{
  MissionFsm fsm(FsmConfig{20.0});
  FsmInput in = idle_in(1.0);
  in.loiter_reached = true;
  FsmOutput out = fsm.step(in);
  EXPECT_EQ(out.phase, Phase::MONITOR_DONE);
  EXPECT_TRUE(out.authorize_now);

  // pulse hanya sekali
  FsmOutput out2 = fsm.step(idle_in(1.1));
  EXPECT_FALSE(out2.authorize_now);
  EXPECT_EQ(out2.phase, Phase::MONITOR_DONE);
}

TEST(MissionFsm, PlanAvailableGoesGuided)
{
  MissionFsm fsm(FsmConfig{20.0});
  FsmInput a = idle_in(1.0); a.loiter_reached = true; fsm.step(a);

  FsmInput b = idle_in(2.0); b.plan_available = true;
  FsmOutput out = fsm.step(b);
  EXPECT_EQ(out.phase, Phase::DROP_APPROACH);
  EXPECT_EQ(out.mode_cmd, ModeCmd::GUIDED);
}

TEST(MissionFsm, TimeoutWithoutPlanResumesAutoAndFails)
{
  MissionFsm fsm(FsmConfig{20.0});
  FsmInput a = idle_in(1.0); a.loiter_reached = true; fsm.step(a);

  FsmOutput out = fsm.step(idle_in(21.5));  // 20.5 s > timeout 20 s
  EXPECT_EQ(out.phase, Phase::RESUME_AUTO);
  EXPECT_EQ(out.mode_cmd, ModeCmd::AUTO);
  EXPECT_TRUE(out.drop_failed);
}

TEST(MissionFsm, DropSuccessResumesAutoNoFail)
{
  MissionFsm fsm(FsmConfig{20.0});
  FsmInput a = idle_in(1.0); a.loiter_reached = true; fsm.step(a);
  FsmInput b = idle_in(2.0); b.plan_available = true; fsm.step(b);

  FsmInput c = idle_in(30.0); c.drop_finished = true; c.drop_success = true;
  FsmOutput out = fsm.step(c);
  EXPECT_EQ(out.phase, Phase::RESUME_AUTO);
  EXPECT_EQ(out.mode_cmd, ModeCmd::AUTO);
  EXPECT_FALSE(out.drop_failed);
}

TEST(MissionFsm, DropAbortResumesAutoWithFail)
{
  MissionFsm fsm(FsmConfig{20.0});
  FsmInput a = idle_in(1.0); a.loiter_reached = true; fsm.step(a);
  FsmInput b = idle_in(2.0); b.plan_available = true; fsm.step(b);

  FsmInput c = idle_in(30.0); c.drop_finished = true; c.drop_success = false;
  FsmOutput out = fsm.step(c);
  EXPECT_EQ(out.phase, Phase::RESUME_AUTO);
  EXPECT_TRUE(out.drop_failed);
}
