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

// --- urutan survey-first ---

static FsmInput survey_in(double t)
{
  FsmInput in{};
  in.now_s = t;
  return in;
}

static FsmConfig survey_cfg()
{
  FsmConfig cfg;
  cfg.plan_wait_timeout_s = 10.0;
  cfg.survey_enabled = true;
  return cfg;
}

TEST(MissionFsmSurvey, FullSequenceTransitSurveyLoiterAuthorizes)
{
  MissionFsm fsm(survey_cfg());
  EXPECT_EQ(fsm.phase(), Phase::TRANSIT_TO_DZ);

  FsmInput a = survey_in(1.0); a.survey_start_reached = true;
  EXPECT_EQ(fsm.step(a).phase, Phase::SURVEY);

  FsmInput b = survey_in(2.0);
  b.survey_start_reached = true; b.survey_end_reached = true;
  EXPECT_EQ(fsm.step(b).phase, Phase::MONITOR_LOITER);

  // loiter belum selesai -> belum boleh otorisasi
  FsmOutput hold = fsm.step(b);
  EXPECT_EQ(hold.phase, Phase::MONITOR_LOITER);
  EXPECT_FALSE(hold.authorize_now);

  FsmInput c = survey_in(3.0);
  c.survey_start_reached = true; c.survey_end_reached = true; c.loiter_reached = true;
  FsmOutput out = fsm.step(c);
  EXPECT_EQ(out.phase, Phase::MONITOR_DONE);
  EXPECT_TRUE(out.authorize_now);
  EXPECT_FALSE(out.order_violation);
}

TEST(MissionFsmSurvey, DisabledSurveyKeepsLegacyBehaviour)
{
  FsmConfig cfg;
  cfg.plan_wait_timeout_s = 10.0;
  cfg.survey_enabled = false;
  MissionFsm fsm(cfg);

  FsmInput a = survey_in(1.0); a.loiter_reached = true;
  FsmOutput out = fsm.step(a);
  EXPECT_EQ(out.phase, Phase::MONITOR_DONE);
  EXPECT_TRUE(out.authorize_now);
  EXPECT_FALSE(out.order_violation);
}

TEST(MissionFsmSurvey, LoiterDuringTransitStillAuthorizesButFlagsOrder)
{
  MissionFsm fsm(survey_cfg());

  FsmInput a = survey_in(1.0); a.loiter_reached = true;
  FsmOutput out = fsm.step(a);
  EXPECT_EQ(out.phase, Phase::MONITOR_DONE);
  EXPECT_TRUE(out.authorize_now);
  EXPECT_TRUE(out.order_violation);
}

TEST(MissionFsmSurvey, LoiterDuringSurveyStillAuthorizesButFlagsOrder)
{
  MissionFsm fsm(survey_cfg());

  FsmInput a = survey_in(1.0); a.survey_start_reached = true;
  ASSERT_EQ(fsm.step(a).phase, Phase::SURVEY);

  FsmInput b = survey_in(2.0);
  b.survey_start_reached = true; b.loiter_reached = true;
  FsmOutput out = fsm.step(b);
  EXPECT_EQ(out.phase, Phase::MONITOR_DONE);
  EXPECT_TRUE(out.authorize_now);
  EXPECT_TRUE(out.order_violation);
}

TEST(MissionFsmSurvey, SurveyEndWithoutStartSkipsToMonitorLoiter)
{
  MissionFsm fsm(survey_cfg());

  FsmInput a = survey_in(1.0); a.survey_end_reached = true;
  FsmOutput out = fsm.step(a);
  EXPECT_EQ(out.phase, Phase::MONITOR_LOITER);
  EXPECT_FALSE(out.authorize_now);
  EXPECT_FALSE(out.order_violation);
}

TEST(MissionFsmSurvey, OrderViolationIsPulseNotLevel)
{
  MissionFsm fsm(survey_cfg());

  FsmInput a = survey_in(1.0); a.loiter_reached = true;
  ASSERT_TRUE(fsm.step(a).order_violation);

  FsmInput b = survey_in(1.1); b.loiter_reached = true;
  FsmOutput out = fsm.step(b);
  EXPECT_FALSE(out.order_violation);
  EXPECT_FALSE(out.authorize_now);
}

// --- invarian keselamatan (Fix 6, review whole-branch) ---

TEST(MissionFsmSafety, ModeCmdStaysNoneThroughSurveyAndMonitorLoiter)
{
  MissionFsm fsm(survey_cfg());

  FsmInput a = survey_in(1.0); a.survey_start_reached = true;
  FsmOutput out_survey = fsm.step(a);
  ASSERT_EQ(out_survey.phase, Phase::SURVEY);
  EXPECT_EQ(out_survey.mode_cmd, ModeCmd::NONE);

  FsmInput b = survey_in(2.0);
  b.survey_start_reached = true; b.survey_end_reached = true;
  FsmOutput out_loiter = fsm.step(b);
  ASSERT_EQ(out_loiter.phase, Phase::MONITOR_LOITER);
  EXPECT_EQ(out_loiter.mode_cmd, ModeCmd::NONE);

  // beberapa tick lagi menunggu loiter, mode_cmd tetap NONE
  FsmOutput out_hold = fsm.step(b);
  EXPECT_EQ(out_hold.phase, Phase::MONITOR_LOITER);
  EXPECT_EQ(out_hold.mode_cmd, ModeCmd::NONE);
}

TEST(MissionFsmSafety, MonitorLoiterNeverTimesOutOrAuthorizes)
{
  // Paling kritis: bila MONITOR_LOITER pernah timeout, drop bisa terotorisasi
  // TANPA loiter pernah diterbangkan sama sekali.
  MissionFsm fsm(survey_cfg());

  FsmInput a = survey_in(1.0); a.survey_start_reached = true;
  ASSERT_EQ(fsm.step(a).phase, Phase::SURVEY);

  FsmInput b = survey_in(2.0);
  b.survey_start_reached = true; b.survey_end_reached = true;
  ASSERT_EQ(fsm.step(b).phase, Phase::MONITOR_LOITER);

  // now_s maju jauh melewati 10x plan_wait_timeout_s (10.0 pada survey_cfg()),
  // loiter_reached tetap false.
  FsmInput c = survey_in(2.0 + 10.0 * survey_cfg().plan_wait_timeout_s);
  c.survey_start_reached = true; c.survey_end_reached = true;
  FsmOutput out = fsm.step(c);

  EXPECT_EQ(out.phase, Phase::MONITOR_LOITER);
  EXPECT_FALSE(out.authorize_now);
  EXPECT_FALSE(out.drop_failed);
  EXPECT_EQ(out.mode_cmd, ModeCmd::NONE);
}

TEST(MissionFsmSafety, SurveyDisabledIgnoresSurveyReachedInputs)
{
  FsmConfig cfg;
  cfg.plan_wait_timeout_s = 10.0;
  cfg.survey_enabled = false;
  MissionFsm fsm(cfg);

  FsmInput a = survey_in(1.0);
  a.survey_start_reached = true; a.survey_end_reached = true;
  FsmOutput out = fsm.step(a);

  EXPECT_EQ(out.phase, Phase::TRANSIT_TO_DZ);
  EXPECT_FALSE(out.authorize_now);
}

TEST(MissionFsmConfig, PlanWaitTimeoutDefaultIsTenSeconds)
{
  FsmConfig cfg;
  EXPECT_DOUBLE_EQ(cfg.plan_wait_timeout_s, 10.0);
}
