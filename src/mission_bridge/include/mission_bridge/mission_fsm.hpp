// mission_fsm.hpp
// FSM fase-misi DZ, ROS-free & header-only agar bisa di-unit-test tanpa
// ROS graph (pola sama dengan math airdrop_planning).
//   TRANSIT_TO_DZ -> (loiter 2x selesai) -> MONITOR_DONE
//   MONITOR_DONE  -> (plan ada) -> DROP_APPROACH (GUIDED)
//                 -> (timeout, belum lock) -> RESUME_AUTO (gagal)
//   DROP_APPROACH -> (drop selesai) -> RESUME_AUTO
#ifndef MISSION_BRIDGE__MISSION_FSM_HPP_
#define MISSION_BRIDGE__MISSION_FSM_HPP_

#include <cstdint>

namespace mission_bridge
{

enum class Phase : uint8_t {
  TRANSIT_TO_DZ = 0,
  MONITOR_DONE  = 1,
  DROP_APPROACH = 2,
  RESUME_AUTO   = 3
};

enum class ModeCmd : uint8_t { NONE = 0, GUIDED = 1, AUTO = 2 };

struct FsmInput {
  bool loiter_reached;   // mission/reached untuk item loiter DZ
  bool plan_available;   // AirdropPlan diterima sejak otorisasi
  bool drop_finished;    // airdrop sudah RELEASED atau abort/timeout
  bool drop_success;     // true bila RELEASED, false bila abort
  double now_s;          // waktu monotonik [s]
};

struct FsmConfig {
  double plan_wait_timeout_s{20.0};
};

struct FsmOutput {
  Phase phase{Phase::TRANSIT_TO_DZ};
  bool authorize_now{false};   // pulse: publish DropAuthorization(true)
  ModeCmd mode_cmd{ModeCmd::NONE};
  bool drop_failed{false};     // pulse: drop DZ ini gagal
};

class MissionFsm
{
public:
  explicit MissionFsm(const FsmConfig & cfg) : cfg_(cfg) {}

  FsmOutput step(const FsmInput & in)
  {
    FsmOutput out;
    switch (phase_) {
      case Phase::TRANSIT_TO_DZ:
        if (in.loiter_reached) {
          phase_ = Phase::MONITOR_DONE;
          monitor_done_time_s_ = in.now_s;
          out.authorize_now = true;
        }
        break;

      case Phase::MONITOR_DONE:
        if (in.plan_available) {
          phase_ = Phase::DROP_APPROACH;
          out.mode_cmd = ModeCmd::GUIDED;
        } else if (in.now_s - monitor_done_time_s_ >= cfg_.plan_wait_timeout_s) {
          phase_ = Phase::RESUME_AUTO;
          out.mode_cmd = ModeCmd::AUTO;
          out.drop_failed = true;
        }
        break;

      case Phase::DROP_APPROACH:
        if (in.drop_finished) {
          phase_ = Phase::RESUME_AUTO;
          out.mode_cmd = ModeCmd::AUTO;
          out.drop_failed = !in.drop_success;
        }
        break;

      case Phase::RESUME_AUTO:
        break;
    }
    out.phase = phase_;
    return out;
  }

  Phase phase() const { return phase_; }

private:
  FsmConfig cfg_;
  Phase phase_{Phase::TRANSIT_TO_DZ};
  double monitor_done_time_s_{0.0};
};

}  // namespace mission_bridge

#endif  // MISSION_BRIDGE__MISSION_FSM_HPP_
