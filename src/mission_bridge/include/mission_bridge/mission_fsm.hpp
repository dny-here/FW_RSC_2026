// mission_fsm.hpp
// FSM fase-misi DZ, ROS-free & header-only agar bisa di-unit-test tanpa
// ROS graph (pola sama dengan math airdrop_planning).
//   TRANSIT_TO_DZ  -> (survey_start)      -> SURVEY
//   SURVEY         -> (survey_end)        -> MONITOR_LOITER
//   MONITOR_LOITER -> (loiter 2x selesai) -> MONITOR_DONE [authorize]
//   MONITOR_DONE   -> (plan ada)          -> DROP_APPROACH (GUIDED)
//                  -> (timeout)           -> RESUME_AUTO (gagal)
//   DROP_APPROACH  -> (drop selesai)      -> RESUME_AUTO
//
// Deteksi terpal terjadi di fase SURVEY (wings-level), bukan saat loiter:
// pusat loiter berada 45 deg off-nadir sementara tilt badan hanya ~22 deg.
// Loiter 2x tetap SELALU dieksekusi (poin pemantauan) dan memberi estimasi
// angin segar tepat sebelum drop.
//
// Bila survey_enabled = false, TRANSIT_TO_DZ langsung ke MONITOR_DONE pada
// loiter_reached — perilaku sebelum urutan survey-first, dipertahankan.
#ifndef MISSION_BRIDGE__MISSION_FSM_HPP_
#define MISSION_BRIDGE__MISSION_FSM_HPP_

#include <cstdint>

namespace mission_bridge
{

enum class Phase : uint8_t {
  TRANSIT_TO_DZ  = 0,
  SURVEY         = 1,
  MONITOR_LOITER = 2,
  MONITOR_DONE   = 3,
  DROP_APPROACH  = 4,
  RESUME_AUTO    = 5
};

enum class ModeCmd : uint8_t { NONE = 0, GUIDED = 1, AUTO = 2 };

struct FsmInput {
  bool loiter_reached{false};   // mission/reached untuk item loiter DZ
  bool plan_available{false};   // AirdropPlan diterima sejak otorisasi
  bool drop_finished{false};    // airdrop sudah RELEASED atau abort/timeout
  bool drop_success{false};     // true bila RELEASED, false bila abort
  double now_s{0.0};            // waktu monotonik [s]
  // Field baru ditaruh setelah now_s agar aggregate-init lama tetap valid.
  bool survey_start_reached{false};  // mission/reached item pertama grid
  bool survey_end_reached{false};    // mission/reached item terakhir grid
};

struct FsmConfig {
  double plan_wait_timeout_s{10.0};
  bool survey_enabled{false};
};

struct FsmOutput {
  Phase phase{Phase::TRANSIT_TO_DZ};
  bool authorize_now{false};    // pulse: publish DropAuthorization(true)
  ModeCmd mode_cmd{ModeCmd::NONE};
  bool drop_failed{false};      // pulse: drop DZ ini gagal
  bool order_violation{false};  // pulse: loiter selesai sebelum survey tuntas
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
          // Loiter benar-benar selesai (sinyal dari FCU). Tetap otorisasi:
          // kehilangan drop karena indeks wp salah isi lebih mahal daripada
          // log kotor. order_violation hanya bermakna bila survey diaktifkan.
          out.order_violation = cfg_.survey_enabled;
          enterMonitorDone(in, out);
        } else if (cfg_.survey_enabled && in.survey_end_reached) {
          // Misi di-resume di tengah grid: lompat langsung, jangan mandek.
          //
          // survey_end_reached SENGAJA dicek sebelum survey_start_reached (bukan
          // sebaliknya) supaya resume-di-tengah-grid di atas berfungsi. Konsekuensi:
          // bila konfigurasi punya survey_start_wp_index == survey_end_wp_index
          // (diizinkan oleh validateSurveyIndices(), syaratnya start <= end, bukan
          // <), satu event mission/reached men-set survey_start_reached DAN
          // survey_end_reached pada tick yang sama, cabang ini menang duluan, dan
          // FSM lompat langsung ke MONITOR_LOITER — fase SURVEY tidak pernah
          // muncul di log/phase. Untuk grid survei sungguhan pakai start < end.
          phase_ = Phase::MONITOR_LOITER;
        } else if (cfg_.survey_enabled && in.survey_start_reached) {
          phase_ = Phase::SURVEY;
        }
        break;

      case Phase::SURVEY:
        if (in.loiter_reached) {
          out.order_violation = true;
          enterMonitorDone(in, out);
        } else if (in.survey_end_reached) {
          phase_ = Phase::MONITOR_LOITER;
        }
        break;

      case Phase::MONITOR_LOITER:
        if (in.loiter_reached) {
          enterMonitorDone(in, out);
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
  void enterMonitorDone(const FsmInput & in, FsmOutput & out)
  {
    phase_ = Phase::MONITOR_DONE;
    monitor_done_time_s_ = in.now_s;
    out.authorize_now = true;
  }

  FsmConfig cfg_;
  Phase phase_{Phase::TRANSIT_TO_DZ};
  double monitor_done_time_s_{0.0};
};

}  // namespace mission_bridge

#endif  // MISSION_BRIDGE__MISSION_FSM_HPP_
