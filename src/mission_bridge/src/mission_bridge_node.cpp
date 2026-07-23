// mission_bridge_node.cpp
// Konduktor fase-misi DZ (Opsi C hybrid). Menerjemahkan output MissionFsm
// menjadi aksi MAVROS: otorisasi drop, switch GUIDED/AUTO, dan streaming
// setpoint global menuju entry -> release (L1/TECS ArduPilot yang terbang).
#include <cmath>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geographic_msgs/msg/geo_pose_stamped.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "interfaces/msg/airdrop_plan.hpp"
#include "interfaces/msg/airdrop_status.hpp"
#include "interfaces/msg/drop_authorization.hpp"

#include "mission_bridge/mission_fsm.hpp"

using namespace std::chrono_literals;

namespace mission_bridge
{

class MissionBridgeNode : public rclcpp::Node
{
public:
  MissionBridgeNode()
  : Node("mission_bridge")
  {
    declare_parameter("dz_index", 0);
    declare_parameter("dz_loiter_wp_index", -1);
    declare_parameter("survey_start_wp_index", -1);
    declare_parameter("survey_end_wp_index", -1);
    declare_parameter("plan_wait_timeout", 10.0);
    declare_parameter("approach_timeout", 60.0);
    declare_parameter("entry_reach_radius", 30.0);
    declare_parameter("guided_mode_name", std::string("GUIDED"));
    declare_parameter("auto_mode_name", std::string("AUTO"));

    dz_index_ = static_cast<uint8_t>(get_parameter("dz_index").as_int());
    dz_loiter_wp_index_ = static_cast<int>(get_parameter("dz_loiter_wp_index").as_int());
    survey_start_wp_index_ = static_cast<int>(get_parameter("survey_start_wp_index").as_int());
    survey_end_wp_index_ = static_cast<int>(get_parameter("survey_end_wp_index").as_int());
    approach_timeout_ = get_parameter("approach_timeout").as_double();
    entry_reach_radius_ = get_parameter("entry_reach_radius").as_double();
    guided_mode_name_ = get_parameter("guided_mode_name").as_string();
    auto_mode_name_ = get_parameter("auto_mode_name").as_string();

    FsmConfig cfg;
    cfg.plan_wait_timeout_s = get_parameter("plan_wait_timeout").as_double();
    cfg.survey_enabled = validateSurveyIndices();
    fsm_ = std::make_unique<MissionFsm>(cfg);

    auto sensor_qos = rclcpp::SensorDataQoS();
    rclcpp::QoS latched(1); latched.transient_local();

    sub_reached_ = create_subscription<mavros_msgs::msg::WaypointReached>(
      "mavros/mission/reached", rclcpp::QoS(10),
      std::bind(&MissionBridgeNode::onReached, this, std::placeholders::_1));
    sub_globalpos_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "mavros/global_position/global", sensor_qos,
      std::bind(&MissionBridgeNode::onGlobalPos, this, std::placeholders::_1));
    sub_plan_ = create_subscription<interfaces::msg::AirdropPlan>(
      "airdrop/plan", 10,
      std::bind(&MissionBridgeNode::onPlan, this, std::placeholders::_1));
    sub_status_ = create_subscription<interfaces::msg::AirdropStatus>(
      "airdrop/status", 10,
      std::bind(&MissionBridgeNode::onStatus, this, std::placeholders::_1));

    pub_auth_ = create_publisher<interfaces::msg::DropAuthorization>(
      "mission/drop_authorized", latched);
    pub_setpoint_ = create_publisher<geographic_msgs::msg::GeoPoseStamped>(
      "mavros/setpoint_position/global", 10);

    cli_set_mode_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");

    timer_ = create_wall_timer(100ms, std::bind(&MissionBridgeNode::update, this));

    // Baris kesiapan dicetak sebelum log "Fase survey AKTIF/NONAKTIF" (yang
    // ditulis dari validateSurveyIndices() di atas) supaya urutan log tidak
    // terbalik, dan mencerminkan fase yang sebenarnya sedang ditunggu.
    if (cfg.survey_enabled) {
      RCLCPP_INFO(get_logger(),
        "mission_bridge siap (DZ %u, survey wp %d..%d, loiter wp %d). Menunggu survey mulai...",
        dz_index_, survey_start_wp_index_, survey_end_wp_index_, dz_loiter_wp_index_);
    } else {
      RCLCPP_INFO(get_logger(),
        "mission_bridge siap (DZ %u, loiter wp %d). Menunggu loiter selesai...",
        dz_index_, dz_loiter_wp_index_);
    }

    logSurveyValidation();

    if (dz_loiter_wp_index_ < 0) {
      RCLCPP_WARN(get_logger(),
        "dz_loiter_wp_index TIDAK diset (sentinel -1) — GATE DROP NONAKTIF sampai "
        "parameter ini diisi dengan wp_seq item LOITER_TURNS DZ!");
    }
  }

private:
  using AirdropStatus = interfaces::msg::AirdropStatus;

  enum class SurveyDiagLevel { kDisabled, kInvalid, kEnabled };

  // Syarat: 0 <= start <= end < loiter. Gagal syarat -> fase survey dimatikan
  // (fallback perilaku lama), BUKAN crash dan BUKAN mematikan otorisasi drop.
  //
  // CATATAN: fungsi ini MEVALIDASI *dan* MENSANITASI — pada kegagalan syarat ia
  // menge-nol-kan (sentinel -1) survey_start_wp_index_/survey_end_wp_index_,
  // bukan cuma melaporkan error. Ini disengaja: onReached() memakai kedua
  // anggota ini langsung, jadi men-sanitasi di sini mengeraskan onReached()
  // terhadap indeks yang setengah-valid tanpa perlu cek ulang di sana.
  //
  // Pencatatan log hasil validasi SENGAJA ditunda (lihat logSurveyValidation()
  // dan pemanggilnya di konstruktor) agar baris "mission_bridge siap" tetap
  // tercetak duluan, bukan setelah baris "Fase survey AKTIF/NONAKTIF".
  bool validateSurveyIndices()
  {
    if (survey_start_wp_index_ < 0 && survey_end_wp_index_ < 0) {
      survey_diag_level_ = SurveyDiagLevel::kDisabled;
      return false;
    }
    if (survey_start_wp_index_ < 0 || survey_end_wp_index_ < 0 ||
      survey_start_wp_index_ > survey_end_wp_index_ ||
      dz_loiter_wp_index_ < 0 ||
      survey_end_wp_index_ >= dz_loiter_wp_index_)
    {
      survey_diag_level_ = SurveyDiagLevel::kInvalid;
      // Simpan nilai mentah (sebelum disanitasi) untuk pesan error di bawah.
      survey_diag_start_ = survey_start_wp_index_;
      survey_diag_end_ = survey_end_wp_index_;
      survey_start_wp_index_ = -1;
      survey_end_wp_index_ = -1;
      return false;
    }
    survey_diag_level_ = SurveyDiagLevel::kEnabled;
    return true;
  }

  // Mencetak hasil validateSurveyIndices(). Dipanggil dari konstruktor SETELAH
  // baris kesiapan "mission_bridge siap" — lihat catatan di validateSurveyIndices().
  void logSurveyValidation()
  {
    switch (survey_diag_level_) {
      case SurveyDiagLevel::kDisabled:
        // Cetak nilai aktual (bukan literal "-1" hardcoded) — cabang ini
        // dieksekusi untuk pasangan negatif apa pun, tidak hanya -1 persis.
        RCLCPP_INFO(get_logger(),
          "Fase survey NONAKTIF (survey_start_wp_index=%d, survey_end_wp_index=%d) — "
          "otorisasi drop langsung saat loiter selesai.",
          survey_start_wp_index_, survey_end_wp_index_);
        break;
      case SurveyDiagLevel::kInvalid:
        RCLCPP_ERROR(get_logger(),
          "Indeks survey TIDAK VALID (start=%d, end=%d, loiter=%d) — syarat: "
          "0 <= start <= end < loiter. Fase survey DINONAKTIFKAN.",
          survey_diag_start_, survey_diag_end_, dz_loiter_wp_index_);
        break;
      case SurveyDiagLevel::kEnabled:
        RCLCPP_INFO(get_logger(), "Fase survey AKTIF: wp %d..%d, loiter wp %d.",
          survey_start_wp_index_, survey_end_wp_index_, dz_loiter_wp_index_);
        break;
    }
  }

  static const char * phaseName(Phase p)
  {
    switch (p) {
      case Phase::TRANSIT_TO_DZ:  return "TRANSIT_TO_DZ";
      case Phase::SURVEY:         return "SURVEY";
      case Phase::MONITOR_LOITER: return "MONITOR_LOITER";
      case Phase::MONITOR_DONE:   return "MONITOR_DONE";
      case Phase::DROP_APPROACH:  return "DROP_APPROACH";
      case Phase::RESUME_AUTO:    return "RESUME_AUTO";
    }
    return "?";
  }

  void onReached(const mavros_msgs::msg::WaypointReached::SharedPtr msg)
  {
    if (dz_loiter_wp_index_ < 0) {
      RCLCPP_WARN_ONCE(get_logger(),
        "mission/reached diterima tapi dz_loiter_wp_index belum diset (sentinel -1) — "
        "event diabaikan, gate drop tetap nonaktif.");
      return;
    }
    const int seq = static_cast<int>(msg->wp_seq);
    // Selalu dicetak (bukan hanya saat cocok) — ini satu-satunya cara operator
    // melihat, dari log, bahwa mission yang di-load tidak cocok dengan
    // survey_*_wp_index/dz_loiter_wp_index (mis. mission file lama termuat
    // sementara param mengacu ke mission baru): tanpa baris ini, wp_seq yang
    // tidak pernah match hanya membuat FSM diam di satu fase tanpa jejak.
    RCLCPP_INFO(get_logger(),
      "mission/reached wp_seq=%d (survey %d..%d, loiter %d)",
      seq, survey_start_wp_index_, survey_end_wp_index_, dz_loiter_wp_index_);
    if (survey_start_wp_index_ >= 0 && seq == survey_start_wp_index_) {
      survey_start_reached_ = true;
    }
    if (survey_end_wp_index_ >= 0 && seq == survey_end_wp_index_) {
      survey_end_reached_ = true;
    }
    if (seq == dz_loiter_wp_index_) {
      loiter_reached_ = true;
    }
  }

  void onGlobalPos(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    veh_lat_ = msg->latitude;
    veh_lon_ = msg->longitude;
    veh_alt_ = msg->altitude;
    have_veh_pos_ = true;
  }

  void onPlan(const interfaces::msg::AirdropPlan::SharedPtr msg)
  {
    plan_ = *msg;
    plan_available_ = true;
  }

  void onStatus(const AirdropStatus::SharedPtr msg)
  {
    if (msg->state == AirdropStatus::STATE_RELEASED) {
      saw_released_ = true;
    }
  }

  // haversine [m]
  static double distMeters(double lat1, double lon1, double lat2, double lon2)
  {
    constexpr double R = 6378137.0;
    const double dlat = (lat2 - lat1) * M_PI / 180.0;
    const double dlon = (lon2 - lon1) * M_PI / 180.0;
    const double a = std::sin(dlat / 2) * std::sin(dlat / 2) +
      std::cos(lat1 * M_PI / 180.0) * std::cos(lat2 * M_PI / 180.0) *
      std::sin(dlon / 2) * std::sin(dlon / 2);
    return 2.0 * R * std::asin(std::min(1.0, std::sqrt(a)));
  }

  void publishSetpoint(double lat, double lon, double alt)
  {
    geographic_msgs::msg::GeoPoseStamped sp;
    sp.header.stamp = now();
    sp.header.frame_id = "map";
    sp.pose.position.latitude = lat;
    sp.pose.position.longitude = lon;
    sp.pose.position.altitude = alt;
    sp.pose.orientation.w = 1.0;
    pub_setpoint_->publish(sp);
  }

  void setMode(const std::string & mode)
  {
    if (!cli_set_mode_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "set_mode service belum siap; gagal minta mode %s",
        mode.c_str());
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    req->custom_mode = mode;
    cli_set_mode_->async_send_request(req,
      [this, mode](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture fut) {
        RCLCPP_INFO(get_logger(), "set_mode %s -> mode_sent=%d",
          mode.c_str(), fut.get()->mode_sent);
      });
  }

  void update()
  {
    // hitung drop_finished/success dari status + timeout backstop
    bool drop_finished = false;
    bool drop_success = false;
    if (fsm_->phase() == Phase::DROP_APPROACH) {
      const double elapsed = (now() - approach_start_).seconds();
      drop_finished = saw_released_ || (elapsed > approach_timeout_);
      drop_success = saw_released_;
    }

    FsmInput in;
    in.loiter_reached = loiter_reached_;
    in.plan_available = plan_available_;
    in.drop_finished = drop_finished;
    in.drop_success = drop_success;
    in.now_s = now().seconds();
    in.survey_start_reached = survey_start_reached_;
    in.survey_end_reached = survey_end_reached_;

    const Phase before = fsm_->phase();
    FsmOutput out = fsm_->step(in);

    if (before != out.phase) {
      RCLCPP_INFO(get_logger(), "Fase: %s -> %s",
        phaseName(before), phaseName(out.phase));
    }

    if (out.order_violation) {
      RCLCPP_ERROR(get_logger(),
        "URUTAN MISSION SALAH: loiter wp %d tercapai saat fase masih %s — survey "
        "wp %d..%d belum tuntas. Otorisasi TETAP diterbitkan. Periksa urutan item "
        "mission dan nilai survey_*_wp_index / dz_loiter_wp_index, ATAU event "
        "mission/reached untuk survey_end tidak diterima (node restart / pesan "
        "hilang — mission/reached bersifat one-shot & tidak di-latch, jadi mission "
        "bisa saja sudah benar).",
        dz_loiter_wp_index_, phaseName(before),
        survey_start_wp_index_, survey_end_wp_index_);
    }

    if (out.authorize_now) {
      interfaces::msg::DropAuthorization a;
      a.header.stamp = now();
      a.dz_index = dz_index_;
      a.authorized = true;
      pub_auth_->publish(a);
      RCLCPP_INFO(get_logger(),
        "Loiter 2x selesai -> OTORISASI drop DZ %u", dz_index_);
    }

    if (before != Phase::DROP_APPROACH && out.phase == Phase::DROP_APPROACH) {
      approach_start_ = now();
      hold_alt_amsl_ = have_veh_pos_ ? veh_alt_ : plan_.release_altitude_agl;
      entry_reached_ = false;
      RCLCPP_INFO(get_logger(), "Mulai DROP_APPROACH (hold alt %.1f m AMSL)",
        hold_alt_amsl_);
    }

    if (out.mode_cmd == ModeCmd::GUIDED) {setMode(guided_mode_name_);}
    if (out.mode_cmd == ModeCmd::AUTO) {setMode(auto_mode_name_);}

    if (out.drop_failed) {
      RCLCPP_WARN(get_logger(), "Drop DZ %u DICATAT GAGAL.", dz_index_);
    }

    // streaming setpoint approach: entry dulu, lalu release
    if (fsm_->phase() == Phase::DROP_APPROACH && plan_available_ && have_veh_pos_) {
      if (!entry_reached_) {
        const double d = distMeters(veh_lat_, veh_lon_,
          plan_.entry_point_lat, plan_.entry_point_lon);
        if (d < entry_reach_radius_) {entry_reached_ = true;}
      }
      if (entry_reached_) {
        publishSetpoint(plan_.release_point_lat, plan_.release_point_lon, hold_alt_amsl_);
      } else {
        publishSetpoint(plan_.entry_point_lat, plan_.entry_point_lon, hold_alt_amsl_);
      }
    }
  }

  // --- anggota ---
  std::unique_ptr<MissionFsm> fsm_;

  uint8_t dz_index_{0};
  int dz_loiter_wp_index_{-1};
  int survey_start_wp_index_{-1};
  int survey_end_wp_index_{-1};
  double approach_timeout_{60.0};
  double entry_reach_radius_{30.0};
  std::string guided_mode_name_{"GUIDED"};
  std::string auto_mode_name_{"AUTO"};

  // Hasil validateSurveyIndices(), dicetak belakangan oleh logSurveyValidation()
  // — lihat catatan pada validateSurveyIndices(). survey_diag_start_/end_ hanya
  // dipakai untuk pesan kInvalid (menyimpan nilai sebelum disanitasi ke -1).
  SurveyDiagLevel survey_diag_level_{SurveyDiagLevel::kDisabled};
  int survey_diag_start_{-1};
  int survey_diag_end_{-1};

  // Semua flag "reached"/"available"/"seen" di bawah adalah latch one-shot:
  // sekali true, tidak pernah direset. Untuk DZ tunggal itu benar (proses
  // linear TRANSIT->...->RESUME_AUTO). Implementasi multi-DZ di masa depan
  // WAJIB mereset flag-flag ini (dan dz_index_) saat pindah ke DZ berikutnya,
  // atau state DZ sebelumnya akan bocor ke DZ baru.
  bool loiter_reached_{false};
  bool survey_start_reached_{false};
  bool survey_end_reached_{false};
  bool plan_available_{false};
  bool saw_released_{false};
  bool have_veh_pos_{false};
  bool entry_reached_{false};

  interfaces::msg::AirdropPlan plan_;
  double veh_lat_{0.0}, veh_lon_{0.0}, veh_alt_{0.0};
  double hold_alt_amsl_{100.0};
  rclcpp::Time approach_start_;

  rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr sub_reached_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_globalpos_;
  rclcpp::Subscription<interfaces::msg::AirdropPlan>::SharedPtr sub_plan_;
  rclcpp::Subscription<interfaces::msg::AirdropStatus>::SharedPtr sub_status_;
  rclcpp::Publisher<interfaces::msg::DropAuthorization>::SharedPtr pub_auth_;
  rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr pub_setpoint_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr cli_set_mode_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace mission_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<mission_bridge::MissionBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
