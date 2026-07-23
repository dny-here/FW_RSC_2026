// mission_bridge_node.cpp
// Konduktor fase-misi DZ (Opsi C hybrid). Menerjemahkan output MissionFsm
// menjadi aksi MAVROS: otorisasi drop, switch GUIDED/AUTO, dan perintah
// tujuan GUIDED menuju loiter center -> overshoot (L1/TECS ArduPilot yang terbang).
//
// PENTING — kenapa DO_REPOSITION dan bukan setpoint_position/global:
// ArduPlane MEMBUANG lat/lon pada SET_POSITION_TARGET_GLOBAL_INT (pesan yang
// dikirim mavros/setpoint_position/global). Lihat ArduPlane/GCS_MAVLink_Plane.cpp
// handle_set_position_target_global_int(): ia hanya memanggil
// handle_change_alt_request() dengan lat=0,lng=0 — posisi tidak pernah diproses.
// Akibatnya wahana mengorbit titik tempat GUIDED diaktifkan, entry point p tak
// pernah dilewati, dan planner parkir selamanya di STATE_LOITER (terverifikasi
// dari dataflash: 111 s di GUIDED, NTUN.TLat/TLng hanya berisi SATU nilai).
// MAV_CMD_DO_REPOSITION (COMMAND_INT) adalah satu-satunya jalur yang benar-benar
// memanggil plane.set_guided_WP().
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "mavros_msgs/msg/waypoint_reached.hpp"
#include "mavros_msgs/srv/command_int.hpp"
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
    declare_parameter("overshoot_distance", 250.0);
    declare_parameter("guided_mode_name", std::string("GUIDED"));
    declare_parameter("auto_mode_name", std::string("AUTO"));

    dz_index_ = static_cast<uint8_t>(get_parameter("dz_index").as_int());
    dz_loiter_wp_index_ = static_cast<int>(get_parameter("dz_loiter_wp_index").as_int());
    survey_start_wp_index_ = static_cast<int>(get_parameter("survey_start_wp_index").as_int());
    survey_end_wp_index_ = static_cast<int>(get_parameter("survey_end_wp_index").as_int());
    approach_timeout_ = get_parameter("approach_timeout").as_double();
    overshoot_distance_ = get_parameter("overshoot_distance").as_double();
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
    // Ketinggian RELATIF terhadap home — bukan altitude NavSatFix, yang bernilai
    // tinggi ELIPSOID WGS-84. Mengirim tinggi elipsoid sebagai AMSL menaikkan
    // perintah sebesar separasi geoid (terukur ~19 m di SITL Canberra: wahana
    // memanjat 100 -> 119 m). Frame relative-alt tidak punya ambiguitas itu.
    sub_rel_alt_ = create_subscription<std_msgs::msg::Float64>(
      "mavros/global_position/rel_alt", sensor_qos,
      [this](const std_msgs::msg::Float64::SharedPtr m) {
        rel_alt_m_ = m->data;
        have_rel_alt_ = true;
      });
    sub_plan_ = create_subscription<interfaces::msg::AirdropPlan>(
      "airdrop/plan", 10,
      std::bind(&MissionBridgeNode::onPlan, this, std::placeholders::_1));
    sub_status_ = create_subscription<interfaces::msg::AirdropStatus>(
      "airdrop/status", 10,
      std::bind(&MissionBridgeNode::onStatus, this, std::placeholders::_1));

    pub_auth_ = create_publisher<interfaces::msg::DropAuthorization>(
      "mission/drop_authorized", latched);
    cli_set_mode_ = create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
    cli_reposition_ = create_client<mavros_msgs::srv::CommandInt>("mavros/cmd/command_int");

    // WAJIB: rclcpp::Time yang dibiarkan default memakai RCL_SYSTEM_TIME (2),
    // sedangkan now() memakai RCL_ROS_TIME (1). Mengurangkan keduanya MELEMPAR
    // std::runtime_error "can't subtract times with different time sources",
    // exception itu keluar dari timer callback dan mematikan node tanpa jejak
    // (file log tersisa 0 byte karena buffer tak sempat di-flush). Menginisialisasi
    // di sini menyamakan clock source keduanya sejak awal.
    approach_start_ = now();
    repos_last_attempt_ = now();

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

    // Radius orbit dikirim EKSPLISIT pada param3 DO_REPOSITION, jadi geometri
    // titik singgung p tidak lagi bergantung pada WP_LOITER_RAD FCU kebetulan
    // bernilai sama dengan approach.loiter_radius planner. WP_LOITER_RAD tetap
    // berlaku untuk loiter mission biasa, tapi bukan lagi kopling senyap di
    // jalur drop.
    RCLCPP_INFO_ONCE(get_logger(),
      "Plan diterima: orbit approach akan diperintahkan r=%.0f m searah jarum jam.",
      msg->loiter_radius);
  }

  void onStatus(const AirdropStatus::SharedPtr msg)
  {
    // Sumber kebenaran tunggal untuk "sudah sampai di p": planner. Bridge tidak
    // lagi menghitung sendiri jarak ke entry point — uji jarak murni bisa trip
    // di setiap putaran orbit (p berada DI orbit) sementara syarat course-nya
    // belum terpenuhi, memutus orbit sebelum planner sempat capture.
    planner_state_ = msg->state;
    if (msg->state == AirdropStatus::STATE_RELEASED) {
      saw_released_ = true;
    }
  }

  // Geser (lat,lon) sejauh d meter pada heading psi [rad, dari North, CW positif].
  // Equirectangular — konsisten dengan geo_utils planner, memadai untuk d << R.
  static std::pair<double, double> offsetLatLon(
    double lat, double lon, double heading, double d)
  {
    constexpr double R = 6378137.0;
    constexpr double kRad2Deg = 180.0 / M_PI;
    const double dn = d * std::cos(heading);
    const double de = d * std::sin(heading);
    const double coslat = std::cos(lat * M_PI / 180.0);
    return {
      lat + (dn / R) * kRad2Deg,
      lon + (de / (R * (std::abs(coslat) < 1e-9 ? 1e-9 : coslat))) * kRad2Deg};
  }

  // Perintahkan tujuan GUIDED via MAV_CMD_DO_REPOSITION (COMMAND_INT).
  //
  // Ini PERINTAH, bukan stream: dikirim sekali per perubahan tujuan, lalu
  // di-retry hanya bila FCU tidak mengiyakan. Menembakkannya 10 Hz akan
  // membanjiri link dan me-reset guided WP tiap tick.
  //
  // param3 = radius loiter dan param4 = 0 -> loiter_ccw = 0 (SEARAH JARUM JAM,
  // lihat handle_command_int_do_reposition). Dengan radius dikirim eksplisit
  // dari plan_.loiter_radius, geometri titik singgung p tidak lagi bergantung
  // pada WP_LOITER_RAD FCU kebetulan bernilai sama — konsisten by construction.
  //
  // param2 = MAV_DO_REPOSITION_FLAGS_CHANGE_MODE: ArduPlane sendiri yang masuk
  // GUIDED, sehingga tidak ada balapan antara set_mode dan perintah ini.
  void sendReposition(double lat, double lon, double alt_rel, double radius)
  {
    if (!cli_reposition_->service_is_ready()) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
        "Service %s belum siap — tujuan GUIDED tidak bisa diperintahkan.",
        cli_reposition_->get_service_name());
      return;
    }
    auto req = std::make_shared<mavros_msgs::srv::CommandInt::Request>();
    req->broadcast = false;
    req->frame = 6;    // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    req->command = 192;  // MAV_CMD_DO_REPOSITION
    req->current = 0;
    req->autocontinue = 0;
    req->param1 = 0.0f;   // ground speed: 0 = pakai default kendaraan
    req->param2 = 1.0f;   // MAV_DO_REPOSITION_FLAGS_CHANGE_MODE
    req->param3 = static_cast<float>(radius);
    req->param4 = 0.0f;   // yaw 0/NaN -> clockwise
    req->x = static_cast<int32_t>(std::lround(lat * 1e7));
    req->y = static_cast<int32_t>(std::lround(lon * 1e7));
    req->z = static_cast<float>(alt_rel);

    repos_sent_lat_ = lat;
    repos_sent_lon_ = lon;
    repos_have_sent_ = true;
    repos_ack_ok_ = false;
    repos_last_attempt_ = now();

    cli_reposition_->async_send_request(req,
      [this, lat, lon, radius](
        rclcpp::Client<mavros_msgs::srv::CommandInt>::SharedFuture fut) {
        const bool ok = fut.get()->success;
        repos_ack_ok_ = ok;
        if (ok) {
          RCLCPP_INFO(get_logger(),
            "DO_REPOSITION diterima: (%.7f, %.7f) r=%.0f m CW", lat, lon, radius);
        } else {
          RCLCPP_ERROR(get_logger(),
            "DO_REPOSITION DITOLAK FCU untuk (%.7f, %.7f) — akan di-retry. "
            "Penyebab lazim: bukan mode GUIDED, atau tujuan di luar geofence.",
            lat, lon);
        }
      });
  }

  // Kirim ulang hanya bila tujuan berubah (mis. planner replan, atau pindah ke
  // titik overshoot) atau bila FCU belum mengiyakan yang terakhir.
  void commandTarget(double lat, double lon, double alt_rel, double radius)
  {
    constexpr double kSameTargetDeg = 1e-6;   // ~0,11 m
    const bool moved = !repos_have_sent_ ||
      std::abs(lat - repos_sent_lat_) > kSameTargetDeg ||
      std::abs(lon - repos_sent_lon_) > kSameTargetDeg;
    // repos_have_sent_ mendahului sengaja: sebelum perintah pertama terkirim,
    // repos_last_attempt_ tidak bermakna dan tidak boleh ikut dievaluasi.
    const bool retry_due = repos_have_sent_ && !repos_ack_ok_ &&
      (now() - repos_last_attempt_).seconds() > 1.0;
    if (moved || retry_due) {
      sendReposition(lat, lon, alt_rel, radius);
    }
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

  // Pembungkus timer. Exception apa pun yang lolos dari callback timer akan
  // menghentikan spin() dan mematikan node TANPA jejak apa pun di file log
  // (buffer tidak sempat di-flush, log tersisa 0 byte). Itu sudah terjadi sekali
  // — clock source mismatch pada rclcpp::Time — dan menghabiskan satu
  // penerbangan penuh: wahana mengorbit selamanya karena tidak ada lagi yang
  // memerintah maupun men-timeout-nya. Node yang hidup dan berisik jauh lebih
  // baik daripada node yang mati diam-diam.
  void update()
  {
    try {
      updateStep();
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
        "EXCEPTION di update(): %s — siklus ini dilewati, node tetap hidup.",
        e.what());
    }
  }

  void updateStep()
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
      hold_alt_rel_ = have_rel_alt_ ? rel_alt_m_ : plan_.release_altitude_agl;
      repos_have_sent_ = false;   // paksa perintah pertama pada fase ini
      RCLCPP_INFO(get_logger(),
        "Mulai DROP_APPROACH (hold alt %.1f m relatif home%s) -> giring ke loiter center s",
        hold_alt_rel_, have_rel_alt_ ? "" : ", FALLBACK release_altitude_agl");
    }

    if (out.mode_cmd == ModeCmd::GUIDED) {setMode(guided_mode_name_);}
    if (out.mode_cmd == ModeCmd::AUTO) {setMode(auto_mode_name_);}

    if (out.drop_failed) {
      RCLCPP_WARN(get_logger(), "Drop DZ %u DICATAT GAGAL.", dz_index_);
    }

    // Streaming setpoint approach — prec_drop Sec. 2.3 / Fig. 5.
    //
    // Referensi PERTAMA adalah PUSAT LOITER s, bukan entry point p ("Its center s
    // will be the first reference sent to the guidance and control subsystem...
    // The UAV is supposed to fly towards the point s, and upon approaching it,
    // loiter around it in a clockwise direction"). p adalah titik SINGGUNG pada
    // orbit tersebut: di sana course wahana sejajar approach_heading sekaligus
    // cross-track nol, sehingga syarat capture planner bisa terpenuhi. Menaruh
    // setpoint di p justru membuat ArduPlane mengorbit p — dan pada orbit itu
    // cross kecil dan course selaras saling meniadakan, jadi capture tidak
    // pernah terjadi.
    //
    // Setelah planner masuk STATE_FINAL_APPROACH, tujuan pindah ke titik
    // OVERSHOOT di luar release point sepanjang approach_heading — bukan release
    // point itu sendiri. Tujuan GUIDED tepat di release point membuat ArduPlane
    // mulai membelok masuk orbit SEBELUM melewatinya, sehingga wahana tak pernah
    // melintas lurus dan cross-track velocity saat rilis tidak nol; prec_drop
    // Sec. 2.4 menyebut justru komponen itu yang mendominasi drop error. Pemicu
    // drop tetap milik planner (proximity circle), bukan tercapainya setpoint.
    //
    // planner_state_ dibaca setiap tick (bukan latch): bila planner jatuh ke
    // STATE_MISSED lalu replan, bridge otomatis kembali menggiring wahana ke s.
    if (fsm_->phase() == Phase::DROP_APPROACH && plan_available_ && have_veh_pos_) {
      if (planner_state_ == AirdropStatus::STATE_FINAL_APPROACH) {
        const auto ov = offsetLatLon(
          plan_.release_point_lat, plan_.release_point_lon,
          plan_.approach_heading, overshoot_distance_);
        commandTarget(ov.first, ov.second, hold_alt_rel_, plan_.loiter_radius);
      } else {
        commandTarget(plan_.loiter_center_lat, plan_.loiter_center_lon,
          hold_alt_rel_, plan_.loiter_radius);
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
  double overshoot_distance_{250.0};
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
  bool have_rel_alt_{false};

  // Bukan latch — cermin state planner saat ini, dipakai untuk memilih setpoint
  // (loiter center s vs titik overshoot). Sengaja bisa mundur lagi ke PLANNING
  // saat missed target supaya bridge ikut kembali menggiring wahana ke s.
  uint8_t planner_state_{AirdropStatus::STATE_IDLE};

  interfaces::msg::AirdropPlan plan_;
  double veh_lat_{0.0}, veh_lon_{0.0}, veh_alt_{0.0};
  double rel_alt_m_{0.0};
  double hold_alt_rel_{100.0};
  rclcpp::Time approach_start_;

  // Status perintah DO_REPOSITION terakhir. repos_ack_ok_ dipakai commandTarget()
  // untuk memutuskan retry; tanpa itu satu penolakan FCU akan membuat wahana
  // mengorbit titik lama tanpa jejak bahwa perintahnya tidak pernah berlaku.
  bool repos_have_sent_{false};
  bool repos_ack_ok_{false};
  double repos_sent_lat_{0.0};
  double repos_sent_lon_{0.0};
  rclcpp::Time repos_last_attempt_;

  rclcpp::Subscription<mavros_msgs::msg::WaypointReached>::SharedPtr sub_reached_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_globalpos_;
  rclcpp::Subscription<interfaces::msg::AirdropPlan>::SharedPtr sub_plan_;
  rclcpp::Subscription<interfaces::msg::AirdropStatus>::SharedPtr sub_status_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_rel_alt_;
  rclcpp::Publisher<interfaces::msg::DropAuthorization>::SharedPtr pub_auth_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr cli_set_mode_;
  rclcpp::Client<mavros_msgs::srv::CommandInt>::SharedPtr cli_reposition_;
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
