// airdrop_planner_node.cpp
// Node "Airdrop Planning" — KRTI 2026 Divisi Fixed-Wing.
//
// Node SWAKELOLA sesuai gambar arsitektur sistem: SATU node yang merencanakan
// DAN menerbangkan approach drop (tanpa mission_bridge terpisah), agar hanya
// airdrop_planning + target_recognition yang berjalan di Raspberry Pi.
//
// ref prec_drop Sec. 2.3:
//   input  : ACTION GOAL interfaces/action/TargetAirdrop dari node
//            target_recognition (berisi TargetEstimate + bay_index)
//            posisi & kecepatan wahana + estimasi angin (dari MAVROS/FCU)
//   proses : hitung release point (Algorithm 1, model balistik eq. 3-5),
//            bangun truncated Dubins approach (loiter s -> entry p -> release),
//            GIRING wahana lewat setpoint GUIDED (DO_REPOSITION) — L1/TECS
//            ArduPilot yang menerbangkan; kita TIDAK menghitung roll/pitch
//            sendiri (kotak "LOS Guidance" di rancangan tidak diimplementasi),
//            replan release point saat mencapai titik p (ground speed aktual,
//            heading DIKUNCI), pelepasan payload dengan MENEMBAK servo bay
//            langsung ke FCU (MAV_CMD_DO_SET_SERVO via mavros/cmd/command),
//            deteksi missed target -> replan/abort, kembalikan wahana ke AUTO.
//   output : ACTION FEEDBACK (AirdropStatus) + ACTION RESULT (drop_successful)
//            interfaces/AirdropPlan   (telemetri geometri rencana utk GCS)
//            interfaces/AirdropStatus (topik telemetri utk GCS/logging)
//            MAV_CMD_DO_REPOSITION via mavros/cmd/command_int (tujuan GUIDED)
//            MAV_CMD_DO_SET_SERVO via mavros/cmd/command (lepas payload)
//            mavros/set_mode (kembali ke AUTO saat misi drop selesai)
//
// Servo bay ditembak LANGSUNG ke FCU (tanpa node relay terpisah) agar cukup
// airdrop_planning + target_recognition yang jalan di Raspi. Peta bay_index ->
// channel servo + PWM ada di parameter drop.servo_channel_bay / drop.pwm_release.
// Channel bay HARUS bebas di ArduPilot (SERVOx_FUNCTION tidak dipakai FC) —
// lihat commit "pindahkan servo bay ke channel 12/13 yang bebas".
//
// PENTING — kenapa DO_REPOSITION dan bukan setpoint_position/global:
// ArduPlane membuang lat/lon pada SET_POSITION_TARGET_GLOBAL_INT (hanya altitude
// yang berlaku), sehingga wahana mengorbit tempat GUIDED diaktifkan dan capture
// entry point tak pernah terjadi. MAV_CMD_DO_REPOSITION (COMMAND_INT) adalah
// satu-satunya jalur yang benar-benar memindahkan tujuan GUIDED fixed-wing.
// (Lihat CLAUDE.md, diagnosa 2026-07-23 dari logs/00000020.BIN.)

#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "mavros_msgs/srv/command_int.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "mavros_msgs/srv/set_mode.hpp"

#include "interfaces/msg/target_estimate.hpp"
#include "interfaces/msg/airdrop_plan.hpp"
#include "interfaces/msg/airdrop_status.hpp"
#include "interfaces/action/target_airdrop.hpp"

#include "airdrop_planning/ballistic_model.hpp"
#include "airdrop_planning/release_point_solver.hpp"
#include "airdrop_planning/geo_utils.hpp"

using namespace std::chrono_literals;

namespace airdrop_planning
{

class AirdropPlannerNode : public rclcpp::Node
{
public:
  using TargetAirdrop = interfaces::action::TargetAirdrop;
  using GoalHandle = rclcpp_action::ServerGoalHandle<TargetAirdrop>;
  using State = interfaces::msg::AirdropStatus;

  AirdropPlannerNode()
  : Node("airdrop_planner"),
    drop_sent_time_(now()),
    repos_last_attempt_(now())
  {
    declareParams();
    loadParams();

    solver_ = std::make_unique<ReleasePointSolver>(ballistic_params_, approach_params_);

    // --- I/O ---
    auto sensor_qos = rclcpp::SensorDataQoS();

    sub_global_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      get_parameter("topics.global_position").as_string(), sensor_qos,
      std::bind(&AirdropPlannerNode::onGlobalPos, this, std::placeholders::_1));

    sub_rel_alt_ = create_subscription<std_msgs::msg::Float64>(
      get_parameter("topics.rel_alt").as_string(), sensor_qos,
      [this](const std_msgs::msg::Float64::SharedPtr m) {
        rel_alt_m_ = m->data;
        have_rel_alt_ = true;
      });

    sub_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      get_parameter("topics.velocity").as_string(), sensor_qos,
      std::bind(&AirdropPlannerNode::onVelocity, this, std::placeholders::_1));

    sub_wind_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      get_parameter("topics.wind").as_string(), sensor_qos,
      std::bind(&AirdropPlannerNode::onWind, this, std::placeholders::_1));

    pub_plan_ = create_publisher<interfaces::msg::AirdropPlan>("airdrop/plan", 10);
    pub_status_ = create_publisher<interfaces::msg::AirdropStatus>("airdrop/status", 10);

    cli_command_ = create_client<mavros_msgs::srv::CommandLong>(
      get_parameter("topics.command_long").as_string());
    cli_reposition_ = create_client<mavros_msgs::srv::CommandInt>(
      get_parameter("topics.command_int").as_string());
    cli_set_mode_ = create_client<mavros_msgs::srv::SetMode>(
      get_parameter("topics.set_mode").as_string());

    action_server_ = rclcpp_action::create_server<TargetAirdrop>(
      this, "execute_airdrop",
      std::bind(&AirdropPlannerNode::handleGoal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&AirdropPlannerNode::handleCancel, this, std::placeholders::_1),
      std::bind(&AirdropPlannerNode::handleAccepted, this, std::placeholders::_1));

    timer_ = create_wall_timer(10ms, std::bind(&AirdropPlannerNode::update, this));

    RCLCPP_INFO(get_logger(),
      "airdrop_planner siap. Action server 'execute_airdrop' menunggu goal...");
  }

private:
  // ------------------------- parameter -------------------------
  void declareParams()
  {
    // balistik — identifikasi C_D lewat drop test
    declare_parameter("ballistic.mass_kg", 0.5);
    declare_parameter("ballistic.drag_coeff", 0.8);
    declare_parameter("ballistic.ref_area_m2", 0.01);
    declare_parameter("ballistic.air_density", 1.225);
    declare_parameter("ballistic.dt", 0.005);

    // approach
    declare_parameter("approach.release_alt_agl", 100.0);
    declare_parameter("approach.release_airspeed", 18.0);
    declare_parameter("approach.distance", 300.0);
    declare_parameter("approach.loiter_radius", 80.0);
    declare_parameter("approach.min_wind_for_heading", 1.0);
    declare_parameter("approach.overshoot_distance", 250.0);  // [m] target lewat release

    // trigger release
    declare_parameter("release.proximity_radius", 12.0);     // [m]
    declare_parameter("release.mechanism_latency", 0.15);    // [s] servo + bay
    declare_parameter("release.entry_capture_radius", 25.0); // [m] cross-track maks di p
    declare_parameter("release.heading_tolerance", 0.35);    // [rad] ~20 deg
    declare_parameter("release.speed_reduction", 0.0);       // [m/s] paper: 2.0 bila
                                                             // motor off sblm rilis
    declare_parameter("release.gps_latency", 0.1);           // [s] proyeksi posisi
    declare_parameter("release.drop_ack_timeout", 2.0);      // [s]

    // misi
    declare_parameter("mission.num_payloads", 2);
    declare_parameter("mission.max_replans", 3);
    declare_parameter("mission.num_bays", 2);

    // topik sensor MAVROS (mudah di-remap)
    declare_parameter("topics.global_position",
      std::string("mavros/global_position/global"));
    declare_parameter("topics.rel_alt",
      std::string("mavros/global_position/rel_alt"));
    declare_parameter("topics.velocity",
      std::string("mavros/local_position/velocity_local"));
    declare_parameter("topics.wind", std::string("mavros/wind_estimation"));

    // service perintah FCU (tujuan GUIDED + tembak servo + switch mode)
    declare_parameter("topics.command_int", std::string("mavros/cmd/command_int"));
    declare_parameter("topics.command_long", std::string("mavros/cmd/command"));
    declare_parameter("topics.set_mode", std::string("mavros/set_mode"));
    declare_parameter("mode.auto_name", std::string("AUTO"));

    // mekanisme drop: peta bay_index -> channel servo AUX + PWM posisi "lepas".
    // Channel bay HARUS bebas di ArduPilot (SERVOx_FUNCTION tidak dipakai FC).
    declare_parameter("drop.servo_channel_bay", std::vector<int64_t>{12, 13});
    declare_parameter("drop.pwm_release", 1900);
  }

  void loadParams()
  {
    ballistic_params_.mass_kg = get_parameter("ballistic.mass_kg").as_double();
    ballistic_params_.drag_coeff = get_parameter("ballistic.drag_coeff").as_double();
    ballistic_params_.ref_area_m2 = get_parameter("ballistic.ref_area_m2").as_double();
    ballistic_params_.air_density = get_parameter("ballistic.air_density").as_double();
    ballistic_params_.dt = get_parameter("ballistic.dt").as_double();

    approach_params_.release_alt_agl = get_parameter("approach.release_alt_agl").as_double();
    approach_params_.release_airspeed = get_parameter("approach.release_airspeed").as_double();
    approach_params_.approach_distance = get_parameter("approach.distance").as_double();
    approach_params_.loiter_radius = get_parameter("approach.loiter_radius").as_double();
    approach_params_.min_wind_for_heading =
      get_parameter("approach.min_wind_for_heading").as_double();
    overshoot_distance_ = get_parameter("approach.overshoot_distance").as_double();

    proximity_radius_ = get_parameter("release.proximity_radius").as_double();
    mechanism_latency_ = get_parameter("release.mechanism_latency").as_double();
    capture_gate_.entry_capture_radius =
      get_parameter("release.entry_capture_radius").as_double();
    capture_gate_.heading_tolerance =
      get_parameter("release.heading_tolerance").as_double();
    speed_reduction_ = get_parameter("release.speed_reduction").as_double();
    gps_latency_ = get_parameter("release.gps_latency").as_double();
    drop_ack_timeout_ = get_parameter("release.drop_ack_timeout").as_double();

    payloads_remaining_ =
      static_cast<uint8_t>(get_parameter("mission.num_payloads").as_int());
    max_replans_ = static_cast<int>(get_parameter("mission.max_replans").as_int());
    num_bays_ = static_cast<size_t>(get_parameter("mission.num_bays").as_int());

    auto_mode_name_ = get_parameter("mode.auto_name").as_string();
    servo_channels_ = get_parameter("drop.servo_channel_bay").as_integer_array();
    pwm_release_ = static_cast<int>(get_parameter("drop.pwm_release").as_int());
  }

  // ------------------------- callback sensor -------------------------
  void onGlobalPos(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!frame_.hasDatum()) {
      frame_.setDatum(msg->latitude, msg->longitude);
      RCLCPP_INFO(get_logger(), "Datum lokal diset: %.7f, %.7f",
        msg->latitude, msg->longitude);
    }
    uav_geo_ = GeoPoint{msg->latitude, msg->longitude};
  }

  void onVelocity(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    // mavros local_position/velocity_local : ENU -> NED.
    const auto & v = msg->twist.linear;
    ground_vel_ned_ = {v.y, v.x, -v.z};
    have_vel_ = true;
  }

  void onWind(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    // mavros wind_estimation: ENU -> NED. Vektor = arah KE mana udara bergerak.
    wind_ned_ = {msg->twist.twist.linear.y, msg->twist.twist.linear.x};
    have_wind_ = true;
  }

  // ------------------------- action server -------------------------
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const TargetAirdrop::Goal> goal)
  {
    if (state_ != State::STATE_IDLE) {
      RCLCPP_WARN(get_logger(), "Goal ditolak: misi drop lain sedang berjalan.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (payloads_remaining_ == 0) {
      RCLCPP_WARN(get_logger(), "Goal ditolak: payload habis.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->bay_index >= num_bays_) {
      RCLCPP_WARN(get_logger(), "Goal ditolak: bay_index %u tidak dikenal.",
        goal->bay_index);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!frame_.hasDatum() || !uav_geo_ || !have_vel_) {
      RCLCPP_WARN(get_logger(),
        "Goal ditolak: state wahana belum lengkap (GPS/velocity).");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandle>)
  {
    // Cancel diterima selama payload belum dilepas; dieksekusi di update().
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();

    target_ = goal->gps_estimation;
    active_bay_ = goal->bay_index;
    replan_count_ = 0;
    release_miss_ = -1.0;

    // Bekukan ketinggian tahan (relatif home) untuk seluruh approach GUIDED:
    // wahana sudah terbang ~100 m AGL dari mission AUTO, jadi tahan yang ada
    // ketimbang memerintah perubahan altitude yang bikin transien saat lintas.
    hold_alt_rel_ = have_rel_alt_ ? rel_alt_m_ : approach_params_.release_alt_agl;
    repos_have_sent_ = false;   // paksa perintah GUIDED pertama pada goal ini

    state_ = State::STATE_PLANNING;

    RCLCPP_INFO(get_logger(),
      "Goal diterima: target %u lat=%.7f lon=%.7f (%u obs), bay %u, hold_alt %.1f m%s",
      target_->target_id, target_->latitude, target_->longitude,
      target_->num_observations, active_bay_, hold_alt_rel_,
      have_rel_alt_ ? "" : " (FALLBACK release_alt_agl)");
  }

  // ------------------------- state machine -------------------------
  // Pembungkus timer. Exception apa pun yang lolos dari callback timer akan
  // menghentikan spin() dan mematikan node TANPA jejak di file log (buffer tak
  // sempat di-flush, log 0 byte). Sudah terjadi sekali — clock source mismatch
  // pada rclcpp::Time — dan menghabiskan satu penerbangan penuh. Node hidup dan
  // berisik jauh lebih baik daripada node mati diam-diam. (Lihat CLAUDE.md.)
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
    // Cancel dari client — hanya sebelum payload lepas.
    if (goal_handle_ && goal_handle_->is_canceling() &&
      state_ != State::STATE_RELEASED && state_ != State::STATE_IDLE)
    {
      finish(rclcpp_action::ResultCode::CANCELED, false,
        "Dibatalkan oleh client sebelum rilis.");
      publishStatusAndFeedback();
      return;
    }

    switch (state_) {
      case State::STATE_IDLE:
        break;

      case State::STATE_PLANNING: {
          // Param angin dibekukan saat perencanaan (paper: angin tidak di-update
          // lagi setelah titik p).
          wind_at_plan_ = have_wind_ ? wind_ned_ : std::array<double, 2>{0.0, 0.0};

          const Ned2D target_ned = frame_.toNed(
            GeoPoint{target_->latitude, target_->longitude});
          plan_ = solver_->solve(target_ned, uavNed(), wind_at_plan_);
          publishPlan();
          min_dist_seen_ = 1e9;
          state_ = State::STATE_TRANSIT;
          break;
        }

      case State::STATE_TRANSIT:
      case State::STATE_LOITER: {
          // Guidance membawa wahana ke loiter center s lalu loiter CW.
          // Telemetri: TRANSIT vs LOITER dibedakan dari jarak ke s.
          const double d_s = distTo(plan_->loiter_center);
          state_ = (d_s < 1.5 * approach_params_.loiter_radius) ?
            State::STATE_LOITER : State::STATE_TRANSIT;

          const double course = std::atan2(ground_vel_ned_[1], ground_vel_ned_[0]);
          if (have_vel_ && entryCaptured(*plan_, uavNed(), course, capture_gate_)) {
            const double gs = std::hypot(ground_vel_ned_[0], ground_vel_ned_[1]);
            const double gs_eff = std::max(0.0, gs - speed_reduction_);
            const Ned2D target_ned = frame_.toNed(
              GeoPoint{target_->latitude, target_->longitude});
            plan_ = solver_->solveAlongHeading(
              target_ned, plan_->approach_heading, gs_eff, wind_at_plan_);
            publishPlan();
            min_dist_seen_ = 1e9;
            state_ = State::STATE_FINAL_APPROACH;
            RCLCPP_INFO(get_logger(),
              "Entry point p dilewati (gs=%.1f m/s) -> final approach", gs);
          }
          break;
        }

      case State::STATE_FINAL_APPROACH: {
          const double d = distTo(plan_->release_point);
          min_dist_seen_ = std::min(min_dist_seen_, d);
          const bool moving_away = d > min_dist_seen_ + 0.5;

          // Kompensasi latensi mekanisme: lepaskan lebih awal sejauh
          // v_ground * latency.
          const double gs = std::hypot(ground_vel_ned_[0], ground_vel_ned_[1]);
          const double trigger_radius = std::max(
            proximity_radius_, gs * mechanism_latency_);

          if (d < trigger_radius || (moving_away && min_dist_seen_ < proximity_radius_)) {
            release_miss_ = d;
            releasePayload();
            state_ = State::STATE_RELEASED;
          } else if (moving_away && min_dist_seen_ >= proximity_radius_) {
            RCLCPP_WARN(get_logger(),
              "MISSED TARGET (min dist %.1f m) -> replan", min_dist_seen_);
            state_ = State::STATE_MISSED;
          }
          break;
        }

      case State::STATE_MISSED: {
          ++replan_count_;
          if (replan_count_ > max_replans_) {
            finish(rclcpp_action::ResultCode::ABORTED, false,
              "Missed target melebihi batas replan (" +
              std::to_string(max_replans_) + "x).");
          } else {
            state_ = State::STATE_PLANNING;  // hitung ulang dari posisi sekarang
          }
          break;
        }

      case State::STATE_RELEASED: {
          const double waited = (now() - drop_sent_time_).seconds();
          if (drop_ack_ == DropAck::OK) {
            if (payloads_remaining_ > 0) {--payloads_remaining_;}
            finish(rclcpp_action::ResultCode::SUCCEEDED, true,
              "Payload dilepas, miss " + std::to_string(release_miss_) + " m.");
          } else if (drop_ack_ == DropAck::FAILED) {
            finish(rclcpp_action::ResultCode::ABORTED, false,
              "Drop service melaporkan gagal (success=false).");
          } else if (waited > drop_ack_timeout_) {
            finish(rclcpp_action::ResultCode::ABORTED, false,
              "Timeout menunggu ACK drop service.");
          }
          break;
        }

      default:
        break;
    }

    // Setelah transisi state, giring wahana lewat setpoint GUIDED.
    commandApproachSetpoint();
    publishStatusAndFeedback();
  }

  // ------------------------- guidance (setpoint GUIDED) -------------------------
  // Streaming setpoint approach — prec_drop Sec. 2.3 / Fig. 5. Kita hanya kirim
  // POSISI; L1/TECS ArduPilot yang menerbangkan (kotak "LOS Guidance" pada
  // rancangan sengaja tidak diimplementasi).
  //
  // Referensi PERTAMA adalah PUSAT LOITER s, bukan entry point p ("Its center s
  // will be the first reference sent to the guidance and control subsystem...
  // upon approaching it, loiter around it in a clockwise direction"). p adalah
  // titik SINGGUNG pada orbit itu: di sana course sejajar approach_heading DAN
  // cross-track nol, sehingga entryCaptured() bisa terpenuhi. Menaruh setpoint
  // di p justru membuat ArduPlane mengorbit p — capture tak pernah terjadi.
  //
  // Setelah masuk STATE_FINAL_APPROACH, tujuan pindah ke titik OVERSHOOT di luar
  // release point sepanjang approach_heading — bukan release point itu sendiri.
  // Tujuan GUIDED tepat di release point membuat ArduPlane membelok masuk orbit
  // SEBELUM melewatinya (cross-track velocity saat rilis tidak nol; prec_drop
  // Sec. 2.4 menyebut komponen itu mendominasi drop error). Pemicu drop tetap
  // milik proximity circle, bukan tercapainya setpoint.
  void commandApproachSetpoint()
  {
    if (!plan_ || !uav_geo_) {return;}
    if (state_ != State::STATE_TRANSIT && state_ != State::STATE_LOITER &&
      state_ != State::STATE_FINAL_APPROACH)
    {
      return;
    }

    GeoPoint tgt;
    if (state_ == State::STATE_FINAL_APPROACH) {
      Ned2D os;
      os.north = plan_->release_point.north +
        overshoot_distance_ * std::cos(plan_->approach_heading);
      os.east = plan_->release_point.east +
        overshoot_distance_ * std::sin(plan_->approach_heading);
      tgt = frame_.toGeo(os);
    } else {
      tgt = frame_.toGeo(plan_->loiter_center);
    }
    commandTarget(tgt.lat_deg, tgt.lon_deg, hold_alt_rel_,
      approach_params_.loiter_radius);
  }

  // Kirim ulang hanya bila tujuan berubah (replan / pindah ke overshoot) atau
  // bila FCU belum mengiyakan yang terakhir. Menembakkannya tiap tick tanpa
  // dedup akan me-reset guided WP tiap 10 ms.
  void commandTarget(double lat, double lon, double alt_rel, double radius)
  {
    constexpr double kSameTargetDeg = 1e-6;   // ~0,11 m
    const bool moved = !repos_have_sent_ ||
      std::abs(lat - repos_sent_lat_) > kSameTargetDeg ||
      std::abs(lon - repos_sent_lon_) > kSameTargetDeg;
    const bool retry_due = repos_have_sent_ && !repos_ack_ok_ &&
      (now() - repos_last_attempt_).seconds() > 1.0;
    if (moved || retry_due) {
      sendReposition(lat, lon, alt_rel, radius);
    }
  }

  // MAV_CMD_DO_REPOSITION (COMMAND_INT). param2 = CHANGE_MODE: ArduPlane masuk
  // GUIDED sendiri (tak ada balapan dengan set_mode). param3 = radius loiter dan
  // param4 = 0 -> CW; radius dikirim eksplisit sehingga geometri titik singgung
  // p konsisten by construction, tidak bergantung WP_LOITER_RAD FCU.
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
    req->frame = 6;      // MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
    req->command = 192;  // MAV_CMD_DO_REPOSITION
    req->current = 0;
    req->autocontinue = 0;
    req->param1 = 0.0f;   // ground speed: 0 = default kendaraan
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
            "DO_REPOSITION DITOLAK FCU (%.7f, %.7f) — akan di-retry. Penyebab "
            "lazim: bukan mode GUIDED, atau tujuan di luar geofence.", lat, lon);
        }
      });
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

  // ------------------------- helper -------------------------
  // Posisi wahana NED, diproyeksikan gps_latency detik ke depan
  // (kompensasi delay solusi GPS, paper Sec. 5.4).
  Ned2D uavNed() const
  {
    Ned2D u = frame_.toNed(*uav_geo_);
    u.north += ground_vel_ned_[0] * gps_latency_;
    u.east += ground_vel_ned_[1] * gps_latency_;
    return u;
  }

  double distTo(const Ned2D & pt) const
  {
    if (!uav_geo_) {return 1e9;}
    const Ned2D u = uavNed();
    return std::hypot(pt.north - u.north, pt.east - u.east);
  }

  // Lepas payload: tembak servo bay LANGSUNG ke FCU via MAV_CMD_DO_SET_SERVO
  // (CommandLong). bay_index -> channel dari drop.servo_channel_bay, PWM =
  // drop.pwm_release. Tanpa node relay; pemetaan hardware ada di parameter.
  void releasePayload()
  {
    drop_ack_ = DropAck::PENDING;
    drop_sent_time_ = now();

    if (active_bay_ >= servo_channels_.size()) {
      RCLCPP_ERROR(get_logger(),
        "bay_index %u di luar drop.servo_channel_bay (ukuran %zu)!",
        active_bay_, servo_channels_.size());
      drop_ack_ = DropAck::FAILED;
      return;
    }
    if (!cli_command_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Service %s tidak tersedia — servo tak ditembak!",
        cli_command_->get_service_name());
      drop_ack_ = DropAck::FAILED;
      return;
    }

    const int channel = static_cast<int>(servo_channels_[active_bay_]);
    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->broadcast = false;
    req->command = 183;  // MAV_CMD_DO_SET_SERVO
    req->confirmation = 0;
    req->param1 = static_cast<float>(channel);       // nomor servo output
    req->param2 = static_cast<float>(pwm_release_);  // PWM posisi "lepas"

    cli_command_->async_send_request(req,
      [this, channel](rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedFuture fut) {
        const auto res = fut.get();
        drop_ack_ = res->success ? DropAck::OK : DropAck::FAILED;
        RCLCPP_INFO(get_logger(), "DO_SET_SERVO ch=%d ack: success=%d result=%u",
          channel, res->success, res->result);
      });

    RCLCPP_INFO(get_logger(), "RELEASE! bay=%u -> servo ch=%d pwm=%d (miss %.1f m)",
      active_bay_, channel, pwm_release_, release_miss_);
  }

  void finish(rclcpp_action::ResultCode code, bool success, const std::string & msg)
  {
    if (goal_handle_) {
      auto result = std::make_shared<TargetAirdrop::Result>();
      result->drop_successful = success;
      switch (code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          goal_handle_->succeed(result); break;
        case rclcpp_action::ResultCode::CANCELED:
          goal_handle_->canceled(result); break;
        default:
          goal_handle_->abort(result); break;
      }
      goal_handle_.reset();
    }
    // Kembalikan wahana ke mission AUTO — jangan tinggalkan ia yatim mengorbit
    // di GUIDED, apa pun hasil (sukses / abort / cancel).
    setMode(auto_mode_name_);
    repos_have_sent_ = false;

    RCLCPP_INFO(get_logger(), "Misi drop selesai: %s (sisa payload=%u)",
      msg.c_str(), payloads_remaining_);
    state_ = State::STATE_IDLE;
    plan_.reset();
    target_.reset();
    drop_ack_ = DropAck::NONE;
  }

  void publishPlan()
  {
    if (!plan_) {return;}
    interfaces::msg::AirdropPlan m;
    m.header.stamp = now();
    m.header.frame_id = "local_ned";
    m.target_id = target_ ? target_->target_id : 0;
    m.release_point_ned.x = plan_->release_point.north;
    m.release_point_ned.y = plan_->release_point.east;
    m.release_point_ned.z = -approach_params_.release_alt_agl;
    m.entry_point_ned.x = plan_->entry_point.north;
    m.entry_point_ned.y = plan_->entry_point.east;
    m.entry_point_ned.z = -approach_params_.release_alt_agl;
    m.loiter_center_ned.x = plan_->loiter_center.north;
    m.loiter_center_ned.y = plan_->loiter_center.east;
    m.loiter_center_ned.z = -approach_params_.release_alt_agl;
    m.loiter_radius = approach_params_.loiter_radius;
    m.approach_heading = plan_->approach_heading;
    m.release_altitude_agl = approach_params_.release_alt_agl;
    m.fall_time = plan_->fall_time;
    m.impact_offset_north = plan_->offset_north;
    m.impact_offset_east = plan_->offset_east;
    const GeoPoint rp_geo = frame_.toGeo(plan_->release_point);
    const GeoPoint ep_geo = frame_.toGeo(plan_->entry_point);
    const GeoPoint lc_geo = frame_.toGeo(plan_->loiter_center);
    m.release_point_lat = rp_geo.lat_deg;
    m.release_point_lon = rp_geo.lon_deg;
    m.entry_point_lat = ep_geo.lat_deg;
    m.entry_point_lon = ep_geo.lon_deg;
    m.loiter_center_lat = lc_geo.lat_deg;
    m.loiter_center_lon = lc_geo.lon_deg;
    pub_plan_->publish(m);
  }

  void publishStatusAndFeedback()
  {
    interfaces::msg::AirdropStatus s;
    s.header.stamp = now();
    s.state = state_;
    s.target_id = target_ ? target_->target_id : 0;
    s.payloads_remaining = payloads_remaining_;
    s.distance_to_release = plan_ ? distTo(plan_->release_point) : -1.0;
    s.wind_speed = std::hypot(wind_ned_[0], wind_ned_[1]);
    s.wind_direction = wrapPi(std::atan2(-wind_ned_[1], -wind_ned_[0]));

    pub_status_->publish(s);

    if (goal_handle_) {
      auto fb = std::make_shared<TargetAirdrop::Feedback>();
      fb->current_status = s;
      goal_handle_->publish_feedback(fb);
    }
  }

  // ------------------------- anggota -------------------------
  enum class DropAck {NONE, PENDING, OK, FAILED};

  BallisticParams ballistic_params_;
  ApproachParams approach_params_;
  std::unique_ptr<ReleasePointSolver> solver_;

  LocalFrame frame_;
  std::optional<GeoPoint> uav_geo_;
  std::array<double, 3> ground_vel_ned_{0, 0, 0};
  std::array<double, 2> wind_ned_{0, 0};
  std::array<double, 2> wind_at_plan_{0, 0};
  bool have_vel_{false};
  bool have_wind_{false};
  double rel_alt_m_{0.0};
  bool have_rel_alt_{false};
  double hold_alt_rel_{100.0};

  std::optional<interfaces::msg::TargetEstimate> target_;
  std::optional<ApproachPlanNed> plan_;
  std::shared_ptr<GoalHandle> goal_handle_;

  uint8_t state_{State::STATE_IDLE};
  uint8_t active_bay_{0};
  uint8_t payloads_remaining_{2};
  int replan_count_{0};
  int max_replans_{3};
  double min_dist_seen_{1e9};
  double release_miss_{-1.0};

  DropAck drop_ack_{DropAck::NONE};
  rclcpp::Time drop_sent_time_;

  double proximity_radius_{12.0};
  double mechanism_latency_{0.15};
  CaptureGate capture_gate_{};
  double speed_reduction_{0.0};
  double gps_latency_{0.1};
  double drop_ack_timeout_{2.0};
  size_t num_bays_{2};
  double overshoot_distance_{250.0};
  std::string auto_mode_name_{"AUTO"};

  // Mekanisme drop (servo bay langsung ke FCU).
  std::vector<int64_t> servo_channels_{12, 13};
  int pwm_release_{1900};

  // Pelacakan DO_REPOSITION (dedup + retry).
  double repos_sent_lat_{0.0};
  double repos_sent_lon_{0.0};
  bool repos_have_sent_{false};
  bool repos_ack_ok_{false};
  rclcpp::Time repos_last_attempt_;

  rclcpp_action::Server<TargetAirdrop>::SharedPtr action_server_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_global_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_rel_alt_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_wind_;
  rclcpp::Publisher<interfaces::msg::AirdropPlan>::SharedPtr pub_plan_;
  rclcpp::Publisher<interfaces::msg::AirdropStatus>::SharedPtr pub_status_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cli_command_;
  rclcpp::Client<mavros_msgs::srv::CommandInt>::SharedPtr cli_reposition_;
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr cli_set_mode_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace airdrop_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<airdrop_planning::AirdropPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
