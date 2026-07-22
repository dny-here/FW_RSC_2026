// airdrop_planner_node.cpp
// Node "Airdrop Planning" — KRTI 2026 Divisi Fixed-Wing.
//
// ref prec_drop Sec. 2.3:
//   input  : ACTION GOAL interfaces/action/TargetAirdrop dari node
//            target_recognition (berisi TargetEstimate + bay_index)
//            posisi & kecepatan wahana + estimasi angin (dari MAVROS/FCU)
//   proses : hitung release point (Algorithm 1, model balistik eq. 3-5)
//            bangun truncated Dubins approach (loiter s -> entry p -> release)
//            replan release point saat mencapai titik p (ground speed aktual,
//            heading DIKUNCI), trigger pelepasan payload via service
//            interfaces/DropPayload, deteksi missed target -> replan/abort
//   output : ACTION FEEDBACK (AirdropStatus) + ACTION RESULT (drop_successful)
//            interfaces/AirdropPlan   (dikonsumsi guidance / mission bridge)
//            interfaces/AirdropStatus (topik telemetri utk GCS/logging)
//

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
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"

#include "interfaces/msg/target_estimate.hpp"
#include "interfaces/msg/airdrop_plan.hpp"
#include "interfaces/msg/airdrop_status.hpp"
#include "interfaces/action/target_airdrop.hpp"
#include "interfaces/srv/drop_payload.hpp"

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
  : Node("airdrop_planner")
  {
    declareParams();
    loadParams();

    solver_ = std::make_unique<ReleasePointSolver>(ballistic_params_, approach_params_);

    // --- I/O ---
    auto sensor_qos = rclcpp::SensorDataQoS();

    sub_global_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      get_parameter("topics.global_position").as_string(), sensor_qos,
      std::bind(&AirdropPlannerNode::onGlobalPos, this, std::placeholders::_1));

    sub_vel_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      get_parameter("topics.velocity").as_string(), sensor_qos,
      std::bind(&AirdropPlannerNode::onVelocity, this, std::placeholders::_1));

    sub_wind_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      get_parameter("topics.wind").as_string(), sensor_qos,
      std::bind(&AirdropPlannerNode::onWind, this, std::placeholders::_1));

    pub_plan_ = create_publisher<interfaces::msg::AirdropPlan>("airdrop/plan", 10);
    pub_status_ = create_publisher<interfaces::msg::AirdropStatus>("airdrop/status", 10);

    cli_drop_ = create_client<interfaces::srv::DropPayload>(
      get_parameter("topics.drop_service").as_string());

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
    declare_parameter("approach.release_alt_agl", 50.0);
    declare_parameter("approach.release_airspeed", 18.0);
    declare_parameter("approach.distance", 300.0);
    declare_parameter("approach.loiter_radius", 80.0);
    declare_parameter("approach.min_wind_for_heading", 1.0);

    // trigger release
    declare_parameter("release.proximity_radius", 15.0);     // [m]
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
    declare_parameter("topics.velocity",
      std::string("mavros/local_position/velocity_local"));
    declare_parameter("topics.wind", std::string("mavros/wind_estimation"));

    // service pemicu drop (diimplementasi node mekanisme drop / relay)
    declare_parameter("topics.drop_service", std::string("drop"));
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

    proximity_radius_ = get_parameter("release.proximity_radius").as_double();
    mechanism_latency_ = get_parameter("release.mechanism_latency").as_double();
    entry_capture_radius_ = get_parameter("release.entry_capture_radius").as_double();
    heading_tolerance_ = get_parameter("release.heading_tolerance").as_double();
    speed_reduction_ = get_parameter("release.speed_reduction").as_double();
    gps_latency_ = get_parameter("release.gps_latency").as_double();
    drop_ack_timeout_ = get_parameter("release.drop_ack_timeout").as_double();

    payloads_remaining_ =
      static_cast<uint8_t>(get_parameter("mission.num_payloads").as_int());
    max_replans_ = static_cast<int>(get_parameter("mission.max_replans").as_int());

    num_bays_ = static_cast<size_t>(get_parameter("mission.num_bays").as_int());
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
    state_ = State::STATE_PLANNING;

    RCLCPP_INFO(get_logger(),
      "Goal diterima: target %u lat=%.7f lon=%.7f (%u obs), bay %u",
      target_->target_id, target_->latitude, target_->longitude,
      target_->num_observations, active_bay_);
  }

  // ------------------------- state machine -------------------------
  void update()
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
          // Param angin froze
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

          const Ned2D u = uavNed();
          const double qn = std::cos(plan_->approach_heading);
          const double qe = std::sin(plan_->approach_heading);
          const double rel_n = u.north - plan_->entry_point.north;
          const double rel_e = u.east - plan_->entry_point.east;
          const double along = rel_n * qn + rel_e * qe;         // (p_u - p)·q
          const double cross = std::abs(rel_n * qe - rel_e * qn);

          if (along >= 0.0 && cross < entry_capture_radius_ && headingAligned()) {
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
              "FCU menolak perintah DO_SET_SERVO.");
          } else if (waited > drop_ack_timeout_) {
            finish(rclcpp_action::ResultCode::ABORTED, false,
              "Timeout menunggu ACK drop service.");
          }
          break;
        }

      default:
        break;
    }

    publishStatusAndFeedback();
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

  bool headingAligned() const
  {
    if (!plan_ || !have_vel_) {return false;}
    const double course = std::atan2(ground_vel_ned_[1], ground_vel_ned_[0]);
    return std::abs(wrapPi(course - plan_->approach_heading)) < heading_tolerance_;
  }

  void releasePayload()
  {
    drop_ack_ = DropAck::PENDING;
    drop_sent_time_ = now();

    if (!cli_drop_->service_is_ready()) {
      RCLCPP_ERROR(get_logger(), "Drop service %s tidak tersedia!",
        cli_drop_->get_service_name());
      drop_ack_ = DropAck::FAILED;
      return;
    }

    auto req = std::make_shared<interfaces::srv::DropPayload::Request>();
    req->bay_index = active_bay_;

    cli_drop_->async_send_request(req,
      [this](rclcpp::Client<interfaces::srv::DropPayload>::SharedFuture fut) {
        const auto res = fut.get();
        drop_ack_ = res->success ? DropAck::OK : DropAck::FAILED;
        RCLCPP_INFO(get_logger(), "Drop ack: success=%d msg='%s'",
          res->success, res->message.c_str());
      });

    RCLCPP_INFO(get_logger(), "RELEASE! bay=%u (miss %.1f m)",
      active_bay_, release_miss_);
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

  double proximity_radius_{15.0};
  double mechanism_latency_{0.15};
  double entry_capture_radius_{25.0};
  double heading_tolerance_{0.35};
  double speed_reduction_{0.0};
  double gps_latency_{0.1};
  double drop_ack_timeout_{2.0};
  size_t num_bays_{2};

  rclcpp_action::Server<TargetAirdrop>::SharedPtr action_server_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_global_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_vel_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_wind_;
  rclcpp::Publisher<interfaces::msg::AirdropPlan>::SharedPtr pub_plan_;
  rclcpp::Publisher<interfaces::msg::AirdropStatus>::SharedPtr pub_status_;
  rclcpp::Client<interfaces::srv::DropPayload>::SharedPtr cli_drop_;
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
