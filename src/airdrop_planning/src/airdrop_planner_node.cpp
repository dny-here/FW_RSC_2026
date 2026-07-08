// airdrop_planner_node.cpp
// Node "Airdrop Planning" — KRTI 2026 Divisi Fixed-Wing.
//
// Peran (sesuai arsitektur tim & prec_drop Sec. 2.3):
//   input  : interfaces/TargetEstimate  (dari node target_recognition)
//            posisi & kecepatan wahana + estimasi angin (dari MAVROS/FCU)
//   proses : hitung release point (Algorithm 1, model balistik eq. 3-5)
//            bangun truncated Dubins approach (loiter s -> entry p -> release)
//            replan release point saat mencapai titik p (pakai ground velocity
//            aktual), trigger pelepasan payload, deteksi missed target
//   output : interfaces/AirdropPlan   (dikonsumsi guidance / mission bridge)
//            interfaces/AirdropStatus (telemetri state machine)
//            std_msgs/UInt8 pada topik drop command (bay index) untuk node
//            mekanisme dropping
//
// Node ini SENGAJA tidak mengirim setpoint roll/pitch langsung —
// guidance & control tetap di ArduPilot/node guidance, sesuai pemisahan
// subsistem pada paper ("modular structure ... as long as the interface
// is maintained").

#include <array>
#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "std_msgs/msg/u_int8.hpp"

#include "interfaces/msg/target_estimate.hpp"
#include "interfaces/msg/airdrop_plan.hpp"
#include "interfaces/msg/airdrop_status.hpp"
#include "interfaces/srv/start_airdrop.hpp"

#include "airdrop_planning/ballistic_model.hpp"
#include "airdrop_planning/release_point_solver.hpp"
#include "airdrop_planning/geo_utils.hpp"

using namespace std::chrono_literals;

namespace airdrop_planning
{

class AirdropPlannerNode : public rclcpp::Node
{
public:
  AirdropPlannerNode()
  : Node("airdrop_planner")
  {
    declareParams();
    loadParams();

    solver_ = std::make_unique<ReleasePointSolver>(ballistic_params_, approach_params_);

    // --- I/O ---
    auto sensor_qos = rclcpp::SensorDataQoS();

    sub_target_ = create_subscription<interfaces::msg::TargetEstimate>(
      "target_recognition/target_estimate", 10,
      std::bind(&AirdropPlannerNode::onTarget, this, std::placeholders::_1));

    sub_global_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      get_parameter("topics.global_position").as_string(), sensor_qos,
      std::bind(&AirdropPlannerNode::onGlobalPos, this, std::placeholders::_1));

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("topics.odom").as_string(), sensor_qos,
      std::bind(&AirdropPlannerNode::onOdom, this, std::placeholders::_1));

    sub_wind_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      get_parameter("topics.wind").as_string(), sensor_qos,
      std::bind(&AirdropPlannerNode::onWind, this, std::placeholders::_1));

    pub_plan_ = create_publisher<interfaces::msg::AirdropPlan>("airdrop/plan", 10);
    pub_status_ = create_publisher<interfaces::msg::AirdropStatus>("airdrop/status", 10);
    pub_drop_ = create_publisher<std_msgs::msg::UInt8>("airdrop/drop_command", 10);

    srv_start_ = create_service<interfaces::srv::StartAirdrop>(
      "airdrop/start",
      std::bind(&AirdropPlannerNode::onStartRequest, this,
        std::placeholders::_1, std::placeholders::_2));

    timer_ = create_wall_timer(50ms, std::bind(&AirdropPlannerNode::update, this));

    RCLCPP_INFO(get_logger(), "airdrop_planner siap. Menunggu target estimate...");
  }

private:
  using State = interfaces::msg::AirdropStatus;

  // ------------------------- parameter -------------------------
  void declareParams()
  {
    // balistik — identifikasi C_D lewat drop test!
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
    declare_parameter("release.proximity_radius", 15.0);   // [m]
    declare_parameter("release.mechanism_latency", 0.15);  // [s] delay servo+jatuh bebas awal
    declare_parameter("release.entry_capture_radius", 25.0); // [m] dianggap sampai titik p

    // misi
    declare_parameter("mission.num_payloads", 2);

    // topik MAVROS (mudah di-remap)
    declare_parameter("topics.global_position", std::string("mavros/global_position/global"));
    declare_parameter("topics.odom", std::string("mavros/local_position/odom"));
    declare_parameter("topics.wind", std::string("mavros/wind_estimation"));
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
    payloads_remaining_ = static_cast<uint8_t>(get_parameter("mission.num_payloads").as_int());
  }

  // ------------------------- callback input -------------------------
  void onTarget(const interfaces::msg::TargetEstimate::SharedPtr msg)
  {
    last_target_ = *msg;
    RCLCPP_INFO(get_logger(),
      "Target %u diterima: lat=%.7f lon=%.7f (%u observasi)",
      msg->target_id, msg->latitude, msg->longitude, msg->num_observations);
  }

  void onGlobalPos(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!frame_.hasDatum()) {
      frame_.setDatum(msg->latitude, msg->longitude);
      RCLCPP_INFO(get_logger(), "Datum lokal diset: %.7f, %.7f",
        msg->latitude, msg->longitude);
    }
    uav_geo_ = GeoPoint{msg->latitude, msg->longitude};
  }

  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // mavros local_position/odom : frame ENU -> konversi ke NED.
    const auto & v = msg->twist.twist.linear;
    ground_vel_ned_ = {v.y, v.x, -v.z};
    have_odom_ = true;
  }

  void onWind(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    // mavros wind_estimation: ENU -> NED. Vektor = arah KE mana udara bergerak.
    wind_ned_ = {msg->twist.twist.linear.y, msg->twist.twist.linear.x};
    have_wind_ = true;
  }

  void onStartRequest(
    const interfaces::srv::StartAirdrop::Request::SharedPtr req,
    interfaces::srv::StartAirdrop::Response::SharedPtr res)
  {
    if (!last_target_ || last_target_->target_id != req->target_id) {
      res->accepted = false;
      res->message = "Belum ada estimasi target untuk target_id tsb.";
      return;
    }
    if (payloads_remaining_ == 0) {
      res->accepted = false;
      res->message = "Payload habis.";
      return;
    }
    if (!frame_.hasDatum() || !uav_geo_ || !have_odom_) {
      res->accepted = false;
      res->message = "State wahana belum lengkap (GPS/odom).";
      return;
    }
    active_bay_ = req->bay_index;
    state_ = State::STATE_PLANNING;
    res->accepted = true;
    res->message = "Sekuens airdrop dimulai.";
    RCLCPP_INFO(get_logger(), "Airdrop dimulai utk target %u, bay %u",
      req->target_id, req->bay_index);
  }

  // ------------------------- state machine -------------------------
  void update()
  {
    switch (state_) {
      case State::STATE_IDLE:
        break;

      case State::STATE_PLANNING: {
          plan_ = computePlan(/*use_ground_vel=*/false);
          publishPlan();
          min_dist_seen_ = 1e9;
          state_ = State::STATE_TRANSIT;
          break;
        }

      case State::STATE_TRANSIT: {
          // Guidance membawa wahana ke loiter center s lalu loiter CW.
          // Kita hanya memantau kapan wahana tiba di sekitar titik p
          // dengan heading yang kira-kira sudah sejajar approach.
          const double d_entry = distTo(plan_->entry_point);
          if (d_entry < entry_capture_radius_ && headingAligned()) {
            // Sesuai paper: replan release point di titik p dengan
            // GROUND VELOCITY aktual (bukan prediksi airspeed+angin),
            // arah garis dipertahankan.
            plan_ = computePlan(/*use_ground_vel=*/true, plan_->approach_heading);
            publishPlan();
            min_dist_seen_ = 1e9;
            state_ = State::STATE_FINAL_APPROACH;
            RCLCPP_INFO(get_logger(), "Entry point tercapai -> final approach");
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
            releasePayload();
            state_ = State::STATE_RELEASED;
          } else if (moving_away && min_dist_seen_ >= proximity_radius_) {
            // Lewat tanpa pernah masuk proximity -> missed target (Fig. 1).
            RCLCPP_WARN(get_logger(),
              "MISSED TARGET (min dist %.1f m) -> replan", min_dist_seen_);
            state_ = State::STATE_MISSED;
          }
          break;
        }

      case State::STATE_MISSED:
        state_ = State::STATE_PLANNING;   // hitung ulang dari posisi sekarang
        break;

      case State::STATE_RELEASED:
        // Selesai; mission manager memutuskan langkah berikutnya
        // (lanjut mapping / DZ berikutnya). Kembali idle.
        state_ = State::STATE_IDLE;
        break;

      default:
        break;
    }

    publishStatus();
  }

  // ------------------------- helper -------------------------
  std::optional<ApproachPlanNed> computePlan(
    bool use_ground_vel, std::optional<double> fixed_heading = std::nullopt)
  {
    if (!last_target_ || !uav_geo_) {return std::nullopt;}

    const Ned2D target_ned = frame_.toNed(
      GeoPoint{last_target_->latitude, last_target_->longitude});
    const Ned2D uav_ned = frame_.toNed(*uav_geo_);

    auto wind = have_wind_ ? wind_ned_ : std::array<double, 2>{0.0, 0.0};

    // Paper: setelah heading approach ditetapkan, update angin TIDAK
    // dipakai lagi (robust thd fluktuasi angin lemah). Saat replan di p,
    // heading dipertahankan (fixed_heading).
    if (fixed_heading) {
      // paksa arah dengan mengganti "angin" sintetis berlawanan heading
      // hanya bila angin asli terlalu lemah — jika angin kuat, heading
      // hasil solve akan sama karena angin tidak berubah drastis.
      // Cara paling sederhana: solve normal lalu timpa heading & geometri.
    }

    const std::array<double, 3> * gv =
      use_ground_vel && have_odom_ ? &ground_vel_ned_ : nullptr;

    ApproachPlanNed p = solver_->solve(target_ned, uav_ned, wind, gv);
    if (fixed_heading) {
      // Pertahankan arah garis: bangun ulang geometri dgn heading lama.
      const double h = *fixed_heading;
      p.approach_heading = h;
      p.entry_point.north = p.release_point.north -
        approach_params_.approach_distance * std::cos(h);
      p.entry_point.east = p.release_point.east -
        approach_params_.approach_distance * std::sin(h);
      p.loiter_center.north = p.entry_point.north -
        approach_params_.loiter_radius * std::sin(h);
      p.loiter_center.east = p.entry_point.east +
        approach_params_.loiter_radius * std::cos(h);
    }
    return p;
  }

  double distTo(const Ned2D & pt) const
  {
    if (!uav_geo_) {return 1e9;}
    const Ned2D u = frame_.toNed(*uav_geo_);
    return std::hypot(pt.north - u.north, pt.east - u.east);
  }

  bool headingAligned() const
  {
    if (!plan_ || !have_odom_) {return false;}
    const double course = std::atan2(ground_vel_ned_[1], ground_vel_ned_[0]);
    return std::abs(wrapPi(course - plan_->approach_heading)) < 0.35;  // ~20 deg
  }

  void releasePayload()
  {
    std_msgs::msg::UInt8 cmd;
    cmd.data = active_bay_;
    pub_drop_->publish(cmd);
    if (payloads_remaining_ > 0) {--payloads_remaining_;}
    RCLCPP_INFO(get_logger(), "RELEASE! bay=%u, sisa payload=%u",
      active_bay_, payloads_remaining_);
  }

  void publishPlan()
  {
    if (!plan_) {return;}
    interfaces::msg::AirdropPlan m;
    m.header.stamp = now();
    m.header.frame_id = "local_ned";
    m.target_id = last_target_ ? last_target_->target_id : 0;
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

  void publishStatus()
  {
    interfaces::msg::AirdropStatus s;
    s.header.stamp = now();
    s.state = state_;
    s.target_id = last_target_ ? last_target_->target_id : 0;
    s.payloads_remaining = payloads_remaining_;
    s.distance_to_release = plan_ ? distTo(plan_->release_point) : -1.0;
    s.wind_speed = std::hypot(wind_ned_[0], wind_ned_[1]);
    // arah meteorologis: DARI mana angin bertiup
    s.wind_direction = wrapPi(std::atan2(-wind_ned_[1], -wind_ned_[0]));
    pub_status_->publish(s);
  }

  // ------------------------- anggota -------------------------
  BallisticParams ballistic_params_;
  ApproachParams approach_params_;
  std::unique_ptr<ReleasePointSolver> solver_;

  LocalFrame frame_;
  std::optional<GeoPoint> uav_geo_;
  std::array<double, 3> ground_vel_ned_{0, 0, 0};
  std::array<double, 2> wind_ned_{0, 0};
  bool have_odom_{false};
  bool have_wind_{false};

  std::optional<interfaces::msg::TargetEstimate> last_target_;
  std::optional<ApproachPlanNed> plan_;

  uint8_t state_{State::STATE_IDLE};
  uint8_t active_bay_{0};
  uint8_t payloads_remaining_{2};
  double min_dist_seen_{1e9};

  double proximity_radius_{15.0};
  double mechanism_latency_{0.15};
  double entry_capture_radius_{25.0};

  rclcpp::Subscription<interfaces::msg::TargetEstimate>::SharedPtr sub_target_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_global_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_wind_;
  rclcpp::Publisher<interfaces::msg::AirdropPlan>::SharedPtr pub_plan_;
  rclcpp::Publisher<interfaces::msg::AirdropStatus>::SharedPtr pub_status_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr pub_drop_;
  rclcpp::Service<interfaces::srv::StartAirdrop>::SharedPtr srv_start_;
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
