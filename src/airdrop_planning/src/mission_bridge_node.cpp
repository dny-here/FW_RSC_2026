// mission_bridge_node.cpp
// Peran:
//   input  : interfaces/AirdropPlan pada topik "airdrop/plan"
//            (loiter_center s -> entry_point p -> release_point, frame NED lokal)
//            posisi global wahana (MAVROS NavSatFix) untuk menetapkan datum.
//   proses : konversi titik-titik NED plan -> lat/lon (WGS-84) memakai datum
//            yang sama seperti airdrop_planner (fix pertama = home), lalu
//            bangun mission ArduPilot dan push lewat service MAVROS WaypointPush.
//   output : mavros_msgs/srv/WaypointPush -> ArduPilot menerbangkan approach
//            dengan L1/TECS-nya sendiri (guidance TETAP di ArduPilot).
//
// Bridge menyalurkan geometri approach menjadi waypoint, satu arah.

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include "mavros_msgs/msg/waypoint.hpp"
#include "mavros_msgs/srv/waypoint_push.hpp"

#include "interfaces/msg/airdrop_plan.hpp"
#include "interfaces/msg/airdrop_status.hpp" 

#include "airdrop_planning/geo_utils.hpp"

using namespace std::chrono_literals;

namespace airdrop_planning
{

// Konstanta MAVLink
namespace mav
{
constexpr uint8_t FRAME_GLOBAL_RELATIVE_ALT = 3;   // alt relatif terhadap home
constexpr uint16_t CMD_NAV_WAYPOINT = 16;
constexpr uint16_t CMD_NAV_LOITER_TO_ALT = 31;
}  // namespace mav

class MissionBridgeNode : public rclcpp::Node
{
public:
  MissionBridgeNode()
  : Node("mission_bridge")
  {
    declareParams();
    loadParams();

    auto sensor_qos = rclcpp::SensorDataQoS();

    // Datum: fix GPS pertama (samakan dengan airdrop_planner).
    sub_global_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      global_topic_, sensor_qos,
      std::bind(&MissionBridgeNode::onGlobalPos, this, std::placeholders::_1));

    // Plan reliable (bukan sensor QoS) — transisi state, frekuensi rendah.
    sub_plan_ = create_subscription<interfaces::msg::AirdropPlan>(
      plan_topic_, 10,
      std::bind(&MissionBridgeNode::onPlan, this, std::placeholders::_1));

    sub_status_ = create_subscription<interfaces::msg::AirdropStatus>(
      status_topic_, 10,
      std::bind(&MissionBridgeNode::onStatus, this, std::placeholders::_1));

    cli_push_ = create_client<mavros_msgs::srv::WaypointPush>(push_service_);

    RCLCPP_INFO(get_logger(),
      "mission_bridge siap. Menunggu datum (%s) & plan (%s)...",
      global_topic_.c_str(), plan_topic_.c_str());
  }

private:
  // ------------------------- parameter -------------------------
  void declareParams()
  {
    declare_parameter<std::string>("topics.plan", "airdrop/plan");
    declare_parameter<std::string>("topics.global_position",
      "mavros/global_position/global");
    declare_parameter<std::string>("topics.waypoint_push_service",
      "mavros/mission/push");

    declare_parameter<double>("mission.loiter_radius", 80.0);
    declare_parameter<double>("mission.overshoot_distance", 150.0);
    declare_parameter<double>("mission.acceptance_radius", 0.0);
    declare_parameter<std::string>("topics.status", "airdrop/status");
    // declare_parameter<double>("mission.min_replan_move", 5.0);
  }

  void loadParams()
  {
    plan_topic_ = get_parameter("topics.plan").as_string();
    global_topic_ = get_parameter("topics.global_position").as_string();
    push_service_ = get_parameter("topics.waypoint_push_service").as_string();

    loiter_radius_ = get_parameter("mission.loiter_radius").as_double();
    overshoot_distance_ = get_parameter("mission.overshoot_distance").as_double();
    acceptance_radius_ = get_parameter("mission.acceptance_radius").as_double();
    status_topic_ = get_parameter("topics.status").as_string();
    // min_replan_move_ = get_parameter("mission.min_replan_move").as_double();
  }

  // ------------------------- callback -------------------------
  void onGlobalPos(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    if (!frame_.hasDatum()) {
      frame_.setDatum(msg->latitude, msg->longitude);
      home_alt_ = msg->altitude;
      RCLCPP_INFO(get_logger(),
        "Datum diset dari fix pertama: lat=%.7f lon=%.7f",
        msg->latitude, msg->longitude);
    }
  }

  void onStatus(const interfaces::msg::AirdropStatus::SharedPtr msg)
    {
      using S = interfaces::msg::AirdropStatus;
      // Re-arm untuk push berikutnya HANYA di awal misi (IDLE) atau saat
      // missed-target — di kedua kasus wahana memang harus (di)putar & di-push
      // ulang. Transisi ke FINAL_APPROACH sengaja TIDAK me-re-arm: replan di
      // titik p tak boleh mengganti mission saat wahana di garis lurus kritis.
      if (msg->state == S::STATE_IDLE || msg->state == S::STATE_MISSED) {
        armed_ = true;
      }
      last_state_ = msg->state;
    }

    void onPlan(const interfaces::msg::AirdropPlan::SharedPtr msg)
    {
      if (!frame_.hasDatum()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Plan diterima tapi datum belum ada (belum ada fix GPS) — di-skip.");
        return;
      }

      // Gerbang fase: push hanya rencana PERTAMA tiap percobaan (awal / setelah
      // missed). Rencana KEDUA tiap goal adalah replan di titik p — di-skip agar
      // mission tak diganti saat wahana sudah di segmen lurus final.
      if (!armed_) {
        RCLCPP_DEBUG(get_logger(),
          "Plan diterima tapi bridge tak armed (kemungkinan replan titik p) "
          "— tidak push.");
        return;
      }

      pushMission(*msg);
    }

  // void onPlan(const interfaces::msg::AirdropPlan::SharedPtr msg)
  // {
  //   if (!frame_.hasDatum()) {
  //     RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
  //       "Plan diterima tapi datum belum ada (belum ada fix GPS) — di-skip.");
  //     return;
  //   }

  //   // Debounce: jangan push ulang bila release point praktis tidak bergerak
  //   // (plan diterbitkan tiap replan; hindari spam WaypointPush).
  //   if (have_last_release_) {
  //     const double d = std::hypot(
  //       msg->release_point_ned.x - last_release_ned_.north,
  //       msg->release_point_ned.y - last_release_ned_.east);
  //     if (d < min_replan_move_) {
  //       RCLCPP_DEBUG(get_logger(),
  //         "Plan baru ~identik (%.1f m) — tidak push ulang.", d);
  //       return;
  //     }
  //   }

  //   pushMission(*msg);
  // }

  // ------------------------- inti -------------------------
  // Susun mission ArduPilot: home -> LOITER_TO_ALT(s) -> WAYPOINT(p) ->
  // WAYPOINT(release) -> WAYPOINT(overshoot). Overshoot menjaga garis lurus
  // melewati release point agar L1 stabil saat drop.
  void pushMission(const interfaces::msg::AirdropPlan & plan)
  {
    if (!cli_push_->service_is_ready()) {
      RCLCPP_WARN(get_logger(),
        "Service WaypointPush (%s) belum siap — plan tidak dikirim.",
        push_service_.c_str());
      return;
    }
    armed_ = false;

    const double alt = plan.release_altitude_agl;  // AGL relatif home

    auto req = std::make_shared<mavros_msgs::srv::WaypointPush::Request>();
    req->start_index = 0;

    // seq 0: home placeholder (ArduPilot menimpanya dengan home sebenarnya).
    req->waypoints.push_back(makeWp(
      mav::CMD_NAV_WAYPOINT, frame_.datum().lat_deg, frame_.datum().lon_deg,
      alt, /*is_current=*/true));

    // s: loiter untuk menyelaraskan approach & mencapai alt rilis.
    {
      Ned2D s{plan.loiter_center_ned.x, plan.loiter_center_ned.y};
      const GeoPoint g = frame_.toGeo(s);
      const double radius =
        (plan.loiter_radius > 0.0) ? plan.loiter_radius : loiter_radius_;
      auto wp = makeWp(mav::CMD_NAV_LOITER_TO_ALT, g.lat_deg, g.lon_deg, alt);
      wp.param1 = 0.0f;                        // heading required: tidak
      wp.param2 = static_cast<float>(radius);  // radius loiter [m]
      req->waypoints.push_back(wp);
    }

    // p: titik masuk segmen lurus final.
    pushNed(req, {plan.entry_point_ned.x, plan.entry_point_ned.y}, alt);

    // release point.
    pushNed(req, {plan.release_point_ned.x, plan.release_point_ned.y}, alt);

    // overshoot: perpanjang garis melewati release sesuai approach_heading
    // (heading dari North, CW: north=cos, east=sin).
    if (overshoot_distance_ > 0.0) {
      Ned2D ov{
        plan.release_point_ned.x + std::cos(plan.approach_heading) * overshoot_distance_,
        plan.release_point_ned.y + std::sin(plan.approach_heading) * overshoot_distance_};
      pushNed(req, ov, alt);
    }

    const size_t n = req->waypoints.size();
    cli_push_->async_send_request(
      req,
      [this, n](rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedFuture fut) {
        auto res = fut.get();
        if (res->success) {
          RCLCPP_INFO(get_logger(),
            "Mission ter-push: %u/%zu waypoint diterima ArduPilot.",
            res->wp_transfered, n);
        } else {
          RCLCPP_ERROR(get_logger(),
            "WaypointPush GAGAL (success=false, %u waypoint).",
            res->wp_transfered);
        }
      });
  }

  void pushNed(
    const mavros_msgs::srv::WaypointPush::Request::SharedPtr & req,
    const Ned2D & ned, double alt)
  {
    const GeoPoint g = frame_.toGeo(ned);
    req->waypoints.push_back(
      makeWp(mav::CMD_NAV_WAYPOINT, g.lat_deg, g.lon_deg, alt));
  }

  mavros_msgs::msg::Waypoint makeWp(
    uint16_t command, double lat, double lon, double alt,
    bool is_current = false)
  {
    mavros_msgs::msg::Waypoint wp;
    wp.frame = mav::FRAME_GLOBAL_RELATIVE_ALT;
    wp.command = command;
    wp.is_current = is_current;
    wp.autocontinue = true;
    wp.param1 = 0.0f;
    wp.param2 = static_cast<float>(acceptance_radius_);  // acceptance radius
    wp.param3 = 0.0f;
    wp.param4 = 0.0f;
    wp.x_lat = lat;
    wp.y_long = lon;
    wp.z_alt = alt;
    return wp;
  }

  // ------------------------- anggota -------------------------
  LocalFrame frame_;
  double home_alt_{0.0};

  // bool have_last_release_{false};
  // Ned2D last_release_ned_{};

  // std::string plan_topic_;
  // std::string global_topic_;
  // std::string push_service_;
  // double loiter_radius_{80.0};
  // double overshoot_distance_{150.0};
  // double acceptance_radius_{0.0};
  // double min_replan_move_{5.0};

  // rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_global_;
  // rclcpp::Subscription<interfaces::msg::AirdropPlan>::SharedPtr sub_plan_;
  // rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr cli_push_;


  bool armed_{true};
  uint8_t last_state_{0};

  std::string plan_topic_;
  std::string global_topic_;
  std::string status_topic_;
  std::string push_service_;
  double loiter_radius_{80.0};
  double overshoot_distance_{150.0};
  double acceptance_radius_{0.0};

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_global_;
  rclcpp::Subscription<interfaces::msg::AirdropPlan>::SharedPtr sub_plan_;
  rclcpp::Subscription<interfaces::msg::AirdropStatus>::SharedPtr sub_status_;
  rclcpp::Client<mavros_msgs::srv::WaypointPush>::SharedPtr cli_push_;
};

}  // namespace airdrop_planning

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<airdrop_planning::MissionBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
