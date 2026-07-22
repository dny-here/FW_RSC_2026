// drop_relay_node.cpp
// Node "Drop Relay" — jembatan antara airdrop_planning dan FCU.
//
// Peran:
//   server : interfaces/srv/DropPayload pada service "drop"
//            (dipanggil airdrop_planner saat FINAL_APPROACH)
//   proses : peta bay_index -> channel servo + PWM, kirim MAVLink
//            DO_SET_SERVO ke FCU via service MAVROS mavros/cmd/command
//            (CommandLong), tunggu COMMAND_ACK
//   balas  : DropPayload response {success, message} dari hasil ACK
//
// Pemetaan bay_index (abstrak) -> servo channel/PWM (hardware) berada
// DI SINI, sehingga airdrop_planning tetap bebas MAVROS/hardware.
//
// Karena memanggil service (MAVROS) dari DALAM callback service (/drop),
// client MAVROS ditempatkan di callback group Reentrant terpisah dan node
// dijalankan dengan MultiThreadedExecutor -> future.wait_for() di callback
// tidak deadlock.

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "mavros_msgs/srv/command_long.hpp"
#include "interfaces/srv/drop_payload.hpp"

namespace drop_mechanism
{

class DropRelayNode : public rclcpp::Node
{
public:
  DropRelayNode()
  : Node("drop_relay")
  {
    declare_parameter("drop_service", std::string("drop"));
    declare_parameter("command_service", std::string("mavros/cmd/command"));
    declare_parameter("servo_channel_bay", std::vector<int64_t>{9, 10});
    declare_parameter("pwm_release", 1900);
    declare_parameter("command_timeout", 2.0);

    servo_channels_ = get_parameter("servo_channel_bay").as_integer_array();
    pwm_release_ = static_cast<int>(get_parameter("pwm_release").as_int());
    command_timeout_ = get_parameter("command_timeout").as_double();

    // Client MAVROS di callback group Reentrant terpisah agar response-nya
    // bisa diproses thread lain saat callback service mem-block future.
    client_cbg_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cli_command_ = create_client<mavros_msgs::srv::CommandLong>(
      get_parameter("command_service").as_string(),
      rmw_qos_profile_services_default, client_cbg_);

    srv_drop_ = create_service<interfaces::srv::DropPayload>(
      get_parameter("drop_service").as_string(),
      std::bind(&DropRelayNode::onDrop, this,
        std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "drop_relay siap. Service '%s' -> MAVROS '%s' (DO_SET_SERVO), %zu bay.",
      get_parameter("drop_service").as_string().c_str(),
      get_parameter("command_service").as_string().c_str(),
      servo_channels_.size());
  }

private:
  void onDrop(
    const std::shared_ptr<interfaces::srv::DropPayload::Request> req,
    std::shared_ptr<interfaces::srv::DropPayload::Response> res)
  {
    // 1. validasi bay
    if (req->bay_index >= servo_channels_.size()) {
      res->success = false;
      res->message = "bay_index " + std::to_string(req->bay_index) +
        " di luar jangkauan (jumlah bay=" +
        std::to_string(servo_channels_.size()) + ").";
      RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
      return;
    }

    // 2. MAVROS siap?
    if (!cli_command_->service_is_ready()) {
      res->success = false;
      res->message = "Service MAVROS '" +
        std::string(cli_command_->get_service_name()) + "' tidak tersedia.";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    // 3. bangun DO_SET_SERVO
    const int channel = static_cast<int>(servo_channels_[req->bay_index]);
    auto cmd = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    cmd->broadcast = false;
    cmd->command = 183;  // MAV_CMD_DO_SET_SERVO
    cmd->confirmation = 0;
    cmd->param1 = static_cast<float>(channel);   // nomor servo output
    cmd->param2 = static_cast<float>(pwm_release_);  // PWM

    RCLCPP_INFO(get_logger(),
      "Drop bay=%u -> DO_SET_SERVO ch=%d pwm=%d", req->bay_index, channel,
      pwm_release_);

    auto future = cli_command_->async_send_request(cmd);

    // 4. tunggu ACK (block thread callback ini; client di cbg lain)
    const std::chrono::duration<double> timeout(command_timeout_);
    if (future.wait_for(timeout) != std::future_status::ready) {
      res->success = false;
      res->message = "Timeout menunggu ACK MAVROS.";
      RCLCPP_ERROR(get_logger(), "%s", res->message.c_str());
      return;
    }

    const auto ack = future.get();
    res->success = ack->success;
    res->message = ack->success ?
      ("DO_SET_SERVO diterima FCU (ch " + std::to_string(channel) + ").") :
      ("FCU menolak DO_SET_SERVO (result=" +
        std::to_string(ack->result) + ").");
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  std::vector<int64_t> servo_channels_;
  int pwm_release_{1900};
  double command_timeout_{2.0};

  rclcpp::CallbackGroup::SharedPtr client_cbg_;
  rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr cli_command_;
  rclcpp::Service<interfaces::srv::DropPayload>::SharedPtr srv_drop_;
};

}  // namespace drop_mechanism

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drop_mechanism::DropRelayNode>();
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
