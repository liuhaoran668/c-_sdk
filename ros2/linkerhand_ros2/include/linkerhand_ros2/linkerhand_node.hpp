#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "linkerhand_msgs/msg/force_sensors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace linkerhand::hand::l6 {
class L6;
}  // namespace linkerhand::hand::l6

namespace linkerhand_ros2 {

class LinkerHandNode final : public rclcpp::Node {
 public:
  explicit LinkerHandNode(const rclcpp::NodeOptions& options);
  ~LinkerHandNode() override;

 private:
  std::unique_ptr<linkerhand::hand::l6::L6> hand_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<linkerhand_msgs::msg::ForceSensors>::SharedPtr force_sensors_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostics_pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string side_ = "left";
  std::string can_interface_ = "can0";
  double publish_rate_ = 100.0;
  std::vector<std::string> joint_names_;
  bool enable_force_sensors_ = true;
  double timeout_ms_ = 50.0;

  void publish_callback();
  void publish_diagnostics(std::uint8_t level, const std::string& message);
  void cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
};

}  // namespace linkerhand_ros2
