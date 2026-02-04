#include "linkerhand_ros2/linkerhand_node.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <utility>

#include "diagnostic_msgs/msg/key_value.hpp"
#include "linkerhand/exceptions.hpp"
#include "linkerhand/hand/l6/l6.hpp"
#include "linkerhand_msgs/msg/force_sensor_data.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

double degrees_to_radians(double degrees) {
  return degrees * kPi / 180.0;
}

double radians_to_degrees(double radians) {
  return radians * 180.0 / kPi;
}

}  // namespace

namespace linkerhand_ros2 {

LinkerHandNode::LinkerHandNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("linkerhand_node", options) {
  side_ = declare_parameter<std::string>("side", "left");
  can_interface_ = declare_parameter<std::string>("can_interface", "can0");
  publish_rate_ = declare_parameter<double>("publish_rate", 100.0);
  timeout_ms_ = declare_parameter<double>("timeout_ms", 50.0);
  enable_force_sensors_ = declare_parameter<bool>("enable_force_sensors", true);
  joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"thumb", "index", "middle", "ring", "little", "thumb_rotation"});

  if (publish_rate_ <= 0.0) {
    RCLCPP_WARN(get_logger(), "publish_rate must be positive, fallback to 100.0");
  }
  const double safe_publish_rate = publish_rate_ > 0.0 ? publish_rate_ : 100.0;
  publish_rate_ = safe_publish_rate;

  if (joint_names_.size() != 6) {
    RCLCPP_WARN(
        get_logger(),
        "joint_names must contain 6 elements, fallback to default list");
    joint_names_ = {"thumb", "index", "middle", "ring", "little", "thumb_rotation"};
  }

  try {
    hand_ = std::make_unique<linkerhand::hand::l6::L6>(side_, can_interface_);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Failed to initialize LinkerHand SDK: %s", e.what());
    throw;
  }

  joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  force_sensors_pub_ = create_publisher<linkerhand_msgs::msg::ForceSensors>("force_sensors", 10);
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticStatus>("diagnostics", 10);

  cmd_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "cmd_joint_states",
      10,
      [this](sensor_msgs::msg::JointState::SharedPtr msg) { cmd_callback(std::move(msg)); });

  const std::chrono::duration<double> period_s(1.0 / safe_publish_rate);
  publish_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period_s),
      [this] { publish_callback(); });
}

LinkerHandNode::~LinkerHandNode() = default;

void LinkerHandNode::publish_diagnostics(std::uint8_t level, const std::string& message) {
  if (!diagnostics_pub_) {
    return;
  }

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.level = level;
  status.name = "linkerhand/" + side_;
  status.message = message;
  status.hardware_id = can_interface_;

  status.values.reserve(8);
  auto add_kv = [&](const std::string& key, const std::string& value) {
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = key;
    kv.value = value;
    status.values.push_back(std::move(kv));
  };

  const bool stream_active = publish_timer_ && !publish_timer_->is_canceled();
  add_kv("side", side_);
  add_kv("can_interface", can_interface_);
  add_kv("stream_active", stream_active ? "true" : "false");
  add_kv("hand_initialized", hand_ ? "true" : "false");
  add_kv("publish_rate", std::to_string(publish_rate_));
  add_kv("timeout_ms", std::to_string(timeout_ms_));
  add_kv("enable_force_sensors", enable_force_sensors_ ? "true" : "false");
  add_kv("joint_names_size", std::to_string(joint_names_.size()));

  diagnostics_pub_->publish(status);
}

void LinkerHandNode::publish_callback() {
  if (!hand_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "SDK is not initialized");
    publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SDK is not initialized");
    return;
  }

  bool ok = true;

  try {
    const auto angles = hand_->angle.get_angles_blocking(timeout_ms_);

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;
    msg.position.resize(angles.angles.size());
    for (std::size_t i = 0; i < angles.angles.size(); ++i) {
      msg.position[i] = degrees_to_radians(static_cast<double>(angles.angles[i]));
    }
    joint_state_pub_->publish(msg);
  } catch (const linkerhand::TimeoutError& e) {
    ok = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Angle read timeout: %s", e.what());
    publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::WARN, std::string("Angle timeout: ") + e.what());
  } catch (const linkerhand::LinkerHandError& e) {
    ok = false;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Angle read error: %s", e.what());
    publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string("Angle error: ") + e.what());
  } catch (const std::exception& e) {
    ok = false;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Angle read error: %s", e.what());
    publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string("Angle error: ") + e.what());
  }

  if (!enable_force_sensors_) {
    if (ok) {
      publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
    }
    return;
  }

  try {
    const auto data = hand_->force_sensor.get_data_blocking(timeout_ms_);

    linkerhand_msgs::msg::ForceSensors out;
    out.header.stamp = now();
    out.fingers.reserve(5);

    auto append_finger = [&](const std::string& name, const linkerhand::hand::l6::ForceSensorData& sensor) {
      linkerhand_msgs::msg::ForceSensorData msg;
      msg.header = out.header;
      msg.finger_name = name;
      msg.pressure_values.resize(sensor.values.size());
      for (std::size_t i = 0; i < sensor.values.size(); ++i) {
        msg.pressure_values[i] = static_cast<double>(sensor.values[i]);
      }
      out.fingers.push_back(std::move(msg));
    };

    append_finger("thumb", data.thumb);
    append_finger("index", data.index);
    append_finger("middle", data.middle);
    append_finger("ring", data.ring);
    append_finger("little", data.pinky);

    force_sensors_pub_->publish(out);
  } catch (const linkerhand::TimeoutError& e) {
    ok = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Force sensor read timeout: %s", e.what());
    publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::WARN, std::string("Force sensor timeout: ") + e.what());
  } catch (const linkerhand::LinkerHandError& e) {
    ok = false;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Force sensor read error: %s", e.what());
    publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string("Force sensor error: ") + e.what());
  } catch (const std::exception& e) {
    ok = false;
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Force sensor read error: %s", e.what());
    publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR, std::string("Force sensor error: ") + e.what());
  }

  if (ok) {
    publish_diagnostics(diagnostic_msgs::msg::DiagnosticStatus::OK, "OK");
  }
}

void LinkerHandNode::cmd_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  if (!hand_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "SDK is not initialized");
    return;
  }
  if (!msg) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Received null JointState message");
    return;
  }
  if (msg->position.size() != 6) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "cmd_joint_states.position must have 6 elements, got %zu",
        msg->position.size());
    return;
  }

  std::array<int, 6> angles_deg{};
  for (std::size_t i = 0; i < angles_deg.size(); ++i) {
    const double deg = radians_to_degrees(msg->position[i]);
    angles_deg[i] = static_cast<int>(std::lround(deg));
  }

  try {
    hand_->angle.set_angles(angles_deg);
  } catch (const linkerhand::ValidationError& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Invalid angle command: %s", e.what());
  } catch (const linkerhand::StateError& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "SDK state error: %s", e.what());
  } catch (const linkerhand::CANError& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "CAN error: %s", e.what());
  } catch (const linkerhand::LinkerHandError& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Command error: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Command error: %s", e.what());
  }
}

}  // namespace linkerhand_ros2

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<linkerhand_ros2::LinkerHandNode>(rclcpp::NodeOptions{}));
  rclcpp::shutdown();
  return 0;
}
