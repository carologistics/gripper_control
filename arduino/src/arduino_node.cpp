// Copyright (c) 2024 Carologistics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "arduino/arduino_node.hpp"

ArduinoNode::ArduinoNode() : Node("arduino_node") {
  // Declare parameters
  this->declare_parameter("port", "/dev/arduino");
  this->declare_parameter("baud_rate", 115200);

  auto port = this->get_parameter("port").as_string();
  auto baud_rate = this->get_parameter("baud_rate").as_int();

  // Initialize serial port
  try {
    port_ = std::make_unique<SerialPort>(
        port,
        std::bind(&ArduinoNode::handle_serial_message, this,
                  std::placeholders::_1),
        std::bind(&ArduinoNode::handle_disconnect, this), baud_rate);
  } catch (const boost::system::system_error &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s",
                 e.what());
    throw;
  }

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Create timer for periodic checks
  timer_ = this->create_wall_timer(
      std::chrono::seconds(1), std::bind(&ArduinoNode::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Arduino node initialized");

  // Create action servers
  home_action_server_ = rclcpp_action::create_server<Home>(
      this, "arduino/home",
      std::bind(&ArduinoNode::handle_home_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ArduinoNode::handle_home_cancel, this, std::placeholders::_1),
      std::bind(&ArduinoNode::handle_home_accepted, this,
                std::placeholders::_1));

  calibrate_action_server_ = rclcpp_action::create_server<Calibrate>(
      this, "arduino/calibrate",
      std::bind(&ArduinoNode::handle_calibrate_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArduinoNode::handle_calibrate_cancel, this,
                std::placeholders::_1),
      std::bind(&ArduinoNode::handle_calibrate_accepted, this,
                std::placeholders::_1));

  // Create cmd_vel subscription
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&ArduinoNode::cmd_vel_callback, this, std::placeholders::_1));

  // Create services
  get_status_service_ = create_service<arduino::srv::GetStatus>(
      "arduino/get_status",
      std::bind(&ArduinoNode::handle_get_status, this, std::placeholders::_1,
                std::placeholders::_2));

  reset_device_service_ = create_service<arduino::srv::ResetDevice>(
      "arduino/reset_device",
      std::bind(&ArduinoNode::handle_reset_device, this, std::placeholders::_1,
                std::placeholders::_2));
}

ArduinoNode::~ArduinoNode() { port_.reset(); }

void ArduinoNode::timer_callback() {
  // Periodic checks and updates
  if (port_) {
    // Add monitoring code here
    // Check if waypoint sensor is enabled
    if (wp_sensor_enable_) {
      std::stringstream cmd;
      cmd << "R " << (wp_sensor_analog_ ? "A" : "D") << " " << wp_sensor_pin_;
      if (wp_sensor_analog_) {
        cmd << " " << wp_sensor_analog_threshold_;
      }
      send_arduino_command(cmd.str());
    }

    // Add periodic status request
    send_arduino_command("?"); // Query device status periodically

    // Add watchdog functionality
    if ((std::chrono::steady_clock::now() - comm_stats_.last_msg_time) >
        std::chrono::seconds(5)) {
      handle_error("Communication timeout");
    }
  }
}

void ArduinoNode::handle_serial_message(const std::string &msg) {
  RCLCPP_DEBUG(this->get_logger(), "Received message: %s", msg.c_str());
  // Add message handling code here
  // Parse message format: "<type> <data>"
  std::istringstream iss(msg);
  std::string type;
  iss >> type;

  if (type == "W") { // Waypoint sensor data
    int value;
    iss >> value;
    wp_sensed_ = value > 0;

    // Broadcast transform if waypoint detected
    if (wp_sensed_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "base_link";
      t.child_frame_id = "waypoint";
      t.transform.translation.x = 0.0;
      t.transform.translation.y = 0.0;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = 0.0;
      t.transform.rotation.y = 0.0;
      t.transform.rotation.z = 0.0;
      t.transform.rotation.w = 1.0;
      tf_broadcaster_->sendTransform(t);
    }
  }

  // Add status message handling
  if (type == "S") { // Status message
    std::string status;
    iss >> status;
    if (status == "OK") {
      device_state_ = DeviceState::OPERATIONAL;
    }
    comm_stats_.last_msg_time = std::chrono::steady_clock::now();
  }
}

void ArduinoNode::handle_disconnect() {
  RCLCPP_WARN(this->get_logger(), "Serial port disconnected");
  // Handle reconnection logic here
  // Try to reconnect every 5 seconds
  while (rclcpp::ok()) {
    try {
      auto port = this->get_parameter("port").as_string();
      auto baud_rate = this->get_parameter("baud_rate").as_int();

      port_ = std::make_unique<SerialPort>(
          port,
          std::bind(&ArduinoNode::handle_serial_message, this,
                    std::placeholders::_1),
          std::bind(&ArduinoNode::handle_disconnect, this), baud_rate);

      RCLCPP_INFO(this->get_logger(),
                  "Successfully reconnected to serial port");
      return;
    } catch (const boost::system::system_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Reconnection failed: %s", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(5));
    }
  }
}

void ArduinoNode::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  handle_move_command(msg->linear.x, msg->angular.z);
}

void ArduinoNode::handle_move_command(float linear_x, float angular_z) {
  std::stringstream cmd;
  if (linear_x == 0.0 && angular_z == 0.0) {
    handle_stop_command();
    return;
  }

  cmd << "M" << linear_x << " "
      << angular_z; // Removed space after M to match Fawkes
  send_arduino_command(cmd.str());
}

void ArduinoNode::handle_stop_command() { send_arduino_command("S"); }

void ArduinoNode::send_arduino_command(const std::string &cmd) {
  if (port_) {
    port_->write("AT " + cmd + "+"); // Add AT prefix and + suffix
  }
}

// Action server implementations
rclcpp_action::GoalResponse
ArduinoNode::handle_home_goal(const rclcpp_action::GoalUUID &,
                              std::shared_ptr<const Home::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArduinoNode::handle_home_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Home>>) {
  handle_stop_command();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArduinoNode::handle_home_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Home>> goal_handle) {
  send_arduino_command("H");
  auto result = std::make_shared<Home::Result>();
  result->success = true;
  result->message = "Homing completed";
  goal_handle->succeed(result);
}

rclcpp_action::GoalResponse
ArduinoNode::handle_calibrate_goal(const rclcpp_action::GoalUUID &,
                                   std::shared_ptr<const Calibrate::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArduinoNode::handle_calibrate_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Calibrate>>) {
  handle_stop_command();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArduinoNode::handle_calibrate_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Calibrate>> goal_handle) {
  auto goal = goal_handle->get_goal();
  send_arduino_command(goal->double_calibrate ? "D" : "C");

  auto result = std::make_shared<Calibrate::Result>();
  result->success = true;
  result->message = goal->double_calibrate ? "Double calibration completed"
                                           : "Calibration completed";
  goal_handle->succeed(result);
}

void ArduinoNode::handle_get_status(
    const std::shared_ptr<arduino::srv::GetStatus::Request>,
    std::shared_ptr<arduino::srv::GetStatus::Response> response) {
  response->connected = (port_ != nullptr);
  response->state = [this]() {
    switch (device_state_) {
    case DeviceState::DISCONNECTED:
      return "DISCONNECTED";
    case DeviceState::CONNECTING:
      return "CONNECTING";
    case DeviceState::INITIALIZING:
      return "INITIALIZING";
    case DeviceState::OPERATIONAL:
      return "OPERATIONAL";
    case DeviceState::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
    }
  }();
  response->bytes_received = comm_stats_.bytes_received;
  response->bytes_sent = comm_stats_.bytes_sent;
  response->errors = comm_stats_.errors;
}

void ArduinoNode::handle_reset_device(
    const std::shared_ptr<arduino::srv::ResetDevice::Request>,
    std::shared_ptr<arduino::srv::ResetDevice::Response> response) {
  port_.reset();
  retry_count_ = 0;

  if (attempt_reconnect()) {
    response->success = true;
    response->message = "Device reset successful";
  } else {
    response->success = false;
    response->message = "Failed to reset device";
  }
}

bool ArduinoNode::attempt_reconnect() {
  if (retry_count_ >= MAX_RETRIES) {
    device_state_ = DeviceState::ERROR;
    return false;
  }

  try {
    auto port = this->get_parameter("port").as_string();
    auto baud_rate = this->get_parameter("baud_rate").as_int();

    port_ = std::make_unique<SerialPort>(
        port,
        std::bind(&ArduinoNode::handle_serial_message, this,
                  std::placeholders::_1),
        std::bind(&ArduinoNode::handle_disconnect, this), baud_rate);

    device_state_ = DeviceState::OPERATIONAL;
    retry_count_ = 0;
    return true;
  } catch (const boost::system::system_error &e) {
    retry_count_++;
    return false;
  }
}

void ArduinoNode::handle_error(const std::string &error_msg) {
  RCLCPP_ERROR(this->get_logger(), "Error: %s", error_msg.c_str());
  comm_stats_.errors++;
  device_state_ = DeviceState::ERROR;
  publish_diagnostics();
}

void ArduinoNode::publish_diagnostics() {
  auto diag_msg = diagnostic_msgs::msg::DiagnosticArray();
  diag_msg.header.stamp = this->get_clock()->now();

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Arduino Device";
  status.hardware_id = this->get_parameter("port").as_string();

  if (device_state_ == DeviceState::OPERATIONAL) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "Device operational";
  } else if (device_state_ == DeviceState::ERROR) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = "Device error";
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = "Device not fully operational";
  }

  diagnostic_msgs::msg::KeyValue kv;
  kv.key = "State";
  kv.value = [this]() {
    switch (device_state_) {
    case DeviceState::DISCONNECTED:
      return "DISCONNECTED";
    case DeviceState::CONNECTING:
      return "CONNECTING";
    case DeviceState::INITIALIZING:
      return "INITIALIZING";
    case DeviceState::OPERATIONAL:
      return "OPERATIONAL";
    case DeviceState::ERROR:
      return "ERROR";
    default:
      return "UNKNOWN";
    }
  }();
  status.values.push_back(kv);

  diag_msg.status.push_back(status);
  diagnostics_pub_->publish(diag_msg);
}

rcl_interfaces::msg::SetParametersResult ArduinoNode::parameterCallback(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto &param : parameters) {
    if (param.get_name() == "wp_sensor_enable") {
      wp_sensor_enable_ = param.as_bool();
    } else if (param.get_name() == "wp_sensor_analog") {
      wp_sensor_analog_ = param.as_bool();
    } else if (param.get_name() == "wp_sensor_pin") {
      wp_sensor_pin_ = param.as_int();
    } else if (param.get_name() == "wp_sensor_analog_threshold") {
      wp_sensor_analog_threshold_ = param.as_double();
    }
  }

  return result;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoNode>());
  rclcpp::shutdown();
  return 0;
}
