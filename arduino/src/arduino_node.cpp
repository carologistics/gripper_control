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
  // Initialize CommStats at the start
  comm_stats_.last_msg_time = std::chrono::steady_clock::now();
  comm_stats_.bytes_received = 0;
  comm_stats_.bytes_sent = 0;
  comm_stats_.errors = 0;

  // Initialize device state and reconnection timing
  device_state_ = DeviceState::DISCONNECTED;
  last_reconnect_attempt_ = std::chrono::steady_clock::now();

  // Create diagnostics publisher first
  diagnostics_pub_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          "diagnostics", 10);

  // Register parameter callback (Add this before parameter declarations)
  param_callback_ = this->add_on_set_parameters_callback(
      std::bind(&ArduinoNode::parameterCallback, this, std::placeholders::_1));

  // Declare parameters
  declare_parameters();

  try {
    auto port = this->get_parameter("port").as_string();
    auto baud_rate = this->get_parameter("baud_rate").as_int();

    // Check if device exists
    if (access(port.c_str(), F_OK) == -1) {
      throw SerialPortError("Device file does not exist: " + port);
    }

    RCLCPP_INFO(this->get_logger(), "Attempting to connect to %s at %ld baud",
                port.c_str(), baud_rate);

    port_ = std::make_unique<SerialPort>(
        port,
        std::bind(&ArduinoNode::handle_serial_message, this,
                  std::placeholders::_1),
        std::bind(&ArduinoNode::handle_disconnect, this), baud_rate);

    // Test communication
    device_state_ = DeviceState::INITIALIZING;
    send_arduino_command("?"); // Send initial status request

    // Wait briefly for response
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    if (device_state_ == DeviceState::INITIALIZING) {
      throw SerialPortError("No response from device");
    }

    RCLCPP_INFO(this->get_logger(), "Successfully connected to Arduino");
  } catch (const SerialPortError &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s",
                 e.what());
    device_state_ = DeviceState::DISCONNECTED;
    // Don't rethrow - allow node to keep running
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
    device_state_ = DeviceState::DISCONNECTED;
    // Don't rethrow - allow node to keep running
  }

  // Initialize TF broadcaster
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  // Create timer for periodic checks
  timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / config_.status_frequency),
      std::bind(&ArduinoNode::timer_callback, this));

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

  move_xyz_action_server_ = rclcpp_action::create_server<MoveXYZ>(
      this, "arduino/move_xyz",
      std::bind(&ArduinoNode::handle_move_xyz_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ArduinoNode::handle_move_xyz_cancel, this,
                std::placeholders::_1),
      std::bind(&ArduinoNode::handle_move_xyz_accepted, this,
                std::placeholders::_1));

  gripper_action_server_ = rclcpp_action::create_server<Gripper>(
      this, "arduino/gripper",
      std::bind(&ArduinoNode::handle_gripper_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&ArduinoNode::handle_gripper_cancel, this,
                std::placeholders::_1),
      std::bind(&ArduinoNode::handle_gripper_accepted, this,
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

  // Create status publisher
  status_pub_ =
      this->create_publisher<arduino::msg::Status>("arduino/status", 10);

  // Initialize feedback tracking
  last_feedback_time_ = std::chrono::steady_clock::now();
}

ArduinoNode::~ArduinoNode() { cleanup(); }

void ArduinoNode::cleanup() {
  // Stop all motion
  handle_stop_command();

  // Reset port
  port_.reset();

  // Clean up subscribers and publishers
  cmd_vel_sub_.reset();
  status_pub_.reset();
  diagnostics_pub_.reset();

  // Clean up services
  get_status_service_.reset();
  reset_device_service_.reset();

  // Clean up action servers
  home_action_server_.reset();
  calibrate_action_server_.reset();

  // Add action server cleanup
  if (active_move_goal_) {
    auto result = std::make_shared<MoveXYZ::Result>();
    result->success = false;
    result->message = "Node shutting down";
    active_move_goal_->abort(result);
  }

  if (active_gripper_goal_) {
    auto result = std::make_shared<Gripper::Result>();
    result->success = false;
    result->message = "Node shutting down";
    active_gripper_goal_->abort(result);
  }

  move_xyz_action_server_.reset();
  gripper_action_server_.reset();
}

void ArduinoNode::timer_callback() {
  if (!port_) {
    auto now = std::chrono::steady_clock::now();
    if (now - last_reconnect_attempt_ > reconnect_interval_) {
      last_reconnect_attempt_ = now;
      if (attempt_reconnect()) {
        RCLCPP_INFO(this->get_logger(), "Successfully reconnected to Arduino");
      }
    }
    return;
  }

  try {
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
    auto now = std::chrono::steady_clock::now();
    if ((now - comm_stats_.last_msg_time) > std::chrono::seconds(5)) {
      if (device_state_ == DeviceState::OPERATIONAL) {
        handle_error("Communication timeout");
        attempt_reconnect();
      }
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(this->get_logger(), "Timer callback error: %s", e.what());
    handle_error(e.what());
  }

  // Add feedback monitoring
  check_action_timeouts();
  handle_feedback_updates();
}

void ArduinoNode::handle_serial_message(const std::string &msg) {
  // Update timestamp at start of message handling
  comm_stats_.last_msg_time = std::chrono::steady_clock::now();
  comm_stats_.bytes_received += msg.length();

  RCLCPP_DEBUG(this->get_logger(), "Received message: %s", msg.c_str());
  // Add message handling code here
  // Parse message format: "<type> <data>"
  std::istringstream iss(msg);
  std::string type;
  iss >> type;

  // Add checksum verification
  size_t plus_pos = msg.find_last_of('+');
  if (plus_pos != std::string::npos && plus_pos < msg.length() - 1) {
    int received_checksum = std::stoi(msg.substr(plus_pos + 1));
    int calculated_checksum = 0;
    for (size_t i = 0; i < plus_pos; i++) {
      calculated_checksum += msg[i];
    }
    calculated_checksum = (calculated_checksum + 128) % 256;

    if (calculated_checksum != received_checksum) {
      RCLCPP_WARN(this->get_logger(), "Checksum mismatch!");
      return;
    }
  }

  if (type == "E") { // Error message
    std::string error_msg = msg.substr(msg.find(" ") + 1);
    handle_error("Arduino error: " + error_msg);
    return;
  }

  if (type == "W" &&
      config_.enable_tf_broadcast) { // Only broadcast TF if enabled
    int value;
    iss >> value;
    wp_sensed_ = value > 0;

    // Broadcast transform if waypoint detected
    if (wp_sensed_) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = config_.base_frame;    // Use configured frame
      t.child_frame_id = config_.waypoint_frame; // Use configured frame
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
      if (device_state_ == DeviceState::INITIALIZING) {
        RCLCPP_INFO(this->get_logger(), "Initial communication successful");
      }
      device_state_ = DeviceState::OPERATIONAL;
    }
    comm_stats_.last_msg_time = std::chrono::steady_clock::now();
  }

  if (type == "P") { // Position update
    iss >> status_.x_position >> status_.y_position >> status_.z_position;
  } else if (type == "G") { // Gripper status
    int closed;
    iss >> closed;
    status_.gripper_closed = (closed > 0);
  } else if (type == "L") { // Limits
    iss >> status_.x_max >> status_.y_max >> status_.z_max;
  } else if (type == "M") { // Motion status
    std::string motion_status;
    iss >> motion_status;
    status_.final = (motion_status == "IDLE");

    if (motion_status == "MOVING") {
      arduino_status_ = ArduinoStatus::MOVING;
    } else if (motion_status == "IDLE") {
      arduino_status_ = ArduinoStatus::IDLE;
      // Check if goals completed
      if (active_move_goal_) {
        auto result = std::make_shared<MoveXYZ::Result>();
        result->success = true;
        result->message = "Move completed";
        active_move_goal_->succeed(result);
        active_move_goal_.reset();
      }
      if (active_gripper_goal_) {
        auto result = std::make_shared<Gripper::Result>();
        result->success = true;
        result->message = "Gripper action completed";
        active_gripper_goal_->succeed(result);
        active_gripper_goal_.reset();
      }
    } else if (motion_status.find("ERROR") != std::string::npos) {
      // Handle errors for active goals
      if (active_move_goal_) {
        auto result = std::make_shared<MoveXYZ::Result>();
        result->success = false;
        result->message = "Motion error: " + motion_status;
        active_move_goal_->abort(result);
        active_move_goal_.reset();
      }
      if (active_gripper_goal_) {
        auto result = std::make_shared<Gripper::Result>();
        result->success = false;
        result->message = "Motion error: " + motion_status;
        active_gripper_goal_->abort(result);
        active_gripper_goal_.reset();
      }
    }

    // Publish feedback for active goals
    publish_move_xyz_feedback();
    publish_gripper_feedback();
  }

  // Increment message ID for commands
  if (type == "M" || type == "H" || type == "C" || type == "D") {
    status_.cmd_msgid++;
  }
  status_.msgid++;

  // Publish current status
  auto status_msg = arduino::msg::Status();
  status_msg.x_position = status_.x_position;
  status_msg.y_position = status_.y_position;
  status_msg.z_position = status_.z_position;
  status_msg.gripper_closed = status_.gripper_closed;
  status_msg.wp_sensed = wp_sensed_;
  status_msg.x_max = status_.x_max;
  status_msg.y_max = status_.y_max;
  status_msg.z_max = status_.z_max;
  status_msg.final = status_.final;
  status_msg.msgid = status_.msgid;
  status_msg.status = status_.status;

  status_pub_->publish(status_msg);
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
    std::string full_cmd = "AT " + cmd + "+";
    if (port_->write(full_cmd)) {
      comm_stats_.bytes_sent += full_cmd.length();
    }
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

void ArduinoNode::declare_parameters() {
  // Device parameters
  this->declare_parameter("port", "/dev/arduino");
  this->declare_parameter("baud_rate", 115200);

  // Waypoint sensor parameters
  this->declare_parameter("wp_sensor_enable", true);
  this->declare_parameter("wp_sensor_analog", true);
  this->declare_parameter("wp_sensor_pin", 0);
  this->declare_parameter("wp_sensor_analog_threshold", 0.7);

  // Configuration parameters
  this->declare_parameter("enable_tf_broadcast", config_.enable_tf_broadcast);
  this->declare_parameter("base_frame", config_.base_frame);
  this->declare_parameter("waypoint_frame", config_.waypoint_frame);
  this->declare_parameter("status_frequency", config_.status_frequency);

  // Get configuration
  config_.enable_tf_broadcast =
      this->get_parameter("enable_tf_broadcast").as_bool();
  config_.base_frame = this->get_parameter("base_frame").as_string();
  config_.waypoint_frame = this->get_parameter("waypoint_frame").as_string();
  config_.status_frequency =
      this->get_parameter("status_frequency").as_double();
}

void ArduinoNode::publish_move_xyz_feedback() {
  if (active_move_goal_) {
    auto feedback = std::make_shared<MoveXYZ::Feedback>();
    feedback->x_position = status_.x_position;
    feedback->y_position = status_.y_position;
    feedback->z_position = status_.z_position;
    feedback->moving = (arduino_status_ == ArduinoStatus::MOVING);
    active_move_goal_->publish_feedback(feedback);
  }
}

void ArduinoNode::publish_gripper_feedback() {
  if (active_gripper_goal_) {
    auto feedback = std::make_shared<Gripper::Feedback>();
    feedback->gripper_closed = status_.gripper_closed;
    feedback->moving = (arduino_status_ == ArduinoStatus::MOVING);
    active_gripper_goal_->publish_feedback(feedback);
  }
}

void ArduinoNode::execute_move_xyz(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveXYZ>> goal_handle) {
  const auto goal = goal_handle->get_goal();
  active_move_goal_ = goal_handle;

  std::stringstream cmd;
  cmd << (goal->relative ? "MR " : "MA ") << goal->x << " " << goal->y << " "
      << goal->z;
  if (!goal->relative && !goal->target_frame.empty()) {
    cmd << " " << goal->target_frame;
  }

  send_arduino_command(cmd.str());
}

void ArduinoNode::execute_gripper(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>> goal_handle) {
  const auto goal = goal_handle->get_goal();
  active_gripper_goal_ = goal_handle;

  send_arduino_command(goal->close ? "G" : "O");
}

// Add action server handlers
rclcpp_action::GoalResponse
ArduinoNode::handle_move_xyz_goal(const rclcpp_action::GoalUUID &,
                                  std::shared_ptr<const MoveXYZ::Goal>) {
  if (active_move_goal_ != nullptr) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArduinoNode::handle_move_xyz_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveXYZ>>) {
  handle_stop_command();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArduinoNode::handle_move_xyz_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveXYZ>> goal_handle) {
  execute_move_xyz(goal_handle);
}

void ArduinoNode::handle_feedback_updates() {
  if (!port_ || device_state_ != DeviceState::OPERATIONAL) {
    return;
  }

  // Update last feedback time
  last_feedback_time_ = std::chrono::steady_clock::now();

  // Publish feedback for all active goals
  publish_move_xyz_feedback();
  publish_gripper_feedback();

  // Request status update from Arduino
  send_arduino_command("S");
}

void ArduinoNode::check_action_timeouts() {
  auto now = std::chrono::steady_clock::now();
  if (now - last_feedback_time_ > feedback_timeout_) {
    if (active_move_goal_) {
      auto result = std::make_shared<MoveXYZ::Result>();
      result->success = false;
      result->message = "Action timed out";
      active_move_goal_->abort(result);
      active_move_goal_.reset();
    }
    if (active_gripper_goal_) {
      auto result = std::make_shared<Gripper::Result>();
      result->success = false;
      result->message = "Action timed out";
      active_gripper_goal_->abort(result);
      active_gripper_goal_.reset();
    }
  }
}

rclcpp_action::GoalResponse
ArduinoNode::handle_gripper_goal(const rclcpp_action::GoalUUID &,
                                 std::shared_ptr<const Gripper::Goal>) {
  if (active_gripper_goal_ != nullptr) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArduinoNode::handle_gripper_cancel(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>>) {
  handle_stop_command();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArduinoNode::handle_gripper_accepted(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>> goal_handle) {
  execute_gripper(goal_handle);
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArduinoNode>());
  rclcpp::shutdown();
  return 0;
}
