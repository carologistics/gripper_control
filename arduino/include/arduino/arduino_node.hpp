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

#ifndef ARDUINO_NODE_HPP
#define ARDUINO_NODE_HPP

#include "arduino/action/calibrate.hpp"
#include "arduino/action/gripper.hpp"
#include "arduino/action/home.hpp"
#include "arduino/action/move_xyz.hpp"
#include "arduino/commands.h" // Add this include
#include "arduino/msg/status.hpp"
#include "arduino/serial_port.hpp"
#include "arduino/srv/get_status.hpp"
#include "arduino/srv/reset_device.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <memory>
#include <string>

class ArduinoNode : public rclcpp::Node {
public:
  using Home = arduino::action::Home;
  using Calibrate = arduino::action::Calibrate;

  ArduinoNode();
  ~ArduinoNode();

private:
  using MoveXYZ = arduino::action::MoveXYZ;
  using Gripper = arduino::action::Gripper;

  // Move retry count initialization to header
  int retry_count_{0};
  const int MAX_RETRIES{3};
  std::chrono::steady_clock::time_point last_reconnect_attempt_;
  const std::chrono::seconds reconnect_interval_{5};

  void timer_callback();
  void handle_serial_message(const std::string &msg);
  void handle_disconnect();
  void handle_error(const std::string &error_msg);
  bool attempt_reconnect();

  // Action server callbacks
  rclcpp_action::GoalResponse
  handle_home_goal(const rclcpp_action::GoalUUID &,
                   std::shared_ptr<const Home::Goal>);
  rclcpp_action::CancelResponse handle_home_cancel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<Home>>);
  void handle_home_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<Home>>);

  // Similar callbacks for calibrate action
  rclcpp_action::GoalResponse
  handle_calibrate_goal(const rclcpp_action::GoalUUID &,
                        std::shared_ptr<const Calibrate::Goal>);
  rclcpp_action::CancelResponse handle_calibrate_cancel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<Calibrate>>);
  void handle_calibrate_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<Calibrate>>);

  // Movement control
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  std::unique_ptr<SerialPort> port_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  bool wp_sensed_ = false;
  bool wp_sensor_enable_ = true;
  bool wp_sensor_analog_ = true;
  int wp_sensor_pin_ = 0;
  float wp_sensor_analog_threshold_ = 0.7;

  // Action servers
  rclcpp_action::Server<Home>::SharedPtr home_action_server_;
  rclcpp_action::Server<Calibrate>::SharedPtr calibrate_action_server_;
  rclcpp_action::Server<MoveXYZ>::SharedPtr move_xyz_action_server_;
  rclcpp_action::Server<Gripper>::SharedPtr gripper_action_server_;

  // Movement subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  // Command handlers
  void handle_move_command(float linear_x, float angular_z);
  void handle_stop_command();
  void send_arduino_command(const std::string &cmd);

  OnSetParametersCallbackHandle::SharedPtr param_callback_;
  rcl_interfaces::msg::SetParametersResult
  parameterCallback(const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      diagnostics_pub_;
  void publish_diagnostics();

  struct CommStats {
    int bytes_sent{0};
    int bytes_received{0};
    int errors{0};
    std::chrono::steady_clock::time_point last_msg_time;
  } comm_stats_;

  enum class DeviceState {
    DISCONNECTED,
    CONNECTING,
    INITIALIZING,
    OPERATIONAL,
    ERROR
  };
  DeviceState device_state_{DeviceState::DISCONNECTED};

  enum class ArduinoStatus {
    IDLE = 0,
    MOVING = 1,
    ERROR_OUT_OF_RANGE_X = 2,
    ERROR_OUT_OF_RANGE_Y = 3,
    ERROR_OUT_OF_RANGE_Z = 4
  };

  ArduinoStatus arduino_status_{ArduinoStatus::IDLE};

  struct Status {
    float x_position{0.0f}; // Combined with old Position struct
    float y_position{0.0f};
    float z_position{0.0f};
    bool gripper_closed{false};
    float x_max{0.0f};
    float y_max{0.0f};
    float z_max{0.0f};
    bool final{true};
    uint32_t msgid{0};
    uint32_t cmd_msgid{0}; // Track commands separately
    uint32_t status{0};
  } status_;

  // Service servers
  rclcpp::Service<arduino::srv::GetStatus>::SharedPtr get_status_service_;
  rclcpp::Service<arduino::srv::ResetDevice>::SharedPtr reset_device_service_;

  // Service handlers
  void handle_get_status(
      const std::shared_ptr<arduino::srv::GetStatus::Request>,
      std::shared_ptr<arduino::srv::GetStatus::Response> response);
  void handle_reset_device(
      const std::shared_ptr<arduino::srv::ResetDevice::Request>,
      std::shared_ptr<arduino::srv::ResetDevice::Response> response);

  // Status publisher
  rclcpp::Publisher<arduino::msg::Status>::SharedPtr status_pub_;

  // Add new action handlers
  rclcpp_action::GoalResponse
  handle_move_xyz_goal(const rclcpp_action::GoalUUID &,
                       std::shared_ptr<const MoveXYZ::Goal>);
  rclcpp_action::CancelResponse handle_move_xyz_cancel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveXYZ>>);
  void handle_move_xyz_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveXYZ>>);
  void execute_move_xyz(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveXYZ>>);

  rclcpp_action::GoalResponse
  handle_gripper_goal(const rclcpp_action::GoalUUID &,
                      std::shared_ptr<const Gripper::Goal>);
  rclcpp_action::CancelResponse handle_gripper_cancel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>>);
  void handle_gripper_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>>);
  void execute_gripper(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>>);

  // Add goal tracking
  std::shared_ptr<rclcpp_action::ServerGoalHandle<MoveXYZ>> active_move_goal_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>>
      active_gripper_goal_;

  // Add helper methods for goal handling
  void check_goal_progress();
  void publish_move_xyz_feedback();
  void publish_gripper_feedback();

  // Add parameter declarations
  void declare_parameters();

  // Add config parameters
  struct Config {
    bool enable_tf_broadcast{true};
    std::string base_frame{"base_link"};
    std::string waypoint_frame{"waypoint"};
    double status_frequency{1.0};
  } config_;

  // Add cleanup helper
  void cleanup();

  // Add goal tracking and feedback handling
  void handle_feedback_updates();
  void check_action_timeouts();
  std::chrono::steady_clock::time_point last_feedback_time_;
  const std::chrono::seconds feedback_timeout_{5}; // 5 second timeout

  // Add new helper functions for message handling
  void parse_position_and_gripper(std::istringstream &iss);
  void handle_goals_completed();
  void handle_goals_error(); // Add this function too
};
#endif // ARDUINO_NODE_HPP
