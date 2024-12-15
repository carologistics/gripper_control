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
#include "arduino/action/home.hpp"
#include "arduino/serial_port.hpp"
#include "arduino/srv/get_status.hpp"
#include "arduino/srv/reset_device.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <diagnostic_updater/diagnostic_updater.hpp>

#include <memory>
#include <string>

class ArduinoNode : public rclcpp::Node {
public:
  using Home = arduino::action::Home;
  using Calibrate = arduino::action::Calibrate;

  ArduinoNode();
  ~ArduinoNode();

private:
  void timer_callback();
  void handle_serial_message(const std::string &msg);
  void handle_disconnect();
  void handle_error(const std::string &error_msg);
  int retry_count_ = 0;
  const int MAX_RETRIES = 3;
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
};
#endif // ARDUINO_NODE_HPP
