// Copyright (c) 2025 Carologistics
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

#include "lifecycle_msgs/msg/transition.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <boost/asio.hpp>

#include <msgpack.hpp>

#include "gigatino_msgs/action/calibrate.hpp"
#include "gigatino_msgs/action/gripper.hpp"
#include "gigatino_msgs/action/home.hpp"
#include "gigatino_msgs/action/move.hpp"
#include "gigatino_msgs/action/stop.hpp"
#include "gigatino_msgs/msg/feedback.hpp"
#include "gigatino_msgs/msg/status_code.hpp"

namespace gigatino_ros {
constexpr size_t BUFFER_SIZE = 1024;
class GigatinoROS : public nav2_util::LifecycleNode {
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Home = gigatino_msgs::action::Home;
  using Calibrate = gigatino_msgs::action::Calibrate;
  using Move = gigatino_msgs::action::Move;
  using Stop = gigatino_msgs::action::Stop;
  using Gripper = gigatino_msgs::action::Gripper;
  using Feedback = gigatino_msgs::msg::Feedback;

public:
  // **New constructor required for component registration**
  explicit GigatinoROS(const rclcpp::NodeOptions &options);
  ~GigatinoROS();

private:
  void publish();
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

  uint8_t current_command_result_;

  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint recv_endpoint_;
  boost::asio::ip::udp::endpoint send_endpoint_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  float gripper_open_pos_;
  float gripper_close_pos_;
  float max_x_;
  float max_z_;
  float max_yaw_;
  float min_z_;
  float min_x_;
  float min_yaw_;
  float bound_threshold_z_;
  float bound_threshold_x_;
  float bound_threshold_yaw_;

  float target_mot_x_, target_mot_yaw_, target_mot_z_;

  std::thread io_thread_;
  bool io_running_;

  size_t max_send_attempts_;
  std::chrono::milliseconds command_timeout_;

  void stop_io_service();
  void start_io_service();

  // for protecting everything below
  std::mutex feedback_mtx_;
  Feedback current_feedback_;
  bool action_running_ = false;
  bool cancel_action_ = false;
  int curr_command_index_;
  std::condition_variable action_cv_;
  // end protection of feedback_mtx_
  rclcpp::TimerBase::SharedPtr timer_;
  double feedback_time_;

  std::array<char, BUFFER_SIZE> recv_buffer_;

  std::string namespace_;
  std::string tf_prefix_;

  void start_receive();

  void cancel_action(bool stop = false);
  void close_socket();

  void unpack_msgpack_data(size_t size);

  uint8_t send_udp_message(std::map<std::string, msgpack::object> &data);

  void print_buffer(const msgpack::sbuffer &buffer);

  template <typename GoalHandleType, typename ResultType>
  void handle_result(std::shared_ptr<GoalHandleType> goal_handle,
                     uint8_t result_code) {
    auto result = std::make_shared<ResultType>();
    result->status_code = result_code;

    switch (result_code) {
    case gigatino_msgs::msg::StatusCode::SUCCESS:
      RCLCPP_INFO(get_logger(), "[uuid %s] Action done",
                  rclcpp_action::to_string(goal_handle->get_goal_id()).c_str());
      goal_handle->succeed(result);
      break;
    case gigatino_msgs::msg::StatusCode::GENERIC_FAILURE: {
      result->message = "Command failed";
      RCLCPP_ERROR(get_logger(), "[uuid %s], %s",
                   rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
                   result->message.c_str());
      goal_handle->abort(result);
      break;
    }
    case gigatino_msgs::msg::StatusCode::EMERGENCY_STOP: {
      result->message = "Emergency stop";
      RCLCPP_ERROR(get_logger(), "[uuid %s], %s",
                   rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
                   result->message.c_str());
      goal_handle->abort(result);
      break;
    }
    case gigatino_msgs::msg::StatusCode::COMM_LOST: {
      result->message = "Command not acknowledged";
      RCLCPP_ERROR(get_logger(), "[uuid %s], %s",
                   rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
                   result->message.c_str());
      goal_handle->abort(result);
      break;
    }
    case gigatino_msgs::msg::StatusCode::TIMEOUT: {
      result->message = "Timeout while executing";
      RCLCPP_ERROR(get_logger(), "[uuid %s], %s",
                   rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
                   result->message.c_str());
      goal_handle->abort(result);
      break;
    }
    case gigatino_msgs::msg::StatusCode::CANCELLED: {
      result->message = "Cancelled by other action";
      RCLCPP_WARN(get_logger(), "[uuid %s], %s",
                  rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
                  result->message.c_str());
      goal_handle->abort(result);
      break;
    }
    default: {
      result->message = "Unknown result";
      RCLCPP_ERROR(get_logger(), "[uuid %s], %s",
                   rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
                   result->message.c_str());
      goal_handle->abort(result);
      break;
    }
    }
  }

  template <typename ActionT, typename GoalT>
  std::shared_ptr<rclcpp_action::Server<ActionT>>
  setup_server(const std::string &action_name,
               std::function<void(
                   msgpack::zone &,
                   std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>)>
                   goal_processor,
               const rcl_action_server_options_t &server_options,
               rclcpp::CallbackGroup::SharedPtr cb_group) {

    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;

    return rclcpp_action::create_server<ActionT>(
        this, action_name,
        // Goal handler
        [this](const rclcpp_action::GoalUUID &uuid,
               std::shared_ptr<const GoalT>) {
          if constexpr (std::is_same_v<GoalT, Calibrate::Goal> or
                        std::is_same_v<GoalT, Gripper::Goal> or
                        std::is_same_v<GoalT, Stop::Goal>) {
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          } else {
            std::scoped_lock lk(feedback_mtx_);
            if (current_feedback_.referenced) {
              return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            } else {
              RCLCPP_ERROR(get_logger(),
                           "[uuid %s] Rejected, system not calibrated",
                           rclcpp_action::to_string(uuid).c_str());
              return rclcpp_action::GoalResponse::REJECT;
            }
          }
        },
        // Cancel handler
        [this](std::shared_ptr<GoalHandle>) -> rclcpp_action::CancelResponse {
          cancel_action(true);
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        // Accepted handler
        [this, action_name,
         goal_processor](std::shared_ptr<GoalHandle> goal_handle) {
          RCLCPP_INFO(
              get_logger(), "[uuid %s] Starting action %s",
              rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
              action_name.c_str());
          msgpack::zone zone;
          goal_processor(zone, goal_handle);
        },
        server_options, cb_group);
  }

  rclcpp::Publisher<Feedback>::SharedPtr feedback_pub_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp_action::Server<Home>::SharedPtr home_action_server_;
  rclcpp_action::Server<Calibrate>::SharedPtr calibrate_action_server_;
  rclcpp_action::Server<Move>::SharedPtr move_action_server_;
  rclcpp_action::Server<Gripper>::SharedPtr gripper_action_server_;
  rclcpp_action::Server<Stop>::SharedPtr stop_action_server_;
};
} // namespace gigatino_ros
