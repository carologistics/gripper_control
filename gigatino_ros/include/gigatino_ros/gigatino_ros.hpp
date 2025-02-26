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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include <boost/asio.hpp>

#include <msgpack.hpp>

#include "gigatino_msgs/action/calibrate.hpp"
#include "gigatino_msgs/action/home.hpp"
#include "gigatino_msgs/action/move.hpp"
#include "gigatino_msgs/msg/feedback.hpp"

namespace gigatino_ros {
constexpr size_t BUFFER_SIZE = 1024;
class GigatinoROS : public rclcpp_lifecycle::LifecycleNode {
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using Home = gigatino_msgs::action::Home;
  using Calibrate = gigatino_msgs::action::Calibrate;
  using Move = gigatino_msgs::action::Move;
  using Feedback = gigatino_msgs::msg::Feedback;

public:
  // **New constructor required for component registration**
  explicit GigatinoROS(const rclcpp::NodeOptions &options);

private:
  enum GigatinoResult { SUCCESS, FAILED, IGNORED, TIMEOUT, CANCELLED };
  void publish();
  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

  rclcpp_action::GoalResponse
  handle_home_goal(const rclcpp_action::GoalUUID &,
                   std::shared_ptr<const Home::Goal>);

  rclcpp_action::CancelResponse handle_home_cancel(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<Home>>);

  boost::asio::io_service io_service_;
  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint recv_endpoint_;
  boost::asio::ip::udp::endpoint send_endpoint_;

  std::string remote_ip_addr_;

  std::thread io_thread_;
  bool io_running_;

  size_t max_send_attempts_;
  std::chrono::milliseconds command_timeout_;

  void stop_io_service();
  void start_io_service();

  // for protecting everything below
  std::mutex feedback_mtx_;
  Feedback current_feedback_;
  bool action_running_;
  bool cancel_action_ = true;
  int curr_command_index_;
  std::condition_variable action_cv_;
  // end protection of feedback_mtx_

  std::array<char, BUFFER_SIZE> recv_buffer_;

  void start_receive();

  void cancel_action(bool stop = false);
  void close_socket();

  void unpack_msgpack_data(size_t size);

  GigatinoROS::GigatinoResult
  send_udp_message(std::map<std::string, msgpack::object> &data);

  void print_buffer(const msgpack::sbuffer &buffer);

  template <typename GoalHandleType, typename ResultType>
  void handle_result(std::shared_ptr<GoalHandleType> goal_handle,
                     GigatinoResult result_code) {
    auto result = std::make_shared<ResultType>();

    switch (result_code) {
    case GigatinoResult::SUCCESS:
      goal_handle->succeed(result);
      break;
    case GigatinoResult::FAILED: {
      result->message = "Command failed";
      goal_handle->abort(result);
      break;
    }
    case GigatinoResult::IGNORED: {
      result->message = "Command not acknowledged";
      goal_handle->abort(result);
      break;
    }
    case GigatinoResult::TIMEOUT: {
      result->message = "Timeout while executing";
      goal_handle->abort(result);
      break;
    }
    case GigatinoResult::CANCELLED: {
      result->message = "Cancelled by other action";
      goal_handle->abort(result);
      break;
    }
    default: {
      result->message = "Unknown result";
      goal_handle->abort(result);
      break;
    }
    }
  }

  void handle_home_accepted(
      std::shared_ptr<rclcpp_action::ServerGoalHandle<Home>> goal_handle);

  rclcpp::Publisher<Feedback>::SharedPtr feedback_pub_;
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp_action::Server<Home>::SharedPtr home_action_server_;
  rclcpp_action::Server<Calibrate>::SharedPtr calibrate_action_server_;
  rclcpp_action::Server<Move>::SharedPtr move_action_server_;
  // rclcpp_action::Server<Gripper>::SharedPtr gripper_action_server_;
};
} // namespace gigatino_ros
