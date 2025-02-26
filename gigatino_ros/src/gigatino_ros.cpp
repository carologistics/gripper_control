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

#include "gigatino_ros/gigatino_ros.hpp"

#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;
using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

using boost::asio::ip::udp;

using namespace gigatino_ros;

// **New constructor required for component registration**
GigatinoROS::GigatinoROS(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("gigatino_ros", "", options),
      socket_(io_service_) { // Change "gigatino_ros" to your actual node name
                             // Initialization code
  // declare_parameter("remote_ip_address", "192.168.1.100");
  declare_parameter("remote_ip_address", "127.0.0.1");
  declare_parameter("send_port", 8888);
  declare_parameter("recv_port", 8889);
  declare_parameter("command_timeout_ms", 15000);
  declare_parameter("max_send_attempts", 3);
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

void GigatinoROS::publish() {
  // pub_->publish(std::move(msg));
}
CallbackReturn GigatinoROS::on_configure(const rclcpp_lifecycle::State &) {
  // This callback is supposed to be used for initialization and
  // configuring purposes.
  RCLCPP_INFO(get_logger(), "on_configure() is called.");
  createBond();
  command_timeout_ =
      std::chrono::milliseconds(get_parameter("command_timeout_ms").as_int());
  max_send_attempts_ = get_parameter("max_send_attempts").as_int();

  remote_ip_addr_ = get_parameter("remote_ip_address").as_string();
  // recv_endpoint_ =
  // udp::endpoint(boost::asio::ip::address::from_string("192.168.1.2"),
  // get_parameter("recv_port").as_int());
  recv_endpoint_ = udp::endpoint(boost::asio::ip::address_v4::loopback(),
                                 get_parameter("recv_port").as_int());

  // send_endpoint_ =
  // udp::endpoint(boost::asio::ip::address::from_string("192.168.1.100"),
  // get_parameter("send_port").as_int());
  send_endpoint_ = udp::endpoint(boost::asio::ip::address_v4::loopback(),
                                 get_parameter("send_port").as_int());

  rcl_action_server_options_t server_options =
      rcl_action_server_get_default_options();
  // rmw_qos_profile_t qos_profile;
  // qos_profile.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
  // qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
  // server_options.goal_service_qos = qos_profile;
  // server_options.cancel_service_qos = qos_profile;
  // server_options.result_service_qos = qos_profile;

  feedback_pub_ = rclcpp::create_publisher<Feedback>(
      this, "feedback", rclcpp::QoS(1).best_effort().durability_volatile());

  home_action_server_ = rclcpp_action::create_server<Home>(
      this, "gigatino/home",
      // Lambda for goal handler
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const Home::Goal> goal)
          -> rclcpp_action::GoalResponse {
        RCLCPP_INFO(get_logger(), "I ACCEPT AND EXECUTE");
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // Lambda for cancel handler
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Home>> goal_handle)
          -> rclcpp_action::CancelResponse {
        cancel_action(true);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Lambda for accepted handler
      [this](
          std::shared_ptr<rclcpp_action::ServerGoalHandle<Home>> goal_handle) {
        msgpack::zone zone;
        std::map<std::string, msgpack::object> data = {
            {"command", msgpack::object("HOME", zone)},
        };
        handle_result<rclcpp_action::ServerGoalHandle<Home>, Home::Result>(
            goal_handle, send_udp_message(data));
      },
      server_options, cb_group_);

  calibrate_action_server_ = rclcpp_action::create_server<Calibrate>(
      this, "gigatino/calibrate",
      // Lambda for goal handler
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const Calibrate::Goal> goal)
          -> rclcpp_action::GoalResponse {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // Lambda for cancel handler
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Calibrate>>
                 goal_handle) -> rclcpp_action::CancelResponse {
        cancel_action(true);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Lambda for accepted handler
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Calibrate>>
                 goal_handle) {
        msgpack::zone zone;
        std::map<std::string, msgpack::object> data = {
            {"command", msgpack::object("CALIBRATE", zone)},
        };
        handle_result<rclcpp_action::ServerGoalHandle<Calibrate>,
                      Calibrate::Result>(goal_handle, send_udp_message(data));
      },
      server_options, cb_group_);

  move_action_server_ = rclcpp_action::create_server<Move>(
      this, "gigatino/move",
      // Lambda for goal handler
      [this](const rclcpp_action::GoalUUID &uuid,
             std::shared_ptr<const Move::Goal> goal)
          -> rclcpp_action::GoalResponse {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      // Lambda for cancel handler
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>> goal_handle)
          -> rclcpp_action::CancelResponse {
        cancel_action(true);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Lambda for accepted handler
      [this](
          std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>> goal_handle) {
        msgpack::zone zone;
        // TODO: compute right abolute positions for axis
        std::map<std::string, msgpack::object> data = {
            {"command", msgpack::object("MOVE", zone)},
            {"target_mot_x",
             msgpack::object(1.5, zone)}, // absolute position of axis
            {"target_mot_yaw",
             msgpack::object(1.5, zone)}, // absolute position of axis
            {"target_mot_z",
             msgpack::object(1.5, zone)}, // absolute position of axis
        };
        handle_result<rclcpp_action::ServerGoalHandle<Move>, Move::Result>(
            goal_handle, send_udp_message(data));
      },
      server_options, cb_group_);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}
void GigatinoROS::stop_io_service() {
  if (io_running_) {
    io_running_ = false;
    io_service_.stop(); // Stop async operations
    io_service_.reset();
    if (io_thread_.joinable()) {
      io_thread_.join(); // Wait for thread to finish
    }
  }
}
void GigatinoROS::start_io_service() {
  if (!io_running_) {
    io_running_ = true;
    io_thread_ = std::thread([this]() {
      while (io_running_) {
        io_service_.run();
        io_service_.reset();
      }
    });
  }
}

CallbackReturn GigatinoROS::on_activate(const rclcpp_lifecycle::State &state) {
  LifecycleNode::on_activate(state);
  // init socket
  socket_.open(udp::v4());
  socket_.bind(recv_endpoint_);
  RCLCPP_INFO(get_logger(), "Socket bound to interface: %s, port: %d",
              recv_endpoint_.address().to_string().c_str(),
              recv_endpoint_.port());
  start_io_service();
  start_receive();
  RCLCPP_INFO(get_logger(), "on_activate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void GigatinoROS::close_socket() {
  if (socket_.is_open()) {
    boost::system::error_code ec;
    socket_.shutdown(boost::asio::ip::udp::socket::shutdown_both, ec);
    socket_.close(ec);
    if (ec) {
      RCLCPP_WARN(get_logger(), "Error closing socket: %s",
                  ec.message().c_str());
    } else {
      RCLCPP_INFO(get_logger(), "Socket closed successfully.");
    }
  }
}
CallbackReturn
GigatinoROS::on_deactivate(const rclcpp_lifecycle::State &state) {
  LifecycleNode::on_deactivate(state);
  close_socket();
  stop_io_service();
  destroyBond();
  RCLCPP_INFO(get_logger(), "on_deactivate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

CallbackReturn GigatinoROS::on_cleanup(const rclcpp_lifecycle::State &) {
  // In our cleanup phase, we release the shared pointers to the
  // timer and publisher. These entities are no longer available
  // and our node is "clean".
  // timer_.reset();
  // pub_.reset();

  RCLCPP_INFO(get_logger(), "on cleanup is called.");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void GigatinoROS::start_receive() {
  // Start receiving asynchronously
  socket_.async_receive_from(
      boost::asio::buffer(recv_buffer_), recv_endpoint_,
      [this](boost::system::error_code ec, std::size_t bytes_received) {
        RCLCPP_DEBUG(get_logger(), "received %lu bytes", bytes_received);
        if (!ec) {
          {
            std::scoped_lock lock(feedback_mtx_);
            unpack_msgpack_data(bytes_received);
            feedback_pub_->publish(current_feedback_);
            // TODO: compute transform from abs positions
          }
          action_cv_.notify_all();
        }
        // After receiving, continue to listen for more data
        if (io_running_) {
          start_receive();
        }
      });
}

void GigatinoROS::unpack_msgpack_data(size_t size) {
  try {
    // Deserialize the MessagePack data
    msgpack::unpacked msg;
    msgpack::unpack(msg, recv_buffer_.data(), size);

    msgpack::object obj = msg.get();

    // Check if the object is a map using the new API (v2)
    if (obj.type == msgpack::type::MAP) {
      // Iterate through the map (obj.via.map) using the correct API
      for (msgpack::object_kv &kv : obj.via.map) {
        const std::string &key = kv.key.as<std::string>();
        msgpack::object &value = kv.val;

        // Check and process each key-value pair
        if (key == "stepper_positions" && value.type == msgpack::type::ARRAY) {
          for (size_t i = 0; i < value.via.array.size && i < 4; ++i) {
            current_feedback_.stepper_positions[i] =
                value.via.array.ptr[i].as<float>(); // Corrected access pattern
          }
        } else if (key == "servo_positions" &&
                   value.type == msgpack::type::ARRAY) {
          for (size_t i = 0; i < value.via.array.size && i < 2; ++i) {
            current_feedback_.servo_positions[i] =
                value.via.array.ptr[i].as<float>(); // Corrected access pattern
          }
        } else if (key == "stepper_directions" &&
                   value.type == msgpack::type::ARRAY) {
          for (size_t i = 0; i < value.via.array.size && i < 4; ++i) {
            current_feedback_.stepper_directions[i] =
                value.via.array.ptr[i].as<bool>(); // Corrected access pattern
          }
        } else if (key == "stepper_endstops" &&
                   value.type == msgpack::type::ARRAY) {
          for (size_t i = 0; i < value.via.array.size && i < 4; ++i) {
            current_feedback_.stepper_endstops[i] =
                value.via.array.ptr[i].as<bool>(); // Corrected access pattern
          }
        } else if (key == "wp_sensor" && value.type == msgpack::type::BOOLEAN) {
          current_feedback_.wp_sensor = value.as<bool>();
        } else if (key == "busy" && value.type == msgpack::type::BOOLEAN) {
          current_feedback_.busy = value.as<bool>();
        } else if (key == "referenced" &&
                   value.type == msgpack::type::BOOLEAN) {
          current_feedback_.referenced = value.as<bool>();
        } else if (key == "command_index" &&
                   value.type == msgpack::type::POSITIVE_INTEGER) {
          current_feedback_.command_index = value.as<int>();
        }
      }
    } else {
      RCLCPP_ERROR(
          get_logger(),
          "Error while receiving current_feedback_. Root object is not a map!");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "Error while unpacking MessagePack: %s",
                 e.what());
  }
}

CallbackReturn GigatinoROS::on_shutdown(const rclcpp_lifecycle::State &state) {
  close_socket();
  stop_io_service();
  destroyBond();
  RCLCPP_INFO(get_logger(), "on shutdown is called from state %s.",
              state.label().c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void GigatinoROS::cancel_action(bool stop) {
  {
    std::lock_guard<std::mutex> lk(feedback_mtx_);
    if (action_running_) {
      RCLCPP_WARN(get_logger(), "Cancelling action");
    }
    cancel_action_ = true;
  }
  action_cv_.notify_all();
  if (stop) { // send unacknowledged STOP message
    msgpack::zone zone;
    std::map<std::string, msgpack::object> data = {
        {"command", msgpack::object("STOP", zone)},
    };
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, data);
    socket_.send_to(boost::asio::buffer(sbuf.data(), sbuf.size()),
                    send_endpoint_);
  }
}

void GigatinoROS::print_buffer(const msgpack::sbuffer &buffer) {
  std::ostringstream oss;
  // Access the underlying vector through the data and size methods
  const char *data = buffer.data();
  size_t size = buffer.size();

  for (size_t i = 0; i < size; ++i) {
    oss << std::hex << std::setw(2) << std::setfill('0')
        << (int)(unsigned char)data[i] << " ";
    if ((i + 1) % 16 == 0) {
      oss << "\n";
    }
  }
  RCLCPP_INFO(get_logger(), "Buffer contents:\n%s", oss.str().c_str());
}

GigatinoROS::GigatinoResult
GigatinoROS::send_udp_message(std::map<std::string, msgpack::object> &data) {
  // this function should be called after terminating concurrently running
  // actions.
  cancel_action();
  {
    std::unique_lock<std::mutex> lock(feedback_mtx_);
    // otherwise, wait for it to finish
    RCLCPP_INFO(get_logger(), "Waiting for action to finish");
    action_cv_.wait(lock, [&]() {
      // Check if the command index has increased and the feedback is not busy
      return !action_running_;
    });
    RCLCPP_INFO(get_logger(), "Done, start my action");
    action_running_ = true;
    curr_command_index_ = current_feedback_.command_index + 1;
    msgpack::zone zone;
    data["command_index"] = msgpack::object(curr_command_index_, zone);
    cancel_action_ = false;
  }
  msgpack::sbuffer sbuf;

  // Serialize the std::map directly into MessagePack format
  msgpack::pack(sbuf, data);
  bool action_acknowledged = false;
  size_t send_attempts = 0;
  while (!action_acknowledged) {
    if (send_attempts > max_send_attempts_) {
      action_running_ = false;
      return GigatinoResult::IGNORED;
    }
    send_attempts++;
    try {
      socket_.send_to(boost::asio::buffer(sbuf.data(), sbuf.size()),
                      send_endpoint_);

      print_buffer(sbuf);
      RCLCPP_INFO(get_logger(), "Message sent");
      std::unique_lock<std::mutex> lock(feedback_mtx_);
      action_cv_.wait_for(lock, 1s, [&]() {
        return cancel_action_ ||
               (curr_command_index_ == current_feedback_.command_index);
      });
      if (cancel_action_) {
        return GigatinoResult::CANCELLED;
      }
      action_acknowledged =
          (curr_command_index_ == current_feedback_.command_index);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "Error: %s", e.what());
    }
  }
  std::unique_lock<std::mutex> lock(feedback_mtx_);
  RCLCPP_INFO(get_logger(), "Action sent, wait for it to finish");
  // action already done
  if (!current_feedback_.busy) {
    action_running_ = false;
    return GigatinoResult::SUCCESS;
  }
  // otherwise, wait for it to finish
  action_cv_.wait_for(lock, command_timeout_, [&]() {
    // Check if the command index has increased and the feedback is not busy
    return cancel_action_ || !current_feedback_.busy;
  });
  RCLCPP_INFO(get_logger(), "took some time, now it is done");
  action_running_ = false;
  if (cancel_action_) {
    return GigatinoResult::CANCELLED;
  }
  if (current_feedback_.busy) {
    return GigatinoResult::TIMEOUT;
  }
  return GigatinoResult::SUCCESS;
}
RCLCPP_COMPONENTS_REGISTER_NODE(gigatino_ros::GigatinoROS)
