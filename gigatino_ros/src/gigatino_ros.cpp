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
#include <cmath>
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
  declare_parameter("local_ip_address", "127.0.0.1");
  declare_parameter("remote_ip_address", "127.0.0.1");
  declare_parameter("tf_prefix", "");
  declare_parameter("send_port", 8888);
  declare_parameter("recv_port", 8889);
  declare_parameter("command_timeout_ms", 15000);
  declare_parameter("max_send_attempts", 3);
  declare_parameter("gripper_open_pos", 110.0);
  declare_parameter("gripper_close_pos", 0.0);
  declare_parameter("z_min", 0.0);
  declare_parameter("z_max", 145.0);
  declare_parameter("x_min", 0.0);
  declare_parameter("x_max", 245.0);
  declare_parameter("yaw_min", 0.0);
  declare_parameter("yaw_max", 80.0);
  cb_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
}

CallbackReturn GigatinoROS::on_configure(const rclcpp_lifecycle::State &) {
  RCLCPP_DEBUG(get_logger(), "on_configure() is called.");
  createBond();
  tf_prefix_ = get_parameter("tf_prefix").as_string();
  command_timeout_ =
      std::chrono::milliseconds(get_parameter("command_timeout_ms").as_int());
  max_send_attempts_ = get_parameter("max_send_attempts").as_int();
  gripper_open_pos_ = get_parameter("gripper_open_pos").as_double();
  gripper_close_pos_ = get_parameter("gripper_close_pos").as_double();
  max_x_ = get_parameter("x_max").as_double();
  min_x_ = get_parameter("x_min").as_double();
  bound_threshold_x_ = get_parameter("x_bound_threshold").as_double();
  max_yaw_ = get_parameter("yaw_max").as_double();
  min_yaw_ = get_parameter("yaw_min").as_double();
  bound_threshold_yaw_ = get_parameter("yaw_bound_threshold").as_double();
  max_z_ = get_parameter("z_max").as_double();
  min_z_ = get_parameter("z_min").as_double();
  bound_threshold_z_ = get_parameter("z_bound_threshold").as_double();
  std::string local_ip_addr = get_parameter("local_ip_address").as_string();
  std::string remote_ip_addr = get_parameter("remote_ip_address").as_string();
  RCLCPP_INFO(get_logger(), "local address %s and port %li",
              local_ip_addr.c_str(), get_parameter("recv_port").as_int());
  recv_endpoint_ =
      udp::endpoint(boost::asio::ip::address::from_string(local_ip_addr),
                    get_parameter("recv_port").as_int());

  RCLCPP_INFO(get_logger(), "Remote address %s and port %li",
              remote_ip_addr.c_str(), get_parameter("send_port").as_int());
  send_endpoint_ =
      udp::endpoint(boost::asio::ip::address::from_string(remote_ip_addr),
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
      this, "gigatino/feedback",
      rclcpp::QoS(1).best_effort().durability_volatile());

  home_action_server_ = setup_server<Home, Home::Goal>(
      "gigatino/home",
      [this](msgpack::zone &zone,
             std::shared_ptr<rclcpp_action::ServerGoalHandle<Home>> g)
          -> std::map<std::string, msgpack::object> {
        (void)g;
        return {
            {"command", msgpack::object("MOVE", zone)},
            {"target_mot_x", msgpack::object(0.0f, zone)},
            {"target_mot_yaw", msgpack::object(52.0f, zone)},
            {"target_mot_z", msgpack::object(0.0f, zone)},
        };
      },
      server_options, cb_group_);

  calibrate_action_server_ = setup_server<Calibrate, Calibrate::Goal>(
      "gigatino/calibrate",
      [this](msgpack::zone &zone,
             std::shared_ptr<rclcpp_action::ServerGoalHandle<Calibrate>> g_h)
          -> std::map<std::string, msgpack::object> {
        std::map<std::string, msgpack::object> data = {
            {"command", msgpack::object("CALIBRATE", zone)},
            {"target_mot_x", msgpack::object(0.0f, zone)},
        };
        GigatinoResult res = send_udp_message(data);
        if (res != GigatinoResult::SUCCESS) {
          handle_result<rclcpp_action::ServerGoalHandle<Calibrate>,
                        Calibrate::Result>(g_h, res);
          return {};
        } else {
          return {
              {"command", msgpack::object("CALIBRATE", zone)},
          };
        }
      },
      server_options, cb_group_);

  move_action_server_ = rclcpp_action::create_server<Move>(
      this, "gigatino/move",
      // Goal handler
      [this](
          const rclcpp_action::GoalUUID &,
          std::shared_ptr<const Move::Goal> g) -> rclcpp_action::GoalResponse {
        try {
          geometry_msgs::msg::PoseStamped target_point;
          geometry_msgs::msg::PoseStamped goal_pose =
              geometry_msgs::msg::PoseStamped();
          goal_pose.header.frame_id = g->target_frame;
          goal_pose.pose.position.x = g->x; //-> for pointer
          goal_pose.pose.position.y = g->y;
          goal_pose.pose.position.z = g->z;
          goal_pose.header.stamp.sec = 0;
          goal_pose.header.stamp.nanosec = 0;
          tf_buffer_->transform(goal_pose, target_point,
                                tf_prefix_ + "gripper_home_origin");
          float z_abs = target_point.pose.position.z;
          geometry_msgs::msg::PoseStamped target_to_yaw;
          tf_buffer_->transform(goal_pose, target_to_yaw,
                                tf_prefix_ + "gripper_yaw_origin");
          geometry_msgs::msg::TransformStamped offset_end_effector =
              tf_buffer_->lookupTransform(tf_prefix_ + "gripper_x_dyn",
                                          tf_prefix_ + "gripper_end_effector",
                                          tf2::TimePointZero);
          geometry_msgs::msg::TransformStamped end_effector_to_yaw =
              tf_buffer_->lookupTransform(tf_prefix_ + "gripper_yaw_dyn",
                                          tf_prefix_ + "gripper_end_effector",
                                          tf2::TimePointZero);
          float d = end_effector_to_yaw.transform.translation.y;
          float T_distance =
              std::sqrt(std::pow(target_to_yaw.pose.position.x, 2) +
                        std::pow(target_to_yaw.pose.position.y, 2));
          float beta = acos(abs(d) / abs(T_distance));
          geometry_msgs::msg::TransformStamped x_origin_to_yaw_dyn =
              tf_buffer_->lookupTransform(tf_prefix_ + "gripper_x_origin",
                                          tf_prefix_ + "gripper_yaw_dyn",
                                          tf2::TimePointZero);
          float x_static = abs(x_origin_to_yaw_dyn.transform.translation.x) +
                           abs(offset_end_effector.transform.translation.x);
          float x_delta = T_distance * sin(beta);
          float x_abs = x_delta - x_static;
          float t_x = target_to_yaw.pose.position.x;
          float t_y = target_to_yaw.pose.position.y;
          float tetha;
          float alpha = atan(abs(t_x) / abs(t_y));
          if (t_y == 0) {
            tetha = 0;
          } else if (t_y >= 0) {
            tetha = beta - alpha;
          } else {
            tetha = (M_PI - beta - alpha) * -1;
          }
          float target_mot_y = -tetha * 180 / M_PI;
          {
            std::scoped_lock lk(feedback_mtx_);
            target_mot_x_ = x_abs * 1000;
            target_mot_yaw_ = target_mot_y;
            target_mot_z_ = z_abs * 1000;
            if ((min_x_ < target_mot_x_ + bound_threshold_x_ &&
                 target_mot_x_ - bound_threshold_x_ < max_x_) &&
                (min_z_ < target_mot_z_ + bound_threshold_z_ &&
                 target_mot_z_ - bound_threshold_z_ < max_z_) &&
                (min_yaw_ < target_mot_yaw_ + bound_threshold_yaw_ &&
                 target_mot_yaw_ - bound_threshold_yaw_ < max_yaw_)) {
              target_mot_x_ = std::clamp(target_mot_x_, min_x_, max_x_);
              target_mot_yaw_ = std::clamp(target_mot_yaw_, min_yaw_, max_yaw_);
              target_mot_z_ = std::clamp(target_mot_z_, min_z_, max_z_);
              RCLCPP_INFO(get_logger(), "x_abs : %.6f", target_mot_x_);
              RCLCPP_INFO(get_logger(), "yaw_angle : %.6f", target_mot_yaw_);
              RCLCPP_INFO(get_logger(), "z_abs : %.6f", target_mot_z_);
              return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            }
          }
        } catch (tf2::LookupException &e) {
          RCLCPP_ERROR(get_logger(), "Reject: %s", e.what());
          return rclcpp_action::GoalResponse::REJECT;
        } catch (tf2::InvalidArgumentException &e) {
          RCLCPP_ERROR(get_logger(), "Reject: %s", e.what());
          return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_ERROR(get_logger(), "x.abs : %.6f", target_mot_x_);
        RCLCPP_ERROR(get_logger(), "yaw_angle : %.6f", target_mot_yaw_);
        RCLCPP_ERROR(get_logger(), "z_abs : %.6f", target_mot_z_);
        return rclcpp_action::GoalResponse::REJECT;
      },
      // Cancel handler
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>>)
          -> rclcpp_action::CancelResponse {
        cancel_action(true);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      // Accepted handler
      [this](
          std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>> goal_handle) {
        msgpack::zone zone;
        std::map<std::string, msgpack::object> data;
        bool move_x_first = false;
        {
          std::scoped_lock lk(feedback_mtx_);
          move_x_first = target_mot_x_ < current_feedback_.stepper_positions[0];
          if (move_x_first) {
            data = {
                {"command", msgpack::object("MOVE", zone)},
                {"target_mot_x", msgpack::object(target_mot_x_, zone)},

            };
          } else {
            data = {
                {"command", msgpack::object("MOVE", zone)},
                {"target_mot_yaw", msgpack::object(target_mot_yaw_, zone)},
                {"target_mot_z", msgpack::object(target_mot_z_, zone)},

            };
          }
        }
        RCLCPP_INFO(
            get_logger(), "[uuid %s] Starting action %s",
            rclcpp_action::to_string(goal_handle->get_goal_id()).c_str(),
            "MOVE");
        GigatinoResult outcome = send_udp_message(data);
        if (outcome != GigatinoResult::SUCCESS) {
          handle_result<rclcpp_action::ServerGoalHandle<Move>, Move::Result>(
              goal_handle, outcome);
        } else {
          {
            std::scoped_lock lk(feedback_mtx_);
            if (move_x_first) {
              data = {
                  {"command", msgpack::object("MOVE", zone)},
                  {"target_mot_yaw", msgpack::object(target_mot_yaw_, zone)},
                  {"target_mot_z", msgpack::object(target_mot_z_, zone)},

              };
            } else {
              data = {
                  {"command", msgpack::object("MOVE", zone)},
                  {"target_mot_x", msgpack::object(target_mot_x_, zone)},

              };
            }
          }
          handle_result<rclcpp_action::ServerGoalHandle<Move>, Move::Result>(
              goal_handle, send_udp_message(data));
        }
      },
      server_options, cb_group_);

  gripper_action_server_ = setup_server<Gripper, Gripper::Goal>(
      "gigatino/gripper",
      [this](msgpack::zone &zone,
             std::shared_ptr<rclcpp_action::ServerGoalHandle<Gripper>> g_h)
          -> std::map<std::string, msgpack::object> {
        float target_servo_pos = gripper_close_pos_;
        if (g_h->get_goal()->open) {
          target_servo_pos = gripper_open_pos_;
        }
        return {
            {"command", msgpack::object("MOVE", zone)},
            {"target_servo_gripper", msgpack::object(target_servo_pos, zone)},
        };
      },
      server_options, cb_group_);

  stop_action_server_ = setup_server<Stop, Stop::Goal>(
      "gigatino/stop",
      [this](msgpack::zone &zone,
             std::shared_ptr<rclcpp_action::ServerGoalHandle<Stop>> g_h)
          -> std::map<std::string, msgpack::object> {
        (void)g_h;
        return {{"command", msgpack::object("STOP", zone)}};
      },
      server_options, cb_group_);
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}
void GigatinoROS::stop_io_service() {
  if (io_running_) {
    io_running_ = false;
    io_service_.stop();
    io_service_.reset();
    if (io_thread_.joinable()) {
      io_thread_.join();
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
  RCLCPP_DEBUG(get_logger(), "on_activate() is called.");

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
  RCLCPP_INFO(get_logger(), "destroy on deactivate");
  destroyBond();
  RCLCPP_DEBUG(get_logger(), "on_deactivate() is called.");

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

CallbackReturn GigatinoROS::on_cleanup(const rclcpp_lifecycle::State &) {
  home_action_server_.reset();
  calibrate_action_server_.reset();
  move_action_server_.reset();
  gripper_action_server_.reset();
  feedback_pub_.reset();
  cb_group_.reset();
  RCLCPP_DEBUG(get_logger(), "on cleanup is called.");
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
            geometry_msgs::msg::TransformStamped transf_yaw;
            transf_yaw.header.stamp = this->get_clock()->now();
            transf_yaw.header.frame_id = tf_prefix_ + "gripper_yaw_origin";
            transf_yaw.child_frame_id = tf_prefix_ + "gripper_yaw_dyn";
            geometry_msgs::msg::TransformStamped transf_x;
            transf_x.header.stamp = this->get_clock()->now();
            transf_x.header.frame_id = tf_prefix_ + "gripper_x_origin";
            transf_x.child_frame_id = tf_prefix_ + "gripper_x_dyn";
            geometry_msgs::msg::TransformStamped transf_z;
            transf_z.header.stamp = this->get_clock()->now();
            transf_z.header.frame_id = tf_prefix_ + "gripper_z_origin";
            transf_z.child_frame_id = tf_prefix_ + "gripper_z_dyn";
            tf2::Quaternion q;
            q.setRPY(0, 0,
                     -current_feedback_.stepper_positions[1] / 180.f *
                         M_PI); // 0=x,1=yaw,2=z
            transf_yaw.transform.rotation.x = q.x();
            transf_yaw.transform.rotation.y = q.y();
            transf_yaw.transform.rotation.z = q.z();
            transf_yaw.transform.rotation.w = q.w();
            transf_x.transform.translation.x =
                current_feedback_.stepper_positions[0] / 1000.f;
            transf_z.transform.translation.z =
                current_feedback_.stepper_positions[2] / 1000.f;
            std::vector<geometry_msgs::msg::TransformStamped>
                transforms; // function to pack multiple transforms into 1 array
            transforms.push_back(transf_x);
            transforms.push_back(transf_yaw);
            transforms.push_back(transf_z);
            tf_broadcaster_->sendTransform(transforms);
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
    current_feedback_ = Feedback();

    // Check if the object is a map using the new API (v2)
    if (obj.type == msgpack::type::MAP) {
      for (msgpack::object_kv &kv : obj.via.map) {
        const std::string &key = kv.key.as<std::string>();
        msgpack::object &value = kv.val;

        if (key == "stepper_positions" && value.type == msgpack::type::ARRAY) {
          for (size_t i = 0; i < value.via.array.size && i < 4; ++i) {
            current_feedback_.stepper_positions[i] =
                value.via.array.ptr[i].as<float>();
          }
        } else if (key == "servo_positions" &&
                   value.type == msgpack::type::ARRAY) {
          for (size_t i = 0; i < value.via.array.size && i < 2; ++i) {
            current_feedback_.servo_positions[i] =
                value.via.array.ptr[i].as<float>();
          }
        } else if (key == "stepper_directions" &&
                   value.type == msgpack::type::ARRAY) {
          for (size_t i = 0; i < value.via.array.size && i < 4; ++i) {
            current_feedback_.stepper_directions[i] =
                value.via.array.ptr[i].as<bool>();
          }
        } else if (key == "stepper_endstops" &&
                   value.type == msgpack::type::ARRAY) {
          for (size_t i = 0; i < value.via.array.size && i < 4; ++i) {
            current_feedback_.stepper_endstops[i] =
                value.via.array.ptr[i].as<bool>();
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
  // close_socket();
  // stop_io_service();
  RCLCPP_INFO(get_logger(), "destroy on shutdown");
  RCLCPP_DEBUG(get_logger(), "on shutdown is called from state %s.",
               state.label().c_str());

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::
      CallbackReturn::SUCCESS;
}

void GigatinoROS::cancel_action(bool stop) {
  {
    std::lock_guard<std::mutex> lk(feedback_mtx_);
    if (action_running_) {
      RCLCPP_WARN(get_logger(), "Cancelling previous action for new one");
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
  msgpack::zone zone;
  {
    std::unique_lock<std::mutex> lock(feedback_mtx_);
    action_cv_.wait_for(lock, 1s, [this]() { return !action_running_; });
    if (action_running_) {
      RCLCPP_WARN(get_logger(), "Other action still busy, overriding it");
    }
    action_running_ = true;
    curr_command_index_ = current_feedback_.command_index + 1;

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

      // print_buffer(sbuf);
      RCLCPP_DEBUG(get_logger(), "Message sent");
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
