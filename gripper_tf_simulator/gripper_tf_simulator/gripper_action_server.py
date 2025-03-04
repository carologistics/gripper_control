# Copyright (c) 2025 Carologistics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import collections
import math
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Twist
from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.action import GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from gripper_msgs.action import Gripper
from transforms3d.euler import euler2quat

class GripperActionServer(Node):

    def __init__(self):
        super().__init__("gripper_action_server")

        # TF2 buffer and listener to query transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF2 broadcaster to broadcast transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Action server
        self._action_server = ActionServer(
            self,
            Gripper,
            "gripper",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        self.publisher_ = self.create_publisher(TransformStamped, "/tf", 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel_gripper", 10)
        self.feedback_timer = None  # Timer for feedback updates

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal to move to target position: {goal_handle.request}")

        result = Gripper.Result()
        target_reached = False

        def feedback_and_publish():
            nonlocal target_reached

            try:

                posestamp = PoseStamped()
                posestamp.header.frame_id = goal_handle.request.frame
                posestamp.pose.position.x = goal_handle.request.x_target
                posestamp.pose.position.y = goal_handle.request.y_target
                posestamp.pose.position.z = goal_handle.request.z_target
                target_point = self.tf_buffer.transform(posestamp, "gripper_home_origin")
                z_abs =  target_point.pose.position.z
                target_to_yaw = self.tf_buffer.transform(posestamp, "gripper_yaw_origin")

                offset_end_effector = self.tf_buffer.lookup_transform(target_frame='gripper_x_dyn',source_frame='gripper_end_effector',time=rclpy.time.Time())
                end_effector_to_yaw = self.tf_buffer.lookup_transform(target_frame='gripper_end_effector',source_frame='gripper_yaw_origin',time=rclpy.time.Time())
                d = self.tf_buffer.lookup_transform(target_frame='gripper_x_origin',source_frame='gripper_yaw_origin',time=rclpy.time.Time())
                d_rel = math.sqrt(d.transform.translation.x**2+d.transform.translation.y**2)
                g = math.sqrt(target_to_yaw.pose.position.x**2 + target_to_yaw.pose.position.y **2)
                beta = math.acos(abs(end_effector_to_yaw.transform.translation.y)/g) 
                x_static =  abs(d.transform.translation.x) +  offset_end_effector.transform.translation.x
                x_delta = g*math.sin(beta)
                x_abs = x_delta - x_static
                t_x = target_to_yaw.pose.position.x
                t_y = target_to_yaw.pose.position.y
                alpha = math.atan(abs(t_x) / abs(t_y))
                if posestamp.pose.position.y <= abs(d.transform.translation.y) :
                    tetha = (90*math.pi/180)- beta + alpha
                elif posestamp.pose.position.y > abs(d.transform.translation.y) :
                    tetha = (math.pi) - (math.pi/2 -beta) -  beta - alpha

                # Publish the 
                transform_msg = TransformStamped()
                transform_msg.header.stamp = self.get_clock().now().to_msg()
                transform_msg.header.frame_id = "base_link"
                transform_msg.child_frame_id = "gripper_end_effector"

                # Set translation values
                transform_msg.transform.translation.x = x_abs
                transform_msg.transform.translation.y = 0
                transform_msg.transform.translation.z = z_abs

                # Set rotation (yaw_after needs to be converted to quaternion)

                q = euler2quat(0, 0, tetha)
                transform_msg.transform.rotation.x = q[0]
                transform_msg.transform.rotation.y = q[1]
                transform_msg.transform.rotation.z = q[2]
                transform_msg.transform.rotation.w = q[3]

                # Publish the transform
                self.publisher_.publish(transform_msg)
                self.get_logger().info(f"x:{x_abs}, g:{offset_end_effector.transform.translation.x}, z:{z_abs}, current angle{tetha}, beta:{beta},alpha:{alpha}, d:x{d.transform.translation.x},y{d.transform.translation.x},end_efector:{end_effector_to_yaw.transform.translation.y},target_point:{target_point.pose.position.x},{target_point.pose.position.y},{target_point.pose.position.z}")
            except Exception as e:
                self.get_logger().error(f"Error in feedback/publish loop: {str(e)}")
                goal_handle.abort()
                result.success = False
                if self.feedback_timer:
                    self.feedback_timer.cancel()

        # Start a timer to call feedback_and_publish at 1 ms intervals
        self.feedback_timer = self.create_timer(0.001, feedback_and_publish)

        # Block until the goal completes or is canceled
        while not target_reached and rclpy.ok():
            time.sleep(0.1)
        result.success = True
        return result


def main(args=None):

    try:
        rclpy.init(args=args)
        gripper_action_server = GripperActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(gripper_action_server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        gripper_action_server.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()