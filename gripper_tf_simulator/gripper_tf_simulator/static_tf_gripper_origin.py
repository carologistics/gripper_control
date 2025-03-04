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
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):

    def __init__(self):
        super().__init__("fixed_frame_tf2_broadcaster")
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)

    def broadcast_timer_callback(self):
        t = TransformStamped()

       t.header.stamp = self.get_clock().now().to_msg()
       t.header.frame_id = 'base_link'
       t.child_frame_id = 'gripper_z_origin'
       t.transform.translation.x = 0.0
       t.transform.translation.y = 0.0
       t.transform.translation.z = 2.5
       t.transform.rotation.x = 0.0
       t.transform.rotation.y = 0.0
       t.transform.rotation.z = 0.0
       t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        t2 = TransformStamped()

       t2.header.stamp = self.get_clock().now().to_msg()
       t2.header.frame_id = 'gripper_z_dyn'
       t2.child_frame_id = 'gripper_yaw_origin'
       t2.transform.translation.x = 1.0
       t2.transform.translation.y = 1.0
       t2.transform.translation.z = 0.0
       t2.transform.rotation.x = 0.0
       t2.transform.rotation.y = 0.0
       t2.transform.rotation.z = 0.0
       t2.transform.rotation.w = 1.0
       self.tf_broadcaster.sendTransform(t2)

        t3 = TransformStamped()

       t3.header.stamp = self.get_clock().now().to_msg()
       t3.header.frame_id = 'gripper_yaw'
       t3.child_frame_id = 'gripper_x_origin'
       t3.transform.translation.x = 4.0
       t3.transform.translation.y = 3.0
       t3.transform.translation.z = 2.0
       t3.transform.rotation.x = 0.0
       t3.transform.rotation.y = 0.0
       t3.transform.rotation.z = 0.0
       t3.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t3)

        t4 = TransformStamped()

       t4.header.stamp = self.get_clock().now().to_msg()
       t4.header.frame_id = 'gripper_x_dyn'
       t4.child_frame_id = 'gripper_end_effector'
       t4.transform.translation.x = 0.0 #to be determined
       t4.transform.translation.y = 0.0
       t4.transform.translation.z = 0.0
       t4.transform.rotation.x = 0.0
       t4.transform.rotation.y = 0.0
       t4.transform.rotation.z = 0.0
       t4.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t4)
       t5 = TransformStamped()

        # from gripper_z_origin to gripper_yaw_origin (only y currently, also in x in the future) and to gripper_end_effector in z
       t5.header.stamp = self.get_clock().now().to_msg()
       t5.header.frame_id = 'gripper_z_origin'
       t5.child_frame_id = 'gripper_home_origin'
       t5.transform.translation.x = 0.0
       t5.transform.translation.y = 0.05
       t5.transform.translation.z = 2.0
       t5.transform.rotation.x = 0.0
       t5.transform.rotation.y = 0.0
       t5.transform.rotation.z = 0.0
       t5.transform.rotation.w = 1.0
       self.tf_broadcaster.sendTransform(t5)






def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
