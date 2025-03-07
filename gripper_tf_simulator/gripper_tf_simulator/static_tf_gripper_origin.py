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
        t.header.frame_id = "gripper_y_dyn"
        t.child_frame_id = "gripper_x_origin"
        t.transform.translation.x = 0.8125
        t.transform.translation.y = 0.54
        t.transform.translation.z = -0.18
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)

        t2 = TransformStamped()

        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = "gripper_z_dyn"
        t2.child_frame_id = "gripper_y_origin"
        t2.transform.translation.x = -0.1325
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.65
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t2)

        t3 = TransformStamped()

        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = "base_link"
        t3.child_frame_id = "gripper_z_origin"
        t3.transform.translation.x = 0.245
        t3.transform.translation.y = 0.0
        t3.transform.translation.z = 2.5
        t3.transform.rotation.x = 0.0
        t3.transform.rotation.y = 0.0
        t3.transform.rotation.z = 0.0
        t3.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t3)

        t4 = TransformStamped()

        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = "gripper_x_dyn"
        t4.child_frame_id = "gripper_final"
        t4.transform.translation.x = 0.46528
        t4.transform.translation.y = 0.0
        t4.transform.translation.z = -0.405
        t4.transform.rotation.x = 0.0
        t4.transform.rotation.y = 0.0
        t4.transform.rotation.z = 0.0
        t4.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t4)

    #    t5 = TransformStamped()

    #    t5.header.stamp = self.get_clock().now().to_msg()
    #    t5.header.frame_id = 'base_link'
    #    t5.child_frame_id = 'machine_link'
    #    t5.transform.translation.x = -0.350
    #    t5.transform.translation.y = 0.175
    #    t5.transform.translation.z = 0.0
    #    t5.transform.rotation.x = 0.0
    #    t5.transform.rotation.y = 0.0
    #    t5.transform.rotation.z = 1.570795
    #    t5.transform.rotation.w = 1.0

    #    self.tf_broadcaster.sendTransform(t5)

    #    t6 = TransformStamped()

    #    t6.header.stamp = self.get_clock().now().to_msg()
    #    t6.header.frame_id = 'gripper_y_dyn'
    #    t6.child_frame_id = 'plate_top'
    #    t6.transform.translation.x = 0.0
    #    t6.transform.translation.y = 0.0
    #    t6.transform.translation.z = 0.1
    #    t6.transform.rotation.x = 0.0
    #    t6.transform.rotation.y = 0.0
    #    t6.transform.rotation.z = 0.0
    #    t6.transform.rotation.w = 1.0

    #    self.tf_broadcaster.sendTransform(t6)

    #    t7 = TransformStamped()

    #    t7.header.stamp = self.get_clock().now().to_msg()
    #    t7.header.frame_id = 'base_link'
    #    t7.child_frame_id = 'front_laser'
    #    t7.transform.translation.x = 0.094
    #    t7.transform.translation.y = 0.0
    #    t7.transform.translation.z = 0.239
    #    t7.transform.rotation.x = 3.1415
    #    t7.transform.rotation.y = 0.0
    #    t7.transform.rotation.z = 0.0
    #    t7.transform.rotation.w = 0.0

    #    self.tf_broadcaster.sendTransform(t7)

    #    t8 = TransformStamped()

    #    t8.header.stamp = self.get_clock().now().to_msg()
    #    t8.header.frame_id = 'plate_top'
    #    t8.child_frame_id = 'end_effector_home'
    #    t8.transform.translation.x = 0.139028
    #    t8.transform.translation.y = 0.054
    #    t8.transform.translation.z = 0.105
    #    t8.transform.rotation.x = 0.0
    #    t8.transform.rotation.y = 0.0
    #    t8.transform.rotation.z = 0.0
    #    t8.transform.rotation.w = 1.0

    #    self.tf_broadcaster.sendTransform(t8)

    #    t9 = TransformStamped()

    #    t9.header.stamp = self.get_clock().now().to_msg()
    #    t9.header.frame_id = 'base_link'
    #    t9.child_frame_id = 'cam_tag'
    #    t9.transform.translation.x = 0.0546
    #    t9.transform.translation.y = 0.0
    #    t9.transform.translation.z = 0.4675
    #    t9.transform.rotation.x = -1.57079632679
    #    t9.transform.rotation.y = 0.0
    #    t9.transform.rotation.z = -1.57079632679
    #    t9.transform.rotation.w = 0.0

    #    self.tf_broadcaster.sendTransform(t9)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
