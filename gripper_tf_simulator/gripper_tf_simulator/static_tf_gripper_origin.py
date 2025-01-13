from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):

   def __init__(self):
       super().__init__('fixed_frame_tf2_broadcaster')
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
       t2.transform.translation.x = 0.0
       t2.transform.translation.y = 0.05
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
       t3.transform.translation.x = 0.0
       t3.transform.translation.y = 0.0
       t3.transform.translation.z = 2.0
       t3.transform.rotation.x = 0.0
       t3.transform.rotation.y = 0.0
       t3.transform.rotation.z = 0.0
       t3.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t3)

       t4 = TransformStamped()

       t4.header.stamp = self.get_clock().now().to_msg()
       t4.header.frame_id = 'gripper_x_dyn'
       t4.child_frame_id = 'gripper_final'
       t4.transform.translation.x = 0.46528
       t4.transform.translation.y = 0.0
       t4.transform.translation.z = 0.0
       t4.transform.rotation.x = 0.0
       t4.transform.rotation.y = 0.0
       t4.transform.rotation.z = 0.0
       t4.transform.rotation.w = 1.0

       self.tf_broadcaster.sendTransform(t4)






def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
