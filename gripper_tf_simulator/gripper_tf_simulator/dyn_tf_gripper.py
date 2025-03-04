import numpy as np
from geometry_msgs.msg import TransformStamped, Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat

class DynamicFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        
        # Transform for yaw
        self.transf_yaw = TransformStamped()
        self.transf_yaw.header.frame_id = 'gripper_yaw_origin'
        self.transf_yaw.child_frame_id = 'gripper_yaw'
        
        # Transform for x-axis
        self.transf_x = TransformStamped()
        self.transf_x.header.frame_id = 'gripper_x_origin'
        self.transf_x.child_frame_id = 'gripper_x_dyn'
        
        # Transform for z-axis
        self.transf_z = TransformStamped()
        self.transf_z.header.frame_id = 'gripper_z_origin'
        self.transf_z.child_frame_id = 'gripper_z_dyn'
        
        # Initialize position attributes
        self.position_x = 0.0
        self.position_z = 0.0  
        # Initialize velocity attributes 
        self.linear_x = 0.0
        self.linear_z = 0.0
        # Initialize yaw attributes
        self.yaw = 0.0
        self.angular_yaw = 0.0
        
        # Initialize default quaternions
        self.transf_yaw.transform.rotation.x = 0.0
        self.transf_yaw.transform.rotation.y = 0.0
        self.transf_yaw.transform.rotation.z = 0.0
        self.transf_yaw.transform.rotation.w = 1.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_time = self.get_clock().now()     
        self.timer = self.create_timer(0.1, self.tf_timer)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel_gripper", self.arm_callback, 10)       

    def arm_callback(self, msg: Twist):
        # Update linear and angular velocities
        self.linear_x = msg.linear.x
        self.linear_z = msg.linear.z
        self.angular_yaw = msg.angular.z
        
        self.get_logger().info(f"Received: linear_x={self.linear_x}, linear_z={self.linear_z}, yaw={self.angular_yaw}")

    def tf_timer(self):
        # Get the current time and calculate elapsed time
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds
        
        # Update position based on velocity and elapsed time
        self.position_x += self.linear_x * dt
        self.position_z += self.linear_z * dt
        self.yaw += self.angular_yaw * dt
        
        quat_yaw = euler2quat(0, 0, self.yaw)
        
        # Update yaw transformation
        self.transf_yaw.header.stamp = current_time.to_msg()
        self.transf_yaw.transform.rotation.w = quat_yaw[0]
        self.transf_yaw.transform.rotation.x = quat_yaw[1]
        self.transf_yaw.transform.rotation.y = quat_yaw[2]
        self.transf_yaw.transform.rotation.z = quat_yaw[3]
        self.tf_broadcaster.sendTransform(self.transf_yaw)
        
        # Update x-axis transformation
        self.transf_x.header.stamp = current_time.to_msg()
        self.transf_x.transform.translation.x = self.position_x
        self.tf_broadcaster.sendTransform(self.transf_x)
        
        # Update z-axis transformation
        self.transf_z.header.stamp = current_time.to_msg()
        self.transf_z.transform.translation.z = self.position_z
        self.tf_broadcaster.sendTransform(self.transf_z)
        
        # Update previous time
        self.prev_time = current_time


def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
