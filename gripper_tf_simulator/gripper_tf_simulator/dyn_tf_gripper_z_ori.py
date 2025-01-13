import numpy as np
from geometry_msgs.msg import TransformStamped, Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from transforms3d.euler import euler2quat


class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('fixed_frame_tf2_broadcaster')
        self.transf = TransformStamped()
        self.transf.header.frame_id = 'gripper_z_origin'
        self.transf.child_frame_id = 'gripper_z_dyn'
        # Initialize position attributes
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0  
        # Initialize velocity attributes
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.linear_z = 0.0
        # Initialize angle attributes
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        # Initialize angular velocities for roll, pitch, yaw
        self.angular_roll = 0.0
        self.angular_pitch = 0.0
        self.angular_yaw = 0.0
        # Initialize default quaternion
        self.transf.transform.rotation.x = 0.0
        self.transf.transform.rotation.y = 0.0
        self.transf.transform.rotation.z = 0.0
        self.transf.transform.rotation.w = 1.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.prev_time = self.get_clock().now()     
        self.timer = self.create_timer(0.1, self.tf_timer)
        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.arm_callback, 10)
        

    def arm_callback(self, msg: Twist):
        # Save the linear components to instance variables
        self.linear_x = 0
        self.linear_y = 0
        self.linear_z = msg.linear.z
        self.angular_roll = 0
        self.angular_pitch = 0
        self.angular_yaw = 0
        # Convert yaw (angular.z) to quaternion
        quat = euler2quat(0.0, 0.0, msg.angular.z)
        self.transf.transform.rotation.w = quat[0]
        self.transf.transform.rotation.x = quat[1]
        self.transf.transform.rotation.y = quat[2]
        self.transf.transform.rotation.z = quat[3]
        self.get_logger().info(f"Received angular: roll={self.angular_roll}, pitch={self.angular_pitch}, yaw={self.angular_yaw}")
    
    def tf_timer(self):
        # Get the current time and calculate the elapsed time
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds * 1e-9  # Convert nanoseconds to seconds
        
        # Update position based on velocity and elapsed time
        self.position_x += self.linear_x * dt
        self.position_y += self.linear_y * dt
        self.position_z += self.linear_z * dt
        self.yaw += self.angular_yaw * dt
        quat = euler2quat(0, 0, 0)
        # Update the transform
        self.transf.header.stamp = current_time.to_msg()
        self.transf.transform.translation.x = self.position_x
        self.transf.transform.translation.y = self.position_y
        self.transf.transform.translation.z = self.position_z
        self.transf.transform.rotation.w = quat[0]
        self.transf.transform.rotation.x = quat[1]
        self.transf.transform.rotation.y = quat[2]
        self.transf.transform.rotation.z = quat[3]
        
        self.tf_broadcaster.sendTransform(self.transf)
        # Update the previous time
        self.prev_time = current_time
    
def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
