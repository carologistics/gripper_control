import rclpy
import threading
import collections
import math
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf2_geometry_msgs
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped,Vector3
from transforms3d.euler import euler2quat

class FixedFrameBroadcaster(Node):

   def __init__(self):
       super().__init__('fixed_frame_tf2_broadcaster')
       self.get_logger().info("hello") # TF2 buffer and listener to query transforms
       self.tf_buffer = Buffer()
       self.tf_listener = TransformListener(self.tf_buffer, self)

       self.timer = self.create_timer(3, self.calculate)
   def calculate(self):
       target =  Vector3()
       target.x = 3
       target.y = 6
       target.z = 6
       posestamp = PoseStamped()
       posestamp.header.frame_id = "base_link"
       posestamp.pose.position.x = target.x
       posestamp.pose.position.y = target.y
       posestamp.pose.position.z = target.z
       posestamp.header.stamp.sec = 0
       posestamp.header.stamp.nanosec = 0
       target_point = self.tf_buffer.transform(posestamp, "gripper_home_origin")
       self.get_logger().info(f"target_point:{target_point}")
       z_abs =  target_point.pose.position.z
       self.get_logger().info(f"z:{z_abs}")
       target_to_yaw = self.tf_buffer.transform(posestamp, "gripper_yaw_origin")
       
       offset_end_effector = self.tf_buffer.lookup_transform(target_frame='gripper_x_dyn',source_frame='gripper_end_effector',time=rclpy.time.Time())
       self.get_logger().info(f"offset_end_effector:{offset_end_effector}")
       end_effector_to_yaw = self.tf_buffer.lookup_transform(target_frame='gripper_yaw',source_frame='gripper_end_effector',time=rclpy.time.Time())
       self.get_logger().info(f"end_effector_to_yaw:{end_effector_to_yaw}")
       d = end_effector_to_yaw.transform.translation.y
       g = math.sqrt(target_to_yaw.pose.position.x**2 + target_to_yaw.pose.position.y **2)
       beta = math.acos(abs(d)/abs(g))
       x_origin_to_yaw_dyn = self.tf_buffer.lookup_transform(target_frame='gripper_x_origin',source_frame='gripper_yaw',time=rclpy.time.Time())
       x_static = abs(x_origin_to_yaw_dyn.transform.translation.x) + abs(offset_end_effector.transform.translation.x)
       x_delta = g*math.sin(beta)
       x_abs = x_delta - x_static
       t_x = target_to_yaw.pose.position.x
       t_y = target_to_yaw.pose.position.y
       alpha = math.atan(abs(t_x) / abs(t_y))
       
       if target_to_yaw.pose.position.y >= 0 :
           tetha = (beta - alpha)
       else :
           tetha = ((math.pi) - beta - alpha)*-1
       self.get_logger().info(f"tetha:{tetha}")
       self.get_logger().info(f"alpha:{alpha}")
       self.get_logger().info(f"beta:{beta}")
       self.get_logger().info(f"x_abs:{x_abs}")
       self.get_logger().info(f" target to yaw position: {target_to_yaw.pose.position.y}")
        


       


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
