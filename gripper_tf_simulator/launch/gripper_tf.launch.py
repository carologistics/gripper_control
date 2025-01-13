import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
         Node(
             package='gripper_tf_simulator',
             executable='static_tf_gripper_origin',
             name='static_tf_gripper_origin',
         ),
         Node(
             package='gripper_tf_simulator',
             executable='dyn_tf_gripper_x_ori',
             name='dyn_tf_gripper_x_ori',
         ),
         Node(
             package='gripper_tf_simulator',
             executable='dyn_tf_gripper_z_ori',
             name='dyn_tf_gripper_z_ori',
         ),         
         Node(
             package='gripper_tf_simulator',
             executable='dyn_tf_gripper_yaw',
             name='dyn_tf_gripper_yaw',
         ),
    ])
