import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
              executable='dyn_tf_gripper_y_ori',
              name='dyn_tf_gripper_y_ori',
          ),
         Node(
             package='gripper_tf_simulator',
             executable='dyn_tf_gripper_z_ori',
             name='dyn_tf_gripper_z_ori',
         ),
    ])
