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
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '1.50', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'gripper_z_origin', '--child-frame-id', 'gripper_z_dyn']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '4.544004', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'gripper_x_origin', '--child-frame-id', 'gripper_x_dyn']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '-0.448332', '--pitch', '0', '--roll', '0', '--frame-id', 'gripper_yaw_origin', '--child-frame-id', 'gripper_yaw']
        ),
    ])
