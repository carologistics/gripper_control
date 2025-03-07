#!/usr/bin/env python3
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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from rclpy.logging import get_logger


def launch_with_context(context, *args, **kwargs):
    gigatino_dir = get_package_share_directory("gigatino_ros")

    LaunchConfiguration("namespace")
    config = LaunchConfiguration("config")
    log_level = LaunchConfiguration("log_level")
    mockup = LaunchConfiguration("mockup").perform(context)
    config_file = os.path.join(gigatino_dir, "params", config.perform(context))
    # re-issue warning as it is not colored otherwise ...
    if not os.path.isfile(config_file):
        logger = get_logger("cx_bringup_launch")
        logger.warning(f"Parameter file path is not a file: {config_file}")
    # container = ComposableNodeContainer(
    container = ComposableNodeContainer(
        name="gigatino_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",  # both
        emulate_tty=True,
    )
    mockup_args = {}
    mockup_script = []
    if mockup:
        mockup_script = [
            Node(
                package="gigatino_ros",
                executable="gigatino_mockup.py",  # This is the name in setup.py
                parameters=[config_file],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
                emulate_tty=True,
            )
        ]
        mockup_args = {"local_ip_address": "127.0.0.1", "remote_ip_address": "127.0.0.1"}

    load_composable_nodes = LoadComposableNodes(
        target_container="gigatino_container",
        composable_node_descriptions=[
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager",
                parameters=[{"autostart": True, "node_names": ["gigatino_ros"]}],
            ),
            ComposableNode(
                package="gigatino_ros",
                plugin="gigatino_ros::GigatinoROS",
                name="gigatino_ros",
                parameters=[config_file, mockup_args],
            ),
        ],
    )
    return [container, load_composable_nodes] + mockup_script


def generate_launch_description():
    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()
    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value="debug",
        description="Logging level for cx_node executable",
    )

    declare_namespace_ = DeclareLaunchArgument("namespace", default_value="", description="Default namespace")

    declare_add_mockup_ = DeclareLaunchArgument("mockup", default_value="True", description="Default namespace")

    declare_gigatino_ros_config_ = DeclareLaunchArgument(
        "config",
        default_value="config.yaml",
        description="Name of the node configuration file",
    )

    ld.add_action(declare_log_level_)

    ld.add_action(declare_namespace_)
    ld.add_action(declare_gigatino_ros_config_)
    ld.add_action(declare_add_mockup_)
    ld.add_action(OpaqueFunction(function=launch_with_context))
    return ld
