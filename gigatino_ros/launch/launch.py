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
import sys

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode
from rclpy.logging import get_logger


def find_file(path, locations):
    """
    Check if the file at the given path exists. If not,
    check if the path is absolute. If it is not absolute,
    search for the file in the list of locations.

    Args:
        path (str): The path to the file.
        locations (list): List of locations to search for the file.

    Returns:
        str: The absolute path to the file if found, otherwise None.
    """
    # Check if the file exists
    if os.path.exists(path):
        return path

    # Check if the path is absolute
    if os.path.isabs(path):
        return None

    # Search for the file in the list of locations
    for location in locations:
        file_path = os.path.join(location, path)
        if os.path.exists(file_path):
            return file_path

    # File not found
    return None


def launch_with_context(context, *args, **kwargs):
    gigatino_dir = get_package_share_directory("gigatino_ros")

    logger = get_logger("gigatino_ros_launch")
    LaunchConfiguration("namespace")
    config = LaunchConfiguration("gigatino_config")
    log_level = LaunchConfiguration("log_level")
    tf_config = LaunchConfiguration("tf_config")
    host_config = LaunchConfiguration("host_config")
    mockup = LaunchConfiguration("mockup").perform(context)
    config_file = find_file(config.perform(context), [gigatino_dir + "/params/"])
    if config_file is None:
        logger.error(f"Can not find unknown gigatino config file: {config_file}, abvort")
        sys.exit(1)
    tf_config_file = find_file(tf_config.perform(context), [gigatino_dir + "/params/"])
    if tf_config_file is None:
        logger.warning(f"Skipping unknown tf config file: {tf_config_file}")
    host_config_file = find_file(host_config.perform(context), [gigatino_dir + "/params/"])
    if host_config_file is None:
        logger.warning(f"Skipping unknown host config file: {host_config_file}")
    static_transform_publishers = []
    static_transforms = {}
    with open(tf_config_file, "r") as file:
        transforms = yaml.safe_load(file)
        for key in "/**|ros__parameters|static_transforms".split("|"):
            transforms = transforms.get(key, {})
        for entity, transform in transforms.items():
            if not isinstance(static_transforms.get(entity), dict):
                static_transforms[entity] = {}
            for key in ["translation", "rotation", "parent_frame_id", "child_frame_id"]:
                if key in transform:
                    static_transforms[entity][key] = transform[key]
    with open(host_config_file, "r") as file:
        transforms = yaml.safe_load(file)
        for key in "/**|ros__parameters|static_transforms".split("|"):
            transforms = transforms.get(key, {})
        for entity, transform in transforms.items():
            if not isinstance(static_transforms.get(entity), dict):
                static_transforms[entity] = {}
            for key in ["translation", "rotation", "parent_frame_id", "child_frame_id"]:
                if key in transform:
                    static_transforms[entity][key] = transform[key]
    for transform, values in static_transforms.items():
        translation = values.get("translation", None)
        rotation = values.get("rotation", None)
        frame_id = values.get("parent_frame_id", None)
        child_frame_id = values.get("child_frame_id", None)

        if None in [translation, rotation, frame_id, child_frame_id]:
            static_transform_publishers.append(LogInfo(msg="[WARN] Missing key(s) in transform. Skipping..."))
            continue
        import tf_transformations as tf

        tf.quaternion_from_euler(rotation[0], rotation[1], rotation[2])

        static_transform_publisher_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            name="tf_" + transform,
            arguments=[
                "--x",
                str(translation[0]),
                "--y",
                str(translation[1]),
                "--z",
                str(translation[2]),
                "--yaw",
                str(rotation[0]),
                "--pitch",
                str(rotation[1]),
                "--roll",
                str(rotation[2]),
                "--frame-id",
                frame_id,
                "--child-frame-id",
                child_frame_id,
            ],
        )
        # static_transform_publisher_node = ComposableNode(
        #       name="tf_" + transform,
        #       package="tf2_ros",
        #       plugin="tf2_ros::StaticTransformBroadcasterNode",
        #       parameters=[{"translation.x": translation[0],
        #                    "translation.y": translation[1],
        #                    "translation.z": translation[2],
        #                    "rotation.x": quaternion[0],
        #                    "rotation.y": quaternion[1],
        #                    "rotation.z": quaternion[2],
        #                    "rotation.w": quaternion[3],
        #                    "frame": frame_id,
        #                    "child_frame_id": child_frame_id,
        #                    }],
        # )
        static_transform_publishers.append(static_transform_publisher_node)
    # container = ComposableNodeContainer(
    mockup_args = {}
    mockup_script = []
    if mockup == "true":
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

    container = ComposableNodeContainer(
        name="gigatino_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",  # both
    )
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
        ],  # + static_transform_publishers,
    )
    return [container, load_composable_nodes] + mockup_script + static_transform_publishers


def generate_launch_description():
    # The lauchdescription to populate with defined CMDS
    ld = LaunchDescription()
    declare_log_level_ = DeclareLaunchArgument(
        "log_level",
        default_value="debug",
        description="Logging level for cx_node executable",
    )

    declare_namespace_ = DeclareLaunchArgument("namespace", default_value="", description="Default namespace")

    declare_add_mockup_ = DeclareLaunchArgument("mockup", default_value="true", description="Default namespace")

    declare_gigatino_ros_config_ = DeclareLaunchArgument(
        "gigatino_config",
        default_value="config.yaml",
        description="Name of the node configuration file",
    )
    declare_tf_config_ = DeclareLaunchArgument(
        "tf_config",
        default_value="gripper_tfs.yaml",
        description="Name of the gripper tf file",
    )
    declare_host_config_ = DeclareLaunchArgument(
        "host_config",
        default_value="gripper_tfs.yaml",
        description="Name of the gripper tf file",
    )

    ld.add_action(declare_log_level_)

    ld.add_action(declare_namespace_)
    ld.add_action(declare_gigatino_ros_config_)
    ld.add_action(declare_add_mockup_)
    ld.add_action(declare_tf_config_)
    ld.add_action(declare_host_config_)
    ld.add_action(OpaqueFunction(function=launch_with_context))
    return ld
