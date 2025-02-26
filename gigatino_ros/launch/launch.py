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
from launch import LaunchDescription
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    # container = ComposableNodeContainer(
    container = ComposableNodeContainer(
        name="gigatino_container",
        namespace="",
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
            ComposableNode(package="gigatino_ros", plugin="gigatino_ros::GigatinoROS", name="gigatino_ros"),
        ],
    )
    # Ensure lifecycle nodes are properly shut down
    shutdown_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=container,
            on_exit=[EmitEvent(event=Shutdown())],
        )
    )
    return LaunchDescription([container, load_composable_nodes, shutdown_event_handler])
