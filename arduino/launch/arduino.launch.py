# Copyright (c) 2024 Carologistics
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
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="arduino",
                executable="arduino_node",
                name="arduino_node",
                parameters=[
                    {
                        "port": "/dev/arduino",
                        "baud_rate": 115200,
                        "enable_tf_broadcast": True,
                        "base_frame": "base_link",
                        "waypoint_frame": "waypoint",
                        "status_frequency": 1.0,
                        "wp_sensor_enable": True,
                        "wp_sensor_analog": True,
                        "wp_sensor_pin": 0,
                        "wp_sensor_analog_threshold": 0.7,
                    }
                ],
                output="screen",
            )
        ]
    )
