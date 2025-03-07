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
from glob import glob

from setuptools import find_packages
from setuptools import setup

package_name = "gripper_tf_simulator"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Zhen yan Khaw",
    maintainer_email="zhen-yan.khaw@alumni.fh-aachen.de",
    description="Publisher of the dyn tf for the gripper",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dyn_tf_gripper_y_ori = gripper_tf_simulator.dyn_tf_gripper_y_ori:main",
            "dyn_tf_gripper_x_ori = gripper_tf_simulator.dyn_tf_gripper_x_ori:main",
            "dyn_tf_gripper_z_ori = gripper_tf_simulator.dyn_tf_gripper_z_ori:main",
            "gripper_action_server=gripper_tf_simulator.gripper_action_server:main",
            "gripper_action_client=gripper_tf_simulator.gripper_action_client:main",
            "static_tf_gripper_origin=gripper_tf_simulator.static_tf_gripper_origin:main",
        ],
    },
)
