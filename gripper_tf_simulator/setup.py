import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gripper_tf_simulator'

setup(
    name=package_name,
    version='0.1.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhen yan Khaw',
    maintainer_email='zhen-yan.khaw@alumni.fh-aachen.de',
    description='Publisher of the dyn tf for the new gripper',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dyn_tf_gripper = gripper_tf_simulator.dyn_tf_gripper:main',
            'gripper_action_server=gripper_tf_simulator.gripper_action_server:main',
            'gripper_action_client=gripper_tf_simulator.gripper_action_client:main',
            'static_tf_gripper_origin=gripper_tf_simulator.static_tf_gripper_origin:main',
        ],
    },
)
