# gigatino_ros

## Usage (launch file)

Launch it via:
```bash
ros2 launch gigatino_ros launch.py
```

## Usage (manual bringup)
To run as multithreaded component:
```bash
ros2 run rclcpp_components component_container_mt
```
and in second terminal:
```
ros2 component load /ComponentManager gigatino_ros gigatino_ros::GigatinoROS
```

You can launch the node single-threaded as component:
```bash
ros2 component standalone gigatino_ros gigatino_ros::GigatinoROS
```
Or directly:
```bash
ros2 run gigatino_ros gigatino_node
```

Either way, make sure to transition the node properly:
```bash
ros2 lifecycle set /gigatino_ros configure; ros2 lifecycle set /gigatino_ros activate
```

## Commands
To Calibrate:

ros2 action send_goal /robotinobase1/gigatino/calibrate gigatino_msgs/action/Calibrate {}



ros2 run gigatino_ros test_ethernet.py 192.168.3.199 8888 '{"command": "CALIBRATE", "target_mot_x": 190, "target_mot_yaw": 70, "target_mot_z": 120, "target_servo_gripper": 120}'
