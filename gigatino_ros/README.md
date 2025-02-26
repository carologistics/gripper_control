# gigatino_ros

## usage
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
