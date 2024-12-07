# gripper_control
ROS 2 package to simulate the gripper of the  carologistics gripper.
to start the simulator type the command at terminal.
```bash
ros2 launch gripper_tf_simulator gripper_tf.launch.py
```
in Rviz2 add TF to visualize the transformation
after that run the action server by giving the command.
```bash
ros2 run gripper_tf_simulator gripper_action_server
```
to test whether the server works you may run the action client. 
```bash
ros2 run gripper_tf_simulator gripper_action_client
```
