# gripper_control
ROS 2 packages related to the carologistics gripping components
to start the simulator type the command at terminal
ros2 launch gripper_tf_simulator gripper_tf.launch.py
in Rviz2 add TF to visualize the transformation
after that run the action server by giving the command 
ros2 run gripper_tf_simulator gripper_action_server
to test whether the server works you may run the action client 
ros2 run gripper_tf_simulator gripper_action_client
