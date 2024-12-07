import rclpy
import threading
import collections

import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Twist
from gripper_msgs.action import Gripper

class GripperActionServer(Node):

    def __init__(self):
        super().__init__('gripper_action_server')

        # TF2 buffer and listener to query transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # TF2 broadcaster to broadcast transforms
        self.tf_broadcaster = TransformBroadcaster(self)

        # Action server
        self._action_server = ActionServer(
            self,
            Gripper,
            'gripper',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )
    
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()
        self._current_goal = None

        self.publisher_ = self.create_publisher(TransformStamped, '/tf', 10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.feedback_timer = None  # Timer for feedback updates
    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal to move to target position: {goal_handle.request}")

        feedback_msg = Gripper.Feedback()
        result = Gripper.Result()
        target_reached = False

        def feedback_and_publish():
            nonlocal target_reached

            try:
                # Query the transform
                transform = self.tf_buffer.lookup_transform(
                    target_frame='base_link',                # Frame to query
                    source_frame=goal_handle.request.frame,  # Reference frame
                    time=rclpy.time.Time()
                )

                # Calculate feedback
                current_position = transform.transform.translation
                self.get_logger().info(
                    f"Relative position (base_link -> goal_handle.request.frame): "
                    f"End-effector position: x={current_position.x}, y={current_position.y}, z={current_position.z}"
                )

                # Extract rotation (orientation as a quaternion)
                orientation = transform.transform.rotation
                self.get_logger().info(
                    f"Relative orientation (base_link -> goal_handle.request.frame): "
                    f"End-effector orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}"
                )
                feedback_msg.position_x = goal_handle.request.x_target - current_position.x
                feedback_msg.position_y = goal_handle.request.y_target - current_position.y
                feedback_msg.position_z = goal_handle.request.z_target - current_position.z
                # feedback_msg.x_angle = goal_handle.request.raw_target - orientation.x
                # feedback_msg.y_angle = goal_handle.request.pitch_target - orientation.y
                # feedback_msg.z_angle = goal_handle.request.yaw_target - orientation.z
                # feedback_msg.w_angle = goal_handle.request.qua_target - orientation.w
                goal_handle.publish_feedback(feedback_msg)

                # Compute velocity
                cmd_vel = Twist()
                cmd_vel.linear.x = 3 * feedback_msg.position_x
                cmd_vel.linear.y = 3 * feedback_msg.position_y
                cmd_vel.linear.z = 3 * feedback_msg.position_z

                # Check if the target position is reached (within a small tolerance)
                tolerance = 0.01  # Define your tolerance
                if (abs(feedback_msg.position_x) <= tolerance and
                    abs(feedback_msg.position_y) <= tolerance and
                    abs(feedback_msg.position_z) <= tolerance):
                    target_reached = True
                    self.feedback_timer.cancel()

                    # Stop the robot by publishing zero velocities
                    stop_cmd_vel = Twist()
                    stop_cmd_vel.linear.x = 0.0
                    stop_cmd_vel.linear.y = 0.0
                    stop_cmd_vel.linear.z = 0.0
                    self.cmd_vel_publisher.publish(stop_cmd_vel)
            
                    goal_handle.succeed()
 
                    self.get_logger().info("Target position reached. Robot stopped.")
                    result = Gripper.Result()
                    result.success = True
                    return result


                # Publish the command velocity
                self.cmd_vel_publisher.publish(cmd_vel)

            except Exception as e:
                self.get_logger().error(f"Error in feedback/publish loop: {str(e)}")
                goal_handle.abort()
                result.success = False
                if self.feedback_timer:
                    self.feedback_timer.cancel()

        # Start a timer to call feedback_and_publish at 1 ms intervals
        self.feedback_timer = self.create_timer(0.001, feedback_and_publish)

        # Block until the goal completes or is canceled
        while not target_reached and rclpy.ok():
            time.sleep(0.1)
        result.success = True
        return result

    
def main(args=None):
    
    try:
        rclpy.init(args=args)
        gripper_action_server = GripperActionServer()
        executor = MultiThreadedExecutor()
        executor.add_node(gripper_action_server)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        gripper_action_server.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()