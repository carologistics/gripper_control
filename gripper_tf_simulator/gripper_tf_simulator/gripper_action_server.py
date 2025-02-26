import rclpy
import threading
import collections
import math
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
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel_gripper', 10)
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

                delta_position_x = goal_handle.request.x_target - current_position.x
                delta_position_y = goal_handle.request.y_target - current_position.y
                delta_position_z = goal_handle.request.z_target - current_position.z
                delta_yaw_center = 0.05/current_position.x 
                current_yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y ** 2 + orientation.z ** 2)
                )
                target_yaw = math.atan2(goal_handle.request.y_target, goal_handle.request.x_target)
                yaw_angle = target_yaw - (current_yaw-math.asin2(delta_yaw_center))

                self.get_logger().info(f"yaw_angle in degrees: {yaw_angle}")
                # Compute velocity
                cmd_vel = Twist()
                if goal_handle.request.x_target <0  :
                    cmd_vel.linear.x = -1 * delta_position_x
                else:
                    cmd_vel.linear.x = 1 * delta_position_x    
                cmd_vel.linear.z = 1 * delta_position_z

                cmd_vel.angular.z = 4.0 * yaw_angle
                # Check if the target position is reached (within a small tolerance)
                tolerance = 0.005# Define your tolerance
                if (abs(delta_position_x) <= tolerance and
                    abs(delta_position_y) <= tolerance and
                    abs(yaw_angle) <= tolerance and
                    abs(delta_position_z) <= tolerance ):
                    target_reached = True
                    self.feedback_timer.cancel()

                    # Stop the robot by publishing zero velocities
                    stop_cmd_vel = Twist()
                    self.cmd_vel_publisher.publish(stop_cmd_vel)
            
                    goal_handle.succeed()
 
                    self.get_logger().info("Target position reached. Robot stopped.")
                    result = Gripper.Result()
                    result.success = True
                    return result
                self.get_logger().info(
                    f"Publishing velocities: x={cmd_vel.linear.x}, y={cmd_vel.linear.y}, z={cmd_vel.linear.z}"
                )

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