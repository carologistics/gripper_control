import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from gripper_msgs.action import Gripper


class GripperActionClient(Node):

    def __init__(self):
        super().__init__('gripper_action_client')

        # Initialize the action client
        self._action_client = ActionClient(self, Gripper, 'gripper')

        # Placeholder for goal feedback
        self.feedback = None

    def send_goal(self, x_target, y_target, z_target, frame):
        # Wait for the action server to be ready
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return

        # Create the goal message
        goal_msg = Gripper.Goal()
        goal_msg.x_target = x_target
        goal_msg.y_target = y_target
        goal_msg.z_target = z_target
        goal_msg.frame = frame

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        if result.success:
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error('Goal failed!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    action_client = GripperActionClient()
    try:
        # Set the desired target position and frame
        x_target = -2.0
        y_target = 4.0
        z_target = 3.0
        frame = 'gripper_final'

        action_client.send_goal(x_target, y_target, z_target, frame)
        rclpy.spin(action_client)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
