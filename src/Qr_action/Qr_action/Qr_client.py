import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from swarm_interfaces.action import StringListSend
from rcl_interfaces.msg import SetParametersResult


class StringListActionClient(Node):

    def __init__(self):
        super().__init__('string_list_action_client')

        self.declare_parameter('string_list', ['default'])
        self._action_client = ActionClient(self, StringListSend, 'send_string_list')
        self.last_goal_handle = None
        self._action_client.wait_for_server()

        self.add_on_set_parameters_callback(self.parameter_callback)
        self.get_logger().info("Action client ready and waiting for parameter updates.")

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'string_list' and param.type_ == param.Type.STRING_ARRAY:
                new_list = param.value
                self.get_logger().info(f"New list received: {new_list}")

                if self.last_goal_handle:
                    self.get_logger().info("Canceling previous goal...")
                    cancel_future = self.last_goal_handle.cancel_goal_async()
                    cancel_future.add_done_callback(lambda f: self.get_logger().warn(f"goal canceled."))

                self.send_goal(new_list)
        return SetParametersResult(successful=True)

    def send_goal(self, input_list):
        goal_msg = StringListSend.Goal()
        goal_msg.input = input_list

        self.get_logger().info(f"Sending goal: {input_list}")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Feedback: {feedback_msg.feedback.feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected by server.')
            return

        self.get_logger().info('Goal accepted by server.')
        self.last_goal_handle = goal_handle

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result received from server: {result.result}')


def main(args=None):
    rclpy.init(args=args)
    node = StringListActionClient()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
