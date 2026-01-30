import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from swarm_interfaces.action import StringListSend
from rclpy.executors import MultiThreadedExecutor
import time
import threading

class StringListActionServer(Node):

    def __init__(self):
        super().__init__('string_list_action_server')
        self._action_server = ActionServer(
            self,
            StringListSend,
            'send_string_list',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        self.get_logger().info("Server started!")

    def goal_callback(self, goal_request):
        goal = goal_request.input
        # self.get_logger().warn(f"goal:{goal}")
        #service send it to Uros over service "String array" wait for response like accepted or rejected
        if not goal_request.input or goal_request.input[0] == '':
            self.get_logger().warn("Empty string list received. Rejecting goal.")
            return GoalResponse.REJECT

        self.get_logger().info(f"Goal accepted with list: {goal_request.input}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
         #service send bool cancel to the Uros
        self.get_logger().info("Cancel request received.")
        # if respone in done then ACCEPT else rejected
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        strings = goal_handle.request.input
        self.get_logger().info(f"Executing goal: {strings}")

        for i, item in enumerate(strings):  #Topic while loop feedback[1] is "Executing"
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info("Goal canceled.")
                result = StringListSend.Result()
                result.result = "Goal canceled before completion."
                return result

            feedback_msg = StringListSend.Feedback()
            feedback_msg.feedback = f"Processing item {i+1}/{len(strings)}: {item}"  # subsribe to Uros_feedback and feedback[0] it here
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        # once while loop breaks result feedback[1]

        goal_handle.succeed()
        result = StringListSend.Result()
        result.result = f"Processed {len(strings)} items. First: {strings[0]}"
        self.get_logger().info("Goal completed.")
        return result


def main(args=None):
    rclpy.init(args=args)
    node = StringListActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
