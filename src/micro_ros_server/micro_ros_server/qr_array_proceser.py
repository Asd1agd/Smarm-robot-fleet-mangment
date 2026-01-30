import rclpy
from rclpy.node import Node

from micro_ros_custom_interfaces.srv import MicrorosSendGoal


class MicrorosSendGoalServer(Node):

    def __init__(self):
        super().__init__('microros_send_goal_server')
        self.srv = self.create_service(
            MicrorosSendGoal,
            '/microros_send_goal',
            self.handle_send_goal
        )
        self.get_logger().info('MicrorosSendGoal service is ready.')

    def handle_send_goal(self, request, response):
        self.get_logger().info(f"Received request with input: {request.input}")
        
        # You can process the input here
        result_str = "Success! Received " + str(len(request.input)) + " items."
        # response.result = result_str
        
        self.get_logger().info(f"Responding with: {response.result}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MicrorosSendGoalServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
