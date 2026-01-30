#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from micro_ros_custom_interfaces.srv import MicrorosSendGoal  # auto-generated from .srv

class SendGoalClient(Node):
    def __init__(self):
        super().__init__('send_goal_client')
        self.cli = self.create_client(MicrorosSendGoal, '/microros_send_goal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = MicrorosSendGoal.Request()

    def send_request(self, input_list):
        self.get_logger().info('sending')
        self.req.input = input_list
        self.get_logger().info('send wating for reply...')
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    client = SendGoalClient()
    client.send_request(["QR1", "QR2", "QR3"])

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().error(f'Service call failed: {e}')
            else:
                client.get_logger().info(f'Result: {response.result}')
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
