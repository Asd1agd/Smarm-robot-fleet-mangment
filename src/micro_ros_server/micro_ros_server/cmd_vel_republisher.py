import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped
from std_msgs.msg import Header


class CmdVelRepublisher(Node):
    def __init__(self):
        super().__init__('cmd_vel_republisher')

        # Subscriber to /cmd_vel
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publisher to /diff_drive_controller/cmd_vel
        self.publisher = self.create_publisher(
            TwistStamped,
            '/diff_drive_controller/cmd_vel',
            10
        )

        self.get_logger().info('Node started: listening on /cmd_vel and republishing to /diff_drive_controller/cmd_vel (Stamped)')

    def cmd_vel_callback(self, msg: Twist):
        stamped_msg = TwistStamped()
        stamped_msg.header = Header()
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = "base_link"  # optional
        stamped_msg.twist = msg

        self.publisher.publish(stamped_msg)
        self.get_logger().debug('Republished cmd_vel as stamped')


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

