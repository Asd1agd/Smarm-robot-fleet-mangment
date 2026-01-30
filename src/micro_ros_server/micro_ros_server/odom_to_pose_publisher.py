import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

import math

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # Publisher: publishes x, y, yaw
        self.publisher_ = self.create_publisher(Float32MultiArray, '/microROS_current_pose', 10)

        # Subscriber: subscribes to /odom
        self.subscription = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        # Extract x, y from position
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Prepare message
        pose_array = Float32MultiArray()
        pose_array.data = [x, y, yaw]

        # Publish
        self.publisher_.publish(pose_array)
        self.get_logger().info(f'Published Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

