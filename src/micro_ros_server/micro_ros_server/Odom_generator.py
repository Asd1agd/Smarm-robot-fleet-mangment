import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import math

class OdomBridgeNode(Node):
    def __init__(self):
        super().__init__('Odom_generator')

        # Robot Parameters (update as per your robot)
        self.wheel_radius = 0.05  # meters
        self.ticks_per_rev = 500.0
        self.base_width = 0.30  # meters

        # Internal State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_ticks = None
        self.last_right_ticks = None
        self.last_time = self.get_clock().now()

        # ROS Interfaces
        self.get_logger().info("Subscribing to /esppose")
        self.subscription = self.create_subscription(
            Vector3,
            '/esppose',
            self.pose_callback,
            10
        )

        self.get_logger().info("Publishing to /odom")
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info("TF broadcaster initialized")
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info("OdomBridgeNode initialized and running")

    def pose_callback(self, msg: Vector3):
        current_time = self.get_clock().now()

        # Unpack Vector3
        left_ticks = msg.x
        right_ticks = msg.y
        imu_yaw = math.radians(msg.z)  # Convert degrees to radians
        self.get_logger().info(f"Received ticks: left={left_ticks}, right={right_ticks}, yaw={imu_yaw}")

        # On first run, just store initial values
        if self.last_left_ticks is None or self.last_right_ticks is None:
            self.get_logger().info("First message received, initializing tick counters.")
            self.last_left_ticks = left_ticks
            self.last_right_ticks = right_ticks
            self.theta = imu_yaw
            self.last_time = current_time
            return

        # Calculate deltas
        delta_left = left_ticks - self.last_left_ticks
        delta_right = right_ticks - self.last_right_ticks

        self.get_logger().info(f"Delta ticks: left={delta_left}, right={delta_right}")

        self.last_left_ticks = left_ticks
        self.last_right_ticks = right_ticks

        dist_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        d_left = delta_left * dist_per_tick
        d_right = delta_right * dist_per_tick
        d_center = (d_left + d_right) / 2.0

        self.get_logger().info(f"Wheel distances: left={d_left:.4f} m, right={d_right:.4f} m, center={d_center:.4f} m")

        # Use IMU yaw
        self.theta = imu_yaw

        # Update position
        self.x += d_center * math.cos(self.theta)
        self.y += d_center * math.sin(self.theta)

        dt = (current_time - self.last_time).nanoseconds * 1e-9
        self.last_time = current_time
        vx = d_center / dt if dt > 0 else 0.0
        vth = (d_right - d_left) / self.base_width / dt if dt > 0 else 0.0

        self.get_logger().info(f"Position updated: x={self.x:.4f}, y={self.y:.4f}, θ={self.theta:.4f} rad")
        self.get_logger().info(f"Velocity: linear={vx:.4f} m/s, angular={vth:.4f} rad/s")

        # Build Odometry msg
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.angular.z = vth

        self.odom_pub.publish(odom_msg)
        self.get_logger().info("Published /odom message")

        # Publish TF
        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'  # base_link
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf_msg)
        self.get_logger().info("Published TF from odom → base_footprint")

def main(args=None):
    rclpy.init(args=args)
    node = OdomBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()