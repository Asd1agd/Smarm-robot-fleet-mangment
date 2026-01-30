import rclpy
from rclpy.node import Node
from rclpy.duration import Duration as RclpyDuration
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
import math
from visualization_msgs.msg import MarkerArray



class VirtualQRPublisher(Node):

    def __init__(self):
        super().__init__('virtual_qr_publisher')

        # Parameters
        self.range_m = 0.08  # Distance threshold to publish QR
        self.checked_qrs = set()

        # Virtual QR codes in a 5x5 grid (2m spacing, centered at origin)
        self.virtual_qrs = [
            (-4.0,  4.0, "qr1"),  (-2.0,  4.0, "qr2"),  ( 0.0,  4.0, "qr3"),  ( 2.0,  4.0, "qr4"),  ( 4.0,  4.0, "qr5"),
            (-4.0,  2.0, "qr6"),  (-2.0,  2.0, "qr7"),  ( 0.0,  2.0, "qr8"),  ( 2.0,  2.0, "qr9"),  ( 4.0,  2.0, "qr10"),
            (-4.0,  0.0, "qr11"), (-2.0,  0.0, "qr12"), ( 0.0,  0.0, "qr13"), ( 2.0,  0.0, "qr14"), ( 4.0,  0.0, "qr15"),
            (-4.0, -2.0, "qr16"), (-2.0, -2.0, "qr17"), ( 0.0, -2.0, "qr18"), ( 2.0, -2.0, "qr19"), ( 4.0, -2.0, "qr20"),
            (-4.0, -4.0, "qr21"), (-2.0, -4.0, "qr22"), ( 0.0, -4.0, "qr23"), ( 2.0, -4.0, "qr24"), ( 4.0, -4.0, "qr25"),
        ]

        # Marker publisher
        # self.marker_pub = self.create_publisher(Marker, '/qr_markers', 100)
        self.marker_array_pub = self.create_publisher(MarkerArray, "/qr_marker_array", 50)

        # QR code string publisher
        self.qr_pub = self.create_publisher(String, '/microROS_camera_string', 10)

        # Odometry subscriber
        self.odom_sub = self.create_subscription(
            Odometry,
            '/diff_drive_controller/odom',
            self.odom_callback,
            10
        )

        # Timer for marker publication every 3 seconds
        self.marker_timer = self.create_timer(0.5, self.publish_qr_markers)

        self.get_logger().info("Virtual QR Publisher Node has started.")

    def publish_qr_markers(self):
        marker_array = MarkerArray()

        for idx, (x, y, qr_data) in enumerate(self.virtual_qrs):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "qr_markers"
            marker.id = idx
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            marker.scale.z = 0.4
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.text = qr_data
            marker.lifetime = Duration(sec=0)  # 0 = forever

            marker_array.markers.append(marker)

        self.marker_array_pub.publish(marker_array)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        for qr_x, qr_y, qr_data in self.virtual_qrs:
            dist = math.sqrt((qr_x - x)**2 + (qr_y - y)**2)
            if dist <= self.range_m:
                # self.get_logger().info(f"Detected QR: {qr_data} at ({qr_x:.2f}, {qr_y:.2f})")
                self.qr_pub.publish(String(data=f"{qr_data}."))


def main(args=None):
    rclpy.init(args=args)
    node = VirtualQRPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
