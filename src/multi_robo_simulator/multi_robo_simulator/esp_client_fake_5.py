import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from swarm_interfaces.msg import PathArray
import time
import threading


class PathExecutor(Node):
    def __init__(self):
        super().__init__('esp_client_fake_5')

        self.cmd_pub = self.create_publisher(Twist, '/robot5/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/bot5/status', 10)

        self.create_subscription(PathArray, '/bot5/path', self.path_callback, 10)
        self.create_subscription(JointState, '/robot5/joint_states', self.joint_callback, 10)

        self.prev_positions = [None, None]  # [left, right]
        self.deltas = [0.0, 0.0]  # [left_delta, right_delta]

        self.commands = []
        self.executing = False
        self.execution_thread = None
        self.current_cmd = 0

        self.timer = self.create_timer(0.1, self.execute_command_loop)
        self.execution_thread = threading.Thread(target=self.execute_command)
        self.execution_thread.start()

    def path_callback(self, msg):
        if self.executing:
            self.get_logger().info("Already executing a path. Ignoring new path.")
            return

        self.commands = list(msg.path)
        self.get_logger().info(f"Received path: {self.commands}")
        self.executing = True
        self.publish_status("started")

    def joint_callback(self, msg):
        try:
            left_idx = next(i for i, name in enumerate(msg.name) if "left_wheel_joint" in name)
            right_idx = next(i for i, name in enumerate(msg.name) if "right_wheel_joint" in name)

            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]

            if self.prev_positions[0] is None:
                self.prev_positions = [left_pos, right_pos]
                return

            self.deltas[0] = left_pos - self.prev_positions[0]
            self.deltas[1] = right_pos - self.prev_positions[1]

            self.prev_positions = [left_pos, right_pos]

        except StopIteration:
            self.get_logger().warn("Could not find wheel joints in joint_states!")

    def execute_command_loop(self):
        if not self.executing or self.execution_thread is not None:
            return

        if not self.commands:
            self.executing = False
            self.publish_status("stopped")
            return

        self.current_cmd = self.commands[0]

    def execute_command(self):
        while True:
            if self.commands:
                current_cmd = self.commands[0]
                self.publish_status("moving")
                twist = Twist()

                if current_cmd > 0:
                    twist.linear.x = 1.0
                    target_distance = float(current_cmd)
                    self.get_logger().info(f"Moving forward {target_distance} meters")
                    self.cmd_pub.publish(twist)
                    self.wait_until_distance_reached(target_distance)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.publish_status("reached")
                    time.sleep(0.5)

                elif current_cmd < 0:
                    twist.angular.z = -1.0 if current_cmd == -3 else 1.0
                    direction = "right" if current_cmd == -3 else "left"
                    target_angle = 1.57  # radians ~90°
                    self.get_logger().info(f"Turning {direction}")
                    self.cmd_pub.publish(twist)
                    self.wait_until_angle_reached(target_angle)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.publish_status("reached")
                    time.sleep(0.5)

                else:
                    self.get_logger().info("Skipping zero command.")

                self.stop_robot()
                self.commands.pop(0)

                if not self.commands:
                    self.executing = False
                    self.publish_status("stopped")
                    self.get_logger().info("✅ All path commands executed.")

                self.execution_thread = None  # Allow next command to start

    def wait_until_distance_reached(self, target):
        accumulated = 0.0
        p_pose = self.prev_positions[0]
        wheel_radius = 0.1
        while abs(accumulated) < abs(target):
            # rclpy.spin_once(self, timeout_sec=0.1)
            # delta = (self.deltas[0] + self.deltas[1]) / 2.0
            c_pose = self.prev_positions[0]
            delta = abs(c_pose - p_pose)
            accumulated = delta * wheel_radius
            self.publish_status(f"moving ({accumulated:.2f}/{target})")
            time.sleep(0.001)
        self.publish_status("waiting")

    def wait_until_angle_reached(self, target_rad):
        accumulated = 0.0
        chasis_center_to_wheel = 0.425/2 #438
        wheel_radius = 0.1
        p_pose = self.prev_positions[0]
        # arc_length = (target_rad * chasis_center_to_wheel * 0.1) / 2.0
        arc_length = (3.142 * chasis_center_to_wheel)/(2 * wheel_radius)

        while abs(accumulated) <= abs(arc_length)*1.05:
            # rclpy.spin_once(self, timeout_sec=0.1)
            c_pose = self.prev_positions[0]
            delta = (self.deltas[0] - self.deltas[1]) / 2.0
            accumulated = abs(p_pose) - abs(c_pose)
            self.publish_status(f"turning ({accumulated:.2f}/{arc_length:.2f})")
            time.sleep(0.001)
        self.publish_status("waiting")

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def publish_status(self, msg_str):
        msg = String()
        msg.data = msg_str
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
