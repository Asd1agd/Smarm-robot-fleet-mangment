import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from swarm_interfaces.msg import PathArray
import time
import threading
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math



class PathExecutor(Node):
    def __init__(self):
        super().__init__('esp_client_fake_1')

        self.cmd_pub = self.create_publisher(Twist, '/robot1/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/bot1/status', 10)

        self.create_subscription(PathArray, '/bot1/path', self.path_callback, 10)
        self.create_subscription(JointState, '/robot1/joint_states', self.joint_callback, 10)
        self.pose_log = []  # List to store global x, y, w
        self.create_subscription(Odometry, '/robot1/Odometry', self.odom_callback, 10)


        self.prev_positions = [None, None]  # [left, right]
        self.deltas = [0.0, 0.0]  # [left_delta, right_delta]

        self.commands = []
        self.executing = False
        self.execution_thread = None
        self.current_cmd = 0

        # self.timer = self.create_timer(0.1, self.execute_command_loop)
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


    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation

        # Convert quaternion to Euler angles
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw_rad) = euler_from_quaternion(orientation_list)

        # Convert yaw from radians to degrees
        yaw_deg = math.degrees(yaw_rad)

        x = position.x
        y = position.y

        yaw_deg_n = self.map_value(yaw_deg, -180, 0, 180, 360)

        if yaw_deg >= 0:
            yaw_deg_new = yaw_deg
        else:
            yaw_deg_new = yaw_deg_n

        # Store (x, y, yaw in degrees)
        self.pose_log = [x, y, yaw_deg_new]

        # Optional: Trim list length
        if len(self.pose_log) > 100:
            self.pose_log.pop(0)

        self.get_logger().debug(f"Pose: x={x:.2f}, y={y:.2f}, yaw={yaw_deg:.2f}°")



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
                    # time.sleep(0.2)
                    # self.publish_status("reached")
                    # rclpy.spin_once(self, timeout_sec=0.01)  # If inside a node
                    # time.sleep(0.5)

                elif current_cmd < 0:
                    twist.angular.z = -0.5 if current_cmd == -3 else 0.5
                    direction = "right" if current_cmd == -3 else "left"
                    if direction == "right":
                        target_angle = -90  # radians ~90°
                    else:
                        target_angle = 90  # radians ~90°
                    self.get_logger().info(f"Turning {direction}")
                    self.cmd_pub.publish(twist)
                    self.wait_until_angle_reached(target_angle)
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    # time.sleep(0.2)
                    # self.publish_status("reached")
                    # rclpy.spin_once(self, timeout_sec=0.01) # If inside a node
                    # time.sleep(0.5)

                else:
                    self.get_logger().info("Skipping zero command.")
                
                # time.sleep(0.2)
                # self.get_logger().info(" reached ")
                # self.publish_status("reached")
                # time.sleep(0.5)
                self.stop_robot()
                self.commands.pop(0)

                if not self.commands:
                    self.executing = False
                    self.publish_status("stopped")
                    self.get_logger().info("✅ All path commands executed.")

                self.execution_thread = None  # Allow next command to start

    def map_value(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def classify_angle(self, target_ang, buffer=40):
        # Normalize angle to [0, 360)
        target_ang = target_ang % 360

        if abs(target_ang - 0) <= buffer or abs(target_ang - 360) <= buffer:
            return 0
        elif abs(target_ang - 90) <= buffer:
            return 90
        elif abs(target_ang - 180) <= buffer:
            return 180
        elif abs(target_ang - 270) <= buffer:
            return 270
        else:
            return -1  # Not close to any



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
        # self.publish_status("waiting")
        self.publish_status("reached")
        self.get_logger().info(" reached ")
        time.sleep(0.5)

    def wait_until_angle_reached(self, target_deg):
        accumulated = 0.0
        chasis_center_to_wheel = 0.425/2 #438
        wheel_radius = 0.1
        p_pose = self.prev_positions[0]
        # arc_length = (target_rad * chasis_center_to_wheel * 0.1) / 2.0
        arc_length = (3.142 * chasis_center_to_wheel)/(2 * wheel_radius)

        buffer = 0.5
        pre_w = self.pose_log[-1] # degree 0 to 360
        target_ang = pre_w + target_deg

        if target_ang > 360:
            target_ang -= 360

        target_ang = self.classify_angle(target_ang)

        error = 90


        while abs(error) >= abs(buffer):
            # rclpy.spin_once(self, timeout_sec=0.1)
            cur_w = self.pose_log[-1] # degree 0 to 360
            error = abs(cur_w - target_ang)
            if error > 180:
                error = 360 - error
            self.publish_status(f"turning:{error:.2f} | target:{target_ang})")
            # self.get_logger().info(f"cur_w:{cur_w} | target_ang:{target_ang} | error:{error}")
            time.sleep(0.001)
        # self.publish_status("waiting")
        self.publish_status("reached")
        self.get_logger().info(" reached ")
        time.sleep(0.5)

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
