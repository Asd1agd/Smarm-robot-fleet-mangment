import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from swarm_interfaces.msg import PathArray

import tkinter as tk
from functools import partial
import subprocess


class RobotSpawnerGUI(Node):
    def __init__(self):
        super().__init__('robot_spawner_gui')

        self.bot_ids = {
            "bot1": (0.5, 0.5, 0.0),
            "bot2": (2.5, 2.5, 0.0),
            "bot3": (3.5, 3.5, 0.0),
            "bot4": (5.5, 3.5, 0.0),
            "bot5": (7.5, 9.5, 0.0),
            "bot6": (10.5, 9.5, 0.0),
        }

        self.status_subs = {}
        self.path_subs = {}
        self.pose_pubs = {}
        self.status_pubs = {}

        self.status_labels = {}
        self.path_labels = {}
        self.full_paths = {}
        self.popups = {}

        # GUI Setup
        self.root = tk.Tk()
        self.root.title("Robot Spawner GUI")

        self.setup_gui()

        # Periodically update GUI
        self.timer = self.create_timer(0.5, self.tk_loop)

    def setup_gui(self):
        # Column titles
        tk.Label(self.root, text="Robots", font=('Arial', 14, 'bold')).grid(row=0, column=0)
        tk.Label(self.root, text="Status", font=('Arial', 14, 'bold')).grid(row=0, column=1)
        tk.Label(self.root, text="First 5 Commands", font=('Arial', 14, 'bold')).grid(row=0, column=2)

        for idx, (bot_id, pose) in enumerate(self.bot_ids.items(), start=1):
            btn = tk.Button(self.root, text=bot_id, width=10,
                            command=partial(self.launch_robot, bot_id, pose))
            btn.grid(row=idx, column=0)

            # Status label
            status_lbl = tk.Label(self.root, text="---", width=15)
            status_lbl.grid(row=idx, column=1)
            self.status_labels[bot_id] = status_lbl

            # Path label with click handler
            path_lbl = tk.Label(self.root, text="---", width=30, fg="blue", cursor="hand2")
            path_lbl.grid(row=idx, column=2)
            path_lbl.bind("<Button-1>", partial(self.show_full_path_popup, bot_id))
            self.path_labels[bot_id] = path_lbl

            # ROS communication setup
            self.pose_pubs[bot_id] = self.create_publisher(Pose, f'/{bot_id}/spawn_pose', 10)
            self.status_pubs[bot_id] = self.create_publisher(String, f'/{bot_id}/spawn_status', 10)

            self.status_subs[bot_id] = self.create_subscription(
                String, f'/{bot_id}/status',
                partial(self.status_callback, bot_id), 10)

            self.path_subs[bot_id] = self.create_subscription(
                PathArray, f'/{bot_id}/path',
                partial(self.path_callback, bot_id), 10)

    def launch_robot(self, bot_id, pose_tuple):
        x, y, yaw = pose_tuple
        robot_name = bot_id.replace("bot", "robot")

        # Launch ROS 2 Gazebo robot spawn
        cmd = [
            "ros2", "launch", "goat_robo_discription", "gazibo_goat_spawn.launch.py",
            f"robot_name:={robot_name}",
            f"x:={x}",
            f"y:={y}",
            f"yaw:={yaw}"
        ]

        try:
            subprocess.Popen(cmd)
            self.get_logger().info(f"✅ Launched robot with command: {' '.join(cmd)}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to launch {robot_name}: {e}")

        # Launch ROS 2 Gazebo robot spawn
        cmd2 = [
            "ros2", "run", "multi_robo_simulator", f"esp_client_fake_{robot_name[-1]}",
        ]

        try:
            subprocess.Popen(cmd2)
            self.get_logger().info(f"✅ run clien robot with command: {' '.join(cmd2)}")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to run {robot_name}: {e}")

        # Also publish spawn pose and status (optional)
        pose_msg = Pose()
        pose_msg.position.x = x
        pose_msg.position.y = y
        pose_msg.orientation.z = yaw  # Simplified yaw only

        self.pose_pubs[bot_id].publish(pose_msg)

        status_msg = String()
        status_msg.data = "launched"
        self.status_pubs[bot_id].publish(status_msg)

    def status_callback(self, bot_id, msg):
        self.status_labels[bot_id]['text'] = msg.data

    def path_callback(self, bot_id, msg):
        self.full_paths[bot_id] = msg.path
        path_text = ', '.join(map(str, msg.path[:5])) + ("..." if len(msg.path) > 5 else "")
        self.path_labels[bot_id]['text'] = path_text

    def show_full_path_popup(self, bot_id, event=None):
        if bot_id in self.popups and self.popups[bot_id].winfo_exists():
            self.popups[bot_id].deiconify()
            self.popups[bot_id].lift()
            return

        popup = tk.Toplevel(self.root)
        popup.title(f"{bot_id} Full Path")
        popup.geometry("400x150")
        popup.resizable(False, False)

        path = self.full_paths.get(bot_id, [])
        path_str = ', '.join(map(str, path)) if path else "No path data available."

        tk.Label(popup, text=f"Full path for {bot_id}:", font=('Arial', 12, 'bold')).pack(pady=5)
        path_text_box = tk.Text(popup, wrap="word", width=50, height=5)
        path_text_box.pack()
        path_text_box.insert(tk.END, path_str)
        path_text_box.config(state='disabled')

        self.popups[bot_id] = popup

    def tk_loop(self):
        self.root.update_idletasks()
        self.root.update()


def main(args=None):
    rclpy.init(args=args)
    gui_node = RobotSpawnerGUI()
    try:
        rclpy.spin(gui_node)
    except KeyboardInterrupt:
        pass
    gui_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
