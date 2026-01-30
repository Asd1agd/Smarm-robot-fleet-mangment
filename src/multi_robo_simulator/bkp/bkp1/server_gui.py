import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from swarm_interfaces.msg import PathArray
from ament_index_python.packages import get_package_share_directory

import os
import tkinter as tk
from tkinter import Menu
from PIL import Image, ImageTk
import math

GRID_SIZE = 20   # 1/0.05 

class ServerGUI(Node):
    def __init__(self):
        super().__init__('server_gui')
        self.root = tk.Tk()
        self.root.title("Swarm Robot GUI")

        map_path = os.path.join(get_package_share_directory('goat_robo_discription'), 'map', 'my_map.pgm')
        self.original_img = Image.open(map_path).rotate(-90, expand=True)
        self.zoom_factor = 1.0
        self.offset = [0, 0]
        self.tk_img = None

        self.canvas = tk.Canvas(self.root, bg='black')
        self.canvas.pack(fill=tk.BOTH, expand=True)

        self.canvas.bind("<Button-1>", self.left_click)
        self.canvas.bind("<Double-Button-1>", self.double_click)
        self.canvas.bind("<Button-3>", self.right_click)

        self.drawing_path = False
        self.selected_robot = None
        self.robots = {}
        self.robot_ids = set()
        self.status_buttons = {}
        self.path_cells = {}

        self.path_pubs = {}
        self.move_pubs = {}

        self.pose_received = {}

        self.create_timer(0.1, self.loop)
        self.create_timer(1.0, self.check_new_robots)

    def check_new_robots(self):
        topic_list = self.get_topic_names_and_types()
        for topic, types in topic_list:
            if topic.startswith("/bot") and topic.endswith("/spawn_pose"):
                bot_id = topic.split("/")[1]
                if bot_id not in self.robot_ids:
                    self.robot_ids.add(bot_id)
                    self.init_bot(bot_id)

    def init_bot(self, bot_id):
        self.get_logger().info(f"Initializing robot: {bot_id}")
        self.robots[bot_id] = {
            'status': '---',
            'pos': (0, 0),
            'yaw': None,
            'goal': None,
            'moving': True,
            'color': self.assign_color(bot_id)
        }
        self.path_cells[bot_id] = []
        self.pose_received[bot_id] = False

        self.create_subscription(Pose, f"/{bot_id}/spawn_pose", lambda msg, b=bot_id: self.set_initial_pose(b, msg), 10)
        self.create_subscription(Odometry, f"/{bot_id}/odom", lambda msg, b=bot_id: self.update_odom(b, msg), 10)
        self.create_subscription(String, f"/{bot_id}/spawn_status", lambda msg, b=bot_id: self.update_status(b, msg), 10)
        self.create_subscription(String, f"/{bot_id}/status", lambda msg, b=bot_id: self.update_status(b, msg), 10)

        self.path_pubs[bot_id] = self.create_publisher(PathArray, f'/{bot_id}/path', 10)
        self.move_pubs[bot_id] = self.create_publisher(String, f'/{bot_id}/move_or_not', 10)

    def assign_color(self, bot_id):
        colors = ['red', 'green', 'blue', 'orange', 'cyan', 'yellow', 'magenta']
        return colors[hash(bot_id) % len(colors)]

    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def set_initial_pose(self, bot_id, msg):
        x = msg.position.x * GRID_SIZE
        y = msg.position.y * GRID_SIZE
        self.robots[bot_id]['pos'] = (x, y)
        if self.robots[bot_id]['yaw'] is None:
            self.robots[bot_id]['yaw'] = self.quaternion_to_yaw(msg.orientation)
        self.get_logger().info(f"[SPAWN_POSE] {bot_id} POS: ({x:.2f}, {y:.2f}), YAW: {math.degrees(self.robots[bot_id]['yaw']):.2f} deg")
        self.pose_received[bot_id] = True


    def update_odom(self, bot_id, msg):
        x = msg.pose.pose.position.x * GRID_SIZE
        y = msg.pose.pose.position.y * GRID_SIZE
        self.robots[bot_id]['pos'] = (x, y)
        self.robots[bot_id]['yaw'] = self.quaternion_to_yaw(msg.pose.pose.orientation)
        # self.get_logger().info(f"[ODOM] {bot_id} POS: ({x:.2f}, {y:.2f}), YAW: {math.degrees(self.robots[bot_id]['yaw']):.2f} deg")

        row = int(y // GRID_SIZE)
        col = int(x // GRID_SIZE)
        cell = (row, col)

        if bot_id in self.path_cells and self.path_cells[bot_id]:
            if self.path_cells[bot_id][0] == cell:
                self.path_cells[bot_id].pop(0)

    def update_status(self, bot_id, msg):
        self.robots[bot_id]['status'] = msg.data

    def render_map(self):
        resized = self.original_img.resize((int(self.original_img.width * self.zoom_factor),
                                            int(self.original_img.height * self.zoom_factor)),
                                           Image.ANTIALIAS)
        self.tk_img = ImageTk.PhotoImage(resized)
        self.canvas.delete("all")
        self.canvas.create_image(self.offset[0], self.offset[1], anchor='nw', image=self.tk_img)

        for x in range(0, resized.width, int(GRID_SIZE * self.zoom_factor)):
            self.canvas.create_line(x + self.offset[0], self.offset[1],
                                    x + self.offset[0], resized.height + self.offset[1],
                                    fill='lightgrey')
        for y in range(0, resized.height, int(GRID_SIZE * self.zoom_factor)):
            self.canvas.create_line(self.offset[0], y + self.offset[1],
                                    resized.width + self.offset[0], y + self.offset[1],
                                    fill='lightgrey')

    def loop(self):
        self.render_map()
        for bot_id, data in self.robots.items():
            x, y = data['pos']
            yaw = data['yaw'] if data['yaw'] is not None else 0.0

            # self.get_logger().info(f"[DRAW] {bot_id} POS: ({x:.2f}, {y:.2f}), YAW: {math.degrees(yaw):.2f} deg")

            arrow_dx = 25 * math.cos(yaw)
            arrow_dy = 25 * math.sin(yaw)
            start_x = x
            start_y = y - 15
            end_x = start_x + arrow_dx
            end_y = start_y - arrow_dy
            self.canvas.create_line(start_x, start_y, end_x, end_y, fill='red', arrow=tk.LAST, width=3)

            self.canvas.create_oval(x - 10, y - 10, x + 10, y + 10, fill='blue', tags=('robot', bot_id))
            self.canvas.create_text(x, y - 15, text=bot_id, fill='white', tags=('robot', bot_id))
            self.canvas.create_text(x, y + 15, text=data['status'], fill='white', tags=('robot', bot_id))
            if data['goal']:
                gx, gy = data['goal']
                self.canvas.create_rectangle(gx - 8, gy - 8, gx + 8, gy + 8, outline='green', width=2)
                self.canvas.create_text(gx, gy - 10, text=bot_id, fill='green')

        for bot_id, cells in self.path_cells.items():
            color = self.robots[bot_id]['color']
            for row, col in cells:
                x1 = col * GRID_SIZE
                y1 = row * GRID_SIZE
                x2 = x1 + GRID_SIZE
                y2 = y1 + GRID_SIZE
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=color, stipple='gray25')

        self.root.update_idletasks()
        self.root.update()

    def right_click(self, event):
        items = self.canvas.find_overlapping(event.x, event.y, event.x, event.y)
        for item in items:
            tags = self.canvas.gettags(item)
            for tag in tags:
                if tag in self.robots:
                    self.selected_robot = tag
                    menu = Menu(self.root, tearoff=0)
                    menu.add_command(label="New Order", command=self.new_order)
                    menu.add_command(label="Move/Stop", command=self.move_stop)
                    menu.add_command(label="Cancel Order", command=self.cancel_order)
                    menu.tk_popup(event.x_root, event.y_root)
                    return

    def new_order(self):
        self.drawing_path = True
        self.path_cells[self.selected_robot] = []
        self.root.configure(bg='gray25')

    def left_click(self, event):
        if self.drawing_path and self.selected_robot:
            row = event.y // GRID_SIZE
            col = event.x // GRID_SIZE
            cells = self.path_cells[self.selected_robot]

            if not cells:
                x, y = self.robots[self.selected_robot]['pos']
                r0 = int(y // GRID_SIZE)
                c0 = int(x // GRID_SIZE)
                cells.append((r0, c0)) 
                if r0 == row:
                    step = 1 if c0 < col else -1
                    for c in range(c0 + step, col + step, step):
                        cells.append((r0, c))
                elif c0 == col:
                    step = 1 if r0 < row else -1
                    for r in range(r0 + step, row + step, step):
                        cells.append((r, c0))
            else:
                r0, c0 = cells[-1]
                if r0 == row:
                    step = 1 if c0 < col else -1
                    for c in range(c0 + step, col + step, step):
                        cells.append((r0, c))
                elif c0 == col:
                    step = 1 if r0 < row else -1
                    for r in range(r0 + step, row + step, step):
                        cells.append((r, c0))

    def angle_between(self, p1, p2):
        dy = p2[0] - p1[0]
        dx = p2[1] - p1[1]
        return math.atan2(-dy, dx)

    def angle_diff(self, a1, a2):
        d = a2 - a1
        while d > math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        return d

    def generate_path_commands(self, bot_id, cells, yaw):
        cmds = []
        if len(cells) < 2:
            return cmds

        current_angle = yaw
        i = 0
        buffer = 5

        while i + 1 < len(cells):
            p1 = cells[i]
            p2 = cells[i + 1]
            segment_angle = self.angle_between(p1, p2)
            diff = self.angle_diff(current_angle, segment_angle)
            deg_diff = round(math.degrees(diff))

            if abs(deg_diff) <= buffer:
                cmds.append(1)
                current_angle = segment_angle
                i += 1
            else:
                if abs(deg_diff - 180) <= buffer:
                    cmds.append(-6)
                    cmds.append(-6)
                    current_angle += math.radians(180)
                elif abs(deg_diff - 90) <= buffer or abs(deg_diff + 270) <= buffer:
                    cmds.append(-6)
                    current_angle += math.radians(90)
                elif abs(deg_diff + 90) <= buffer or abs(deg_diff - 270) <= buffer:
                    cmds.append(-3)
                    current_angle -= math.radians(90)
                else:
                    cmds.append(-6)
                    current_angle = segment_angle

        optimized_cmds = []
        count = 0
        for cmd in cmds:
            if cmd == 1:
                count += 1
            else:
                if count > 0:
                    optimized_cmds.append(count)
                    count = 0
                optimized_cmds.append(cmd)
        if count > 0:
            optimized_cmds.append(count)

        return optimized_cmds

    def double_click(self, event):
        if self.drawing_path and self.selected_robot:
            x = event.x
            y = event.y
            self.robots[self.selected_robot]['goal'] = (x, y)
            cells = self.path_cells[self.selected_robot]
            yaw = self.robots[self.selected_robot]['yaw'] if self.robots[self.selected_robot]['yaw'] is not None else 0.0
            commands = []
            if len(cells) >= 2:
                commands = self.generate_path_commands(self.selected_robot, cells, yaw)
            else:
                self.get_logger().warn(f"Not enough path points for {self.selected_robot} to generate commands.")
            self.get_logger().info(f"Commands for {self.selected_robot}: {commands}")
            msg = PathArray()
            msg.path = commands
            self.path_pubs[self.selected_robot].publish(msg)
            self.drawing_path = False
            self.root.configure(bg='SystemButtonFace')

    def move_stop(self):
        bot_id = self.selected_robot
        current_state = self.robots[bot_id].get("moving", False)
        new_state = not current_state
        self.robots[bot_id]["moving"] = new_state

        msg = String()
        msg.data = 'm' if new_state else 's'
        self.move_pubs[bot_id].publish(msg)

    def cancel_order(self):
        if self.selected_robot:
            msg = String()
            msg.data = 's'
            self.move_pubs[self.selected_robot].publish(msg)
            self.robots[self.selected_robot]['goal'] = None
            self.path_cells[self.selected_robot] = []

def main():
    rclpy.init()
    gui = ServerGUI()
    try:
        rclpy.spin(gui)
    except KeyboardInterrupt:
        pass
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
