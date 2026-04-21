import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String


class GuiNode(Node):
    def __init__(self):
        super().__init__('gui_node')

        self.target_pub = self.create_publisher(
            Float32MultiArray,
            'target_position',
            10
        )

        self.angles_sub = self.create_subscription(
            Float32MultiArray,
            'joint_angles',
            self.angles_callback,
            10
        )

        self.obstacle_sub = self.create_subscription(
            String,
            'obstacle_status',
            self.obstacle_status_callback,
            10
        )

        self.ik_status_sub = self.create_subscription(
            String,
            'ik_status',
            self.ik_status_callback,
            10
        )

        self._lock = threading.Lock()
        self.latest_angles_deg = [0.0, 0.0, 0.0]
        self.latest_obstacle_status = 'UNKNOWN'
        self.latest_ik_status = 'UNKNOWN'
        self.last_target = [0.0, 0.0, 0.0]

    def send_target(self, x: float, y: float, z: float):
        msg = Float32MultiArray()
        msg.data = [float(x), float(y), float(z)]
        self.target_pub.publish(msg)
        self.get_logger().info(f'Sent target: {msg.data}')

        with self._lock:
            self.last_target = [float(x), float(y), float(z)]

    def angles_callback(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            return
        with self._lock:
            self.latest_angles_deg = [
                math.degrees(msg.data[0]),
                math.degrees(msg.data[1]),
                math.degrees(msg.data[2]),
            ]

    def obstacle_status_callback(self, msg: String):
        with self._lock:
            self.latest_obstacle_status = str(msg.data)

    def ik_status_callback(self, msg: String):
        with self._lock:
            self.latest_ik_status = str(msg.data)

    def get_ui_state(self):
        with self._lock:
            return (
                list(self.latest_angles_deg),
                self.latest_obstacle_status,
                self.latest_ik_status,
                list(self.last_target),
            )


class ArmGUI:
    def __init__(self, node: GuiNode):
        self.node = node
        self.root = tk.Tk()
        self.root.title('Robotic Arm Control')
        self.root.geometry('560x320')
        self.root.resizable(False, False)

        main = ttk.Frame(self.root, padding=12)
        main.pack(fill='both', expand=True)

        left_frame = ttk.LabelFrame(main, text='Target Position', padding=10)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 10))

        right_frame = ttk.LabelFrame(main, text='IK Output / Status', padding=10)
        right_frame.grid(row=0, column=1, sticky='nsew')

        main.columnconfigure(0, weight=1)
        main.columnconfigure(1, weight=1)

        # --- Inputs ---
        ttk.Label(left_frame, text='X').grid(row=0, column=0, sticky='w', pady=3)
        ttk.Label(left_frame, text='Y').grid(row=1, column=0, sticky='w', pady=3)
        ttk.Label(left_frame, text='Z').grid(row=2, column=0, sticky='w', pady=3)

        self.x_entry = ttk.Entry(left_frame, width=16)
        self.y_entry = ttk.Entry(left_frame, width=16)
        self.z_entry = ttk.Entry(left_frame, width=16)

        self.x_entry.grid(row=0, column=1, pady=3, padx=(6, 0))
        self.y_entry.grid(row=1, column=1, pady=3, padx=(6, 0))
        self.z_entry.grid(row=2, column=1, pady=3, padx=(6, 0))

        self.x_entry.insert(0, '1.0')
        self.y_entry.insert(0, '0.0')
        self.z_entry.insert(0, '0.5')

        move_btn = ttk.Button(left_frame, text='Move Arm', command=self.on_move)
        move_btn.grid(row=3, column=0, columnspan=2, pady=(10, 0), sticky='ew')

        # --- IK outputs shown alongside inputs ---
        output_box = ttk.LabelFrame(left_frame, text='Calculated Angles', padding=8)
        output_box.grid(row=4, column=0, columnspan=2, sticky='ew', pady=(12, 0))

        self.theta1_var = tk.StringVar(value='θ1: --')
        self.theta2_var = tk.StringVar(value='θ2: --')
        self.theta3_var = tk.StringVar(value='θ3: --')

        ttk.Label(output_box, textvariable=self.theta1_var).pack(anchor='w', pady=2)
        ttk.Label(output_box, textvariable=self.theta2_var).pack(anchor='w', pady=2)
        ttk.Label(output_box, textvariable=self.theta3_var).pack(anchor='w', pady=2)

        # --- Status on right ---
        self.target_label = ttk.Label(right_frame, text='Target: --')
        self.target_label.pack(anchor='w', pady=3)

        self.obstacle_label = ttk.Label(right_frame, text='Obstacle: UNKNOWN')
        self.obstacle_label.pack(anchor='w', pady=3)

        self.ik_label = ttk.Label(right_frame, text='IK Status: UNKNOWN')
        self.ik_label.pack(anchor='w', pady=3)

        self.note_label = ttk.Label(
            right_frame,
            text='Angles are updated from joint_angles topic',
            wraplength=200
        )
        self.note_label.pack(anchor='w', pady=(10, 0))

        self.root.protocol('WM_DELETE_WINDOW', self.on_close)
        self.root.after(100, self.refresh_labels)

    def on_move(self):
        try:
            x = float(self.x_entry.get())
            y = float(self.y_entry.get())
            z = float(self.z_entry.get())
            self.node.send_target(x, y, z)
        except ValueError:
            self.obstacle_label.config(text='Obstacle: Invalid input')
            self.get_logger_text('Invalid X, Y, Z input')

    def get_logger_text(self, text):
        self.node.get_logger().warn(text)

    def refresh_labels(self):
        angles, obstacle_status, ik_status, target = self.node.get_ui_state()

        self.target_label.config(
            text=f'Target: X={target[0]:.2f}, Y={target[1]:.2f}, Z={target[2]:.2f}'
        )
        self.obstacle_label.config(text=f'Obstacle: {obstacle_status}')
        self.ik_label.config(text=f'IK Status: {ik_status}')

        self.theta1_var.set(f'θ1: {angles[0]:.2f}°')
        self.theta2_var.set(f'θ2: {angles[1]:.2f}°')
        self.theta3_var.set(f'θ3: {angles[2]:.2f}°')

        self.root.after(100, self.refresh_labels)

    def on_close(self):
        self.root.quit()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = GuiNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = ArmGUI(node)
    try:
        app.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()