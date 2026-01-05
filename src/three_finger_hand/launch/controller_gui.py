#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Point
import tkinter as tk
from tkinter import ttk
import random

JOINT_ORDER = [
    'finger_1_joint_1', 'finger_1_joint_2', 'finger_1_joint_3', 'finger_1_joint_4',
    'finger_2_joint_1', 'finger_2_joint_2', 'finger_2_joint_3', 'finger_2_joint_4',
    'finger_3_joint_1', 'finger_3_joint_2', 'finger_3_joint_3', 'finger_3_joint_4'
]

class HandControlGUI(Node):
    def __init__(self):
        super().__init__('hand_control_gui')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )

        self.ik_pubs = {
            'Finger 1': self.create_publisher(Point, '/finger_1/goal_pose', 10),
            'Finger 2': self.create_publisher(Point, '/finger_2/goal_pose', 10),
            'Finger 3': self.create_publisher(Point, '/finger_3/goal_pose', 10)
        }

        self.joint_positions = [0.0] * 12

        self.root = tk.Tk()
        self.root.title("Control Panel")
        self.root.geometry("400x900")

        self.sliders = []
        for i, joint_name in enumerate(JOINT_ORDER):
            frame = tk.Frame(self.root)
            frame.pack(pady=2, fill='x', padx=20)

            lbl = tk.Label(frame, text=joint_name, width=20, anchor='w')
            lbl.pack(side='left')

            min_limit, max_limit = JointLimits.JOINT_LIMITS[joint_name]

            slider = tk.Scale(frame, from_=min_limit, to=max_limit, resolution=0.01, orient='horizontal', length=200,
                              command=lambda val, idx=i: self.update_joint(idx, val))
            slider.set(0.0)
            slider.pack(side='right')
            self.sliders.append(slider)

        btn_frame_1 = tk.Frame(self.root)
        btn_frame_1.pack(pady=10)

        btn_reset = tk.Button(btn_frame_1, text="Reset", command=self.reset_all, bg="red", fg="white")
        btn_reset.pack(padx=5, side="left", pady=2)

        btn_random = tk.Button(btn_frame_1, text="Random", command=self.random_positions, bg="blue", fg="white")
        btn_random.pack(padx=5, side="left", pady=2)

        btn_close_hand = tk.Button(btn_frame_1, text="Close hand", command=self.close_fingers, bg="green", fg="white")
        btn_close_hand.pack(padx=5, side="left", pady=2)

        btn_frame_2 = tk.Frame(self.root)
        btn_frame_2.pack(pady=5)

        btn_two_finger_pinch = tk.Button(btn_frame_2, text="Two finger pinch", command=self.two_finger_pinch, bg="purple", fg="white")
        btn_two_finger_pinch.pack(padx=5, side="left", pady=1)

        btn_three_finger_pinch = tk.Button(btn_frame_2, text="Three finger pinch", command=self.three_finger_pinch, bg="orange", fg="white")
        btn_three_finger_pinch.pack(padx=5, side="left", pady=1)

        # IK panel
        ik_frame = tk.Frame(self.root, bd=1, relief="solid", padx=10, pady=10)
        ik_frame.pack(pady=5, padx=20, fill="x")

        self.selected_finger = tk.StringVar(value="Finger 1")
        finger_sel = ttk.OptionMenu(ik_frame, self.selected_finger, "Finger 1", "Finger 1", "Finger 2", "Finger 3")
        finger_sel.pack(pady=2)

        tk.Label(ik_frame, text="X:").pack(side="left")
        self.ent_x = tk.Entry(ik_frame, width=5); self.ent_x.insert(0, "0.045"); self.ent_x.pack(side="left", padx=2)
        tk.Label(ik_frame, text="Y:").pack(side="left")
        self.ent_y = tk.Entry(ik_frame, width=5); self.ent_y.insert(0, "0.015"); self.ent_y.pack(side="left", padx=2)
        tk.Label(ik_frame, text="Z:").pack(side="left")
        self.ent_z = tk.Entry(ik_frame, width=5); self.ent_z.insert(0, "0.19"); self.ent_z.pack(side="left", padx=2)

        # IK status display
        self.ik_status = tk.StringVar(value="")
        tk.Label(ik_frame, textvariable=self.ik_status, fg="red", wraplength=320, justify="left").pack(pady=4, anchor="w")

        btn_solve_ik = tk.Button(self.root, text="Solve IK & Move", command=self.send_ik_goal, bg="black", fg="white")
        btn_solve_ik.pack(pady=5)

        # Circle positioning panel
        circle_frame = tk.Frame(self.root, bd=1, relief="solid", padx=10, pady=10)
        circle_frame.pack(pady=5, padx=20, fill="x")
        tk.Label(circle_frame, text="Radius (m):").pack(side="left")
        self.ent_radius = tk.Entry(circle_frame, width=8)
        self.ent_radius.insert(0, "0.05")
        self.ent_radius.pack(side="left", padx=5)
        btn_circle = tk.Button(circle_frame, text="Position Fingers in Circle", command=self.position_fingers_circle, bg="teal", fg="white")
        btn_circle.pack(side="left", padx=5)
        # -----------------------------------------------

        self.create_subscription(String, '/ik_solver/status', self.on_ik_status, 10)

        self.root.after(10, self.loop_ros)
        self.root.mainloop()

    def send_ik_goal(self):
        try:
            msg = Point()
            msg.x = float(self.ent_x.get())
            msg.y = float(self.ent_y.get())
            msg.z = float(self.ent_z.get())
            finger = self.selected_finger.get()
            self.ik_pubs[finger].publish(msg)
        except ValueError:
            print("Invalid XYZ input")

    def update_joint(self, index, value):
        self.joint_positions[index] = float(value)
        self.publish_commands()

    def reset_all(self):
        for slider in self.sliders:
            slider.set(0.0)
        self.joint_positions = [0.0] * 12
        self.publish_commands()

    def on_ik_status(self, msg):
        # Show latest IK status/warning in the IK panel
        self.ik_status.set(msg.data)

    def position_fingers_circle(self):
        try:
            radius = float(self.ent_radius.get())
            z_height = float(self.ent_z.get())
            # Position 3 fingers equally spaced around a circle
            for i in range(3):
                msg = Point()
                msg.x = radius
                msg.y = 0.0
                msg.z = z_height
                finger_name = f"Finger {i+1}"
                self.ik_pubs[finger_name].publish(msg)
        except ValueError:
            print("Invalid radius or Z value")

    def publish_commands(self):
        msg = Float64MultiArray()
        msg.data = self.joint_positions
        self.publisher_.publish(msg)

    def loop_ros(self):
        rclpy.spin_once(self, timeout_sec=0)
        self.root.after(10, self.loop_ros)

    def random_positions(self):
        for i, joint_name in enumerate(JOINT_ORDER):
            min_limit, max_limit = JointLimits.JOINT_LIMITS[joint_name]
            rand_value = random.uniform(min_limit, max_limit)
            self.sliders[i].set(rand_value)
            self.joint_positions[i] = rand_value
        self.publish_commands()

    def close_fingers(self):
        for i, joint_name in enumerate(JOINT_ORDER):
            min_limit, max_limit = JointLimits.JOINT_LIMITS[joint_name]
            self.sliders[i].set(min_limit)
            self.joint_positions[i] = min_limit
        self.publish_commands()

    def two_finger_pinch(self):
        for i, joint_name in enumerate(JOINT_ORDER):
            positions = JointLimits.JOINT_POSITIONS_FOR_2_FINGER_PINCH[joint_name]
            self.sliders[i].set(positions)
            self.joint_positions[i] = positions
        self.publish_commands()

    def three_finger_pinch(self):
        for i, joint_name in enumerate(JOINT_ORDER):
            positions = JointLimits.JOINT_POSITIONS_FOR_3_FINGER_PINCH[joint_name]
            self.sliders[i].set(positions)
            self.joint_positions[i] = positions
        self.publish_commands()

class JointLimits(Exception):
    JOINT_LIMITS = {
        'finger_1_joint_1': (-1.24, 1.24), 'finger_1_joint_2': (-0.61, 1.24),
        'finger_1_joint_3': (-1.24, 1.24), 'finger_1_joint_4': (-1.5, 1.5),
        'finger_2_joint_1': (-1.24, 1.24), 'finger_2_joint_2': (-0.61, 1.24),
        'finger_2_joint_3': (-1.24, 1.24), 'finger_2_joint_4': (-1.5, 1.5),
        'finger_3_joint_1': (-1.24, 1.24), 'finger_3_joint_2': (-0.61, 1.24),
        'finger_3_joint_3': (-1.24, 1.24), 'finger_3_joint_4': (-1.5, 1.5)
    }
    JOINT_POSITIONS_FOR_2_FINGER_PINCH = {
        'finger_1_joint_1': 0.0, 'finger_1_joint_2': -0.3, 'finger_1_joint_3': -0.75, 'finger_1_joint_4': -0.75,
        'finger_2_joint_1': 0.0, 'finger_2_joint_2': -0.3, 'finger_2_joint_3': -0.75, 'finger_2_joint_4': -0.75,
        'finger_3_joint_1': 0.0, 'finger_3_joint_2': -0.61, 'finger_3_joint_3': -1.24, 'finger_3_joint_4': -1.5
    }
    JOINT_POSITIONS_FOR_3_FINGER_PINCH = {
        'finger_1_joint_1': 0.0, 'finger_1_joint_2': -0.3, 'finger_1_joint_3': -0.6, 'finger_1_joint_4': -0.75,
        'finger_2_joint_1': 0.0, 'finger_2_joint_2': -0.3, 'finger_2_joint_3': -0.6, 'finger_2_joint_4': -0.75,
        'finger_3_joint_1': 0.0, 'finger_3_joint_2': -0.3, 'finger_3_joint_3': -0.6, 'finger_3_joint_4': -0.75
    }

def main():
    try:
        print("Starting Hand Control GUI...")
        rclpy.init()
        gui = HandControlGUI()
        gui.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Shutting down...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("--- GUI Closed ---")
if __name__ == '__main__':
    main()
