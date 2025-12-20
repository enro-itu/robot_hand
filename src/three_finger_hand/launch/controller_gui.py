#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
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

        btn_reset = tk.Button(self.root, text="Reset", command=self.reset_all, bg="red", fg="white")
        btn_reset.pack(pady=8)

        btn_random = tk.Button(self.root, text="Random", command=self.random_positions, bg="blue", fg="white")
        btn_random.pack(pady=8)

        btn_close_hand = tk.Button(self.root, text="Close hand", command=self.close_fingers, bg="green", fg="white")
        btn_close_hand.pack(pady=8)

        btn_two_finger_pinch = tk.Button(self.root, text="Two finger pinch", command=self.two_finger_pinch, bg="purple", fg="white")
        btn_two_finger_pinch.pack(pady=8)

        btn_three_finger_pinch = tk.Button(self.root, text="Three finger pinch", command=self.three_finger_pinch, bg="orange", fg="white")
        btn_three_finger_pinch.pack(pady=8)

        self.root.after(10, self.loop_ros)
        self.root.mainloop()

    def update_joint(self, index, value):
        self.joint_positions[index] = float(value)
        self.publish_commands()

    def reset_all(self):
        for slider in self.sliders:
            slider.set(0.0)
        self.joint_positions = [0.0] * 12
        self.publish_commands()

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
        'finger_1_joint_1': (-1.24, 1.24),
        'finger_1_joint_2': (-0.61, 1.24),
        'finger_1_joint_3': (-1.24, 1.24),
        'finger_1_joint_4': (-1.5, 1.5),
        'finger_2_joint_1': (-1.24, 1.24),
        'finger_2_joint_2': (-0.61, 1.24),
        'finger_2_joint_3': (-1.24, 1.24),
        'finger_2_joint_4': (-1.5, 1.5),
        'finger_3_joint_1': (-1.24, 1.24),
        'finger_3_joint_2': (-0.61, 1.24),
        'finger_3_joint_3': (-1.24, 1.24),
        'finger_3_joint_4': (-1.5, 1.5)
    }

    JOINT_POSITIONS_FOR_2_FINGER_PINCH = {
        'finger_1_joint_1': 0.0,
        'finger_1_joint_2': -0.3,
        'finger_1_joint_3': -0.75,
        'finger_1_joint_4': -0.75,
        'finger_2_joint_1': 0.0,
        'finger_2_joint_2': -0.3,
        'finger_2_joint_3': -0.75,
        'finger_2_joint_4': -0.75,
        'finger_3_joint_1': 0.0,
        'finger_3_joint_2': -0.61,
        'finger_3_joint_3': -1.24,
        'finger_3_joint_4': -1.5
    }

    JOINT_POSITIONS_FOR_3_FINGER_PINCH = {
        'finger_1_joint_1': 0.0,
        'finger_1_joint_2': -0.3,
        'finger_1_joint_3': -0.6,
        'finger_1_joint_4': -0.75,
        'finger_2_joint_1': 0.0,
        'finger_2_joint_2': -0.3,
        'finger_2_joint_3': -0.6,
        'finger_2_joint_4': -0.75,
        'finger_3_joint_1': 0.0,
        'finger_3_joint_2': -0.3,
        'finger_3_joint_3': -0.6,
        'finger_3_joint_4': -0.75
    }

def main():
    rclpy.init()
    gui = HandControlGUI()
    gui.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
