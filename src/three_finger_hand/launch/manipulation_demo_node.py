#!/usr/bin/env python3

import sys
import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import String
from geometry_msgs.msg import Point
import ikpy.chain
import numpy as np
from ament_index_python.packages import get_package_share_directory
from functools import partial

class ManipulationDemoNode(Node):
    def __init__(self):
        super().__init__('manipulation_demo_node')

        pkg_share = get_package_share_directory('three_finger_hand')
        xacro_path = os.path.join(pkg_share, 'urdf', 'three_finger_hand.urdf.xacro')
        urdf_temp_path = "/tmp/hand_robot_debug.urdf"

        print(f"Xacro path: {xacro_path}")
        subprocess.run(f"xacro {xacro_path} > {urdf_temp_path}", shell=True, check=True)

        self.chains = {
            'finger_1': ikpy.chain.Chain.from_urdf_file(urdf_temp_path, base_elements=["palm"], name="finger_1"),
            'finger_2': ikpy.chain.Chain.from_urdf_file(urdf_temp_path, base_elements=["palm"], name="finger_2"),
            'finger_3': ikpy.chain.Chain.from_urdf_file(urdf_temp_path, base_elements=["palm"], name="finger_3")
        }

        for name, chain in self.chains.items():
            print(f"{name} chain loaded with {len(chain.links)} links.")

        self.cmd_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)

        # Timer to run demo sequence
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz
        self.state = "OPEN"
        self.state_start_time = self.get_clock().now()

        # Targets for fingers
        self.obj_center = np.array([0.0, 0.0, 0.20])
        self.grasp_radius = 0.07  # 7 cm # for wrapping around object
        self.g_param = 0.0

    def get_wrap_angles(self):
        pass # Probably unnecessary, and may be done by ik_solver_node.py

    def control_loop(self):
        current_time = self.get_clock().now()
        elapsed = (current_time - self.state_start_time).nanoseconds / 1e9

        if self.state == "OPEN":
            self.send_joint_command(self.open_pose)
            if elapsed > 1.0:
                self.state = "WRAP"
                self.state_start_time = current_time

        elif self.state == "WRAP":
            target_angles = self.get_wrap_angles()
            # TODO: Add iterations for smooth movement (optional)
            self.send_joint_command(target_angles)
            if elapsed > 3.0:
                self.state = "CLOSE"
                self.state_start_time = current_time

        elif self.state == "CLOSE":
            # Ramp g_param from 0 to 1 over 1 second
            self.g_param = min(1.0, elapsed / 1.0) # Clamp to 1.0
            current_angles = self.apply_synergy(self.get_wrap_angles(), self.g_param)
            self.send_joint_command(current_angles)
            if elapsed > 1.5:
                self.state = "HOLD"
                self.state_start_time = current_time

        elif self.state == "HOLD":
            # Maintain closed position
            if elapsed > 4.0:
                print("Demo completed successfully.")
            # TODO: Add functionality to a small movement or release after hold (optional)
