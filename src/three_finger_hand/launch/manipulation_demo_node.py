#!/usr/bin/env python3

import os
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import ikpy.chain
import numpy as np
from ament_index_python.packages import get_package_share_directory

class ManipulationDemoNode(Node):
    def __init__(self):
        super().__init__('manipulation_demo_node')

        pkg_share = get_package_share_directory('three_finger_hand')
        xacro_path = os.path.join(pkg_share, 'urdf', 'three_finger_hand.urdf.xacro')
        urdf_temp_path = "/tmp/hand_robot_demo.urdf"
        subprocess.run(f"xacro {xacro_path} > {urdf_temp_path}", shell=True, check=True)

        self.chains = {
            'f1': ikpy.chain.Chain.from_urdf_file(urdf_temp_path, base_elements=["palm"], name="finger_1"),
            'f2': ikpy.chain.Chain.from_urdf_file(urdf_temp_path, base_elements=["palm"], name="finger_2"),
            'f3': ikpy.chain.Chain.from_urdf_file(urdf_temp_path, base_elements=["palm"], name="finger_3")
        }

        for name, chain in self.chains.items():
            self.get_logger().info(f"{name} chain loaded with {len(chain.links)} links.")

        self.cmd_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)

        self.state = "OPEN"
        self.start_time = self.get_clock().now()
        self.joint_state = [0.0] * 12 # Current joint angles
        self.initial_angles = [0.0] * 12 # Initial (open) pose
        self.wrap_angles = [0.0] * 12 # IK-computed wrap pose

        # Target: Cylinder (0, 0, 0.20)
        self.obj_pos = [0.0, 0.0, 0.20]
        self.grasp_radius = 0.07 # Close (wrapped) radius

        # Control Loop (10 Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info("Manipulation Demo Started: State = OPEN")

    def get_ik_for_radius(self, radius):
        angles_12 = [0.0] * 12
        z_target = self.obj_pos[2]
        angles = [0, 4.1888, 2.0944]
        sqrt3 = np.sqrt(3)

        for i in range(3):
            target_local = [radius * np.cos(angles[i]),
                        radius * np.sin(angles[i]),
                        z_target]

            chain = self.chains[f'f{i+1}']
            ik_res = chain.inverse_kinematics(target_local)

            real_angles = [angle for j, angle in enumerate(ik_res) if chain.links[j].name.find("joint") != -1]
            angles_12[i*4 : i*4+len(real_angles)] = real_angles[:4]
        return angles_12

    def apply_synergy(self, base_angles, g):
        tighten_offset = 0.3
        new_angles = list(base_angles)
        for i in range(12):
            if i % 4 in [1, 3]:
                new_angles[i] += g * tighten_offset
        return new_angles

    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        msg = Float64MultiArray()

        if self.state == "OPEN":
            msg.data = [0.0] * 12
            if elapsed > 1.5:
                self.wrap_angles = self.get_ik_for_radius(self.grasp_radius)
                self.transition("WRAP", now)

        elif self.state == "WRAP":
            alpha = min(1.0, elapsed / 2.5) # 2.5 seconds ramp
            msg.data = (np.array(self.initial_angles) * (1-alpha) + np.array(self.wrap_angles) * alpha).tolist()
            if elapsed > 3.0:
                self.transition("CLOSE", now)

        elif self.state == "CLOSE":
            # Grasp Synergy: Ramp g from 0 to 1
            g = min(1.0, elapsed / 1.0)
            msg.data = self.apply_synergy(self.wrap_angles, g)
            if elapsed > 1.5:
                self.transition("HOLD", now)

        elif self.state == "HOLD":
            msg.data = self.apply_synergy(self.wrap_angles, 1.0)
            if elapsed > 4.0:
                self.get_logger().info("Demo Successfully Completed!")
                # Optional: self.transition("OPEN", now) to loop back.

        self.cmd_pub.publish(msg)

    def transition(self, next_state, now):
        self.get_logger().info(f"State Transition: {self.state} -> {next_state}")
        self.state = next_state
        self.start_time = now

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ManipulationDemoNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected, shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
