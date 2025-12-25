#!/usr/bin/env python3

print("Script starting...")

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

class IKSolverNode(Node):
    def __init__(self):
        super().__init__('ik_solver_node')

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

        self.publisher_ = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)
        self.status_pub = self.create_publisher(String, '/ik_solver/status', 10)
        # Track latest commanded joint state so IK updates stay in sync with GUI actions like reset
        self.create_subscription(Float64MultiArray, '/forward_position_controller/commands', self.update_joint_state, 10)
        
        self.create_subscription(Point, '/finger_1/goal_pose', partial(self.calculate_ik_callback, finger_id=1), 10)
        self.create_subscription(Point, '/finger_2/goal_pose', partial(self.calculate_ik_callback, finger_id=2), 10)
        self.create_subscription(Point, '/finger_3/goal_pose', partial(self.calculate_ik_callback, finger_id=3), 10)

        self.joint_state = [0.0] * 12
        print("IK Solver Node initialized for 3 fingers.")

    def update_joint_state(self, msg):
        # Keep internal state aligned with last published GUI command
        if len(msg.data) == 12:
            self.joint_state = list(msg.data)

    def calculate_ik_callback(self, msg, finger_id):
        finger_name = f'finger_{finger_id}'
        print(f"[{finger_name}] Target Received: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")

        try:
            target_pos = [msg.x, msg.y, msg.z]
            chain = self.chains[finger_name]

            ik_angles = chain.inverse_kinematics(target_pos)

            computed_frame = chain.forward_kinematics(ik_angles)
            computed_pos = computed_frame[:3, 3]

            error = np.linalg.norm(np.array(target_pos) - np.array(computed_pos))

            if error > 0.01:
                self.get_logger().warn(f"Target UNREACHABLE for {finger_name}! Error distance: {error:.4f}m")
                warn_msg = String()
                warn_msg.data = f"Unreachable target"
                self.status_pub.publish(warn_msg)
                return

            new_angles = ik_angles[1:5].tolist()

            start_idx = (finger_id - 1) * 4
            self.joint_state[start_idx : start_idx + 4] = new_angles

            cmd = Float64MultiArray()
            cmd.data = self.joint_state
            self.publisher_.publish(cmd)

            ok_msg = String()
            ok_msg.data = "" # Clear any previous warnings
            self.status_pub.publish(ok_msg)

            print(f"[{finger_name}] Move command sent. Precision: {error:.5f}m")

        except Exception as e:
            self.get_logger().error(f"IK Calculation Error for {finger_name}: {e}")
            err_msg = String()
            err_msg.data = "Unreachable target"
            self.status_pub.publish(err_msg)

def main():
    try:
        rclpy.init()
        node = IKSolverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Keyboard Interrupt detected. Shutting down...")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        print("--- Script Closed ---")

if __name__ == '__main__':
    main()
