#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import time
import os
import numpy as np
import subprocess

class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')

        self.ik_pubs = {
            'Finger 1': self.create_publisher(Point, '/finger_1/goal_pose', 10),
            'Finger 2': self.create_publisher(Point, '/finger_2/goal_pose', 10),
            'Finger 3': self.create_publisher(Point, '/finger_3/goal_pose', 10)
        }
        self.joint_cmd_pub = self.create_publisher(Float64MultiArray, '/forward_position_controller/commands', 10)

        self.state = "OPEN"
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.control_loop)

        # File paths
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.waypoints_path = os.path.join(self.script_dir, 'src', 'waypoints.npy')

        self.get_logger().info("Manipulation demo started")

    def run_circle_generator(self): # To run circle_generator.py internally
        try:
            # Operated command: -c 0 0 0.2 -r 0.06 -p 3 -o -90
            command = [
                "python3", "circle_generator.py",
                "-c", "0", "0", "0.2",
                "-n", "0", "0", "1",
                "-r", "0.06", "-p", "3", "-o", "-90"
            ]
            subprocess.run(command, check=True, cwd=self.script_dir)
            self.get_logger().info("Circle generator operated.")
        except Exception as e:
            self.get_logger().error(f"Generator error: {e}")

    def position_fingers_circle(self):
        try:
            if not os.path.exists(self.waypoints_path):
                self.get_logger().error(f"Waypoints not found: {self.waypoints_path}")
                return False

            waypoints = np.load(self.waypoints_path)

            for i in range(min(3, len(waypoints))):
                msg = Point()
                msg.x = float(waypoints[i][0])
                msg.y = float(waypoints[i][1])
                msg.z = float(waypoints[i][2])

                finger_name = f"Finger {i+1}"
                self.ik_pubs[finger_name].publish(msg)
                self.get_logger().info(f"{finger_name} sent to {waypoints[i]}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to read waypoints.npy file: {e}")
            return False

    def control_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if self.state == "OPEN":
            # Open palm
            msg = Float64MultiArray()
            msg.data = [0.0] * 12
            self.joint_cmd_pub.publish(msg)
            if elapsed > 2.0:
                self.run_circle_generator()
                self.transition("WRAP", now)

        elif self.state == "WRAP":
            success = self.position_fingers_circle()
            if success and elapsed > 4.0:
                self.transition("GRASP", now)
            self.state = "FINISHED"

        # TODO Add other tasks later
        elif self.state == "GRASP":
            pass

        elif self.state == "HOLD":
            pass

    def transition(self, next_state, now):
        self.get_logger().info(f"Transition from {self.state} to {next_state}")
        self.state = next_state
        self.start_time = now

def main(args=None):
    try:
        rclpy.init(args=args)
        node = ManipulationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
