#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time

class CircularExecutionNode(Node):
    def __init__(self):
        super().__init__('circular_execution_node')

        # Controller Publisher
        self.publisher_ = self.create_publisher(
            Float64MultiArray, 
            '/forward_position_controller/commands',
            10
        )

        # 2. Load Trajectory Data
        try:
            self.trajectory = np.load('src/joint_trajectory.npy')
            self.get_logger().info(f"Trajectory loaded: {len(self.trajectory)} points found.")
        except FileNotFoundError:
            self.get_logger().error("Error: joint_trajectory.npy not found!")
            return

        # 3. Configuration (Task requirements)
        self.loops = 3              # At least 3 loops
        self.total_duration = 15.0  # Total motion duration (seconds)
        self.dt = self.total_duration / len(self.trajectory) # Wait time between points

        self.execute_motion()

    def execute_motion(self):
        self.get_logger().info(f"Motion starting. Duration: {self.total_duration}s | Loops: {self.loops}")

        for loop in range(self.loops):
            self.get_logger().info(f"Loop {loop + 1} executing...")

            for point in self.trajectory:
                # Send 12-DOF command to Gazebo
                msg = Float64MultiArray()
                msg.data = point.tolist()
                self.publisher_.publish(msg)

                # Ensure smooth flow within the specified duration
                time.sleep(self.dt)

        self.get_logger().info("Task completed! All loops finished.")

def main():
    try:
        print("Execution starting...")
        rclpy.init()
        node = CircularExecutionNode()
        rclpy.shutdown()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Execution finished.")

if __name__ == '__main__':
    main()
