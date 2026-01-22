#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import argparse

class CircularExecutionNode(Node):
    def __init__(self, selected_finger=None):
        super().__init__('circular_execution_node')

        self.selected_finger = selected_finger
        self.finger_slices = {
            'f1': slice(0, 4),
            'f2': slice(4, 8),
            'f3': slice(8, 12),
        }

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

        self.rest_point = np.array(self.trajectory[0])
        if self.selected_finger:
            self.get_logger().info(f"Filtering trajectory to move only {self.selected_finger}.")
        self.zero_idle_point = self._build_zero_idle_point()

        # 3. Configuration (Task requirements)
        self.loops = 3              # At least 3 loops
        self.total_duration = 15.0  # Total motion duration (seconds)
        self.dt = self.total_duration / len(self.trajectory) # Wait time between points

        self.execute_motion()

    def execute_motion(self):
        self.get_logger().info(f"Motion starting. Duration: {self.total_duration}s | Loops: {self.loops}")

        if self.selected_finger:
            self._publish_point(self.zero_idle_point)
            time.sleep(self.dt)

        for loop in range(self.loops):
            self.get_logger().info(f"Loop {loop + 1} executing...")

            for point in self.trajectory:
                point_to_send = self._compose_point(point)
                self._publish_point(point_to_send)

                # Ensure smooth flow within the specified duration
                time.sleep(self.dt)

        if self.selected_finger:
            self._publish_point(self.rest_point)

        self.get_logger().info("Task completed! All loops finished.")

    def _compose_point(self, point):
        if not self.selected_finger:
            return point

        base_point = self.zero_idle_point.copy()
        active_slice = self.finger_slices.get(self.selected_finger)
        if active_slice:
            base_point[active_slice] = point[active_slice]
        return base_point

    def _build_zero_idle_point(self):
        if not self.selected_finger:
            return self.rest_point

        base_point = self.rest_point.copy()
        for name, slc in self.finger_slices.items():
            if name != self.selected_finger:
                base_point[slc] = 0.0
        return base_point

    def _publish_point(self, point):
        msg = Float64MultiArray()
        msg.data = point.tolist()
        self.publisher_.publish(msg)

def main():
    parser = argparse.ArgumentParser(description='Execute a finger trajectory.')
    parser.add_argument('--f1', action='store_true', help='Move only finger 1')
    parser.add_argument('--f2', action='store_true', help='Move only finger 2')
    parser.add_argument('--f3', action='store_true', help='Move only finger 3')
    args = parser.parse_args()

    selected_finger = None
    if args.f1:
        selected_finger = 'f1'
    elif args.f2:
        selected_finger = 'f2'
    elif args.f3:
        selected_finger = 'f3'

    try:
        print("Execution starting...")
        rclpy.init()
        node = CircularExecutionNode(selected_finger=selected_finger)
        rclpy.shutdown()
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        print("Execution finished.")

if __name__ == '__main__':
    main()
