import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np

class WaypointRvizVisualizer(Node):
    def __init__(self):
        super().__init__('waypoint_rviz_visualizer')
        self.publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        # Refresh markers every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_markers)
        self.get_logger().info("Waypoint Visualizer started.")

    def publish_markers(self):
        try:
            waypoints = np.load('src/waypoints.npy')
        except Exception as e:
            self.get_logger().error(f"File could not be loaded: {e}")
            return

        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "palm"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoints"
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD

        # Pick size
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # Pick color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        for pt in waypoints:
            p = Point()
            p.x = float(pt[0])
            p.y = float(pt[1])
            p.z = float(pt[2])
            marker.points.append(p)

        marker_array.markers.append(marker)
        self.publisher.publish(marker_array)

def main():
    rclpy.init()
    node = WaypointRvizVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
