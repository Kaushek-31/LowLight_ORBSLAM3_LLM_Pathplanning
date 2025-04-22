#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json

class SemanticObstacleExtractor(Node):
    def __init__(self):
        super().__init__('semantic_obstacle_extractor')

        self.map_data = None
        self.semantic_objects = []

        # Parameters
        self.declare_parameter('semantic_exclude_labels', ['target_zone'])
        self.exclude_labels = set(self.get_parameter('semantic_exclude_labels').get_parameter_value().string_array_value)

        # Subscribers
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(String, '/object_list', self.object_list_callback, 10)
        self.create_subscription(String, '/exclude_labels', self.exclude_update_callback, 10)

        # Publishers
        self.obstacle_pub = self.create_publisher(String, '/obstacles', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/obstacle_markers', 10)

        # Timer to publish obstacle list and markers every 2 seconds
        self.timer = self.create_timer(2.0, self.publish_obstacles)
        self.get_logger().info("Semantic Obstacle Extractor Node Started")

    def map_callback(self, msg):
        self.map_data = msg

    def object_list_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.semantic_objects = data.get('objects', [])
        except Exception as e:
            self.get_logger().error(f"Failed to parse /object_list JSON: {e}")

    def exclude_update_callback(self, msg):
        try:
            updated_labels = json.loads(msg.data)
            if isinstance(updated_labels, list):
                self.exclude_labels = set(updated_labels)
                self.get_logger().info(f"Updated exclude_labels: {self.exclude_labels}")
            else:
                self.get_logger().warn("Exclude label update was not a list")
        except Exception as e:
            self.get_logger().error(f"Failed to parse /exclude_labels update: {e}")

    def publish_obstacles(self):
        if not self.map_data or not self.semantic_objects:
            self.get_logger().warn("Waiting for both /map and /object_list data...")
            return

        resolution = self.map_data.info.resolution
        origin_x = self.map_data.info.origin.position.x
        origin_y = self.map_data.info.origin.position.y

        grid = self.map_data.data
        width = self.map_data.info.width
        height = self.map_data.info.height

        obstacles = []
        markers = MarkerArray()
        marker_id = 0

        for obj in self.semantic_objects:
            label = obj.get('semantic_label', '')
            if label in self.exclude_labels:
                continue

            pos = obj.get('position', {})
            x = pos.get('x')
            y = pos.get('y')
            if x is None or y is None:
                continue

            # Map (x, y) world coordinates to grid cell
            grid_x = int((x - origin_x) / resolution)
            grid_y = int((y - origin_y) / resolution)

            if 0 <= grid_x < width and 0 <= grid_y < height:
                obstacles.append((grid_x, grid_y))

                # Create a visualization marker
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "semantic_obstacles"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.1
                marker.pose.orientation.w = 1.0
                marker.scale.x = resolution
                marker.scale.y = resolution
                marker.scale.z = 0.1
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                marker.lifetime.sec = 2
                markers.markers.append(marker)
                marker_id += 1

        msg = String()
        msg.data = json.dumps({'obstacles': obstacles})
        self.obstacle_pub.publish(msg)
        self.marker_pub.publish(markers)
        self.get_logger().info(f"Published {len(obstacles)} obstacles and {len(markers.markers)} markers to RViz")


def main(args=None):
    rclpy.init(args=args)
    node = SemanticObstacleExtractor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
