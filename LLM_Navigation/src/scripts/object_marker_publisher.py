#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
import yaml
import os
import json
from ament_index_python.packages import get_package_share_directory

class ObjectMarkerPublisher(Node):
    def __init__(self):
        super().__init__('object_marker_publisher')

        default_yaml = os.path.join(
            get_package_share_directory('bcr_bot'),
            'object_list',
            'rob530_environment_semantic.yaml'
        )

        # 2) Declare the parameter with that default
        self.declare_parameter('environment_yaml', default_yaml)

        # 3) Read back the actual value (may have been overridden)
        yaml_path = self.get_parameter('environment_yaml') \
                         .get_parameter_value() \
                         .string_value

        # 4) Check existence
        if not os.path.isfile(yaml_path):
            self.get_logger().error(f"YAML file not found at: {yaml_path}")
            rclpy.shutdown()
            return

        # 5) Load it
        try:
            with open(yaml_path, 'r') as f:
                self.environment_data = yaml.safe_load(f)
            self.get_logger().info(f"Environment YAML loaded: {yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML ({yaml_path}): {e}")
            rclpy.shutdown()
            return

        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.create_subscription(String, '/obstacles', self.obstacle_callback, 10)
        self.obstacle_cells = []

        # Static values (must be updated if map is available)
        self.map_origin_x = 0.0
        self.map_origin_y = 0.0
        self.map_resolution = 0.1

        self.create_timer(1.0, self.publish_markers)  # 1 Hz

    def obstacle_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.obstacle_cells = data.get('obstacles', [])
        except Exception as e:
            self.get_logger().error(f"Failed to parse /obstacles: {e}")

    def get_color_by_label(self, label):
        color_map = {
            "target_zone": (0.0, 0.0, 1.0),
            "wall": (0.0, 1.0, 0.0),
            "movable_obstacle": (1.0, 0.0, 0.0),
            "furniture": (0.5, 0.3, 0.1),
            "equipment": (0.8, 0.6, 0.0),
            "warning_marker": (1.0, 0.5, 0.0),
            "storage_unit": (0.4, 0.0, 1.0),
            "obstacle": (0.3, 0.3, 0.3)
        }
        return color_map.get(label, (0.5, 0.5, 0.5))

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        index = 0

        for group_key in ['target_objects', 'obstacles']:
            for obj in self.environment_data.get(group_key, []):
                name = obj.get('name', f"object_{index}")
                position = obj.get('position', {})
                size = obj.get('size', None)
                label = obj.get('semantic_label', 'obstacle')
                r, g, b = self.get_color_by_label(label)

                if not all(k in position for k in ('x', 'y', 'z')):
                    self.get_logger().warn(f"Skipping object '{name}' due to missing position.")
                    continue

                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = now
                marker.ns = group_key
                marker.id = index
                marker.action = Marker.ADD
                marker.pose.position.x = position['x']
                marker.pose.position.y = position['y']
                marker.pose.orientation.w = 1.0

                if size:
                    marker.type = Marker.CUBE
                    marker.pose.position.z = position['z'] + size.get('height', 1.0) / 2
                    marker.scale.x = size.get('width', 0.5)
                    marker.scale.y = size.get('length', 0.5)
                    marker.scale.z = size.get('height', 0.5)
                    marker.color.a = 0.7
                else:
                    marker.type = Marker.SPHERE
                    marker.pose.position.z = position['z']
                    marker.scale.x = marker.scale.y = marker.scale.z = 0.3
                    marker.color.a = 1.0

                marker.color.r = r
                marker.color.g = g
                marker.color.b = b
                marker_array.markers.append(marker)

                text_marker = Marker()
                text_marker.header.frame_id = "map"
                text_marker.header.stamp = now
                text_marker.ns = f"{group_key}_label"
                text_marker.id = index + 1000
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = position['x']
                text_marker.pose.position.y = position['y']
                text_marker.pose.position.z = position['z'] + (size.get('height', 1.0) if size else 0.3) + 0.2
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.4
                text_marker.color.a = 1.0
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.text = name
                marker_array.markers.append(text_marker)
                index += 1

        # Add obstacles from /obstacles
        for i, (gx, gy) in enumerate(self.obstacle_cells):
            obstacle_marker = Marker()
            obstacle_marker.header.frame_id = "map"
            obstacle_marker.header.stamp = now
            obstacle_marker.ns = "grid_obstacles"
            obstacle_marker.id = 2000 + i
            obstacle_marker.type = Marker.CUBE
            obstacle_marker.action = Marker.ADD
            obstacle_marker.scale.x = 0.1
            obstacle_marker.scale.y = 0.1
            obstacle_marker.scale.z = 0.1
            obstacle_marker.color.r = 1.0
            obstacle_marker.color.g = 0.0
            obstacle_marker.color.b = 0.0
            obstacle_marker.color.a = 0.9

            obstacle_marker.pose.position.x = self.map_origin_x + (gx + 0.5) * self.map_resolution
            obstacle_marker.pose.position.y = self.map_origin_y + (gy + 0.5) * self.map_resolution
            obstacle_marker.pose.position.z = 0.05
            obstacle_marker.pose.orientation.w = 1.0

            marker_array.markers.append(obstacle_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMarkerPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C pressed. Shutting down Object Marker Publisher.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()