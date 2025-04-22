#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import yaml
import json
import os
from std_msgs.msg import String

class EnvironmentDataProvider(Node):
    def __init__(self):
        super().__init__('environment_data_provider')

        # Declare parameters with default values
        self.declare_parameter('environment_yaml', 'object_list/rob530_environment_semantic.yaml')
        self.declare_parameter('package_path', os.path.join(
            os.getenv('HOME'),
            'bcr_bot', 'src'
        ))

        # Get parameter values
        self.yaml_file = self.get_parameter('environment_yaml').get_parameter_value().string_value
        self.package_path = self.get_parameter('package_path').get_parameter_value().string_value

        # Construct full YAML path (with 'object_list' subdirectory)
        self.yaml_file_path = os.path.join(self.package_path, self.yaml_file)

        # Load raw YAML
        raw_yaml = self.load_environment_data()
        if raw_yaml is None:
            self.get_logger().error("Failed to load environment YAML.")
            return

        # Parse into nested structure
        self.environment_data = raw_yaml.get('environment_data_provider', {}).get('ros__parameters', {})
        if 'environment_width' not in self.environment_data or 'environment_length' not in self.environment_data:
            self.get_logger().error("Missing 'environment_width' or 'environment_length' in YAML.")
            return

        # Publisher
        self.publisher = self.create_publisher(String, '/object_list', 10)

        # Timer to publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_environment_data)
        self.get_logger().info("Environment Data Provider started.")

    def load_environment_data(self):
        try:
            with open(self.yaml_file_path, 'r') as file:
                return yaml.safe_load(file)
        except FileNotFoundError:
            self.get_logger().error(f"YAML file not found: {self.yaml_file_path}")
        except yaml.YAMLError as exc:
            self.get_logger().error(f"YAML parsing error: {exc}")
        return None

    def publish_environment_data(self):
        if not self.environment_data:
            self.get_logger().warn("No environment data loaded.")
            return

        env_width = self.environment_data.get('environment_width')
        env_length = self.environment_data.get('environment_length')

        all_objects = []
        for group_key in ['target_objects', 'obstacles']:
            for obj in self.environment_data.get(group_key, []):
                name = obj.get('name')
                position = obj.get('position', {})
                size = obj.get('size', {})
                label = obj.get('semantic_label', group_key[:-1])

                if not name or not all(k in position for k in ['x', 'y', 'z']):
                    self.get_logger().warn(f"Incomplete object data: {obj}")
                    continue

                obj_entry = {
                    'name': name,
                    'position': position,
                    'semantic_label': label
                }

                if all(k in size for k in ['width', 'length', 'height']):
                    obj_entry['size'] = size

                all_objects.append(obj_entry)

        publish_data = {
            'environment_width': env_width,
            'environment_length': env_length,
            'objects': all_objects
        }

        msg = String()
        msg.data = json.dumps(publish_data)
        self.publisher.publish(msg)
        self.get_logger().debug("Published environment data to /object_list")


def main(args=None):
    rclpy.init(args=args)
    node = EnvironmentDataProvider()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C pressed. Shutting down Environment Data Provider node.")
    finally:
        node.destroy_node()
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass  # Prevent error if shutdown was already called

if __name__ == '__main__':
    main()
