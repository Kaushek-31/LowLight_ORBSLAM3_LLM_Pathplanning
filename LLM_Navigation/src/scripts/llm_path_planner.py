#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry, Path
import yaml
import json
import os
import importlib
import ollama
import sys
import re
from threading import Lock

# TF2 imports
import tf2_ros

class LLMPathPlanner(Node):
    def __init__(self):
        super().__init__('llm_path_planner')

        # Declare parameters
        self.declare_parameter('safe_margin', 0.5)
        self.declare_parameter('waypoint_spacing', 0.7)
        self.declare_parameter('package_path', '')
        self.declare_parameter('environment_yaml', 'rob530_environment_semantic.yaml')
        self.declare_parameter('environment_path', 'object_list')
        self.declare_parameter('environment_script', 'environment_01')
        self.declare_parameter('max_attempts', 3)

        # Get parameters
        self.safe_margin        = self.get_parameter('safe_margin').value
        self.waypoint_spacing   = self.get_parameter('waypoint_spacing').value
        self.package_path       = self.get_parameter('package_path').value or os.path.join(
            os.getenv('HOME'), 'bcr_bot', 'src'
        )
        self.environment_yaml   = self.get_parameter('environment_yaml').value
        self.environment_script = self.get_parameter('environment_script').value
        self.max_attempts       = self.get_parameter('max_attempts').value

        # Placeholder pose until /odom arrives
        self.current_pose = Pose()
        self.current_pose.position.x = -5.0
        self.current_pose.position.y = 8.0
        self.current_pose.position.z = 0.0
        self.current_target_object = None
        self.environment_data      = None
        self.lock                  = Lock()

        # TF2 buffer & listener for transforms
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Load environment YAML
        self.environment_path = self.get_parameter('environment_path').value
        yaml_path = os.path.join(self.package_path, self.environment_path, self.environment_yaml)
        self.load_environment_data(yaml_path)

        # Load LLM-based environment script dynamically
        utils_path = os.path.join(self.package_path, 'scripts', 'utils')
        if utils_path not in sys.path:
            sys.path.append(utils_path)
        self.environment_module = self.load_environment_module(self.environment_script)

        # Connect to Ollama LLM client
        try:
            self.llm_client = ollama.Client()
            self.get_logger().info('Connected to Ollama LLM client.')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to Ollama client: {e}')
            self.destroy_node()
            return

        # ROS subscriptions & publishers
        self.create_subscription(Odometry, '/odom',            self.odom_callback,          10)
        self.create_subscription(String,   '/user_command',    self.user_command_callback,  10)
        self.create_subscription(String,   '/obstacle',        self.obstacle_data_callback,  10)

        self.pose_array_publisher = self.create_publisher(PoseArray, '/llm_waypoints_pose_array',    10)
        self.status_pub           = self.create_publisher(String,    '/waypoint_generation_status', 10)

        self.get_logger().info('LLM Path Planner with TF and target extraction running.')

    def user_command_callback(self, msg):
        with self.lock:
            try:
                command = json.loads(msg.data.strip())
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Invalid JSON command: {e}")
                self.status_pub.publish(String(data="failure:0"))
                return

            to_name = command.get("to")
            constraints = command.get("constraints", {})
            self.safe_margin = constraints.get("safe_margin", self.safe_margin)

            # if self.current_pose is None or self.environment_data is None:
            #     self.get_logger().warn("Waiting for current pose or environment data.")
            #     return

            target_object = self.find_target_object(to_name)
            if not target_object:
                self.get_logger().error(f"Target '{to_name}' not found in environment.")
                self.status_pub.publish(String(data="failure:0"))
                return

            self.current_target_object = target_object
            waypoints = None
            attempt = 0

            while attempt < self.max_attempts and not waypoints:
                attempt += 1
                waypoints = self.environment_module.generate_waypoints_with_navigation_agent(
                    llm_client=self.llm_client,
                    current_pose=self.current_pose,
                    target_object=target_object,
                    environment_data=self.environment_data,
                    safe_margin=self.safe_margin,
                    waypoint_spacing=self.waypoint_spacing
                )
                if waypoints:
                    self.status_pub.publish(String(data=f"success:{attempt}"))
                    self.publish_waypoints(waypoints)
                else:
                    self.get_logger().warn(f"Attempt {attempt} failed.")

            if not waypoints:
                self.get_logger().error("Failed to generate waypoints.")
                self.status_pub.publish(String(data=f"failure:{attempt}"))

    def obstacle_data_callback(self, msg):
        with self.lock:
            if msg.data.strip().lower() != "emergency_stop":
                return

            if not self.current_pose or not self.current_target_object:
                self.status_pub.publish(String(data="failure:0"))
                return

            attempt = 0
            waypoints = None
            while attempt < self.max_attempts and not waypoints:
                attempt += 1
                waypoints = self.environment_module.generate_waypoints_with_navigation_agent(
                    llm_client=self.llm_client,
                    current_pose=self.current_pose,
                    target_object=self.current_target_object,
                    environment_data=self.environment_data,
                    safe_margin=self.safe_margin,
                    waypoint_spacing=self.waypoint_spacing
                )
                if waypoints:
                    self.status_pub.publish(String(data=f"success:{attempt}"))
                    self.publish_waypoints(waypoints)
                else:
                    self.get_logger().warn(f"Replanning attempt {attempt} failed.")

            if not waypoints:
                self.get_logger().error("Replanning failed.")
                self.status_pub.publish(String(data=f"failure:{attempt}"))

    def load_environment_data(self, path):
        try:
            with open(path, 'r') as f:
                full_data = yaml.safe_load(f)
            self.environment_data = full_data.get('environment_data_provider', {}).get('ros__parameters', {})
            if not self.environment_data:
                self.get_logger().error("Environment data could not be extracted from YAML structure.")
        except Exception as e:
            self.get_logger().error(f"Failed to load environment YAML: {e}")

    def load_environment_module(self, module_name):
        try:
            return importlib.import_module(module_name)
        except ImportError as e:
            self.get_logger().error(f"Could not import {module_name}: {e}")
            return None

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
    def refine_waypoints(self, path: Path) -> Path:
        refined_path = Path()
        refined_path.header = path.header  # Retain original header information

        previous_wp = None
        for wp in path.poses:
            # If this is the first waypoint, add it directly
            if previous_wp is None:
                refined_path.poses.append(wp)
                previous_wp = wp
                continue

        # Check if the distance between this waypoint and the previous one exceeds the desired spacing
        distance = self.calculate_distance(previous_wp, wp)
        if distance >= self.waypoint_spacing:
            refined_path.poses.append(wp)
            previous_wp = wp

        self.get_logger().info(f"Refined {len(refined_path.poses)} waypoints from {len(path.poses)}")
        return refined_path

    def calculate_distance(self, wp1, wp2):
        dx = wp2.position.x - wp1.position.x
        dy = wp2.position.y - wp1.position.y
        return (dx ** 2 + dy ** 2) ** 0.5    
    
    def llm_waypoints_callback(self, msg):
        # Handle the incoming waypoints from D* Lite Planner
        self.get_logger().info(f"Received {len(msg.poses)} waypoints from D* Lite Planner.")
        refined_waypoints = self.refine_waypoints(msg)
        self.publish_waypoints(refined_waypoints)

    def find_target_object(self, name):
        for obj in self.environment_data.get('target_objects', []):
            if obj.get('name', '').lower() == name.lower():
                return {'name': name, 'position': obj.get('position', {})}
        return None

    def publish_waypoints(self, waypoints):
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for wp in waypoints:
            pose = Pose()
            pose.position.x = wp['x']
            pose.position.y = wp['y']
            pose.position.z = 0.0
            pose.orientation.w = 1.0
        pose_stamped = PoseStamped()
        pose_stamped.pose = pose
        self.waypoints_pose_array.poses.append(pose_stamped)

        # Create PoseStamped for Path
        path_pose_stamped = PoseStamped()
        path_pose_stamped.pose = pose
        self.path.poses.append(path_pose_stamped)

        # Publish both PoseArray and Path topics
        self.waypoints_pose_array.poses = waypoints
        self.pose_array_publisher.publish(self.waypoints_pose_array)
        self.path_publisher.publish(self.path)
        self.get_logger().info(f"Published {len(pose_array.poses)} LLM waypoints.")

def main(args=None):
    rclpy.init(args=args)
    node = LLMPathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("LLM Path Planner interrupted.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()