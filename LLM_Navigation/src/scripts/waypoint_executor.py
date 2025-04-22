#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf_transformations import euler_from_quaternion
import math
import yaml
import os
import time
import numpy as np

# Fix deprecated np.float alias
if not hasattr(np, 'float'):
    np.float = float

class WaypointExecutor(Node):
    def __init__(self):
        super().__init__('waypoint_executor')

        # Parameters
        self.declare_parameter('environment_yaml', '/home/sanjana/bcr_bot/src/object_list/rob530_environment_semantic.yaml')
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('distance_threshold', 0.2)
        self.declare_parameter('angle_threshold', math.radians(5))
        self.declare_parameter('obstacle_angle_range', math.radians(15))
        self.declare_parameter('self_obstacle_distance', 0.3)
        self.declare_parameter('critical_distance', 0.5)
        self.declare_parameter('max_replans', 3)
        self.declare_parameter('replan_cooldown', 5.0)

        # Load parameter values
        env_yaml_path = self.get_parameter('environment_yaml').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.angle_threshold = self.get_parameter('angle_threshold').value
        self.obstacle_angle_range = self.get_parameter('obstacle_angle_range').value
        self.self_obstacle_distance = self.get_parameter('self_obstacle_distance').value
        self.critical_distance = self.get_parameter('critical_distance').value
        self.max_replans = self.get_parameter('max_replans').value
        self.replan_cooldown = self.get_parameter('replan_cooldown').value
        self.replan_attempts = 0

        # State constants
        self.STATE_NAVIGATING = 'Navigating'
        self.STATE_EMERGENCY_STOPPED = 'Emergency Stopped'
        self.STATE_RESUMING_NAVIGATION = 'Resuming Navigation'
        self.state = self.STATE_NAVIGATING

        # Load environment data
        self.environment_data = self.load_environment_data(env_yaml_path)

        # Robot state and waypoints
        self.current_pose = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.last_replan_time = time.time()

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, 'bcr_bot/cmd_vel', 10)
        self.obstacle_data_pub = self.create_publisher(String, '/obstacle_data', 10)
        self.execution_status_pub = self.create_publisher(String, '/execution_status', 10)

        # Subscribers
        self.create_subscription(Path, '/dstar_path', self.dstar_path_callback, 10)
        self.create_subscription(Path, '/llm_waypoints', self.waypoints_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Waypoint Executor initialized.")

    def load_environment_data(self, yaml_file_path):
        try:
            with open(yaml_file_path, 'r') as file:
                return yaml.safe_load(file)
        except Exception as e:
            self.get_logger().error(f"Failed to load environment YAML: {e}")
            return None

    def get_obstacle_boundaries(self):
        if not self.environment_data:
            self.get_logger().warn("Environment data missing.")
            return []

        boundaries = []
        for obj in self.environment_data.get('objects', []):
            if 'Obstacle' in obj.get('name', ''):
                pos = obj.get('position', {})
                size = obj.get('size', {})
                boundaries.append({
                    'x_min': pos['x'] - size['width'] / 2,
                    'x_max': pos['x'] + size['width'] / 2,
                    'y_min': pos['y'] - size['length'] / 2,
                    'y_max': pos['y'] + size['length'] / 2
                })
        return boundaries

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def waypoints_callback(self, msg):
        self.get_logger().info("Received new waypoints.")
        boundaries = self.get_obstacle_boundaries()
        valid_waypoints = []

        for pose in msg.poses:
            x, y = pose.position.x, pose.position.y
            if any(b['x_min'] <= x <= b['x_max'] and b['y_min'] <= y <= b['y_max'] for b in boundaries):
                self.get_logger().warn(f"Waypoint ({x:.2f}, {y:.2f}) inside obstacle.")
            else:
                valid_waypoints.append((x, y))

        if not valid_waypoints:
            self.get_logger().error("No valid waypoints to follow.")
            return

        self.waypoints = valid_waypoints
        self.current_waypoint_index = 0
        self.state = self.STATE_RESUMING_NAVIGATION
        self.replan_attempts = 0
        self.get_logger().info(f"Updated waypoints: {self.waypoints}")

    def scan_callback(self, msg):
        if self.current_pose is None:
            return

        relevant = []
        for i, dist in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if 0 < dist < self.critical_distance and abs(angle) <= self.obstacle_angle_range and dist > self.self_obstacle_distance:
                relevant.append(dist)

        detected = False
        if len(relevant) >= 3:
            median = sorted(relevant)[len(relevant) // 2]
            detected = median < self.critical_distance
        elif relevant:
            detected = min(relevant) < self.critical_distance

        now = time.time()
        if detected and self.state == self.STATE_NAVIGATING and self.replan_attempts < self.max_replans:
            if now - self.last_replan_time > self.replan_cooldown:
                self.get_logger().warn("Emergency stop triggered.")
                self.state = self.STATE_EMERGENCY_STOPPED
                self.stop_robot()
                self.obstacle_data_pub.publish(String(data='emergency_stop'))
                self.last_replan_time = now
                self.replan_attempts += 1
    def dstar_path_callback(self, msg: Path):
         # When the D* Lite planner publishes the refined path, this callback is triggered.
        self.get_logger().info(f"Received refined path with {len(msg.poses)} waypoints.")
        # Update waypoints from D* Lite
        self.waypoints = [(wp.pose.position.x, wp.pose.position.y) for wp in msg.poses]
        self.current_waypoint_index = 0
        self.state = self.STATE_NAVIGATING
        self.get_logger().info(f"Updated waypoints for navigation: {self.waypoints}")
       
    def timer_callback(self):
        if not self.current_pose or self.state == self.STATE_EMERGENCY_STOPPED:
            self.stop_robot()
            return

        if self.current_waypoint_index >= len(self.waypoints):
            self.stop_robot()
            self.execution_status_pub.publish(String(data='completed'))
            return

        target_x, target_y = self.waypoints[self.current_waypoint_index]
        cur_x = self.current_pose.position.x
        cur_y = self.current_pose.position.y

        dx = target_x - cur_x
        dy = target_y - cur_y
        distance = math.hypot(dx, dy)

        yaw = self.get_yaw()
        desired_angle = math.atan2(dy, dx)
        angle_diff = (desired_angle - yaw + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()
        if abs(angle_diff) > self.angle_threshold:
            twist.angular.z = self.angular_speed * angle_diff
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = self.angular_speed * angle_diff

        self.cmd_vel_pub.publish(twist)

        if distance < self.distance_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index + 1}: ({target_x}, {target_y})")
            self.current_waypoint_index += 1

    def get_yaw(self):
        q = self.current_pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WaypointExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Ctrl+C pressed. Shutting down Waypoint Executor.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
