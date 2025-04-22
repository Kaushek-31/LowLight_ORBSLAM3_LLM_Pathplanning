#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import csv
import os
import math
import time
from threading import Lock

from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan


class MetricsCollector(Node):
    def __init__(self):
        super().__init__('metrics_collector')
        self.get_logger().info("Metrics Collector Node Initialized.")

        # Output CSV path
        csv_dir = os.path.expanduser('~/bcr_bot/src/results/llm_pp')
        os.makedirs(csv_dir, exist_ok=True)
        csv_path = os.path.join(csv_dir, 'metrics_llm_pp.csv')

        try:
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'User_Command',
                'Path_Planning_Time',
                'Execution_Time',
                'Waypoint_Generation_Success_Rate',
                'Path_Length',
                'Collision_Detection_Event',
                'Replanning_Rate'
            ])
            self.csv_file.flush()
        except Exception as e:
            self.get_logger().error(f"CSV file initialization failed: {e}")
            self.destroy_node()

        # Metrics
        self.lock = Lock()
        self.current_command = None
        self.command_start_time = None
        self.waypoint_received_time = None
        self.execution_start_time = None
        self.execution_end_time = None
        self.path_length = 0.0
        self.path_planning_time = 0.0
        self.replanning_attempts = 0
        self.collision_events = 0
        self.successful_waypoint_generations = 0
        self.total_waypoint_generations = 0
        self.current_waypoints = []
        self.previous_pose = None

        # Subscriptions
        self.create_subscription(String, '/user_command', self.user_command_callback, 10)
        self.create_subscription(PoseArray, '/llm_waypoints', self.waypoints_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(String, '/obstacle_data', self.obstacle_data_callback, 10)
        self.create_subscription(String, '/waypoint_generation_status', self.waypoint_generation_status_callback, 10)
        self.create_subscription(String, '/execution_status', self.execution_status_callback, 10)

    def user_command_callback(self, msg):
        with self.lock:
            self.current_command = msg.data.strip()
            self.command_start_time = time.time()
            self.total_waypoint_generations += 1
            self.get_logger().info(f"[User Command] '{self.current_command}'")

    def waypoints_callback(self, msg):
        with self.lock:
            if not self.current_command:
                self.get_logger().warn("Waypoints received before user command.")
                return

            self.waypoint_received_time = time.time()
            self.path_planning_time = self.waypoint_received_time - self.command_start_time
            self.execution_start_time = time.time()
            self.current_waypoints = [(p.position.x, p.position.y) for p in msg.poses]
            self.path_length = 0.0
            self.get_logger().info(f"Waypoints received. Path planning took {self.path_planning_time:.2f} seconds")

    def waypoint_generation_status_callback(self, msg):
        with self.lock:
            status = msg.data.strip().lower()
            try:
                result, attempts = status.split(":")
                attempts = int(attempts)
            except ValueError:
                self.get_logger().warn(f"Unexpected format in waypoint_generation_status: {status}")
                return

            if result == "success":
                self.successful_waypoint_generations += 1
            elif result == "failure":
                pass
            self.get_logger().info(f"[Waypoint Gen] Status: {result.upper()} after {attempts} attempt(s)")

    def execution_status_callback(self, msg):
        with self.lock:
            if msg.data.strip().lower() != "completed":
                return

            self.execution_end_time = time.time()
            execution_time = self.execution_end_time - self.execution_start_time
            success_rate = (self.successful_waypoint_generations / self.total_waypoint_generations) * 100 if self.total_waypoint_generations else 0.0
            replanning_rate = self.replanning_attempts / execution_time if execution_time > 0 else 0.0

            try:
                self.csv_writer.writerow([
                    self.current_command,
                    f"{self.path_planning_time:.2f}",
                    f"{execution_time:.2f}",
                    f"{success_rate:.2f}%",
                    f"{self.path_length:.2f}",
                    f"{self.collision_events}",
                    f"{replanning_rate:.2f}"
                ])
                self.csv_file.flush()
                self.get_logger().info("Metrics recorded.")
            except Exception as e:
                self.get_logger().error(f"Failed to write to CSV: {e}")

            self.reset_metrics()

    def odom_callback(self, msg):
        with self.lock:
            current_pose = msg.pose.pose
            if self.previous_pose:
                dx = current_pose.position.x - self.previous_pose.position.x
                dy = current_pose.position.y - self.previous_pose.position.y
                self.path_length += math.hypot(dx, dy)
            self.previous_pose = current_pose

    def obstacle_data_callback(self, msg):
        if msg.data.strip().lower() == "emergency_stop":
            self.replanning_attempts += 1
            self.collision_events += 1
            self.get_logger().info(f"[Collision] Event detected. Total: {self.collision_events}")

    def scan_callback(self, msg):
        pass  # Optional: live LIDAR monitoring

    def reset_metrics(self):
        self.current_command = None
        self.command_start_time = None
        self.waypoint_received_time = None
        self.execution_start_time = None
        self.execution_end_time = None
        self.path_length = 0.0
        self.path_planning_time = 0.0
        self.replanning_attempts = 0
        self.collision_events = 0
        self.successful_waypoint_generations = 0
        self.total_waypoint_generations = 0
        self.current_waypoints = []
        self.previous_pose = None

    def __del__(self):
        try:
            self.csv_file.close()
        except:
            pass


def main(args=None):
    rclpy.init(args=args)
    collector = MetricsCollector()
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info("Ctrl+C pressed. Shutting down Metrics Collector.")
    finally:
        if rclpy.ok():
            collector.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
