#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from tf2_ros import TransformListener, Buffer
from tf2_ros import TFMessage
import math
from threading import Lock
import numpy as np
from tf2_ros import TransformException

class MPCController(Node):
    def __init__(self):
        super().__init__('mpc_controller')
        self.lock = Lock()

        self.current_pose = None
        self.current_velocity = 0.0
        self.path = []

        # Parameters
        self.lookahead_distance = self.declare_parameter('lookahead_distance', 1.0).value
        self.linear_max_speed = self.declare_parameter('linear_max_speed', 0.25).value
        self.angular_max_speed = self.declare_parameter('angular_max_speed', 1.5).value
        self.goal_tolerance = self.declare_parameter('goal_tolerance', 0.1).value

        # Subscribers
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(PoseArray, '/dstar_waypoints', self.waypoints_callback, 10)
        self.create_subscription(Pose, '/current_pose', self.pose_callback, 10)
        self.create_subscription(TFMessage, '/tf', self.tf_callback, 10)
        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # Timer for control loop
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info("MPC Controller initialized.")
        
    def pose_callback(self, msg: Pose):
        with self.lock:
            self.current_pose = msg
            self.get_logger().info(f"Received Pose: {self.current_pose.position}")

    def odom_callback(self, msg):
        with self.lock:
            self.current_pose = msg.pose.pose
            vx = msg.twist.twist.linear.x
            vy = msg.twist.twist.linear.y
            self.current_velocity = math.sqrt(vx**2 + vy**2)
            self.get_logger().info(f"Received Odometry: {msg.pose.pose.position}")

    def waypoints_callback(self, msg):
        with self.lock:
            self.path = [(p.position.x, p.position.y) for p in msg.poses]
            self.get_logger().info(f"Received {len(self.path)} waypoints.")

    def tf_callback(self, msg):
            # Lookup the transform between 'map' and 'odom'
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            
            # Perform the transformation if possible
            transformed_pose = self.tf_buffer.transform(self.current_pose, 'map')
            transformed_position = transformed_pose.position
            
            self.get_logger().info(f"Transformed Position: {transformed_position}")

    def control_loop(self):
        with self.lock:
            if not self.current_pose or not self.path:
                return

            x = self.current_pose.position.x
            y = self.current_pose.position.y
            orientation_q = self.current_pose.orientation
            (_, _, yaw) = euler_from_quaternion([
                orientation_q.x,
                orientation_q.y,
                orientation_q.z,
                orientation_q.w
            ])

            # Lookahead target selection
            target = None
            for wx, wy in self.path:
                dist = math.hypot(wx - x, wy - y)
                if dist > self.lookahead_distance:
                    target = (wx, wy)
                    break

            if not target:
                target = self.path[-1]

            # Goal reached check
            if math.hypot(x - self.path[-1][0], y - self.path[-1][1]) < self.goal_tolerance:
                self.cmd_pub.publish(Twist())  # Stop the robot
                self.get_logger().info("Goal reached. Stopping.")
                self.path = []
                return

            # Compute control commands
            dx = target[0] - x
            dy = target[1] - y
            angle_to_target = math.atan2(dy, dx)
            angle_error = angle_to_target - yaw
            angle_error = (angle_error + math.pi) % (2 * math.pi) - math.pi

            twist = Twist()
            twist.linear.x = min(self.linear_max_speed, 0.5 * math.hypot(dx, dy))
            twist.angular.z = max(-self.angular_max_speed, min(self.angular_max_speed, 1.0 * angle_error))

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info_throttle(1.0, f"Moving to ({target[0]:.2f}, {target[1]:.2f}) → angle error {angle_error:.2f}")


def main(args=None):
    rclpy.init(args=args)
    controller = MPCController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Ctrl+C pressed. Shutting down MPC Controller.")
    finally:
        if rclpy.ok():
            controller.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
