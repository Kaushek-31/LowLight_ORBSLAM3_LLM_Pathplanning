#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf2_ros import TFMessage
from geometry_msgs.msg import TransformStamped, Pose, Twist
from nav_msgs.msg import Odometry
import math
import tf_transformations

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')

        # Initialize the buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscription to /tf to receive transform data
        self.create_subscription(TFMessage, '/tf', self.transform_callback, 10)
        self.pose_pub = self.create_publisher(Pose, '/current_pose', 10)
        # MPC Controller related publisher and subscriber
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # State variables
        self.current_pose = None
        self.target_pose = None

        self.get_logger().info("Transform Listener initialized.")

    def transform_callback(self, msg: TFMessage):
        # This callback will handle the TF messages
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'base_link':
                # Update the robot's pose based on the received transform
                self.update_pose(transform)

    def update_pose(self, transform: TransformStamped):
        # Convert the received transform into a Pose object
        self.current_pose = Pose()
        self.current_pose.position.x = transform.transform.translation.x
        self.current_pose.position.y = transform.transform.translation.y
        self.current_pose.position.z = transform.transform.translation.z

        # Quaternion to Euler conversion for orientation (if needed)
        orientation_q = transform.transform.rotation
        _, _, yaw = self.euler_from_quaternion(orientation_q)
        
        # Convert Euler angles back to quaternion if necessary for full orientation
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)  # Assuming no roll/pitch for simplicity
        self.current_pose.orientation.x = quaternion[0]
        self.current_pose.orientation.y = quaternion[1]
        self.current_pose.orientation.z = quaternion[2]
        self.current_pose.orientation.w = quaternion[3]
        self.pose_pub.publish(self.current_pose)

        self.get_logger().info(f"Updated Pose: {self.current_pose.position.x}, {self.current_pose.position.y}, {yaw}")

    def euler_from_quaternion(self, quaternion):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw).
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        roll_x, pitch_y, yaw_z = tf_transformations.euler_from_quaternion([x, y, z, w])
        return roll_x, pitch_y, yaw_z

    def odom_callback(self, msg: Odometry):
        # Process the odometry data if necessary
        self.target_pose = msg.pose.pose

    def move_robot(self):
        # Example logic to use the transform data in MPC path computation
        if self.current_pose and self.target_pose:
            twist = Twist()
            # Use transforms for path computation (e.g., adjust velocity based on current and target pose)
            dx = self.target_pose.position.x - self.current_pose.position.x
            dy = self.target_pose.position.y - self.current_pose.position.y

            distance = math.hypot(dx, dy)
            angle = math.atan2(dy, dx)

            twist.linear.x = min(0.5, distance)  # Linear speed proportional to distance
            twist.angular.z = min(1.0, angle)  # Angular speed proportional to angle

            self.cmd_vel_pub.publish(twist)
            self.get_logger().info(f"Moving robot: Linear speed = {twist.linear.x}, Angular speed = {twist.angular.z}")


def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Transform Listener Node interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
