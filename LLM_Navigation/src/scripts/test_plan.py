#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import re

class TestPlanPublisher(Node):
    def __init__(self):
        super().__init__('test_plan')
        self.pub = self.create_publisher(String, '/user_command', 10)
        self.timer = self.create_timer(1.0, self.prompt_input_once)

        self.user_input_pending = True

        self.get_logger().info("Structured Test Plan node started.")
        self.get_logger().info("Examples:")
        self.get_logger().info("  ➤ JSON: {\"command\": \"navigate\", \"from\": \"Chair\", \"to\": \"pine_tree\"}")
        self.get_logger().info("  ➤ Natural: navigate from Chair to pine_tree")

    def prompt_input_once(self):
        if not self.user_input_pending:
            return
        try:
            user_input = input("Enter command (JSON or natural language): ").strip()

            if not user_input:
                return

            # Try JSON parsing first
            try:
                command_json = json.loads(user_input)
            except json.JSONDecodeError:
                # Fallback: Parse natural language input
                command_json = self.parse_natural_command(user_input)
                if not command_json:
                    self.get_logger().error("Could not parse input. Use valid JSON or natural format like: 'navigate from A to B'")
                    return

            if "command" not in command_json or "to" not in command_json:
                self.get_logger().warn("Missing required fields 'command' or 'to'.")
                return

            if "constraints" not in command_json:
                command_json["constraints"] = {}

            json_str = json.dumps(command_json)
            self.get_logger().info("✅ Publishing command:")
            self.get_logger().info(json.dumps(command_json, indent=2))
            self.pub.publish(String(data=json_str))
            self.user_input_pending = False

        except KeyboardInterrupt:
            self.get_logger().info("Keyboard interrupt received. Shutting down.")
            rclpy.shutdown()

    def parse_natural_command(self, command):
        """
        Parses natural language command like:
        'navigate from A to B' or 'go to C'
        """
        patterns = [
            r"navigate from (?P<from>\w+) to (?P<to>\w+)",
            r"go to (?P<to>\w+)",
            r"navigate to (?P<to>\w+)"
        ]
        for pattern in patterns:
            match = re.match(pattern, command, re.IGNORECASE)
            if match:
                data = match.groupdict()
                command_json = {
                    "command": "navigate",
                    "to": data.get("to"),
                    "constraints": {}
                }
                if data.get("from"):
                    command_json["from"] = data["from"]
                return command_json
        return None

def main(args=None):
    rclpy.init(args=args)
    node = TestPlanPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down due to keyboard interrupt.")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
