#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import json
import re
import matplotlib.pyplot as plt
import os

def validate_waypoints(waypoints, environment_data, safe_margin, target_position, node: Node):
    env_width = environment_data.get('environment_width', 100.0)
    env_length = environment_data.get('environment_length', 100.0)

    x_min = -env_width / 2 + safe_margin
    x_max = env_width / 2 - safe_margin
    y_min = -env_length / 2 + safe_margin
    y_max = env_length / 2 - safe_margin

    node.get_logger().info(f"Validating waypoints within environment bounds:")
    node.get_logger().info(f"X: {x_min} to {x_max}")
    node.get_logger().info(f"Y: {y_min} to {y_max}")

    valid_waypoints = []
    for wp in waypoints:
        x, y = wp['x'], wp['y']
        if x_min <= x <= x_max and y_min <= y <= y_max:
            valid_waypoints.append(wp)
        else:
            node.get_logger().warn(f"Waypoint ({x}, {y}) is outside environment bounds and will be ignored.")

    if not valid_waypoints:
        node.get_logger().error("No valid waypoints within environment boundaries.")
        return None

    final_wp = valid_waypoints[-1]
    tx, ty = target_position.get('x'), target_position.get('y')
    tolerance = 0.05
    distance = ((final_wp['x'] - tx) ** 2 + (final_wp['y'] - ty) ** 2) ** 0.5

    node.get_logger().info(f"Final waypoint distance to target: {distance}")
    if distance > tolerance:
        node.get_logger().error("Final waypoint is too far from target.")
        return None

    return valid_waypoints

def agents(llm_client, system_prompt, user_prompt, node: Node):
    try:
        response = llm_client.chat(
            model='llama3.1',
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_prompt}
            ]
        )
        return response['message']['content']
    except Exception as e:
        node.get_logger().error(f"LLM interaction failed: {e}")
        return None

def parse_waypoints(response, node: Node):
    if not response or not isinstance(response, str):
        node.get_logger().error("Empty or invalid response.")
        return None
    try:
        print("parse_waypoints Begin.....")
        response = re.sub(r'^```(json)?\s*', '', response.strip())
        print(response)
        response = re.sub(r'\s*```$', '', response.strip())
        print(response)
        response = response.replace("'", '"')
        print(response)
        match = re.search(r'\[.*]', response, re.DOTALL)
        if not match:
            return None
        waypoints = json.loads(match.group(0))
        return [{'x': float(wp['x']), 'y': float(wp['y'])} for wp in waypoints if 'x' in wp and 'y' in wp]
    except Exception as e:
        node.get_logger().error(f"Failed to parse waypoints: {e}")
        return None

def plot_waypoints(current_position, target_position, waypoints, environment_data, safe_margin):
    width = environment_data.get('environment_width', 100.0)
    length = environment_data.get('environment_length', 100.0)
    x_min = -width / 2 + safe_margin
    x_max = width / 2 - safe_margin
    y_min = -length / 2 + safe_margin
    y_max = length / 2 - safe_margin

    fig, ax = plt.subplots(figsize=(8, 8))
    env_rect = plt.Rectangle((x_min, y_min), x_max - x_min, y_max - y_min,
                             edgecolor='blue', facecolor='lightgray', alpha=0.3, label='Environment')
    ax.add_patch(env_rect)

    ax.plot(current_position['x'], current_position['y'], 'ro', label='Start')
    ax.plot(target_position['x'], target_position['y'], 'go', label='Target')
    if waypoints:
        xs = [wp['x'] for wp in waypoints]
        ys = [wp['y'] for wp in waypoints]
        ax.plot(xs, ys, 'k--', marker='x', label='Waypoints')
        ax.plot([current_position['x'], xs[0]], [current_position['y'], ys[0]], 'k--')

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.grid(True)
    ax.set_aspect("equal")
    plt.title("Waypoints and Environment")
    save_dir = os.path.expanduser("~/Pictures")
    os.makedirs(save_dir, exist_ok=True)
    plt.savefig(os.path.join(save_dir, "waypoints_plot.png"), dpi=300, bbox_inches='tight')
    plt.show()

def generate_waypoints_with_navigation_agent(llm_client, current_pose, target_object, environment_data, safe_margin, waypoint_spacing):
    node = Node('llm_navigation_agent')

    max_attempts = 3

    env_width = environment_data.get('environment_width', 100.0)
    env_length = environment_data.get('environment_length', 100.0)

    x_min = -env_width / 2 + safe_margin
    x_max = env_width / 2 - safe_margin
    y_min = -env_length / 2 + safe_margin
    y_max = env_length / 2 - safe_margin

    current_position = {
        'x': current_pose.position.x,
        'y': current_pose.position.y
    }

    target_position = {
        'x': target_object['position']['x'],
        'y': target_object['position']['y']
    }

    system_prompt = f"""
The robot must navigate from ({current_position['x']:.2f}, {current_position['y']:.2f}) to ({target_position['x']:.2f}, {target_position['y']:.2f}).

Safe margin from boundaries: {safe_margin} meters
Boundaries:
- X: {x_min:.2f} to {x_max:.2f}
- Y: {y_min:.2f} to {y_max:.2f}

Rules:
- Stay within boundaries
- Avoid collisions
- Keep waypoints efficient and direct
- Format strictly as a JSON array with 'x' and 'y' only
"""

    user_prompt = f"""
Start: ({current_position['x']:.2f}, {current_position['y']:.2f})
Goal: ({target_position['x']:.2f}, {target_position['y']:.2f})
Min spacing: {waypoint_spacing}m
Safe margin: {safe_margin}m

Example:
[
  {{ "x": 1.0, "y": 2.0 }},
  {{ "x": 1.0, "y": 4.0 }},
  {{ "x": 1.0, "y": 6.0 }}
]

Waypoints:
"""

    for attempt in range(max_attempts):
        node.get_logger().info(f"Attempt {attempt+1} to generate waypoints.")
        output = agents(llm_client, system_prompt, user_prompt, node)
        waypoints = parse_waypoints(output, node)
        if waypoints:
            waypoints = validate_waypoints(waypoints, environment_data, safe_margin, target_position, node)
            if waypoints:
                plot_waypoints(current_position, target_position, waypoints, environment_data, safe_margin)
                node.destroy_node()
                rclpy.shutdown()
                return waypoints
        node.get_logger().warn("Retrying waypoint generation...")

    node.get_logger().error("All attempts failed.")
    node.destroy_node()
    rclpy.shutdown()
    return None
