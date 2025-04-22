#!/usr/bin/env python3
import numpy as np
from PIL import Image

# === CONFIG ===
TRAJECTORY_FILE = '/home/kaushek/MRob/colcon_ws/KeyFrameTrajectory.txt'  # Path to your file
OUTPUT_PGM_FILE = 'occupancy_map.pgm'       # Output file name
MAP_RESOLUTION = 0.05                       # Each grid cell = 5 cm
MAP_SIZE_METERS = 20                        # 20m x 20m map
GRID_SIZE = int(MAP_SIZE_METERS / MAP_RESOLUTION)
GRID = np.full((GRID_SIZE, GRID_SIZE), 205, dtype=np.uint8)  # 205 = unknown in .pgm (gray)
CENTER = GRID_SIZE // 2
SENSOR_RANGE = 5.0  # meters
FAN_ANGLE_DEG = 90  # degrees
FAN_POINTS = 30     # rays

# === Load Keyframe Trajectory ===
poses = []
with open(TRAJECTORY_FILE, 'r') as f:
    for line in f:
        if line.startswith("#") or not line.strip():
            continue
        data = list(map(float, line.strip().split()))
        _, x, y, z, *_ = data
        poses.append((x, z))  # Use x-z for 2D map

# === Raycast (simulate fan of rays from each pose) ===
def world_to_grid(x, y):
    gx = int(CENTER + x / MAP_RESOLUTION)
    gy = int(CENTER + y / MAP_RESOLUTION)
    return gx, gy

def bresenham(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    while True:
        points.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return points

for (x, z) in poses:
    gx, gy = world_to_grid(x, z)

    # Draw fan rays (simulate map points)
    for angle in np.linspace(-FAN_ANGLE_DEG/2, FAN_ANGLE_DEG/2, FAN_POINTS):
        theta = np.deg2rad(angle)
        dx = SENSOR_RANGE * np.cos(theta)
        dz = SENSOR_RANGE * np.sin(theta)
        px, pz = x + dx, z + dz
        gx1, gy1 = world_to_grid(px, pz)
        for (ix, iy) in bresenham(gx, gy, gx1, gy1)[:-1]:  # free space
            if 0 <= ix < GRID_SIZE and 0 <= iy < GRID_SIZE:
                GRID[iy, ix] = 254
        # last point as occupied
        if 0 <= gx1 < GRID_SIZE and 0 <= gy1 < GRID_SIZE:
            GRID[gy1, gx1] = 0

# === Save as .pgm file ===
img = Image.fromarray(GRID)
img.save(OUTPUT_PGM_FILE)
print(f"Occupancy map saved to {OUTPUT_PGM_FILE}")
