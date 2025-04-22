#!/usr/bin/env python3
import numpy as np
import os
from PIL import Image

class ORBSLAM3OfflineGridMap:
    def __init__(self,
                 trajectory_file='KeyFrameTrajectory.txt',
                 map_points_file='MapPoints.txt',
                 scale_factor=10.0,
                 grid_resolution=0.1,
                 grid_width=500,
                 grid_height=500,
                 free_threshold=0.7,
                 occupied_threshold=0.3,
                 output_file='occupancy_grid_map.pgm'):
        
        self.trajectory_file = trajectory_file
        self.map_points_file = map_points_file
        self.output_file = output_file

        self.scale_factor = scale_factor
        self.grid_resolution = grid_resolution
        self.grid_width = grid_width
        self.grid_height = grid_height
        self.free_threshold = free_threshold
        self.occupied_threshold = occupied_threshold

        self.visit_counter = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)
        self.occupied_counter = np.zeros((self.grid_height, self.grid_width), dtype=np.int32)

        self.keyframes = self.load_keyframes()
        self.map_points = self.load_map_points()

    def load_keyframes(self):
        keyframes = []
        with open(self.trajectory_file, 'r') as f:
            for line in f:
                if line.startswith('#') or line.strip() == '':
                    continue
                parts = line.strip().split()
                x = float(parts[1])
                z = float(parts[3])
                keyframes.append(self.convert_to_grid_indices(x, z))
        print(f'Loaded {len(keyframes)} keyframes.')
        return keyframes

    def load_map_points(self):
        map_points = []
        with open(self.map_points_file, 'r') as f:
            for line in f:
                if line.startswith('#') or line.strip() == '':
                    continue
                parts = line.strip().split()
                x = float(parts[0])
                z = float(parts[2])
                map_points.append((x, z))
        print(f'Loaded {len(map_points)} map points.')
        return map_points

    def convert_to_grid_indices(self, x, z):
        grid_x = int((x * self.scale_factor) + self.grid_width // 2)
        grid_y = int((z * self.scale_factor) + self.grid_height // 2)
        grid_x = min(max(grid_x, 0), self.grid_width - 1)
        grid_y = min(max(grid_y, 0), self.grid_height - 1)
        return (grid_x, grid_y)

    def ray_cast(self, start, end):
        x0, y0 = start
        x1, y1 = end
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        x, y = x0, y0
        while True:
            self.visit_counter[y, x] += 1
            if (x, y) == (x1, y1):
                self.occupied_counter[y, x] += 1
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def build_grid(self):
        for cam_pos in self.keyframes:
            for point in self.map_points:
                grid_point = self.convert_to_grid_indices(point[0], point[1])
                self.ray_cast(cam_pos, grid_point)
        print('Raycasting complete.')

    def generate_occupancy_grid(self):
        grid = np.full((self.grid_height, self.grid_width), 205, dtype=np.uint8)  # Unknown: 205
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                visits = self.visit_counter[y, x]
                occupied = self.occupied_counter[y, x]
                if visits == 0:
                    grid[y, x] = 205
                else:
                    p_free = 1.0 - (occupied / visits)
                    if p_free > self.free_threshold:
                        grid[y, x] = 254  # Free (white)
                    elif p_free < self.occupied_threshold:
                        grid[y, x] = 0    # Occupied (black)
                    else:
                        grid[y, x] = 205  # Unknown (gray)
        return grid

    def save_pgm(self, grid):
        image = Image.fromarray(grid[::-1], mode='L')  # Flip vertically for correct orientation
        image.save(self.output_file)
        print(f'Saved occupancy grid to {self.output_file}')


def main():
    generator = ORBSLAM3OfflineGridMap(
        trajectory_file='/home/kaushek/MRob/colcon_ws/KeyFrameTrajectory.txt',
        map_points_file='/home/kaushek/MRob/colcon_ws/bcr_bot_rob530.osa',  # generated from .osa using map_to_text
        output_file='occupancy_grid_map.pgm'
    )
    generator.build_grid()
    grid = generator.generate_occupancy_grid()
    generator.save_pgm(grid)

if __name__ == '__main__':
    main()
