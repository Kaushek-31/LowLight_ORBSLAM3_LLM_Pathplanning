import os
from pathlib import Path
from typing import Union

import numpy as np

from open3d import * 
import matplotlib.pyplot as plt

def save_pointcloud_from_ORB_SLAM(input_file: Union[Path, str], output_file: Union[Path,str] = "out.ply"):
    """Converts a comma separated list of map point coordinates into
    PLY format for viewing the generated map.

    Args:
        input_file (str or Path): Path to the input file which is expected to
        be a .csv file with the columns pos_x, pos_y, pos_z designating the
        coordinates of the points in the world reference frame.

        output_file (str or Path): Path to the output .ply file, format is
        described here: https://paulbourke.net/dataformats/ply/
    """

    coords = np.genfromtxt(input_file, delimiter=", ", skip_header=1)

    x = coords[:, 0]
    y = coords[:, 1]
    z = coords[:, 2]

    ply_header = 'ply\n' \
                'format ascii 1.0\n' \
                'element vertex %d\n' \
                'property float x\n' \
                'property float y\n' \
                'property float z\n' \
                'end_header' % x.shape[0]

    np.savetxt(output_file, np.column_stack((x, y, z)), fmt='%f %f %f', header=ply_header, comments='')

def save_trajectory_from_ORB_SLAM(input_file: Union[Path, str], output_file: Union[Path,str] = "out_trajectory.ply"):
    """Converts the saved trajectory file from ORB-SLAM3 to a point cloud to then
    show alongside the mapped cloud.

    The input file is expected to be in the format (KITTI format with image path prepended, can ignore it):
    /path/to/image0.png R_00 R_01 R_02 t_0 R_10 R_11 R_12 t_1 R_20 R_21 R_22 t_2
    /path/to/image1.png R_00 R_01 R_02 t_0 R_10 R_11 R_12 t_1 R_20 R_21 R_22 t_2

    Where the R terms are the rotation and t terms are the translation terms
    of the homogeneous transformation matrix T_w_cam0.
    """
    x = []
    y = []
    z = []

    with open(input_file, "r") as file:
        lines = file.readlines()

    for line in lines:
        cols = line.strip().split(" ")

        x.append(float(cols[4]))
        y.append(float(cols[8]))
        z.append(float(cols[12]))

    # Each trajectory point is shown as a "point" in the "point cloud" which is the trajectory
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)

    # RGB values for each point on the trajectory, set to be light green
    r = np.ones_like(x) * 144
    g = np.ones_like(x) * 238
    b = np.ones_like(x) * 144

    ply_header = 'ply\n' \
                'format ascii 1.0\n' \
                'element vertex %d\n' \
                'property float x\n' \
                'property float y\n' \
                'property float z\n' \
                'property uchar red\n' \
                'property uchar green\n' \
                'property uchar blue\n' \
                'end_header' % x.shape

    np.savetxt(output_file, np.column_stack((x, y, z, r, g, b)), fmt='%f %f %f %d %d %d', header=ply_header, comments='')

def pointcloud_to_grid(output_file, img_output, resolution=0.05):

    pcd = io.read_point_cloud(output_file) 
    visualize_pc(output_file)
    # Convert to numpy array
    points = np.asarray(pcd.points)
    print(f"Loaded {len(points)} points")

    # Project to XY plane
    xy_points = points[:, ::2]

    # Bounds
    min_xy = np.min(xy_points, axis=0)
    max_xy = np.max(xy_points, axis=0)
    print(f"Min XY: {min_xy}, Max XY: {max_xy}")

    width = int(np.ceil((max_xy[0] - min_xy[0]) / resolution)) + 1
    height = int(np.ceil((max_xy[1] - min_xy[1]) / resolution)) + 1
    print(f"Grid size: {width} x {height}")

    grid = np.zeros((height, width), dtype=np.uint8)

    # Mark occupied cells
    for x, y in xy_points:
        col = int((x - min_xy[0]) / resolution)
        row = int((y - min_xy[1]) / resolution)
        # Flip Y to make origin bottom-left in image
        row = height - 1 - row
        grid[row, col] = 255

    save_grid_as_image(grid, img_output)
    return grid

def save_grid_as_image(grid, filename="grid_map.png"):
    plt.imsave(filename, grid, cmap="gray")
    print(f"Saved grid map as {filename}")

def visualize_pc(filename):
    cloud = io.read_point_cloud(filename) # Read point cloud
    visualization.draw_geometries([cloud])    # Visualize point cloud      


if __name__ == "__main__":
    input_file = "/home/colcon_ws/PointCloudData.txt" # ReferenceMapPoints seem to work better, use that file
    output_file = "output.ply"
    img_output = "grid_map.png"
    input_trajectory = "/home/colcon_ws/KeyFrameTrajectory.txt"
    output_trajectory = "output_trajectory.ply"

    #save_pointcloud_from_ORB_SLAM(input_file, output_file)
    #visualize_pc(output_file)
    
    pointcloud_to_grid(output_file, img_output)
    #save_trajectory_from_ORB_SLAM(input_trajectory, output_trajectory)
