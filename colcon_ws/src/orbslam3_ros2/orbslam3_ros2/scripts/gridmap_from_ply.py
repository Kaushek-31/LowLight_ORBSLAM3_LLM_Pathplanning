import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt

def pointcloud_to_grid(points, resolution=0.05):
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

    return grid

def save_grid_as_image(grid, filename="grid_map.png"):
    plt.imsave(filename, grid, cmap="gray")
    print(f"Saved grid map as {filename}")

if __name__ == "__main__":
    # Load point cloud
    pcd = o3d.io.read_point_cloud("output.ply") 

    # Convert to numpy array
    points = np.asarray(pcd.points)
    print(f"Loaded {len(points)} points")

    # Generate grid map
    grid = pointcloud_to_grid(points, resolution=0.05)

    # Save as image
    save_grid_as_image(grid, "grid_map.png")

