
# LLM-Based Path Planning for Low-Light Navigation Using ORB-SLAM3

This repository presents an integrated framework for autonomous navigation in low-light conditions by combining ORB-SLAM3 with a Large Language Model (LLM)-based waypoint planner and a global D* Lite path planner. The system accepts high-level semantic commands, generates interpretable waypoints, refines them through obstacle-aware planning, and executes them on a mobile robot platform simulated using the [bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot).

---

## Prerequisites

### OpenCV 4.5.4 with `opencv_contrib` Modules

```bash
sudo apt update && sudo apt install -y build-essential cmake g++ git wget unzip pkg-config \
libgtk-3-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev gfortran \
openexr libatlas-base-dev python3-dev python3-numpy ninja-build

cd ~/rob530
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.4.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.4.zip
unzip opencv.zip && unzip opencv_contrib.zip
mv opencv-4.5.4 opencv && mv opencv_contrib-4.5.4 opencv_contrib

cd opencv && mkdir build && cd build
cmake -GNinja -D CMAKE_BUILD_TYPE=Release \
  -D CMAKE_INSTALL_PREFIX=/usr/local \
  -D OPENCV_EXTRA_MODULES_PATH=~/rob530/opencv_contrib/modules \
  -D BUILD_EXAMPLES=ON ..
ninja
sudo ninja install
sudo ldconfig
python3 -c "import cv2; print(cv2.__version__)"
```

---

## ORB-SLAM3 Setup

- Clone: [https://github.com/UZ-SLAMLab/ORB_SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3)  
- Before building, modify `CMakeLists.txt` to include:
  ```cmake
  add_compile_options(-std=c++14)
  ```
- Follow the build instructions provided in the repository.

---

## bcr_bot Simulation Setup

Use the `bcr_bot` repository for robot simulation:

- Repository: [https://github.com/blackcoffeerobotics/bcr_bot](https://github.com/blackcoffeerobotics/bcr_bot)  
- Make sure Gazebo and ROS 2 Humble are properly installed.

---

## System Execution

### Step 1: Launch ORB-SLAM3 and Occupancy Mapping

```bash
ros2 run orbslam3 mono ~/rob530/ORB_SLAM3/Vocabulary/ORBvoc.txt ./src/orbslam3_ros2/config/monocular/RealSense_R200.yaml true
ros2 run orbslam3 occupancy_grid_node.py
```

### Step 2: Visualize in RViz

```bash
ros2 run rviz2 rviz2
```

- Add a "Map" display and set its topic to `/occupancy_grid_map`.

### Step 3: Launch the Full System

```bash
ros2 launch bcr_bot llm_navigation.launch.py
```

### Step 4: Run the Map Server

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=$HOME/rob530/bcr_bot/maps/bcr_map.yaml
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
```

### Step 5: Start the LLM Server

```bash
ollama serve
ollama pull llama3
ollama run llama3
```

---

## Sending Navigation Commands

### Example

```bash
ros2 topic pub /user_command std_msgs/String "data: '{
  \"command\": \"navigate\",
  \"from\": \"Chair\",
  \"to\": \"pine_tree\",
  \"constraints\": {
    \"safe_margin\": 0.5,
    \"waypoint_spacing\": 0.7
  }
}'"
```

### Supported Parameters

| Key              | Type     | Description                                  | Example        |
|------------------|----------|----------------------------------------------|----------------|
| `safe_margin`    | float    | Minimum distance from obstacles              | `0.5`          |
| `spacing`        | float    | Distance between waypoints                   | `0.7`          |
| `avoid_labels`   | list     | Semantic labels to avoid                     | `["wall"]`     |
| `max_waypoints`  | int      | Maximum number of waypoints                  | `3`            |
| `max_distance`   | float    | Maximum allowed path length (optional)       | `15.0`         |
| `preferred_zones`| list     | Preferred regions to traverse (optional)     | `["furniture"]`|

### Object Labels

| Object Name         | Semantic Label     |
|---------------------|--------------------|
| Chair               | furniture          |
| Bookshelf, Shelf    | storage_unit       |
| Fountain            | equipment          |
| Pine Tree           | natural_object     |
| Marble Table        | furniture          |
| Mailbox             | equipment          |
| Trash Can           | movable_obstacle   |
| Cardboard Box       | movable_obstacle   |
| Construction Cone   | warning_marker     |
| Trisphere Cycle     | movable_obstacle   |

---

## Results

### Demonstration Video

A complete video demonstration of the LLM-based navigation system in a simulated low-light environment is available at:
https://www.youtube.com/watch?v=sBrYsaEU58E

