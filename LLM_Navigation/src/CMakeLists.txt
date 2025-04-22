cmake_minimum_required(VERSION 3.5)
project(bcr_bot)

# Default to C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================
# Dependencies
# ============================
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)

# ============================
# Include Directories
# ============================
include_directories(
  include
  ${gazebo_INCLUDE_DIRS}
)

# ============================
# Python Nodes Installation
# ============================
install(
  PROGRAMS scripts/environment_data_provider.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME environment_data_provider
)

install(
  PROGRAMS scripts/semantic_obstacle_extractor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME semantic_obstacle_extractor
)

install(
  PROGRAMS scripts/object_marker_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_marker_publisher
)

install(
  PROGRAMS scripts/waypoint_executor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_executor
)

install(
  PROGRAMS scripts/llm_path_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME llm_path_planner
)

install(
  PROGRAMS scripts/dstar_lite_planner.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dstar_lite_planner
)

install(
  PROGRAMS scripts/mpc_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mpc_controller
)

install(
  PROGRAMS scripts/metrics_collector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME metrics_collector
)

install(PROGRAMS
  scripts/transform_listener_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME transform_listener_node
)

install(
  PROGRAMS scripts/test_plan.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME test_plan
)

install(
  PROGRAMS scripts/remapper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME remapper
)
# ============================
# Directories to Install
# ============================
install(DIRECTORY scripts/utils
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY object_list
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  models
  rviz
  worlds
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY maps
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# ============================
# Final
# ============================
ament_package()
