# erl_geometry_rviz_plugin

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![ROS1](https://img.shields.io/badge/ROS1-noetic-blue)](http://wiki.ros.org/)
[![ROS2](https://img.shields.io/badge/ROS2-humble-blue)](https://docs.ros.org/)

A collection of custom RViz plugins for visualizing geometry data structures and enhanced point cloud displays. This package provides specialized visualization tools for occupancy trees, enhanced point clouds with normal vectors, and other geometric data.

|                                                         |                                                         |                                                        |
| ------------------------------------------------------- | ------------------------------------------------------- | ------------------------------------------------------ |
| ![](launch/assets/test_better_point_cloud2_display.png) | ![](launch/assets/test_occupancy_tree_grid_display.png) | ![](launch/assets/test_occupancy_tree_map_display.png) |

## Features

- **ROS Support**: Compatible with both ROS1 and ROS2
- **Enhanced Point Cloud Visualization**: Display point clouds with normal vectors and improved performance
- **Occupancy Tree Visualization**: Visualize quadtrees and octrees as grids or occupancy maps
- **Custom Geometry Message Support**: Specialized displays for [erl_geometry_msgs](https://github.com/ExistentialRobotics/erl_geometry_msgs)

## Available RViz Displays

### BetterPointCloud2 Display
Enhanced point cloud visualization with support for displaying normal vectors and improved rendering performance.

**Features:**
- Display normal vectors from point cloud data
- Configurable normal vector properties (length, width, color, stride)
- Multiple color modes for normals (constant color, rainbow)
- Improved rendering performance over standard PointCloud2 display

**Message Type:** `sensor_msgs/PointCloud2`

### OccupancyTreeGrid Display
Visualizes occupancy quadtrees (2D) or octrees (3D) as individual grid cells.

**Features:**
- Display occupancy probability as colored grid cells
- Support for both 2D quadtrees and 3D octrees
- Configurable tree depth visualization
- Height filtering for 3D trees

**Message Type:** `erl_geometry_msgs/OccupancyTreeMsg`

### OccupancyTreeMap Display
Converts occupancy trees into standard occupancy grid maps for visualization.

**Features:**
- Convert quadtree/octree to 2D occupancy grid
- Configurable tree depth and height filtering
- Standard occupancy map visualization
- Compatible with navigation stack displays

**Message Type:** `erl_geometry_msgs/OccupancyTreeMsg`

## Getting Started

### Create Workspace

```bash
mkdir -p <your_workspace>/src && \
vcs import --input https://raw.githubusercontent.com/ExistentialRobotics/erl_geometry_rviz_plugin/refs/heads/main/erl_geometry_rviz_plugin.repos <your_workspace>/src
```
### Prerequisites

- ROS1 Noetic or ROS2 Humble
- C++17 compatible compiler
- CMake 3.24 or higher

### Dependencies

This package depends on the following ERL packages:
- `erl_cmake_tools`
- `erl_common`
- `erl_covariance`
- `erl_geometry`

```bash
# Ubuntu 20.04
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_common/refs/heads/main/scripts/setup_ubuntu_20.04.bash | bash
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_geometry/refs/heads/main/scripts/setup_ubuntu_20.04.bash | bash
# Ubuntu 22.04, 24.04
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_common/refs/heads/main/scripts/setup_ubuntu_22.04_24.04.bash | bash
wget -qO - https://raw.githubusercontent.com/ExistentialRobotics/erl_geometry/refs/heads/main/scripts/setup_ubuntu_22.04_24.04.bash | bash
```

ROS dependencies:

- ROS1
   ```bash
   # Install ROS1 dependencies
   sudo apt install ros-noetic-rviz \
                  ros-noetic-pluginlib \
                  ros-noetic-std-msgs \
                  ros-noetic-nav-msgs \
                  ros-noetic-geometry-msgs \
                  ros-noetic-sensor-msgs \
                  ros-noetic-tf2-ros

   # Install OGRE development libraries
   sudo apt install libogre-1.9-dev qtbase5-dev
   ```

- ROS2
   ```bash
   # Install ROS2 dependencies
   export ROS_DISTRO=humble  # or your desired ROS2 distro
   sudo apt install ros-$ROS_DISTRO-rviz2 \
                  ros-$ROS_DISTRO-rviz-common \
                  ros-$ROS_DISTRO-rviz-rendering \
                  ros-$ROS_DISTRO-rviz-default-plugins \
                  ros-$ROS_DISTRO-pluginlib \
                  ros-$ROS_DISTRO-std-msgs \
                  ros-$ROS_DISTRO-nav-msgs \
                  ros-$ROS_DISTRO-geometry-msgs \
                  ros-$ROS_DISTRO-sensor-msgs \
                  ros-$ROS_DISTRO-tf2-ros

   # Install development tools
   sudo apt install libogre-1.9-dev qtbase5-dev
   ```

### Building the Package

```bash
cd <your_workspace>
# for ROS1
catkin build erl_geometry_ros
source devel/setup.bash
# for ROS2
colcon build --packages-up-to erl_geometry_ros
source install/setup.bash
```

## Usage

### Loading Plugins in RViz

First, remember to source your workspace.

<details>
<summary><b>ROS1 - Loading Plugins</b></summary>

1. Launch RViz: `rosrun rviz rviz`
2. Click "Add" in the Displays panel
3. Select "By display type" tab
4. Find plugins under:
   - `erl_geometry_rviz_plugin/OccupancyTreeGrid`
   - `erl_geometry_rviz_plugin/OccupancyTreeMap`
   - `erl_geometry_rviz_plugin/BetterPointCloud2`

</details>

<details>
<summary><b>ROS2 - Loading Plugins</b></summary>

1. Launch RViz2: `ros2 run rviz2 rviz2`
2. Click "Add" in the Displays panel
3. Select "By display type" tab
4. Find plugins under:
   - `erl_geometry/OccupancyTreeGrid`
   - `erl_geometry/OccupancyTreeMap`
   - `BetterPointCloud2`

</details>

### Test Launch Files

The package includes test launch files to demonstrate plugin functionality:

#### BetterPointCloud2 Display Test

<details>
<summary><b>ROS1</b></summary>

```bash
roslaunch erl_geometry_rviz_plugin test_rviz_plugin_better_point_cloud2_display.launch
```
</details>

<details>
<summary><b>ROS2</b></summary>

```bash
ros2 launch erl_geometry_rviz_plugin test_rviz_plugin_better_point_cloud2_display_launch.py
```
</details>

#### OccupancyTree Display Test

<details>
<summary><b>ROS1</b></summary>

```bash
roslaunch erl_geometry_rviz_plugin test_rviz_plugin_occupancy_tree_display.launch
```
</details>

<details>
<summary><b>ROS2</b></summary>

```bash
ros2 launch erl_geometry_rviz_plugin test_rviz_plugin_occupancy_tree_display_launch.py
```
</details>


## Troubleshooting

1. **Plugins not loaded**: Verify workspace is sourced: `source <devel|install>/setup.bash`
1. **Large point clouds**: Increase normal stride to reduce rendering load
2. **Deep trees**: Limit tree depth visualization to improve performance
3. **Memory usage**: Use float precision instead of double when possible
