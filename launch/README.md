Launch Files
============

- **test_rviz_plugin_occupancy_tree_display**:
  Launches the `test_rviz_plugin_occupancy_tree_display` node to test the rviz plugin for
  visualizing an occupancy quadtree or octree as grids or an occupancy map.

  - arguments:
    - is_3d: true or false
    - is_double: true or false
    - publish_binary_msg: true or false

  <details>
  <summary><b>ROS1 Usage</b></summary>
  ```shell
  roslaunch erl_geometry_rviz_plugin test_rviz_plugin_occupancy_tree_display.launch
  ```
  </details>

  <details>
  <summary><b>ROS2 Usage</b></summary>

  ```shell
  ros2 launch erl_geometry_rviz_plugin test_rviz_plugin_occupancy_tree_display_launch.py
  ```
  Example with arguments:
  ```shell
  ros2 launch erl_geometry_rviz_plugin test_rviz_plugin_occupancy_tree_display_launch.py \
      is_3d:=false is_double:=true publish_binary_msg:=true
  ```
  </details>

  |                                                  |                                                 |
  | ------------------------------------------------ | ----------------------------------------------- |
  | ![](assets/test_occupancy_tree_grid_display.png) | ![](assets/test_occupancy_tree_map_display.png) |

- **test_rviz_plugin_better_point_cloud2_display**:
  Launches the `test_rviz_plugin_better_point_cloud2_display` node to test the rviz plugin for
  visualizing a point cloud2 with better performance.

  <details>
  <summary><b>ROS1 Usage</b></summary>

  ```shell
  roslaunch erl_geometry_rviz_plugin test_rviz_plugin_better_point_cloud2_display.launch
  ```
  </details>

  <details>
  <summary><b>ROS2 Usage</b></summary>

  ```shell
  ros2 launch erl_geometry_rviz_plugin test_rviz_plugin_better_point_cloud2_display_launch.py
  ```
  </details>

  |                                                              |
  | ------------------------------------------------------------ |
  | ![](assets/test_better_point_cloud2_display.png) |
