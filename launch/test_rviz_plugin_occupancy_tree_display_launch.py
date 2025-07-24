#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for testing rviz plugin occupancy tree display."""

    # Declare launch arguments
    is_double_arg = DeclareLaunchArgument(
        "is_double",
        default_value="false",
        description="Whether to use double precision",
    )
    is_3d_arg = DeclareLaunchArgument(
        "is_3d",
        default_value="true",
        description="Whether to use 3D mode",
    )
    publish_binary_msg_arg = DeclareLaunchArgument(
        "publish_binary_msg",
        default_value="false",
        description="Whether to publish binary messages",
    )

    # Test node for rviz plugin occupancy tree display
    test_node = Node(
        package="erl_geometry_rviz_plugin",
        executable="test_rviz_plugin_occupancy_tree_display",
        name="test_rviz_plugin_occupancy_tree_grid_display",
        output="screen",
        parameters=[
            {
                "is_double": LaunchConfiguration("is_double"),
                "is_3d": LaunchConfiguration("is_3d"),
                "publish_binary_msg": LaunchConfiguration("publish_binary_msg"),
                "map_file": PathJoinSubstitution(
                    [FindPackageShare("erl_geometry"), "data", "house_expo_room_1451.json"]
                ),
                "mesh_file": PathJoinSubstitution(
                    [FindPackageShare("erl_geometry"), "data", "house_expo_room_1451.ply"]
                ),
                "traj_file": PathJoinSubstitution(
                    [FindPackageShare("erl_geometry"), "data", "house_expo_room_1451.csv"]
                ),
            }
        ],
    )

    # Static transform publisher (map -> odom)
    map_odom_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_transformer",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--qx",
            "0",
            "--qy",
            "0",
            "--qz",
            "0",
            "--qw",
            "1",
            "--frame-id",
            "map",
            "--child-frame-id",
            "odom",
        ],
    )

    # RViz2 node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    FindPackageShare("erl_geometry_rviz_plugin"),
                    "rviz2",
                    "test_rviz_plugin_occupancy_tree_display.rviz",
                ]
            ),
        ],
    )

    return LaunchDescription(
        [
            # Launch arguments
            is_double_arg,
            is_3d_arg,
            publish_binary_msg_arg,
            # Nodes
            test_node,
            map_odom_transformer,
            rviz_node,
        ]
    )
