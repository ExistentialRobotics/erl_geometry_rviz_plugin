#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for testing rviz plugin frontier display."""

    is_3d_arg = DeclareLaunchArgument(
        "is_3d",
        default_value="true",
        description="3D mode (octree + triangle frontiers) or 2D mode (quadtree + line frontiers)",
    )

    # Test node: publishes both OccupancyTreeMsg and FrontierArray
    test_node = Node(
        package="erl_geometry_rviz_plugin",
        executable="test_rviz_plugin_frontier_display",
        name="test_frontier_display",
        output="screen",
        parameters=[{"is_3d": LaunchConfiguration("is_3d")}],
    )

    # Static transform publisher (map -> odom)
    map_odom_transformer = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_odom_transformer",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",
            "--frame-id", "map",
            "--child-frame-id", "odom",
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
                    "test_rviz_plugin_frontier_display.rviz",
                ]
            ),
        ],
    )

    return LaunchDescription(
        [
            is_3d_arg,
            test_node,
            map_odom_transformer,
            rviz_node,
        ]
    )
