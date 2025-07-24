#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for testing rviz plugin better point cloud2 display."""

    # Test node for rviz plugin
    test_node = Node(
        package="erl_geometry_rviz_plugin",
        executable="test_rviz_plugin_better_point_cloud2_display",
        name="test_rviz_plugin_better_point_cloud2_display",
        output="screen",
        parameters=[{"mesh_file": PathJoinSubstitution([FindPackageShare("erl_geometry"), "data", "bunny_z_up.ply"])}],
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
                    "test_rviz_plugin_better_point_cloud2_display.rviz",
                ]
            ),
        ],
    )

    return LaunchDescription(
        [
            test_node,
            map_odom_transformer,
            rviz_node,
        ]
    )
