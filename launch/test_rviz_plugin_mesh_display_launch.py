#!/usr/bin/env python3

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Generate launch description for testing rviz plugin mesh display."""

    mesh_file_arg = DeclareLaunchArgument(
        "mesh_file",
        default_value=PathJoinSubstitution([FindPackageShare("erl_geometry"), "data", "replica-office-0.ply"]),
        description="Path to the mesh file to display in RViz.",
    )

    # Test node for rviz plugin
    test_node = Node(
        package="erl_geometry_rviz_plugin",
        executable="test_rviz_plugin_mesh_display",
        name="test_rviz_plugin_mesh_display",
        output="screen",
        parameters=[{"mesh_file": LaunchConfiguration("mesh_file")}],
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
                    "test_rviz_plugin_mesh_display.rviz",
                ]
            ),
        ],
    )

    return LaunchDescription(
        [
            mesh_file_arg,
            test_node,
            map_odom_transformer,
            rviz_node,
        ]
    )
