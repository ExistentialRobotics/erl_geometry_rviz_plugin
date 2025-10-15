from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_transform_publisher",
                arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            ),
            Node(
                package="erl_geometry_rviz_plugin",
                executable="test_rviz_plugin_grid_map_display",
                name="test_grid_map_display_node",
                parameters=[
                    {
                        "encoding": "float",
                        "frequency": 2.0,
                        "publish_rate": 5.0,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("erl_geometry_rviz_plugin"),
                            "rviz2",
                            "test_rviz_plugin_grid_map_display.rviz",
                        ]
                    ),
                ],
            ),
        ],
    )
