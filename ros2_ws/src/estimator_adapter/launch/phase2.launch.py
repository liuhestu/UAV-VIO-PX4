import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("estimator_adapter")
    return LaunchDescription([
        DeclareLaunchArgument("output_enabled", default_value="false"),
        DeclareLaunchArgument("replay_mode", default_value="true"),
        Node(
            package="estimator_adapter",
            executable="estimator_adapter_node",
            name="estimator_adapter",
            output="screen",
            parameters=[
                os.path.join(share_dir, "config", "phase2.yaml"),
                os.path.join(share_dir, "config", "extrinsics.yaml"),
                {
                "output_enabled": LaunchConfiguration("output_enabled"),
                "replay_mode": LaunchConfiguration("replay_mode"),
                },
            ],
        ),
    ])
