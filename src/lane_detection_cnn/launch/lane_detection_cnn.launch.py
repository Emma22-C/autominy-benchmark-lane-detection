import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("lane_detection_cnn"),
        "config",
        "params.yaml",
    )

    return LaunchDescription([
        Node(
            package="lane_detection_cnn",
            executable="lane_detection_cnn_node",
            name="lane_detection_cnn",
            output="screen",
            parameters=[config],
        ),
    ])
