# Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
# Licensed under the Apache License, Version 2.0 (the "License").

"""Launch file for the Hough Transform lane detection node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lane_detection_hough')
    default_params = os.path.join(pkg_share, 'config', 'hough_params.yaml')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to the parameters YAML file.',
    )

    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/sensors/camera/color/image_rect_color',
        description='Camera image topic to subscribe to.',
    )

    hough_node = Node(
        package='lane_detection_hough',
        executable='lane_detection_hough_node',
        name='lane_detection_hough',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'input_image_topic': LaunchConfiguration('input_topic')},
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        params_arg,
        input_topic_arg,
        hough_node,
    ])
