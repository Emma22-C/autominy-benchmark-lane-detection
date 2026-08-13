# Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
# Licensed under the Apache License, Version 2.0 (the "License").

"""Run FCM detection + metrics collector. Use this for clean timing runs.

The collector subscribes to the FCM publishers and to the simulator's
ground-truth odometry, writing one row per detection frame to a CSV.
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fcm_share = get_package_share_directory('lane_detection_fcm')
    eval_share = get_package_share_directory('lane_detection_evaluation')

    fcm_params = os.path.join(fcm_share, 'config', 'fcm_params.yaml')
    eval_params = os.path.join(eval_share, 'config', 'evaluation_params.yaml')

    # CSV con timestamp por defecto, sobrescribible.
    default_csv = '/tmp/lane_detection_fcm_' + \
                  datetime.now().strftime('%Y%m%d_%H%M%S') + '.csv'

    csv_arg = DeclareLaunchArgument(
        'csv_path',
        default_value=default_csv,
        description='Output CSV path for the metrics collector.',
    )
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/sensors/camera/color/image_rect_color',
        description='Camera image topic.',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/simulation/odom_ground_truth',
        description='Ground-truth odometry topic from the simulator.',
    )

    fcm_node = Node(
        package='lane_detection_fcm',
        executable='lane_detection_fcm_node',
        name='lane_detection_fcm',
        output='screen',
        parameters=[
            fcm_params,
            {'input_image_topic': LaunchConfiguration('input_topic')},
        ],
        emulate_tty=True,
    )

    collector_node = Node(
        package='lane_detection_evaluation',
        executable='metrics_collector_node',
        name='metrics_collector',
        output='screen',
        parameters=[
            eval_params,
            {
                'algo_label': 'fcm',
                'output_csv_path': LaunchConfiguration('csv_path'),
                'detection_namespace': '/lane_detection_fcm',
                'odom_topic': LaunchConfiguration('odom_topic'),
                'log_fuzzy_metrics': True,
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        csv_arg,
        input_topic_arg,
        odom_topic_arg,
        fcm_node,
        collector_node,
    ])
