# Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
# Licensed under the Apache License, Version 2.0 (the "License").

"""Run Sliding Windows detection + metrics collector.

Mirror of evaluate_fcm.launch.py but for the Sliding Windows algorithm.
Use this for clean timing measurements (run separately from the FCM run
to avoid CPU contention biasing the time_ms column).
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sw_share = get_package_share_directory('lane_detection_sliding_windows')
    eval_share = get_package_share_directory('lane_detection_evaluation')

    sw_params = os.path.join(sw_share, 'config', 'sw_params.yaml')
    eval_params = os.path.join(eval_share, 'config', 'evaluation_params.yaml')

    default_csv = '/tmp/lane_detection_sw_' + \
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

    sw_node = Node(
        package='lane_detection_sliding_windows',
        executable='lane_detection_sliding_windows_node',
        name='lane_detection_sliding_windows',
        output='screen',
        parameters=[
            sw_params,
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
                'algo_label': 'sliding_windows',
                'output_csv_path': LaunchConfiguration('csv_path'),
                'detection_namespace': '/lane_detection_sliding_windows',
                'odom_topic': LaunchConfiguration('odom_topic'),
                # Sliding Windows no publica xie_beni/fpc; el collector
                # simplemente registrará NaN en esas columnas.
                'log_fuzzy_metrics': False,
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        csv_arg,
        input_topic_arg,
        odom_topic_arg,
        sw_node,
        collector_node,
    ])
