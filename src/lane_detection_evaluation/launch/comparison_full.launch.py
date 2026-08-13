# Copyright 2026 J. E. Vidal-Cuevas, F. R. Trejo-Macotela et al.
# Licensed under the Apache License, Version 2.0 (the "License").

"""Run ALL THREE detection algorithms in parallel + three metrics collectors.

WARNING: this launch is intended for QUALITATIVE debugging only (so you
can rqt_image_view the three ~/debug_image topics side-by-side).
Do NOT use the resulting CSVs for the timing column of the comparison
table: CPU contention between the three processes will bias time_ms.
For publication-grade timing data, use evaluate_fcm.launch.py,
evaluate_sw.launch.py and evaluate_hough.launch.py in separate runs
(ideally replaying the same rosbag for each one).
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
    sw_share = get_package_share_directory('lane_detection_sliding_windows')
    hough_share = get_package_share_directory('lane_detection_hough')
    eval_share = get_package_share_directory('lane_detection_evaluation')

    fcm_params = os.path.join(fcm_share, 'config', 'fcm_params.yaml')
    sw_params = os.path.join(sw_share, 'config', 'sw_params.yaml')
    hough_params = os.path.join(hough_share, 'config', 'hough_params.yaml')
    eval_params = os.path.join(eval_share, 'config', 'evaluation_params.yaml')

    ts = datetime.now().strftime('%Y%m%d_%H%M%S')

    csv_fcm_arg = DeclareLaunchArgument(
        'csv_fcm', default_value=f'/tmp/lane_detection_fcm_parallel_{ts}.csv')
    csv_sw_arg = DeclareLaunchArgument(
        'csv_sw', default_value=f'/tmp/lane_detection_sw_parallel_{ts}.csv')
    csv_hough_arg = DeclareLaunchArgument(
        'csv_hough', default_value=f'/tmp/lane_detection_hough_parallel_{ts}.csv')
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/sensors/camera/color/image_rect_color')
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/simulation/odom_ground_truth')

    # ---- Nodos de detección ----
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
    hough_node = Node(
        package='lane_detection_hough',
        executable='lane_detection_hough_node',
        name='lane_detection_hough',
        output='screen',
        parameters=[
            hough_params,
            {'input_image_topic': LaunchConfiguration('input_topic')},
        ],
        emulate_tty=True,
    )

    # ---- Collectors (uno por algoritmo) ----
    collector_fcm = Node(
        package='lane_detection_evaluation',
        executable='metrics_collector_node',
        name='metrics_collector_fcm',
        output='screen',
        parameters=[
            eval_params,
            {
                'algo_label': 'fcm',
                'output_csv_path': LaunchConfiguration('csv_fcm'),
                'detection_namespace': '/lane_detection_fcm',
                'odom_topic': LaunchConfiguration('odom_topic'),
                'log_fuzzy_metrics': True,
            },
        ],
        emulate_tty=True,
    )
    collector_sw = Node(
        package='lane_detection_evaluation',
        executable='metrics_collector_node',
        name='metrics_collector_sw',
        output='screen',
        parameters=[
            eval_params,
            {
                'algo_label': 'sliding_windows',
                'output_csv_path': LaunchConfiguration('csv_sw'),
                'detection_namespace': '/lane_detection_sliding_windows',
                'odom_topic': LaunchConfiguration('odom_topic'),
                'log_fuzzy_metrics': False,
            },
        ],
        emulate_tty=True,
    )
    collector_hough = Node(
        package='lane_detection_evaluation',
        executable='metrics_collector_node',
        name='metrics_collector_hough',
        output='screen',
        parameters=[
            eval_params,
            {
                'algo_label': 'hough',
                'output_csv_path': LaunchConfiguration('csv_hough'),
                'detection_namespace': '/lane_detection_hough',
                'odom_topic': LaunchConfiguration('odom_topic'),
                'log_fuzzy_metrics': False,
            },
        ],
        emulate_tty=True,
    )

    return LaunchDescription([
        csv_fcm_arg,
        csv_sw_arg,
        csv_hough_arg,
        input_topic_arg,
        odom_topic_arg,
        fcm_node,
        sw_node,
        hough_node,
        collector_fcm,
        collector_sw,
        collector_hough,
    ])
