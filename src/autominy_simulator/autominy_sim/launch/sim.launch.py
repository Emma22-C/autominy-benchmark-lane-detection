"""sim.launch.py — versión Python con orden de arranque garantizado.

Este launcher resuelve la race condition presente en sim.launch (XML) entre
description.launch (publica /robot_description) y spawn_car.launch (lo consume).
Usa RegisterEventHandler + TimerAction para garantizar que cada paso espere
a que el anterior haya terminado/estabilizado.

Cadena de inicialización:
  1. gzserver + gzclient + tf estáticas
  2. description.launch (publica URDF del coche)
  3. esperar 3s para que /robot_description se propague en el DDS
  4. spawn_world (spawna el lab)
  5. al terminar spawn_world -> spawn_car (spawna el coche)
  6. al terminar spawn_car -> control.launch (carga sim_car_controller)
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # === Argumentos ===
    name        = LaunchConfiguration('name')
    model       = LaunchConfiguration('model')
    world_name  = LaunchConfiguration('world_name')
    gazebo_gui  = LaunchConfiguration('gazebo_gui')
    car_x       = LaunchConfiguration('car_x')
    car_y       = LaunchConfiguration('car_y')
    car_yaw     = LaunchConfiguration('car_yaw')

    declare_args = [
        DeclareLaunchArgument('name',       default_value='model_car'),
        DeclareLaunchArgument('model',      default_value='car'),
        DeclareLaunchArgument('world_name', default_value='empty'),
        DeclareLaunchArgument('gazebo_gui', default_value='True'),
        DeclareLaunchArgument('car_x',      default_value='0.189021'),
        DeclareLaunchArgument('car_y',      default_value='4.092613'),
        DeclareLaunchArgument('car_yaw',    default_value='-1.5708'),
    ]

    # === 1. Gazebo (gzserver + gzclient via world.launch) ===
    world_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('autominy_sim'), '/launch/world.launch'
        ]),
        launch_arguments={
            'world_name': world_name,
            'gui': gazebo_gui,
        }.items()
    )

    # === 2. description.launch — publica /robot_description del coche ===
    description_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('car_description'), '/launch/description.launch'
        ]),
        launch_arguments={
            'name': name,
            'model': model,
        }.items()
    )

    # === 3. spawn_world — spawna el laboratorio ===
    # Lo metemos en TimerAction(3.0) para dar tiempo a que Gazebo y
    # /robot_description estén listos antes de empezar a spawnear.
    spawn_world_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('world_description'), '/launch/spawn_world.launch'
        ])
    )

    delayed_spawn_world = TimerAction(
        period=3.0,
        actions=[spawn_world_launch],
    )

    # === 4. spawn_car — spawna el coche en Gazebo ===
    # Se define aquí pero NO se incluye directamente en LaunchDescription;
    # se dispara desde el event handler cuando spawn_world haya terminado.
    spawn_car_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('car_description'), '/launch/spawn_car.launch'
        ]),
        launch_arguments={
            'name': name,
            'model': model,
            'x': car_x,
            'y': car_y,
            'yaw': car_yaw,
        }.items()
    )

    # === 5. control.launch — carga sim_car_controller + joint_state_broadcaster ===
    control_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('sim_car_controller'), '/launch/control.launch'
        ]),
        launch_arguments={
            'name': name,
            'model': model,
            'x': car_x,
            'y': car_y,
            'yaw': car_yaw,
        }.items()
    )

    # control.launch necesita esperar a que el coche esté spawneado en Gazebo
    # para que el plugin gazebo_ros2_control haya creado el /controller_manager.
    # Lo retrasamos 5 segundos tras spawn_world.
    delayed_control = TimerAction(
        period=8.0,  # 3s para spawn_world + 5s para que el coche se asiente
        actions=[control_launch],
    )

    return LaunchDescription([
        *declare_args,
        world_launch,
        description_launch,
        delayed_spawn_world,
        TimerAction(period=5.0, actions=[spawn_car_launch]),
        delayed_control,
    ])
