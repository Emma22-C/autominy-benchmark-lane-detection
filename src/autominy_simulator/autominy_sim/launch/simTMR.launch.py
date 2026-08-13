"""simTMR.launch.py — Simulación AutoMiny en pistas TMR.

Variante de sim.launch.py adaptada para usar las pistas del Torneo Mexicano de
Robótica (TMR) en lugar del Berlin Virtual Laboratory.

Diferencias clave respecto a sim.launch.py:
- Carga TMR2021.world (configurable) en lugar de empty.world + lab URDF
- No spawnea el laboratorio (lab_model) porque la pista TMR ya incluye su escenario
- Coordenadas de spawn del coche calibradas para la pista TMR (4.577, 4.08, yaw=3.014)
- world_name puede cambiarse vía CLI para usar otras pistas (TMR2022, etc.)

Uso:
    ros2 launch autominy_sim simTMR.launch.py
    ros2 launch autominy_sim simTMR.launch.py world_name:=TMR2022
    ros2 launch autominy_sim simTMR.launch.py world_name:=TMR2022_World3 gazebo_gui:=False
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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
        DeclareLaunchArgument('name',       default_value='model_car',
                              description='Nombre del robot'),
        DeclareLaunchArgument('model',      default_value='car',
                              description='Modelo del robot (xacro)'),
        DeclareLaunchArgument('world_name', default_value='TMR2021',
                              description='Nombre del .world a cargar (sin extensión). '
                                          'Opciones: TMR2021, TMR2021_1, TMR2022, '
                                          'TMR2022_World, TMR2022_World2, TMR2022_World3'),
        DeclareLaunchArgument('gazebo_gui', default_value='True',
                              description='Lanzar gzclient (GUI de Gazebo)'),
        DeclareLaunchArgument('car_x',      default_value='4.577',
                              description='Posición X inicial del coche'),
        DeclareLaunchArgument('car_y',      default_value='4.08',
                              description='Posición Y inicial del coche'),
        DeclareLaunchArgument('car_yaw',    default_value='3.014',
                              description='Yaw inicial del coche (rad)'),
    ]

    # === 1. Gazebo con pista TMR (gzserver + gzclient via worldTMR.launch) ===
    world_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            FindPackageShare('autominy_sim'), '/launch/worldTMR.launch'
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

    # === 3. spawn_car — spawna el coche tras 5s ===
    # NOTA: no usamos spawn_world.launch porque la pista TMR ya viene con
    # su escenario completo dentro del .world. Solo spawneamos el coche.
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

    # === 4. control.launch — sim_car_controller + joint_state_broadcaster ===
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

    return LaunchDescription([
        *declare_args,
        world_launch,
        description_launch,
        TimerAction(period=5.0, actions=[spawn_car_launch]),
        TimerAction(period=8.0, actions=[control_launch]),
    ])
