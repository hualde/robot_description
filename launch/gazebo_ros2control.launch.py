#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Localiza el directorio share de este paquete
    pkg_share = get_package_share_directory('robot_description')
    gazebo_ros_pkg_share = get_package_share_directory('gazebo_ros')

    # Construye la ruta al fichero XACRO
    urdf_xacro = os.path.join(pkg_share, 'urdf', 'gazebo_ros2control.urdf.xacro')

    # Comando para procesar XACRO en tiempo de lanzamiento
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_xacro
    ])
    robot_description = {'robot_description': robot_description_content}

    # Configuración de Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'pause': 'false',
            'world': os.path.join(gazebo_ros_pkg_share, 'worlds', 'empty.world')
        }.items()
    )

    # Nodo para spawnear el robot en Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'robot',
                  '-topic', '/robot_description',
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.1'],
        output='screen'
    )

    # Nodo que publica el árbol TF a partir del URDF y de /joint_states
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Nodo RViz (solo si rviz:=true)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', os.path.join(pkg_share, 'config', 'robot.rviz')
        ],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    # Cargar y activar los controladores
    controller_params_file = os.path.join(pkg_share, 'config', 'my_controllers.yaml')

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_params_file
        ],
        output='screen',
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    return LaunchDescription([
        # Argumento para activar/desactivar RViz
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Lanzar RViz si es true'
        ),

        # Lanzar Gazebo
        gazebo,

        # Publicar el estado del robot
        robot_state_publisher,

        # Esperar 2 segundos antes de spawnear el robot
        TimerAction(
            period=2.0,
            actions=[spawn_entity]
        ),

        # Lanzar RViz si está habilitado
        rviz,

        # Lanzar el controlador
        controller_manager,

        # Esperar 2 segundos antes de activar los controladores
        TimerAction(
            period=2.0,
            actions=[
                diff_drive_spawner,
                joint_broad_spawner
            ]
        ),
    ])
