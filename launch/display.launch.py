#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Localiza el directorio share de este paquete
    pkg_share = FindPackageShare(package='robot_description')

    # Construye la ruta al fichero XACRO
    urdf_xacro = PathJoinSubstitution([
        pkg_share, 'urdf', 'differential_robot.urdf.xacro'
    ])

    # Comando para procesar XACRO en tiempo de lanzamiento
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', urdf_xacro
    ])
    robot_description = {'robot_description': robot_description_content}

    return LaunchDescription([
        # Argumento para activar/desactivar RViz
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Lanzar RViz si es true'
        ),

        # GUI para manipular joints y publicar /joint_states
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[robot_description]
        ),

        # Nodo que publica el árbol TF a partir del URDF y de /joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        # Nodo RViz (solo si rviz:=true)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', PathJoinSubstitution([pkg_share, 'launch', 'view_robot.rviz'])
            ],
            condition=IfCondition(LaunchConfiguration('rviz'))
        ),
    ])
