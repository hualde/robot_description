#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Nodo para el joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Nodo para el controlador de drive diferencial
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Esperar a que el servicio del controller_manager esté disponible
    wait_for_controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['--wait-for', '/controller_manager/list_controllers'],
        output='screen'
    )

    return LaunchDescription([
        # Esperar a que el controller_manager esté disponible
        wait_for_controller_manager,

        # Esperar 2 segundos después de que el controller_manager esté disponible
        TimerAction(
            period=2.0,
            actions=[
                # Iniciar el joint state broadcaster
                joint_state_broadcaster,
                
                # Esperar 2 segundos más antes de iniciar el controlador diferencial
                TimerAction(
                    period=2.0,
                    actions=[diff_drive_controller]
                )
            ]
        )
    ]) 