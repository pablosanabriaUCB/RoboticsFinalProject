#!/usr/bin/env python3
"""
Launch file para el sistema completo Eye Tracker Arm.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('eye_tracker_bringup')
    
    agent_port = DeclareLaunchArgument(
        'agent_port', default_value='8888',
        description='Puerto UDP del agente micro-ROS'
    )
    
    params_file = PathJoinSubstitution([
        bringup_share, 'config', 'arm_params.yaml'
    ])
    
    # --- Nodo de Visión ---
    vision_node = Node(
        package='eye_tracker_vision',
        executable='vision_node',
        name='eye_tracker_vision',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )
    
    # --- Nodo de Cinemática Inversa ---
    ik_node = Node(
        package='eye_tracker_ik',
        executable='ik_node',
        name='eye_tracker_ik',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )
    
    # --- micro-ROS Agent (Docker) ---
    micro_ros_agent = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'sudo', 'docker', 'run', '-it', '--rm', '--net=host',
                    'microros/micro-ros-agent:humble',
                    'udp4', '--port', LaunchConfiguration('agent_port'), '-v6'
                ],
                output='screen',
                name='micro_ros_agent',
            )
        ]
    )
    
    return LaunchDescription([
        agent_port,
        vision_node,
        ik_node,
        micro_ros_agent,
    ])
