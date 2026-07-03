#!/usr/bin/env python3
"""
Launch file para probar SOLO el nodo de visión.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('eye_tracker_bringup')
    
    params_file = PathJoinSubstitution([
        bringup_share, 'config', 'arm_params.yaml'
    ])
    
    vision_node = Node(
        package='eye_tracker_vision',
        executable='vision_node',
        name='eye_tracker_vision',
        parameters=[params_file],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        vision_node,
    ])
