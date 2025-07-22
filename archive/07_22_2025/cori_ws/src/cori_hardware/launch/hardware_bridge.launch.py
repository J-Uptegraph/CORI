#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cori_hardware',
            executable='hardware_bridge',
            name='cori_hardware_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }]
        ),
    ])