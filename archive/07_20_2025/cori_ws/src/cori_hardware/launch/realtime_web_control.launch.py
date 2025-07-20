#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    # Real-time web control node
    realtime_web_control_node = Node(
        package='cori_hardware',
        executable='realtime_web_control',
        name='cori_realtime_web_control',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # ESP32 hardware bridge node for real servo control
    hardware_bridge_node = Node(
        package='cori_hardware',
        executable='hardware_bridge',
        name='cori_hardware_bridge',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )
    
    # HTTP server to serve the web interface (accessible from internet)
    # Use absolute path to source directory for development
    package_dir = '/home/juptegraph/Workspaces/Robotics/Projects/CORI/cori_ws/src/cori_hardware/cori_hardware'
    web_server_process = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8091', '--bind', '0.0.0.0'],
        cwd=package_dir,
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        realtime_web_control_node,
        hardware_bridge_node,
        web_server_process
    ])