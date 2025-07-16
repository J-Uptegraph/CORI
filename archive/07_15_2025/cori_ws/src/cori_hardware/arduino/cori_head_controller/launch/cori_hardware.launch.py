#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino connection'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='115200',
        description='Serial baudrate for Arduino communication'
    )
    
    auto_detect_arg = DeclareLaunchArgument(
        'auto_detect_port',
        default_value='true',
        description='Auto-detect Arduino serial port'
    )
    
    # Arduino bridge node
    arduino_bridge_node = Node(
        package='cori_hardware',
        executable='arduino_bridge',
        name='cori_arduino_bridge',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
            'auto_detect_port': LaunchConfiguration('auto_detect_port'),
        }],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        serial_port_arg,
        serial_baudrate_arg,
        auto_detect_arg,
        LogInfo(msg="🤖 Launching CORI Hardware Bridge"),
        arduino_bridge_node
    ])