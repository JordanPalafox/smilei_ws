#!/usr/bin/env python3

"""
Archivo de lanzamiento para el servidor de motores Westwood.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    debug = LaunchConfiguration('debug', default='false')
    baudrate = LaunchConfiguration('baudrate', default='8000000')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_debug_cmd = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug logs'
    )
    
    declare_baudrate_cmd = DeclareLaunchArgument(
        'baudrate',
        default_value='8000000',
        description='Baudrate for serial communication'
    )
    
    # Westwood motor server node
    westwood_motor_server_cmd = Node(
        package='westwood_motor_control_sdk_wrapper',
        executable='westwood_motor_server.py',
        name='westwood_motor_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'debug': debug,
            'baudrate': baudrate,
            'motor_ids': [1, 2, 3, 4, 5, 6, 7, 8]
        }]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_debug_cmd)
    ld.add_action(declare_baudrate_cmd)
    
    # Add node
    ld.add_action(westwood_motor_server_cmd)
    
    return ld 