#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('westwood_motor_control_sdk_wrapper'),
            'config',
            'server_params.yaml'
        ]),
        description='Path to the server configuration file'
    )
    
    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode for the motor server'
    )
    
    auto_detect_arg = DeclareLaunchArgument(
        'auto_detect',
        default_value='true',
        description='Enable automatic motor detection'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    debug = LaunchConfiguration('debug')
    auto_detect = LaunchConfiguration('auto_detect')
    
    # Create the motor server node
    westwood_motor_server_node = Node(
        package='westwood_motor_control_sdk_wrapper',
        executable='westwood_motor_server.py',
        name='westwood_motor_server',
        output='screen',
        parameters=[
            config_file,
            {
                'debug': debug,
                'auto_detect': auto_detect,
            }
        ],
        # Add respawn for automatic restart if the node fails
        respawn=True,
        respawn_delay=2.0,
        # Set process priority (requires launch to be run with appropriate permissions)
        # Additional parameters for better real-time performance
        emulate_tty=True,
    )
    
    # Log info about the launch
    log_info = LogInfo(
        msg=[
            'Starting Westwood Motor Server with config: ', config_file,
            ', debug: ', debug,
            ', auto_detect: ', auto_detect
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        debug_arg,
        auto_detect_arg,
        log_info,
        westwood_motor_server_node,
    ])