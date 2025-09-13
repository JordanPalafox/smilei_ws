#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Argumento para el archivo de configuración
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('westwood_motor_control_sdk_wrapper'),
            'config',
            'server_params.yaml'
        ]),
        description='Path to the server configuration file'
    )

    # --- Start of new logic ---
    # Get the path to the config file
    pkg_share = get_package_share_directory('westwood_motor_control_sdk_wrapper')
    config_path = os.path.join(pkg_share, 'config', 'server_params.yaml')

    namespace = ''
    try:
        with open(config_path, 'r') as file:
            config_yaml = yaml.safe_load(file)
            if 'westwood_motor_server' in config_yaml and 'ros__parameters' in config_yaml['westwood_motor_server']:
                if '__ns' in config_yaml['westwood_motor_server']['ros__parameters']:
                    namespace = config_yaml['westwood_motor_server']['ros__parameters']['__ns']
    except Exception as e:
        # Log the error, but continue with an empty namespace
        print(f"Error reading namespace from YAML: {e}")
    # --- End of new logic ---

    # Create the motor server node
    westwood_motor_server_node = Node(
        package='westwood_motor_control_sdk_wrapper',
        executable='westwood_motor_server.py',
        name='westwood_motor_server', # Nombre base del nodo
        namespace=namespace, # Use the namespace from YAML
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        respawn=True,
        respawn_delay=2.0,
        emulate_tty=True,
    )
    
    return LaunchDescription([
        config_file_arg,
        westwood_motor_server_node,
    ])