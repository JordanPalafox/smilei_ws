#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener el directorio del paquete
    pkg_dir = get_package_share_directory('smilei_state_machine')
    params_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')

    # --- Start of new logic ---
    namespace = ''
    try:
        with open(params_file, 'r') as file:
            config_yaml = yaml.safe_load(file)
            if 'state_machine' in config_yaml and 'ros__parameters' in config_yaml['state_machine']:
                if '__ns' in config_yaml['state_machine']['ros__parameters']:
                    namespace = config_yaml['state_machine']['ros__parameters']['__ns']
    except Exception as e:
        # Log the error, but continue with an empty namespace
        print(f"Error reading namespace from YAML: {e}")
    # --- End of new logic ---
    
    # Nodo de la máquina de estados
    state_machine_node = Node(
        package='smilei_state_machine',
        executable='state_machine',
        name='state_machine',  # Nombre base del nodo
        namespace=namespace, # Use the namespace from YAML
        parameters=[params_file],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        state_machine_node
    ])