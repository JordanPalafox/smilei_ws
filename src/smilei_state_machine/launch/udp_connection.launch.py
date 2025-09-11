#!/usr/bin/env python3
"""
Launch file para el nodo de conexión UDP independiente
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Obtener la ruta del archivo de configuración
    package_dir = get_package_share_directory('smilei_state_machine')
    config_file = os.path.join(package_dir, 'config', 'robot_params.yaml')
    
    # Argumentos de lanzamiento
    machine_type_arg = DeclareLaunchArgument(
        'machine_type',
        default_value='A',
        description='Tipo de máquina: A (envía) o B (recibe)'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=config_file,
        description='Ruta al archivo de configuración YAML'
    )
    
    # Nodo UDP de conexión
    udp_connection_node = Node(
        package='smilei_state_machine',
        executable='udp_connection_node',
        name='udp_connection_node',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                # Override is_machine_a based on machine_type argument
                'remote_teleoperation.is_machine_a': True  # Se puede cambiar en runtime
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        machine_type_arg,
        config_file_arg,
        udp_connection_node
    ])