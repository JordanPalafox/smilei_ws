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
    
    # Verificar que el archivo existe, si no usar el del directorio src
    if not os.path.exists(config_file):
        config_file = '/home/jordan/smilei_ws/src/smilei_state_machine/config/robot_params.yaml'
    
    # Nodo UDP de conexión
    udp_connection_node = Node(
        package='smilei_state_machine',
        executable='udp_connection_node',
        name='state_machine_node',  # Usar el mismo nombre que el namespace del YAML
        parameters=[config_file],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        udp_connection_node
    ])