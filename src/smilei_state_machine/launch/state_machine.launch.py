#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Obtener el directorio del paquete
    pkg_dir = get_package_share_directory('smilei_state_machine')
    
    # Ruta al archivo de parámetros
    params_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    
    # Declarar argumentos de lanzamiento
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_file,
        description='Ruta al archivo de parámetros del robot'
    )
    
    # Nodo de la máquina de estados
    state_machine_node = Node(
        package='smilei_state_machine',
        executable='state_machine',
        # name='state_machine_node', # El nombre ahora se establece dinámicamente en el script
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        params_file_arg,
        state_machine_node
    ])