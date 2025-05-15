from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Obtener la ruta del paquete
    pkg_dir = get_package_share_directory('smilei_state_machine')
    
    # Archivo de configuración con los parámetros del robot
    config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    
    # Argumentos de lanzamiento
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([
        # Declarar argumentos
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Usar tiempo de simulación'
        ),
        
        # Nodo del servidor de motores Westwood
        Node(
            package='westwood_motor_control_sdk_wrapper',
            executable='westwood_motor_server.py',
            name='westwood_motor_server',
            parameters=[
                {'baudrate': 8000000},
                {'motor_ids': [1, 2, 3, 4, 5, 6, 7, 8]},
                {'debug': False}
            ],
            output='screen'
        ),
        
        # Nodo de la máquina de estados de SMILEi
        Node(
            package='smilei_state_machine',
            executable='smilei_state_machine_node',
            name='smilei_state_machine',
            parameters=[config_file],
            output='screen'
        )
    ]) 