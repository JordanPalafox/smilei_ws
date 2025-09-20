#!/usr/bin/env python3

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    # Obtener el directorio del paquete
    pkg_dir = get_package_share_directory('smilei_state_machine')
    
    # Ruta por defecto al archivo de parámetros
    default_params_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    
    # --- Cargar configuraciones desde YAML ---
    # Se abre y parsea el archivo YAML para extraer tanto los parámetros del nodo
    # como las configuraciones específicas del lanzamiento (ej. el namespace).
    with open(default_params_file, 'r') as f:
        config_data = yaml.safe_load(f)
    
    # Extraer parámetros para el nodo
    params = config_data['state_machine_node']['ros__parameters']
    
    # Extraer el namespace para el lanzamiento, con un valor de respaldo
    launch_namespace = config_data.get('launch_settings', {}).get('namespace', 'operador')

    # --- Declarar Argumentos de Lanzamiento ---
    
    # Argumento para el namespace, usando el valor del YAML como predeterminado.
    # Esto permite sobreescribirlo desde la línea de comandos si es necesario.
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=launch_namespace,
        description='Namespace to apply to the nodes. Default is read from the params file.'
    )

    # Argumento para el archivo de parámetros
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the robot parameters file'
    )

    # --- Configuración de Nodo y Namespace ---
    
    # Usar un GroupAction para aplicar el namespace al nodo
    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            Node(
                package='smilei_state_machine',
                executable='state_machine',
                name='state_machine_node',
                parameters=[params],
                output='screen',
                emulate_tty=True
            )
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        params_file_arg,
        namespaced_group
    ])
