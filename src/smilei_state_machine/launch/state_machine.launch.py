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
    with open(default_params_file, 'r') as f:
        config_data = yaml.safe_load(f)
    
    # Extraer el namespace
    launch_namespace = config_data.get('state_machine_node', {}).get('ros__parameters', {}).get('launch_namespace', 'operador')

    # Extraer los parámetros para pasarlos como un diccionario
    state_machine_params = config_data.get('state_machine_node', {}).get('ros__parameters', {})
    global_params = config_data.get('/**', {}).get('ros__parameters', {})
    
    # Combinar los parámetros. Los específicos del nodo sobreescriben los globales.
    combined_params = {**global_params, **state_machine_params}

    # --- Declarar Argumentos de Lanzamiento ---
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value=launch_namespace,
        description='Namespace to apply to the nodes.'
    )

    # --- Configuración de Nodo y Namespace ---
    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            Node(
                package='smilei_state_machine',
                executable='state_machine',
                name='state_machine_node',
                # Pasar los parámetros como un diccionario combinado para asegurar la carga
                parameters=[combined_params],
                output='screen',
                emulate_tty=True
            )
        ]
    )
    
    return LaunchDescription([
        namespace_arg,
        namespaced_group
    ])
