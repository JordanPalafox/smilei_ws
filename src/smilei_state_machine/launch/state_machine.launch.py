#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    """
    Launch file simplificado sin dependencia en robot_params.yaml
    Todos los parámetros ahora son controlados desde el dashboard
    """

    # --- Declarar Argumentos de Lanzamiento ---
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='operador',
        description='Namespace to apply to the nodes (operador or seguidor).'
    )

    # --- Configuración de Nodo y Namespace ---
    namespaced_group = GroupAction(
        actions=[
            PushRosNamespace(LaunchConfiguration('namespace')),
            Node(
                package='smilei_state_machine',
                executable='state_machine',
                name='state_machine_node',
                output='screen',
                emulate_tty=True
            )
        ]
    )

    return LaunchDescription([
        namespace_arg,
        namespaced_group
    ])
