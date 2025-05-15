#!/usr/bin/env python3

"""
Archivo de lanzamiento para el nodo de la máquina de estados SMILEi.
Inicia el servidor de motores y la máquina de estados.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    GroupAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths
    westwood_motor_launch_dir = os.path.join(
        get_package_share_directory('westwood_motor_control_sdk_wrapper'),
        'launch'
    )
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Include Westwood motor server launch
    motor_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([westwood_motor_launch_dir, '/westwood_motor_server.launch.py']),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # SMILEi state machine node
    smilei_state_machine_cmd = Node(
        package='smilei_state_machine',
        executable='smilei_state_machine_node',
        name='smilei_state_machine',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add launches
    ld.add_action(motor_server_launch)
    ld.add_action(smilei_state_machine_cmd)
    
    return ld 