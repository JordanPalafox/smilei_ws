#!/usr/bin/env python3
"""
Launch file for Gesture Executor Action Server

Launches the gesture executor node along with visualization tools
"""

import launch
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def generate_xacro_args(params):
    """Convert parameters dictionary to xacro arguments string"""
    args = []
    for key, value in params.items():
        args.append(f"{key}:={value}")
    return ' '.join(args)


def launch_setup(context, *args, **kwargs):
    """Setup function to load parameters and create nodes"""

    # Get package directory
    pkg_share = FindPackageShare('smilei_dual_arm_ik').find('smilei_dual_arm_ik')

    # Paths to files
    xacro_file = os.path.join(pkg_share, 'urdf', 'dual_arm_parametric.urdf.xacro')
    robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')
    gestures_directory = os.path.join(pkg_share, 'config', 'gestures')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'dual_arm.rviz')

    # Load robot parameters from YAML
    with open(robot_params_file, 'r') as f:
        config = yaml.safe_load(f)
    robot_params = config.get('robot_geometry', {})

    # Generate xacro arguments from parameters
    xacro_args = generate_xacro_args(robot_params)

    # Process xacro file with parameters
    robot_desc = Command(['xacro ', xacro_file, ' ', xacro_args])

    # Get launch configuration
    use_rviz = LaunchConfiguration('use_rviz')

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    # Gesture Executor Action Server Node
    gesture_executor_node = Node(
        package='smilei_dual_arm_ik',
        executable='gesture_executor_dual_arm.py',
        name='gesture_executor_dual_arm',
        output='screen',
        parameters=[{
            'gestures_directory': gestures_directory,
            'robot_params_file': robot_params_file,
        }]
    )

    # RViz Node (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=launch.conditions.IfCondition(use_rviz)
    )

    return [
        robot_state_publisher_node,
        gesture_executor_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        OpaqueFunction(function=launch_setup)
    ])
