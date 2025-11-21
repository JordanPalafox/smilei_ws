#!/usr/bin/env python3
"""
Launch file for hardware joint state visualization

This launches:
1. robot_state_publisher (URDF visualization)
2. hardware_joint_state_publisher (reads motor positions from hardware)
3. RViz (visualization)

The URDF in RViz will update in real-time as you physically move the motors!
"""

import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_xacro_args(params):
    """Convert parameters dictionary to xacro arguments string"""
    args = []
    for key, value in params.items():
        args.append(f"{key}:={value}")
    return ' '.join(args)


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('smilei_dual_arm_ik')

    # Paths to config files
    xacro_file = os.path.join(pkg_share, 'urdf', 'dual_arm_parametric.urdf.xacro')
    robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'dual_arm.rviz')

    # Load robot parameters from YAML
    with open(robot_params_file, 'r') as f:
        config = yaml.safe_load(f)
    robot_params = config.get('robot_geometry', {})

    # Generate xacro arguments from parameters
    xacro_args = generate_xacro_args(robot_params)

    # Process xacro file with parameters
    robot_description = Command(['xacro ', xacro_file, ' ', xacro_args])

    # Node 1: robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 50.0
        }]
    )

    # Node 2: Hardware Joint State Publisher
    hardware_joint_state_publisher = Node(
        package='smilei_dual_arm_ik',
        executable='hardware_joint_state_publisher.py',
        name='hardware_joint_state_publisher',
        output='screen',
        parameters=[{
            'hardware_manager.usb_ports': ['/dev/ttyUSB0', '/dev/ttyUSB1'],
            'hardware_manager.baudrate': 8000000,
            'hardware_manager.auto_detect': True,
            'hardware_manager.debug': False,
            'motors.right_arm_ids': [1, 2, 3, 4],
            'motors.left_arm_ids': [5, 6, 7, 8],
            'publish_frequency': 50.0
        }]
    )

    # Node 3: RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        hardware_joint_state_publisher,
        rviz
    ])
