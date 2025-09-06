#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='westwood_motor_control_sdk_wrapper',
            executable='westwood_motor_server.py',
            name='westwood_motor_server',
            output='screen',
            parameters=[{
                'usb_ports': ['/dev/ttyUSB0', '/dev/ttyUSB1'],
                'baudrate': 8000000,
                'motor_ids_usb0': [1, 2, 3, 4],  # IDs locales en USB0
                'motor_ids_usb1': [5, 6, 7, 8],  # IDs globales para USB1 (locales 1-4)
                'debug': False
            }]
        )
    ])