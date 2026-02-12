from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch file for SMILEi Dashboard"""

    dashboard_node = Node(
        package='dashboard',
        executable='dashboard_node',
        name='dashboard_node',
        output='screen',
        parameters=[],
        emulate_tty=True,
    )

    return LaunchDescription([
        dashboard_node,
    ])
