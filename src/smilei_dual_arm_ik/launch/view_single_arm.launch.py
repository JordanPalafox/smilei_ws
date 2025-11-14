from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('smilei_dual_arm_ik').find('smilei_dual_arm_ik')
    xacro_file = os.path.join(pkg_share, 'urdf', 'single_arm.urdf.xacro')

    # Process xacro file to get URDF
    robot_desc = Command(['xacro ', xacro_file])

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time
        }]
    )

    # Joint State Publisher GUI Node (to control joints manually)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz Node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('smilei_dual_arm_ik'), 'rviz', 'single_arm.rviz']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        # If config file doesn't exist, RViz will start with default config
        on_exit=None
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
