from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def load_yaml_params(yaml_file):
    """Load parameters from YAML file"""
    with open(yaml_file, 'r') as f:
        config = yaml.safe_load(f)
    return config.get('robot_geometry', {})


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
    xacro_file = os.path.join(pkg_share, 'urdf', 'single_arm_parametric.urdf.xacro')
    params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'single_arm.rviz')

    # Load robot parameters from YAML
    robot_params = load_yaml_params(params_file)

    # Generate xacro arguments from parameters
    xacro_args = generate_xacro_args(robot_params)

    # Process xacro file with parameters
    robot_desc = Command(['xacro ', xacro_file, ' ', xacro_args])

    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    # Joint State Publisher GUI Node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Forward Kinematics Node (URDF-based with YAML parameters)
    forward_kinematics_node = Node(
        package='smilei_dual_arm_ik',
        executable='forward_kinematics_urdf_based.py',
        name='forward_kinematics_urdf_based',
        output='screen',
        parameters=[{'robot_params_file': params_file}]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        on_exit=None
    )

    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        forward_kinematics_node,
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        OpaqueFunction(function=launch_setup)
    ])
