from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
import os
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
    xacro_file = os.path.join(pkg_share, 'urdf', 'single_arm_parametric.urdf.xacro')
    robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'single_arm.rviz')

    # Load robot parameters from YAML
    with open(robot_params_file, 'r') as f:
        config = yaml.safe_load(f)
    robot_params = config.get('robot_geometry', {})

    # Generate xacro arguments from parameters
    xacro_args = generate_xacro_args(robot_params)

    # Process xacro file with parameters
    robot_desc = Command(['xacro ', xacro_file, ' ', xacro_args])

    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_interactive_marker = LaunchConfiguration('launch_interactive_marker')
    step_size = LaunchConfiguration('step_size')
    initial_x = LaunchConfiguration('initial_x')
    initial_y = LaunchConfiguration('initial_y')
    initial_z = LaunchConfiguration('initial_z')
    enable_ik_validation = LaunchConfiguration('enable_ik_validation')

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

    # Joint State Publisher GUI Node (optional, to see robot)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Interactive Marker Node (optional, usually launched via tmux)
    # Set launch_interactive_marker:=true to launch it from here
    interactive_marker_node = Node(
        package='smilei_dual_arm_ik',
        executable='interactive_marker_node',
        name='interactive_marker_node',
        output='screen',
        parameters=[{
            'step_size': step_size,
            'initial_x': initial_x,
            'initial_y': initial_y,
            'initial_z': initial_z,
            'enable_ik_validation': enable_ik_validation
        }],
        condition=IfCondition(launch_interactive_marker)
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
        interactive_marker_node,
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'launch_interactive_marker',
            default_value='false',
            description='Launch interactive marker node (set to false if using tmux)'
        ),
        DeclareLaunchArgument(
            'step_size',
            default_value='0.01',
            description='Step size for marker movement (meters)'
        ),
        DeclareLaunchArgument(
            'initial_x',
            default_value='0.10',
            description='Initial X position (meters)'
        ),
        DeclareLaunchArgument(
            'initial_y',
            default_value='0.00',
            description='Initial Y position (meters)'
        ),
        DeclareLaunchArgument(
            'initial_z',
            default_value='0.16',
            description='Initial Z position (meters)'
        ),
        DeclareLaunchArgument(
            'enable_ik_validation',
            default_value='true',
            description='Enable IK validation and color markers (green=reachable, red=unreachable)'
        ),
        OpaqueFunction(function=launch_setup)
    ])
