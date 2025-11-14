from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
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
    xacro_file = os.path.join(pkg_share, 'urdf', 'dual_arm_parametric.urdf.xacro')
    robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')
    trajectory_config_file = os.path.join(pkg_share, 'config', 'trajectory_config_dual_arm.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'dual_arm.rviz')

    # Load robot parameters from YAML
    with open(robot_params_file, 'r') as f:
        config = yaml.safe_load(f)
    robot_params = config.get('robot_geometry', {})

    # Generate xacro arguments from parameters
    xacro_args = generate_xacro_args(robot_params)

    # Process xacro file with parameters
    robot_desc = Command(['xacro ', xacro_file, ' ', xacro_args])

    # Load trajectory configuration (if exists)
    trajectory_params = {}
    if os.path.exists(trajectory_config_file):
        with open(trajectory_config_file, 'r') as f:
            trajectory_config = yaml.safe_load(f)
        trajectory_params = trajectory_config.get('trajectory_params', {})

    interpolation_method = trajectory_params.get('interpolation_method', 'cubic')
    steps_per_segment = trajectory_params.get('steps_per_segment', 50)
    execution_rate = trajectory_params.get('execution_rate', 10.0)
    synchronized = trajectory_params.get('synchronized', True)

    # Get launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time')
    execute_on_startup = LaunchConfiguration('execute_on_startup')

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

    # Trajectory Executor Dual Arm Node
    trajectory_executor_node = Node(
        package='smilei_dual_arm_ik',
        executable='trajectory_executor_dual_arm',
        name='trajectory_executor_dual_arm',
        output='screen',
        parameters=[{
            'robot_params_file': robot_params_file,
            'trajectory_config_file': trajectory_config_file,
            'execute_on_startup': execute_on_startup,
            'interpolation_method': interpolation_method,
            'steps_per_segment': steps_per_segment,
            'execution_rate': execution_rate,
            'synchronized': synchronized
        }]
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
        trajectory_executor_node,
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
            'execute_on_startup',
            default_value='true',
            description='Start executing trajectory on startup'
        ),
        OpaqueFunction(function=launch_setup)
    ])
