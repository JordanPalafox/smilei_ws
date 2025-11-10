from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Input camera topic for skeleton detection'
    )

    skeleton_topic_arg = DeclareLaunchArgument(
        'skeleton_topic',
        default_value='/skeleton/image',
        description='Output topic for skeleton visualization'
    )

    # Camera publisher node (oak_publisher.py)
    oak_publisher_node = Node(
        package='',  # Not a package, it's a standalone script
        executable='python3',
        arguments=['/home/smilei/smilei_ws/oak_publisher.py', '/camera/image_raw'],
        name='oak_publisher',
        output='screen',
        emulate_tty=True
    )

    # Skeleton detector node
    skeleton_detector_node = Node(
        package='skeleton_detection',
        executable='skeleton_detector',
        name='skeleton_detector',
        output='screen',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'skeleton_topic': LaunchConfiguration('skeleton_topic'),
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        camera_topic_arg,
        skeleton_topic_arg,
        oak_publisher_node,
        skeleton_detector_node,
    ])
