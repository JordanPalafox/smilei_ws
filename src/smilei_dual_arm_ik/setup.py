from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'smilei_dual_arm_ik'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'gestures'), glob('config/gestures/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rovestrada',
    maintainer_email='rovestrada@gmail.com',
    description='Dual arm inverse kinematics and gesture execution',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'forward_kinematics_node = smilei_dual_arm_ik.forward_kinematics_node:main',
            'forward_kinematics_urdf_based = smilei_dual_arm_ik.forward_kinematics_urdf_based:main',
            'forward_kinematics_dual_arm = smilei_dual_arm_ik.forward_kinematics_dual_arm:main',
            'trajectory_executor_node = smilei_dual_arm_ik.trajectory_executor_node:main',
            'trajectory_executor_dual_arm = smilei_dual_arm_ik.trajectory_executor_dual_arm:main',
            'gesture_executor_dual_arm = smilei_dual_arm_ik.gesture_executor_dual_arm:main',
            'interactive_marker_node = smilei_dual_arm_ik.interactive_marker_node:main',
            'marker_control_node = smilei_dual_arm_ik.marker_control_node:main',
            'hardware_joint_state_publisher = smilei_dual_arm_ik.hardware_joint_state_publisher:main',
            'test_node = smilei_dual_arm_ik.test_node:main',
        ],
    },
)
