#!/usr/bin/env python3
"""
Forward Kinematics for Dual Arm Robot

Calculates FK for both left and right arms independently
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class ForwardKinematicsDualArm(Node):
    """
    Forward kinematics node for dual arm robot
    Calculates FK for both left and right arms
    """

    def __init__(self):
        super().__init__('forward_kinematics_dual_arm')

        # Declare parameter for config file path
        self.declare_parameter('robot_params_file', '')

        # Load robot parameters from YAML
        params_file = self.get_parameter('robot_params_file').value
        if not params_file:
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')

        self.get_logger().info(f'Loading robot parameters from: {params_file}')
        self.load_robot_parameters(params_file)

        self.PI = np.pi

        # Joint states for both arms
        self.right_joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.left_joint_angles = [0.0, 0.0, 0.0, 0.0]

        self.right_joint_names = ['right_joint_0', 'right_joint_1', 'right_joint_2', 'right_joint_3']
        self.left_joint_names = ['left_joint_0', 'left_joint_1', 'left_joint_2', 'left_joint_3']

        # Subscriber to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publishers for calculated poses
        self.right_pose_pub = self.create_publisher(
            PoseStamped,
            '/forward_kinematics/right_pose',
            10
        )

        self.left_pose_pub = self.create_publisher(
            PoseStamped,
            '/forward_kinematics/left_pose',
            10
        )

        # Timer for publishing
        publish_rate = 10.0
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_forward_kinematics)

        self.get_logger().info('Forward Kinematics Dual Arm Node initialized')
        self.get_logger().info(f'Robot parameters loaded: neck_height={self.neck_height}')

    def load_robot_parameters(self, yaml_file):
        """Load robot geometric parameters from YAML file"""
        try:
            with open(yaml_file, 'r') as f:
                config = yaml.safe_load(f)

            params = config.get('robot_geometry', {})

            # Load all geometric parameters
            self.neck_height = params.get('neck_height', 0.34)
            self.base_height = params.get('base_height', 0.34)
            self.link0_length = params.get('link0_length', 0.10)
            self.link0_width = params.get('link0_width', 0.0)
            self.link1_length = params.get('link1_length', 0.15)
            self.link1_width = params.get('link1_width', -0.30)
            self.link2_length = params.get('link2_length', 0.25)
            self.link2_width = params.get('link2_width', 0.20)
            self.link3_length = params.get('link3_length', -0.20)
            self.link4_length = params.get('link4_length', -0.08)
            self.ee_offset = params.get('ee_offset', 0.0)

            self.get_logger().info(f'Successfully loaded {len(params)} parameters from YAML')

        except Exception as e:
            self.get_logger().error(f'Failed to load parameters: {e}')
            # Set defaults
            self.neck_height = 0.34
            self.base_height = 0.34
            self.link0_length = 0.10
            self.link0_width = 0.0
            self.link1_length = 0.15
            self.link1_width = -0.30
            self.link2_length = 0.25
            self.link2_width = 0.20
            self.link3_length = -0.20
            self.link4_length = -0.08
            self.ee_offset = 0.0

    def joint_state_callback(self, msg: JointState):
        """Update joint angles from joint_state topic"""
        # Update right arm joints
        for i, joint_name in enumerate(self.right_joint_names):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.right_joint_angles[i] = msg.position[idx]

        # Update left arm joints
        for i, joint_name in enumerate(self.left_joint_names):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.left_joint_angles[i] = msg.position[idx]

    def create_transform(self, xyz, rpy):
        """Create 4x4 homogeneous transformation matrix"""
        T = np.eye(4)
        T[:3, 3] = xyz
        r = Rotation.from_euler('xyz', rpy)
        T[:3, :3] = r.as_matrix()
        return T

    def create_rotation_z(self, angle):
        """Create rotation matrix around Z axis"""
        c = np.cos(angle)
        s = np.sin(angle)
        R = np.array([
            [c, -s, 0],
            [s,  c, 0],
            [0,  0, 1]
        ])
        T = np.eye(4)
        T[:3, :3] = R
        return T

    def calculate_forward_kinematics_right_arm(self, joint_angles):
        """
        Calculate forward kinematics for RIGHT arm

        Args:
            joint_angles: Array of 4 joint angles

        Returns:
            4x4 transformation matrix from base_link to right_end_effector
        """
        # 1. base_link → neck_base (fixed)
        T = self.create_transform([0, 0, self.neck_height], [0, 0, 0])

        # 2. neck_base → right_link_1 (right_joint_0)
        # URDF: origin xyz="${link0_length} 0 0" rpy="${PI_2} 0 ${PI_2}"
        T_fixed_0 = self.create_transform(
            [self.link0_length, 0, 0],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_0 = self.create_rotation_z(joint_angles[0])
        T = T @ T_fixed_0 @ T_joint_0

        # 3. right_link_1 → right_link_2 (right_joint_1)
        # URDF: origin xyz="${link1_width} 0 ${link1_length}" rpy="${PI_2} 0 ${PI_2}"
        T_fixed_1 = self.create_transform(
            [self.link1_width, 0, self.link1_length],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_1 = self.create_rotation_z(joint_angles[1])
        T = T @ T_fixed_1 @ T_joint_1

        # 4. right_link_2 → right_link_3 (right_joint_2)
        # URDF: origin xyz="0 ${link2_length} ${link2_width}" rpy="${-PI_2} ${-PI_2} 0"
        T_fixed_2 = self.create_transform(
            [0, self.link2_length, self.link2_width],
            [-self.PI/2, -self.PI/2, 0]
        )
        T_joint_2 = self.create_rotation_z(joint_angles[2])
        T = T @ T_fixed_2 @ T_joint_2

        # 5. right_link_3 → right_link_4 (right_joint_3)
        # URDF: origin xyz="0 0 ${link3_length}" rpy="${PI_2} 0 ${PI_2}"
        T_fixed_3 = self.create_transform(
            [0, 0, self.link3_length],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_3 = self.create_rotation_z(joint_angles[3])
        T = T @ T_fixed_3 @ T_joint_3

        # 6. right_link_4 → right_end_effector (fixed)
        # URDF: origin xyz="0 ${link4_length + ee_offset} 0" rpy="${PI_2} 0 ${PI_2}"
        T_ee = self.create_transform(
            [0, self.link4_length + self.ee_offset, 0],
            [self.PI/2, 0, self.PI/2]
        )
        T = T @ T_ee

        return T

    def calculate_forward_kinematics_left_arm(self, joint_angles):
        """
        Calculate forward kinematics for LEFT arm (mirrored)

        Args:
            joint_angles: Array of 4 joint angles

        Returns:
            4x4 transformation matrix from base_link to left_end_effector
        """
        # 1. base_link → neck_base (fixed)
        T = self.create_transform([0, 0, self.neck_height], [0, 0, 0])

        # 2. neck_base → left_link_1 (left_joint_0)
        # URDF: origin xyz="${-link0_length} 0 0" rpy="${PI_2} ${-PI_2} ${-PI_2}"
        T_fixed_0 = self.create_transform(
            [-self.link0_length, 0, 0],
            [self.PI/2, -self.PI/2, -self.PI/2]
        )
        T_joint_0 = self.create_rotation_z(joint_angles[0])
        T = T @ T_fixed_0 @ T_joint_0

        # 3. left_link_1 → left_link_2 (left_joint_1)
        # URDF: origin xyz="0 ${link1_width} ${link1_length}" rpy="${-PI_2} ${-PI_2} 0"
        T_fixed_1 = self.create_transform(
            [0, self.link1_width, self.link1_length],
            [-self.PI/2, -self.PI/2, 0]
        )
        T_joint_1 = self.create_rotation_z(joint_angles[1])
        T = T @ T_fixed_1 @ T_joint_1

        # 4. left_link_2 → left_link_3 (left_joint_2)
        # URDF: origin xyz="${link2_length} 0 ${link2_width}" rpy="${PI_2} 0 ${PI_2}"
        T_fixed_2 = self.create_transform(
            [self.link2_length, 0, self.link2_width],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_2 = self.create_rotation_z(joint_angles[2])
        T = T @ T_fixed_2 @ T_joint_2

        # 5. left_link_3 → left_link_4 (left_joint_3)
        # URDF: origin xyz="0 0 ${link3_length}" rpy="${-PI_2} ${-PI_2} 0"
        T_fixed_3 = self.create_transform(
            [0, 0, self.link3_length],
            [-self.PI/2, -self.PI/2, 0]
        )
        T_joint_3 = self.create_rotation_z(joint_angles[3])
        T = T @ T_fixed_3 @ T_joint_3

        # 6. left_link_4 → left_end_effector (fixed)
        # URDF: origin xyz="${link4_length + ee_offset} 0 0" rpy="0 ${-PI_2} ${-PI_2}"
        T_ee = self.create_transform(
            [self.link4_length + self.ee_offset, 0, 0],
            [0, -self.PI/2, -self.PI/2]
        )
        T = T @ T_ee

        return T

    def matrix_to_pose_stamped(self, T, frame_id='base_link'):
        """Convert transformation matrix to PoseStamped message"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id

        # Extract position
        pose_msg.pose.position.x = T[0, 3]
        pose_msg.pose.position.y = T[1, 3]
        pose_msg.pose.position.z = T[2, 3]

        # Extract rotation and convert to quaternion
        rotation_matrix = T[:3, :3]
        r = Rotation.from_matrix(rotation_matrix)
        quat = r.as_quat()  # Returns [x, y, z, w]

        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        return pose_msg

    def publish_forward_kinematics(self):
        """Calculate and publish forward kinematics for both arms"""
        # Calculate FK for right arm
        T_right = self.calculate_forward_kinematics_right_arm(self.right_joint_angles)
        right_pose = self.matrix_to_pose_stamped(T_right)
        self.right_pose_pub.publish(right_pose)

        # Calculate FK for left arm
        T_left = self.calculate_forward_kinematics_left_arm(self.left_joint_angles)
        left_pose = self.matrix_to_pose_stamped(T_left)
        self.left_pose_pub.publish(left_pose)

        # Log periodically (every 100 calls = 10 seconds at 10Hz)
        if not hasattr(self, '_call_count'):
            self._call_count = 0
        self._call_count += 1

        if self._call_count % 100 == 0:
            self.get_logger().info('='*60)
            self.get_logger().info(f'Right Arm: [{right_pose.pose.position.x:.4f}, '
                                   f'{right_pose.pose.position.y:.4f}, '
                                   f'{right_pose.pose.position.z:.4f}]')
            self.get_logger().info(f'Left Arm:  [{left_pose.pose.position.x:.4f}, '
                                   f'{left_pose.pose.position.y:.4f}, '
                                   f'{left_pose.pose.position.z:.4f}]')
            self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsDualArm()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
