#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from tf2_ros import TransformListener, Buffer
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import os
from ament_index_python.packages import get_package_share_directory


class ForwardKinematicsURDFBased(Node):
    """
    Forward kinematics node that calculates FK using URDF-based transformations
    instead of DH parameters. This directly implements the URDF kinematics.

    Robot parameters are loaded from a YAML configuration file.
    """

    def __init__(self):
        super().__init__('forward_kinematics_urdf_based')

        # Declare parameter for config file path
        self.declare_parameter('robot_params_file', '')

        # Load robot parameters from YAML
        params_file = self.get_parameter('robot_params_file').value
        if not params_file:
            # Use default path
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')

        self.get_logger().info(f'Loading robot parameters from: {params_file}')
        self.load_robot_parameters(params_file)

        self.PI = np.pi

        # Joint state
        self.joint_angles = [0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint_0', 'joint_1', 'joint_2', 'joint_3']

        # Subscriber to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for calculated pose
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/forward_kinematics/pose',
            10
        )

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for publishing
        publish_rate = 10.0
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_forward_kinematics)

        self.get_logger().info('Forward Kinematics URDF-Based Node initialized')
        self.get_logger().info('Using direct URDF transformations (not DH)')
        self.get_logger().info(f'Robot parameters loaded: neck_height={self.neck_height}, base_height={self.base_height}')

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
            self.get_logger().error(f'Failed to load parameters from {yaml_file}: {e}')
            self.get_logger().warn('Using default parameter values')
            # Set default values
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
        for i, joint_name in enumerate(self.joint_names):
            if joint_name in msg.name:
                idx = msg.name.index(joint_name)
                self.joint_angles[i] = msg.position[idx]

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

    def calculate_forward_kinematics(self):
        """
        Calculate forward kinematics using URDF transformations directly

        Returns:
            4x4 transformation matrix from base_link to end_effector
        """
        # 1. base_link → neck_base (fixed)
        T = self.create_transform([0, 0, self.neck_height], [0, 0, 0])

        # 2. neck_base → link_1 (joint_0 with rotation)
        # URDF: origin xyz="0 0 ${base_height}" rpy="${PI_2} 0 ${PI_2}"
        # This is the fixed transform, then add joint rotation
        T_fixed_0 = self.create_transform(
            [self.link0_length, 0, 0],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_0 = self.create_rotation_z(self.joint_angles[0])  # Joint rotation around Z
        T = T @ T_fixed_0 @ T_joint_0

        # 3. link_1 → link_2 (joint_1 with rotation)
        # URDF: origin xyz="${link1_width} 0 ${link1_length}" rpy="${PI_2} 0 ${PI_2}"
        T_fixed_1 = self.create_transform(
            [self.link1_width, 0, self.link1_length],
            [self.PI/2, 0, self.PI/2]
        )
        T_joint_1 = self.create_rotation_z(self.joint_angles[1])
        T = T @ T_fixed_1 @ T_joint_1

        # 4. link_2 → link_3 (joint_2 with rotation)
        # URDF: origin xyz="0 ${link2_length} ${link2_width}" rpy="${PI_2} 0 0"
        T_fixed_2 = self.create_transform(
            [0, self.link2_length, self.link2_width],
            [self.PI/2, 0, 0]
        )
        T_joint_2 = self.create_rotation_z(self.joint_angles[2])
        T = T @ T_fixed_2 @ T_joint_2

        # 5. link_3 → link_4 (joint_3 with rotation)
        # URDF: origin xyz="0 0 ${link3_length}" rpy="${PI_2} 0 0"
        T_fixed_3 = self.create_transform(
            [0, 0, self.link3_length],
            [self.PI/2, 0, 0]
        )
        T_joint_3 = self.create_rotation_z(self.joint_angles[3])
        T = T @ T_fixed_3 @ T_joint_3

        # 6. link_4 → end_effector (fixed)
        # URDF: origin xyz="0 ${link4_length + ee_offset} 0" rpy="-${PI_2} 0 -${PI_2}"
        T_ee = self.create_transform(
            [0, self.link4_length + self.ee_offset, 0],
            [-self.PI/2, 0, -self.PI/2]
        )
        T = T @ T_ee

        return T

    def matrix_to_pose(self, T):
        """Convert transformation matrix to Pose message"""
        pose = Pose()

        # Extract position
        pose.position.x = T[0, 3]
        pose.position.y = T[1, 3]
        pose.position.z = T[2, 3]

        # Extract rotation and convert to quaternion
        rotation_matrix = T[:3, :3]
        r = Rotation.from_matrix(rotation_matrix)
        quat = r.as_quat()  # Returns [x, y, z, w]

        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        return pose

    def get_actual_ee_pose(self):
        """Get actual end effector pose from TF tree"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'end_effector',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation

            return pose
        except Exception:
            return None

    def publish_forward_kinematics(self):
        """Calculate and publish forward kinematics"""
        # Calculate FK using URDF transformations
        T = self.calculate_forward_kinematics()
        calculated_pose = self.matrix_to_pose(T)

        # Create PoseStamped message
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'base_link'
        pose_msg.pose = calculated_pose

        # Publish calculated pose
        self.pose_pub.publish(pose_msg)

        # Get actual pose from TF
        actual_pose = self.get_actual_ee_pose()

        # Calculate and log comparison
        if actual_pose is not None:
            # Calculate position error
            dx = calculated_pose.position.x - actual_pose.position.x
            dy = calculated_pose.position.y - actual_pose.position.y
            dz = calculated_pose.position.z - actual_pose.position.z
            position_error = np.sqrt(dx**2 + dy**2 + dz**2)

            # Log comparison
            self.get_logger().info('='*60)
            self.get_logger().info(f'Joint Angles: [{", ".join([f"{np.rad2deg(a):.2f}°" for a in self.joint_angles])}]')
            self.get_logger().info(f'URDF Calc:  [{calculated_pose.position.x:.4f}, {calculated_pose.position.y:.4f}, {calculated_pose.position.z:.4f}]')
            self.get_logger().info(f'TF Actual:  [{actual_pose.position.x:.4f}, {actual_pose.position.y:.4f}, {actual_pose.position.z:.4f}]')
            self.get_logger().info(f'Position Error: {position_error:.6f} m')
            self.get_logger().info('='*60)


def main(args=None):
    rclpy.init(args=args)
    node = ForwardKinematicsURDFBased()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
