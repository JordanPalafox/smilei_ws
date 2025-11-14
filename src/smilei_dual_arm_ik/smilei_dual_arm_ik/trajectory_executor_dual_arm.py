#!/usr/bin/env python3
"""
Trajectory Executor Node for Dual Arm Robot

Executes synchronized or independent trajectories for both arms
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
import numpy as np
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from smilei_dual_arm_ik.inverse_kinematics_dual_arm import InverseKinematicsDualArm
from smilei_dual_arm_ik.trajectory_planner_dual_arm import TrajectoryPlannerDualArm


class TrajectoryExecutorDualArm(Node):
    """
    ROS2 node that executes trajectories for dual arm robot
    """

    def __init__(self):
        super().__init__('trajectory_executor_dual_arm')

        # Declare parameters
        self.declare_parameter('robot_params_file', '')
        self.declare_parameter('trajectory_config_file', '')
        self.declare_parameter('execute_on_startup', False)
        self.declare_parameter('interpolation_method', 'cubic')
        self.declare_parameter('steps_per_segment', 50)
        self.declare_parameter('execution_rate', 10.0)  # Hz
        self.declare_parameter('synchronized', True)  # Synchronized dual arm motion

        # Load parameters
        robot_params_file = self.get_parameter('robot_params_file').value
        if not robot_params_file:
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')

        trajectory_config_file = self.get_parameter('trajectory_config_file').value
        if not trajectory_config_file:
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            trajectory_config_file = os.path.join(pkg_share, 'config', 'trajectory_config_dual_arm.yaml')

        interpolation_method = self.get_parameter('interpolation_method').value
        self.steps_per_segment = self.get_parameter('steps_per_segment').value
        execution_rate = self.get_parameter('execution_rate').value
        execute_on_startup = self.get_parameter('execute_on_startup').value
        self.synchronized = self.get_parameter('synchronized').value

        # Initialize IK solver and trajectory planner
        self.ik_solver = InverseKinematicsDualArm(robot_params_file)
        self.trajectory_planner = TrajectoryPlannerDualArm(interpolation_method)

        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Publishers for visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/trajectory_markers',
            10
        )

        # State
        self.right_target_waypoints = []
        self.left_target_waypoints = []
        self.trajectory = None
        self.trajectory_index = 0
        self.is_executing = False
        self.last_joint_state = None

        # Timer for trajectory execution
        self.execution_timer = self.create_timer(
            1.0 / execution_rate,
            self.execution_callback
        )

        # Timer for maintaining joint state
        self.maintenance_timer = self.create_timer(
            0.1,  # 10 Hz
            self.maintenance_callback
        )

        # Load trajectory config if it exists
        if os.path.exists(trajectory_config_file):
            self.load_trajectory_config(trajectory_config_file)
            if execute_on_startup and (len(self.right_target_waypoints) > 0 or len(self.left_target_waypoints) > 0):
                self.start_trajectory_execution()

        self.get_logger().info('Trajectory Executor Dual Arm Node initialized')
        self.get_logger().info(f'  Interpolation method: {interpolation_method}')
        self.get_logger().info(f'  Steps per segment: {self.steps_per_segment}')
        self.get_logger().info(f'  Execution rate: {execution_rate} Hz')
        self.get_logger().info(f'  Synchronized: {self.synchronized}')

    def load_trajectory_config(self, config_file):
        """Load target waypoints for both arms from config file"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            # Load right arm waypoints
            right_waypoints_config = config.get('right_arm_waypoints', [])
            self.right_target_waypoints = [
                np.array([wp['x'], wp['y'], wp['z']])
                for wp in right_waypoints_config
            ]

            # Load left arm waypoints
            left_waypoints_config = config.get('left_arm_waypoints', [])
            self.left_target_waypoints = [
                np.array([wp['x'], wp['y'], wp['z']])
                for wp in left_waypoints_config
            ]

            self.get_logger().info(f'Loaded {len(self.right_target_waypoints)} right arm waypoints')
            for i, wp in enumerate(self.right_target_waypoints):
                self.get_logger().info(f'  Right {i}: {wp}')

            self.get_logger().info(f'Loaded {len(self.left_target_waypoints)} left arm waypoints')
            for i, wp in enumerate(self.left_target_waypoints):
                self.get_logger().info(f'  Left {i}: {wp}')

        except Exception as e:
            self.get_logger().error(f'Failed to load trajectory config: {e}')

    def solve_ik_for_both_arms(self):
        """
        Solve IK for all waypoints for both arms

        Returns:
            Tuple of (right_angles_list, left_angles_list)
        """
        self.get_logger().info('='*60)
        self.get_logger().info('Solving IK for both arms...')

        right_joint_angles = []
        left_joint_angles = []
        all_success = True

        # Solve for right arm
        self.get_logger().info('Right Arm:')
        for i, waypoint in enumerate(self.right_target_waypoints):
            solution = self.ik_solver.solve_ik_right_arm_multiple_attempts(waypoint)

            if solution['success']:
                self.get_logger().info(
                    f'  ✅ Waypoint {i}: Target={waypoint}, Error={solution["position_error"]:.6f}m'
                )
                right_joint_angles.append(solution['joint_angles'])
            else:
                self.get_logger().error(
                    f'  ❌ Waypoint {i}: Failed (Error: {solution["position_error"]*100:.2f}cm)'
                )
                all_success = False

        # Solve for left arm
        self.get_logger().info('Left Arm:')
        for i, waypoint in enumerate(self.left_target_waypoints):
            solution = self.ik_solver.solve_ik_left_arm_multiple_attempts(waypoint)

            if solution['success']:
                self.get_logger().info(
                    f'  ✅ Waypoint {i}: Target={waypoint}, Error={solution["position_error"]:.6f}m'
                )
                left_joint_angles.append(solution['joint_angles'])
            else:
                self.get_logger().error(
                    f'  ❌ Waypoint {i}: Failed (Error: {solution["position_error"]*100:.2f}cm)'
                )
                all_success = False

        self.get_logger().info('='*60)

        if not all_success:
            self.get_logger().warn('Some waypoints failed IK. Trajectory may be incomplete.')

        return right_joint_angles, left_joint_angles

    def plan_dual_arm_trajectory(self, right_angles, left_angles):
        """
        Plan smooth trajectory for both arms

        Args:
            right_angles: List of joint angle arrays for right arm
            left_angles: List of joint angle arrays for left arm

        Returns:
            Planned trajectory dictionary
        """
        self.get_logger().info('Planning dual arm trajectory...')

        result = self.trajectory_planner.plan_dual_arm_trajectory(
            right_angles,
            left_angles,
            num_steps_per_segment=self.steps_per_segment,
            synchronized=self.synchronized
        )

        self.get_logger().info(f'✅ Dual arm trajectory planned')
        self.get_logger().info(f'   Right arm: {result["right_arm"]["num_points"]} points')
        self.get_logger().info(f'   Left arm: {result["left_arm"]["num_points"]} points')
        return result

    def start_trajectory_execution(self):
        """Start executing the planned trajectory"""
        if len(self.right_target_waypoints) == 0 and len(self.left_target_waypoints) == 0:
            self.get_logger().warn('No target waypoints defined')
            return

        # Step 1: Solve IK for all waypoints
        right_angles, left_angles = self.solve_ik_for_both_arms()

        if len(right_angles) < 2 and len(left_angles) < 2:
            self.get_logger().error('Not enough valid IK solutions to plan trajectory')
            return

        # Step 2: Plan smooth trajectory
        self.trajectory = self.plan_dual_arm_trajectory(right_angles, left_angles)

        # Step 3: Visualize trajectory
        self.visualize_trajectory()

        # Step 4: Start execution
        self.trajectory_index = 0
        self.is_executing = True
        self.get_logger().info('🚀 Starting dual arm trajectory execution')

    def execution_callback(self):
        """Timer callback for trajectory execution"""
        if not self.is_executing or self.trajectory is None:
            return

        right_traj = self.trajectory['right_arm']['trajectory']
        left_traj = self.trajectory['left_arm']['trajectory']

        if self.trajectory_index >= len(right_traj) or self.trajectory_index >= len(left_traj):
            # Trajectory complete
            self.is_executing = False
            self.get_logger().info('✅ Dual arm trajectory execution complete')
            return

        # Get current joint angles for both arms
        right_angles = right_traj[self.trajectory_index]
        left_angles = left_traj[self.trajectory_index]

        # Create and publish joint state for both arms
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()

        # Right arm joints
        joint_msg.name = [
            'right_joint_0', 'right_joint_1', 'right_joint_2', 'right_joint_3',
            'left_joint_0', 'left_joint_1', 'left_joint_2', 'left_joint_3'
        ]

        # Combine right and left arm positions
        joint_msg.position = list(right_angles) + list(left_angles)

        self.joint_pub.publish(joint_msg)

        # Store last joint state for maintenance
        self.last_joint_state = joint_msg

        # Progress indicator
        if self.trajectory_index % 10 == 0:
            progress = (self.trajectory_index / len(right_traj)) * 100
            self.get_logger().info(
                f'Progress: {progress:.1f}% ({self.trajectory_index}/{len(right_traj)})'
            )

        self.trajectory_index += 1

    def maintenance_callback(self):
        """Timer callback to maintain joint state after trajectory ends"""
        if not self.is_executing and self.last_joint_state is not None:
            self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(self.last_joint_state)

    def visualize_trajectory(self):
        """Visualize planned trajectories for both arms in RViz"""
        if self.trajectory is None:
            return

        marker_array = MarkerArray()

        # Calculate FK for right arm trajectory
        right_positions = []
        for joint_angles in self.trajectory['right_arm']['trajectory']:
            T = self.ik_solver.forward_kinematics_right_arm(joint_angles)
            right_positions.append(T[:3, 3])

        # Calculate FK for left arm trajectory
        left_positions = []
        for joint_angles in self.trajectory['left_arm']['trajectory']:
            T = self.ik_solver.forward_kinematics_left_arm(joint_angles)
            left_positions.append(T[:3, 3])

        # Visualize right arm trajectory (GREEN)
        right_path = Marker()
        right_path.header.frame_id = 'base_link'
        right_path.header.stamp = self.get_clock().now().to_msg()
        right_path.ns = 'right_trajectory'
        right_path.id = 0
        right_path.type = Marker.LINE_STRIP
        right_path.action = Marker.ADD
        right_path.scale.x = 0.005
        right_path.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green

        for pos in right_positions:
            point = Point()
            point.x, point.y, point.z = pos
            right_path.points.append(point)

        marker_array.markers.append(right_path)

        # Visualize left arm trajectory (BLUE)
        left_path = Marker()
        left_path.header.frame_id = 'base_link'
        left_path.header.stamp = self.get_clock().now().to_msg()
        left_path.ns = 'left_trajectory'
        left_path.id = 1
        left_path.type = Marker.LINE_STRIP
        left_path.action = Marker.ADD
        left_path.scale.x = 0.005
        left_path.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.8)  # Blue

        for pos in left_positions:
            point = Point()
            point.x, point.y, point.z = pos
            left_path.points.append(point)

        marker_array.markers.append(left_path)

        # Right arm waypoints (RED)
        for i, waypoint_idx in enumerate(self.trajectory['right_arm']['waypoint_indices']):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'right_waypoints'
            marker.id = i + 100
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            pos = right_positions[waypoint_idx]
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red

            marker_array.markers.append(marker)

        # Left arm waypoints (CYAN)
        for i, waypoint_idx in enumerate(self.trajectory['left_arm']['waypoint_indices']):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'left_waypoints'
            marker.id = i + 200
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            pos = left_positions[waypoint_idx]
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)  # Cyan

            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info('✅ Dual arm trajectory visualization published')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutorDualArm()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
