#!/usr/bin/env python3
"""
Trajectory Executor Node

Integrates:
1. Inverse kinematics solving for target positions
2. Smooth trajectory planning
3. Trajectory execution
4. RViz visualization
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

from smilei_dual_arm_ik.inverse_kinematics_solver import InverseKinematicsSolver
from smilei_dual_arm_ik.trajectory_planner import TrajectoryPlanner


class TrajectoryExecutorNode(Node):
    """
    ROS2 node that executes trajectories through target waypoints
    """

    def __init__(self):
        super().__init__('trajectory_executor_node')

        # Declare parameters
        self.declare_parameter('robot_params_file', '')
        self.declare_parameter('trajectory_config_file', '')
        self.declare_parameter('execute_on_startup', False)
        self.declare_parameter('interpolation_method', 'cubic')
        self.declare_parameter('steps_per_segment', 50)
        self.declare_parameter('execution_rate', 10.0)  # Hz

        # Load parameters
        robot_params_file = self.get_parameter('robot_params_file').value
        if not robot_params_file:
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')

        trajectory_config_file = self.get_parameter('trajectory_config_file').value
        if not trajectory_config_file:
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            trajectory_config_file = os.path.join(pkg_share, 'config', 'trajectory_config.yaml')

        interpolation_method = self.get_parameter('interpolation_method').value
        self.steps_per_segment = self.get_parameter('steps_per_segment').value
        execution_rate = self.get_parameter('execution_rate').value
        execute_on_startup = self.get_parameter('execute_on_startup').value

        # Initialize IK solver and trajectory planner
        self.ik_solver = InverseKinematicsSolver(robot_params_file)
        self.trajectory_planner = TrajectoryPlanner(interpolation_method)

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
        self.target_waypoints = []
        self.trajectory = None
        self.trajectory_index = 0
        self.is_executing = False
        self.last_joint_state = None  # Store last joint state to keep publishing

        # Timer for trajectory execution
        self.execution_timer = self.create_timer(
            1.0 / execution_rate,
            self.execution_callback
        )

        # Timer for maintaining joint state (keep TF alive after trajectory ends)
        self.maintenance_timer = self.create_timer(
            0.1,  # 10 Hz
            self.maintenance_callback
        )

        # Load trajectory config if it exists
        if os.path.exists(trajectory_config_file):
            self.load_trajectory_config(trajectory_config_file)
            if execute_on_startup and len(self.target_waypoints) > 0:
                self.start_trajectory_execution()

        self.get_logger().info('Trajectory Executor Node initialized')
        self.get_logger().info(f'  Interpolation method: {interpolation_method}')
        self.get_logger().info(f'  Steps per segment: {self.steps_per_segment}')
        self.get_logger().info(f'  Execution rate: {execution_rate} Hz')

    def load_trajectory_config(self, config_file):
        """Load target waypoints from config file"""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)

            waypoints_config = config.get('target_waypoints', [])
            self.target_waypoints = [
                np.array([wp['x'], wp['y'], wp['z']])
                for wp in waypoints_config
            ]

            self.get_logger().info(f'Loaded {len(self.target_waypoints)} target waypoints')
            for i, wp in enumerate(self.target_waypoints):
                self.get_logger().info(f'  Waypoint {i}: {wp}')

        except Exception as e:
            self.get_logger().error(f'Failed to load trajectory config: {e}')

    def solve_ik_for_waypoints(self):
        """
        Solve IK for all target waypoints

        Returns:
            List of joint angle solutions
        """
        self.get_logger().info('='*60)
        self.get_logger().info('Solving IK for waypoints...')

        solutions = self.ik_solver.solve_ik_for_waypoints(self.target_waypoints)

        # Check results
        all_success = True
        joint_angles_list = []

        for i, (waypoint, solution) in enumerate(zip(self.target_waypoints, solutions)):
            if solution['success']:
                self.get_logger().info(
                    f'✅ Waypoint {i}: Target={waypoint}, '
                    f'Error={solution["position_error"]:.6f}m'
                )
                joint_angles_list.append(solution['joint_angles'])
            else:
                achieved = solution.get('achieved_position', None)
                self.get_logger().error(
                    f'❌ Waypoint {i}: Failed to find IK solution'
                )
                self.get_logger().error(
                    f'   Target: [{waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f}]'
                )
                if achieved is not None:
                    self.get_logger().error(
                        f'   Closest: [{achieved[0]:.3f}, {achieved[1]:.3f}, {achieved[2]:.3f}]'
                    )
                self.get_logger().error(
                    f'   Distance: {solution["position_error"]:.6f}m ({solution["position_error"]*100:.2f}cm)'
                )
                all_success = False

        self.get_logger().info('='*60)

        if all_success:
            return joint_angles_list
        else:
            self.get_logger().warn('Some waypoints failed IK. Trajectory may be incomplete.')
            return joint_angles_list if len(joint_angles_list) > 0 else None

    def plan_smooth_trajectory(self, waypoint_angles):
        """
        Plan smooth trajectory through waypoint joint angles

        Args:
            waypoint_angles: List of joint angle arrays

        Returns:
            Planned trajectory dictionary
        """
        self.get_logger().info('Planning smooth trajectory...')

        result = self.trajectory_planner.plan_trajectory(
            waypoint_angles,
            num_steps_per_segment=self.steps_per_segment
        )

        self.get_logger().info(f'✅ Trajectory planned: {result["num_points"]} points')
        return result

    def start_trajectory_execution(self):
        """Start executing the planned trajectory"""
        if len(self.target_waypoints) == 0:
            self.get_logger().warn('No target waypoints defined')
            return

        # Step 1: Solve IK for all waypoints
        waypoint_angles = self.solve_ik_for_waypoints()

        if waypoint_angles is None or len(waypoint_angles) < 2:
            self.get_logger().error('Not enough valid IK solutions to plan trajectory')
            return

        # Step 2: Plan smooth trajectory
        self.trajectory = self.plan_smooth_trajectory(waypoint_angles)

        # Step 3: Visualize trajectory
        self.visualize_trajectory()

        # Step 4: Start execution
        self.trajectory_index = 0
        self.is_executing = True
        self.get_logger().info('🚀 Starting trajectory execution')

    def execution_callback(self):
        """Timer callback for trajectory execution"""
        if not self.is_executing or self.trajectory is None:
            return

        if self.trajectory_index >= self.trajectory['num_points']:
            # Trajectory complete
            self.is_executing = False
            self.get_logger().info('✅ Trajectory execution complete')
            self.get_logger().info('   TF frames will be maintained at final position')
            return

        # Get current joint angles from trajectory
        joint_angles = self.trajectory['trajectory'][self.trajectory_index]

        # Create and publish joint state
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['joint_0', 'joint_1', 'joint_2', 'joint_3']
        joint_msg.position = joint_angles.tolist()
        self.joint_pub.publish(joint_msg)

        # Store last joint state for maintenance
        self.last_joint_state = joint_msg

        # Progress indicator
        if self.trajectory_index % 10 == 0:
            progress = (self.trajectory_index / self.trajectory['num_points']) * 100
            self.get_logger().info(f'Progress: {progress:.1f}% ({self.trajectory_index}/{self.trajectory["num_points"]})')

        self.trajectory_index += 1

    def maintenance_callback(self):
        """Timer callback to maintain joint state after trajectory ends"""
        # Only publish if we're not executing and we have a last state
        if not self.is_executing and self.last_joint_state is not None:
            # Update timestamp and republish
            self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(self.last_joint_state)

    def visualize_trajectory(self):
        """Visualize planned trajectory in RViz"""
        if self.trajectory is None:
            return

        marker_array = MarkerArray()

        # Calculate forward kinematics for all trajectory points
        positions = []
        for joint_angles in self.trajectory['trajectory']:
            T = self.ik_solver.forward_kinematics(joint_angles)
            positions.append(T[:3, 3])

        # Marker 1: Trajectory path (line strip)
        path_marker = Marker()
        path_marker.header.frame_id = 'base_link'
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.ns = 'trajectory'
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        path_marker.scale.x = 0.005  # Line width
        path_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)  # Green

        for pos in positions:
            point = Point()
            point.x, point.y, point.z = pos
            path_marker.points.append(point)

        marker_array.markers.append(path_marker)

        # Marker 2: Original waypoints (larger spheres)
        num_waypoints = len(self.trajectory['waypoint_indices'])
        for i, waypoint_idx in enumerate(self.trajectory['waypoint_indices']):
            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = 'base_link'
            waypoint_marker.header.stamp = self.get_clock().now().to_msg()
            waypoint_marker.ns = 'waypoints'
            waypoint_marker.id = i + 1
            waypoint_marker.type = Marker.SPHERE
            waypoint_marker.action = Marker.ADD

            pos = positions[waypoint_idx]
            waypoint_marker.pose.position.x = pos[0]
            waypoint_marker.pose.position.y = pos[1]
            waypoint_marker.pose.position.z = pos[2]
            waypoint_marker.pose.orientation.w = 1.0

            waypoint_marker.scale.x = 0.05
            waypoint_marker.scale.y = 0.05
            waypoint_marker.scale.z = 0.05

            # Color: Red for intermediate waypoints, Pink/Magenta for final waypoint
            if i == num_waypoints - 1:
                # Final waypoint: Pink/Magenta
                waypoint_marker.color = ColorRGBA(r=1.0, g=0.0, b=1.0, a=1.0)  # Magenta/Pink
            else:
                # Other waypoints: Red
                waypoint_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # Red

            marker_array.markers.append(waypoint_marker)

        # Marker 3: Intermediate points (small spheres)
        intermediate_indices = [
            i for i in range(len(positions))
            if i not in self.trajectory['waypoint_indices']
        ]

        for i, idx in enumerate(intermediate_indices[::5]):  # Show every 5th intermediate point
            inter_marker = Marker()
            inter_marker.header.frame_id = 'base_link'
            inter_marker.header.stamp = self.get_clock().now().to_msg()
            inter_marker.ns = 'intermediate'
            inter_marker.id = i + 100
            inter_marker.type = Marker.SPHERE
            inter_marker.action = Marker.ADD

            pos = positions[idx]
            inter_marker.pose.position.x = pos[0]
            inter_marker.pose.position.y = pos[1]
            inter_marker.pose.position.z = pos[2]
            inter_marker.pose.orientation.w = 1.0

            inter_marker.scale.x = 0.02
            inter_marker.scale.y = 0.02
            inter_marker.scale.z = 0.02
            inter_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.6)  # Light blue

            marker_array.markers.append(inter_marker)

        self.marker_pub.publish(marker_array)
        self.get_logger().info('✅ Trajectory visualization published')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryExecutorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
