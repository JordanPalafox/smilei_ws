#!/usr/bin/env python3
"""
Gesture Executor Action Server for Dual Arm Robot

Executes predefined gestures/routines loaded from YAML config files
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from smilei_dual_arm_ik.action import ExecuteGesture
import numpy as np
import yaml
import os
import time
import asyncio
from ament_index_python.packages import get_package_share_directory

from smilei_dual_arm_ik.inverse_kinematics_dual_arm import InverseKinematicsDualArm
from smilei_dual_arm_ik.trajectory_planner_dual_arm import TrajectoryPlannerDualArm


class GestureExecutorDualArm(Node):
    """
    ROS2 Action Server that executes predefined gestures for dual arm robot
    """

    def __init__(self):
        super().__init__('gesture_executor_dual_arm')

        # Declare parameters
        self.declare_parameter('robot_params_file', '')
        self.declare_parameter('gestures_directory', '')

        # Load parameters
        robot_params_file = self.get_parameter('robot_params_file').value
        if not robot_params_file:
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')

        gestures_dir = self.get_parameter('gestures_directory').value
        if not gestures_dir:
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            gestures_dir = os.path.join(pkg_share, 'config', 'gestures')

        self.gestures_directory = gestures_dir

        # Initialize IK solver and trajectory planner
        self.ik_solver = InverseKinematicsDualArm(robot_params_file)
        self.trajectory_planner = TrajectoryPlannerDualArm()

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
        self.last_joint_state = None

        # Timer for maintaining joint state (10 Hz)
        self.maintenance_timer = self.create_timer(
            0.1,
            self.maintenance_callback
        )

        # Initialize default joint state for robot_state_publisher
        self.initialize_default_joint_state()

        # Create action server with reentrant callback group for concurrent execution
        callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ExecuteGesture,
            'execute_gesture',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=callback_group
        )

        self.get_logger().info('Gesture Executor Action Server initialized')
        self.get_logger().info(f'  Gestures directory: {self.gestures_directory}')
        self.get_logger().info(f'  Robot params: {robot_params_file}')

    def initialize_default_joint_state(self):
        """Initialize joint state with default position (all zeros)"""
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = [
            'right_joint_0', 'right_joint_1', 'right_joint_2', 'right_joint_3',
            'left_joint_0', 'left_joint_1', 'left_joint_2', 'left_joint_3'
        ]
        # Initialize all joints to zero position
        joint_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.last_joint_state = joint_msg
        self.joint_pub.publish(joint_msg)
        self.get_logger().info('Published initial joint state (all zeros)')

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action"""
        self.get_logger().info(f'Received goal request for gesture: {goal_request.gesture_name}')

        # Check if gesture file exists
        gesture_file = os.path.join(self.gestures_directory, f'{goal_request.gesture_name}.yaml')
        if not os.path.exists(gesture_file):
            self.get_logger().error(f'Gesture file not found: {gesture_file}')
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action"""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the gesture action"""
        self.get_logger().info(f'Executing gesture: {goal_handle.request.gesture_name}')

        feedback_msg = ExecuteGesture.Feedback()
        result = ExecuteGesture.Result()
        start_time = time.time()

        try:
            # Phase 1: Load gesture configuration
            feedback_msg.current_phase = 'loading'
            feedback_msg.progress = 0.0
            goal_handle.publish_feedback(feedback_msg)

            gesture_config = self.load_gesture_config(goal_handle.request.gesture_name)
            if gesture_config is None:
                result.success = False
                result.message = f'Failed to load gesture: {goal_handle.request.gesture_name}'
                result.execution_time = time.time() - start_time
                return result

            # Phase 2: Solve IK for all waypoints
            feedback_msg.current_phase = 'solving_ik'
            feedback_msg.progress = 0.2
            feedback_msg.total_waypoints = max(
                len(gesture_config['right_waypoints']),
                len(gesture_config['left_waypoints'])
            )
            goal_handle.publish_feedback(feedback_msg)

            right_angles, left_angles = self.solve_ik_for_both_arms(
                gesture_config['right_waypoints'],
                gesture_config['left_waypoints']
            )

            # Check if we have at least 1 valid waypoint for either arm
            if len(right_angles) < 1 and len(left_angles) < 1:
                result.success = False
                result.message = 'No valid IK solutions found'
                result.execution_time = time.time() - start_time
                return result

            # Handle single waypoint case - duplicate waypoint so planner has 2 points
            if len(right_angles) == 1:
                self.get_logger().info('Right arm has 1 waypoint - duplicating for smooth motion')
                right_angles.append(right_angles[0])

            if len(left_angles) == 1:
                self.get_logger().info('Left arm has 1 waypoint - duplicating for smooth motion')
                left_angles.append(left_angles[0])

            # If one arm has no waypoints, use home position (all zeros)
            if len(right_angles) == 0:
                self.get_logger().info('Right arm has no waypoints - using home position')
                home_position = np.zeros(4)
                right_angles = [home_position, home_position]

            if len(left_angles) == 0:
                self.get_logger().info('Left arm has no waypoints - using home position')
                home_position = np.zeros(4)
                left_angles = [home_position, home_position]

            # Phase 3: Plan trajectory
            feedback_msg.current_phase = 'planning'
            feedback_msg.progress = 0.4
            goal_handle.publish_feedback(feedback_msg)

            trajectory = self.plan_dual_arm_trajectory(
                right_angles,
                left_angles,
                gesture_config['steps_per_segment'],
                gesture_config['synchronized'],
                gesture_config['interpolation_method']
            )

            # Visualize trajectory
            self.visualize_trajectory(trajectory)

            # Phase 4: Create transition from current position to first waypoint
            feedback_msg.current_phase = 'transitioning'
            feedback_msg.progress = 0.45
            goal_handle.publish_feedback(feedback_msg)

            transition_trajectory = self.create_transition_trajectory(
                trajectory,
                gesture_config['interpolation_method']
            )

            # Phase 5: Execute trajectory (with transition)
            feedback_msg.current_phase = 'executing'
            feedback_msg.progress = 0.5
            goal_handle.publish_feedback(feedback_msg)

            execution_success = self.execute_trajectory_with_transition(
                transition_trajectory,
                trajectory,
                goal_handle,
                feedback_msg
            )

            if not execution_success:
                result.success = False
                result.message = 'Trajectory execution was cancelled'
                result.execution_time = time.time() - start_time
                return result

            # Success!
            result.success = True
            result.message = f'Gesture "{goal_handle.request.gesture_name}" executed successfully'
            result.execution_time = time.time() - start_time

            self.get_logger().info(
                f'✅ Gesture completed: {result.message} in {result.execution_time:.2f}s'
            )

            goal_handle.succeed()
            return result

        except Exception as e:
            self.get_logger().error(f'Error executing gesture: {e}')
            result.success = False
            result.message = f'Error: {str(e)}'
            result.execution_time = time.time() - start_time
            goal_handle.abort()
            return result

    def load_gesture_config(self, gesture_name):
        """Load gesture configuration from YAML file"""
        try:
            gesture_file = os.path.join(self.gestures_directory, f'{gesture_name}.yaml')
            self.get_logger().info(f'Loading gesture from: {gesture_file}')

            with open(gesture_file, 'r') as f:
                config = yaml.safe_load(f)

            # Extract waypoints
            right_waypoints_config = config.get('right_arm_waypoints', [])
            right_waypoints = [
                np.array([wp['x'], wp['y'], wp['z']])
                for wp in right_waypoints_config
            ]

            left_waypoints_config = config.get('left_arm_waypoints', [])
            left_waypoints = [
                np.array([wp['x'], wp['y'], wp['z']])
                for wp in left_waypoints_config
            ]

            # Extract execution parameters
            synchronized = config.get('synchronized', True)
            interpolation_method = config.get('interpolation_method', 'cubic')
            steps_per_segment = config.get('steps_per_segment', 50)

            self.get_logger().info(
                f'Loaded gesture "{gesture_name}": '
                f'{len(right_waypoints)} right waypoints, '
                f'{len(left_waypoints)} left waypoints, '
                f'synchronized={synchronized}'
            )

            return {
                'right_waypoints': right_waypoints,
                'left_waypoints': left_waypoints,
                'synchronized': synchronized,
                'interpolation_method': interpolation_method,
                'steps_per_segment': steps_per_segment
            }

        except Exception as e:
            self.get_logger().error(f'Failed to load gesture config: {e}')
            return None

    def solve_ik_for_both_arms(self, right_waypoints, left_waypoints):
        """
        Solve IK for all waypoints for both arms

        Returns:
            Tuple of (right_angles_list, left_angles_list)
        """
        self.get_logger().info('='*60)
        self.get_logger().info('Solving IK for both arms...')

        right_joint_angles = []
        left_joint_angles = []

        # Solve for right arm
        if right_waypoints:
            self.get_logger().info('Right Arm:')
            for i, waypoint in enumerate(right_waypoints):
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

        # Solve for left arm
        if left_waypoints:
            self.get_logger().info('Left Arm:')
            for i, waypoint in enumerate(left_waypoints):
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

        self.get_logger().info('='*60)

        return right_joint_angles, left_joint_angles

    def plan_dual_arm_trajectory(self, right_angles, left_angles,
                                   steps_per_segment, synchronized, interpolation_method):
        """
        Plan smooth trajectory for both arms

        Returns:
            Planned trajectory dictionary
        """
        self.get_logger().info('Planning dual arm trajectory...')

        # Create a new planner with the gesture-specific interpolation method
        planner = TrajectoryPlannerDualArm(interpolation_method=interpolation_method)

        result = planner.plan_dual_arm_trajectory(
            right_angles,
            left_angles,
            num_steps_per_segment=steps_per_segment,
            synchronized=synchronized
        )

        self.get_logger().info(f'✅ Dual arm trajectory planned')
        self.get_logger().info(f'   Right arm: {result["right_arm"]["num_points"]} points')
        self.get_logger().info(f'   Left arm: {result["left_arm"]["num_points"]} points')

        return result

    def create_transition_trajectory(self, target_trajectory, interpolation_method='cubic'):
        """
        Create a smooth transition trajectory from current position to first point of target trajectory

        Args:
            target_trajectory: The target trajectory dict with 'right_arm' and 'left_arm' keys
            interpolation_method: Interpolation method to use ('linear', 'cubic', 'quintic')

        Returns:
            Transition trajectory dict with same structure as target_trajectory
        """
        # Get current joint positions
        current_right_angles = np.zeros(4)
        current_left_angles = np.zeros(4)

        if self.last_joint_state is not None:
            # Extract current angles from last joint state
            # Joint order: right_joint_0-3, left_joint_0-3
            current_right_angles = np.array(self.last_joint_state.position[0:4])
            current_left_angles = np.array(self.last_joint_state.position[4:8])

        # Get first point of target trajectory
        first_right_angles = target_trajectory['right_arm']['trajectory'][0]
        first_left_angles = target_trajectory['left_arm']['trajectory'][0]

        # Check if we need a transition (if current position is different from first point)
        right_diff = np.linalg.norm(current_right_angles - first_right_angles)
        left_diff = np.linalg.norm(current_left_angles - first_left_angles)

        # If already at the first position (within tolerance), return empty transition
        if right_diff < 0.01 and left_diff < 0.01:
            self.get_logger().info('Already at first waypoint, skipping transition')
            return {
                'right_arm': {'trajectory': np.array([]), 'num_points': 0},
                'left_arm': {'trajectory': np.array([]), 'num_points': 0}
            }

        self.get_logger().info(
            f'Creating transition: right_diff={right_diff:.3f}rad, left_diff={left_diff:.3f}rad'
        )

        # Create transition trajectory (30 steps = 3 seconds at 10Hz)
        num_transition_steps = 30

        # Use trajectory planner to create smooth transition
        planner = TrajectoryPlannerDualArm(interpolation_method=interpolation_method)

        # Create waypoint lists with current and target positions
        right_waypoints = [current_right_angles, first_right_angles]
        left_waypoints = [current_left_angles, first_left_angles]

        transition = planner.plan_dual_arm_trajectory(
            right_waypoints,
            left_waypoints,
            num_steps_per_segment=num_transition_steps,
            synchronized=True
        )

        self.get_logger().info(
            f'✅ Transition trajectory created: {transition["right_arm"]["num_points"]} points'
        )

        return transition

    def execute_trajectory_with_transition(self, transition_trajectory, main_trajectory,
                                          goal_handle, feedback_msg):
        """
        Execute transition trajectory followed by main trajectory

        Returns:
            True if successful, False if cancelled
        """
        # Execute transition if it exists
        if transition_trajectory['right_arm']['num_points'] > 0:
            self.get_logger().info('🔄 Executing transition to first waypoint...')

            success = self.execute_single_trajectory(
                transition_trajectory,
                goal_handle,
                feedback_msg,
                progress_start=0.5,
                progress_end=0.55  # 5% of total progress for transition
            )

            if not success:
                return False

            self.get_logger().info('✅ Transition complete')

        # Execute main trajectory
        self.get_logger().info('🚀 Executing main gesture trajectory...')
        success = self.execute_single_trajectory(
            main_trajectory,
            goal_handle,
            feedback_msg,
            progress_start=0.55,
            progress_end=1.0  # Remaining 45% for main trajectory
        )

        return success

    def execute_single_trajectory(self, trajectory, goal_handle, feedback_msg,
                                  progress_start=0.5, progress_end=1.0):
        """
        Execute a single trajectory segment with progress tracking

        Returns:
            True if successful, False if cancelled
        """
        right_traj = trajectory['right_arm']['trajectory']
        left_traj = trajectory['left_arm']['trajectory']
        total_points = max(len(right_traj), len(left_traj))

        if total_points == 0:
            return True  # Empty trajectory, nothing to do

        execution_rate = 10.0  # Hz
        sleep_time = 1.0 / execution_rate

        for i in range(total_points):
            # Check if goal is cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                return False

            # Get current joint angles for both arms
            right_angles = right_traj[min(i, len(right_traj) - 1)]
            left_angles = left_traj[min(i, len(left_traj) - 1)]

            # Create and publish joint state
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = [
                'right_joint_0', 'right_joint_1', 'right_joint_2', 'right_joint_3',
                'left_joint_0', 'left_joint_1', 'left_joint_2', 'left_joint_3'
            ]
            joint_msg.position = list(right_angles) + list(left_angles)

            self.joint_pub.publish(joint_msg)
            self.last_joint_state = joint_msg

            # Update feedback with scaled progress
            progress_range = progress_end - progress_start
            progress = progress_start + (progress_range * (i / total_points))
            feedback_msg.progress = progress
            feedback_msg.current_waypoint = i
            feedback_msg.total_waypoints = total_points

            if i % 10 == 0:  # Log every 10 points
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(
                    f'Progress: {progress*100:.1f}% ({i}/{total_points})'
                )

            # Sleep to maintain execution rate
            time.sleep(sleep_time)

        return True

    def execute_trajectory(self, trajectory, goal_handle, feedback_msg):
        """
        Execute the planned trajectory with feedback

        Returns:
            True if successful, False if cancelled
        """
        right_traj = trajectory['right_arm']['trajectory']
        left_traj = trajectory['left_arm']['trajectory']
        total_points = max(len(right_traj), len(left_traj))

        execution_rate = 10.0  # Hz
        sleep_time = 1.0 / execution_rate

        self.get_logger().info(f'🚀 Starting trajectory execution ({total_points} points)')

        for i in range(total_points):
            # Check if goal is cancelled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                return False

            # Get current joint angles for both arms
            right_angles = right_traj[min(i, len(right_traj) - 1)]
            left_angles = left_traj[min(i, len(left_traj) - 1)]

            # Create and publish joint state
            joint_msg = JointState()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = [
                'right_joint_0', 'right_joint_1', 'right_joint_2', 'right_joint_3',
                'left_joint_0', 'left_joint_1', 'left_joint_2', 'left_joint_3'
            ]
            joint_msg.position = list(right_angles) + list(left_angles)

            self.joint_pub.publish(joint_msg)
            self.last_joint_state = joint_msg

            # Update feedback
            progress = 0.5 + (0.5 * (i / total_points))  # 50-100%
            feedback_msg.progress = progress
            feedback_msg.current_waypoint = i
            feedback_msg.total_waypoints = total_points

            if i % 10 == 0:  # Log every 10 points
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(
                    f'Progress: {progress*100:.1f}% ({i}/{total_points})'
                )

            # Sleep to maintain execution rate
            time.sleep(sleep_time)

        self.get_logger().info('✅ Trajectory execution complete')
        return True

    def maintenance_callback(self):
        """Timer callback to maintain joint state"""
        if self.last_joint_state is not None:
            self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(self.last_joint_state)

    def visualize_trajectory(self, trajectory):
        """Visualize planned trajectories for both arms in RViz"""
        marker_array = MarkerArray()

        # Calculate FK for right arm trajectory
        right_positions = []
        for joint_angles in trajectory['right_arm']['trajectory']:
            T = self.ik_solver.forward_kinematics_right_arm(joint_angles)
            right_positions.append(T[:3, 3])

        # Calculate FK for left arm trajectory
        left_positions = []
        for joint_angles in trajectory['left_arm']['trajectory']:
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
        for i, waypoint_idx in enumerate(trajectory['right_arm']['waypoint_indices']):
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
        for i, waypoint_idx in enumerate(trajectory['left_arm']['waypoint_indices']):
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
        self.get_logger().info('✅ Trajectory visualization published')


def main(args=None):
    rclpy.init(args=args)

    node = GestureExecutorDualArm()

    # Use MultiThreadedExecutor to allow concurrent action execution
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
