#!/usr/bin/env python3
"""
Autonomous Gesture Execution Behavior

Behavior that executes predefined gestures using direct motor control.
The gesture name can be set via parameter or topic subscription.
This behavior integrates with the smilei_state_machine and uses the shared HardwareManager.
"""

import py_trees
import rclpy
import yaml
import os
import time
import numpy as np
from std_msgs.msg import String, Bool
from ament_index_python.packages import get_package_share_directory

# Import IK and trajectory planning
from smilei_dual_arm_ik.inverse_kinematics_dual_arm import InverseKinematicsDualArm
from smilei_dual_arm_ik.trajectory_planner_dual_arm import TrajectoryPlannerDualArm


class AutonomousGestureExecution(py_trees.behaviour.Behaviour):
    """
    Behavior that executes predefined gestures for the dual arm robot.

    This behavior directly controls motors using the shared HardwareManager.
    It can receive gesture commands via:
    - Direct parameter (gesture_name)
    - Topic subscription (/gesture_command)
    """

    def __init__(self, name: str, motor_ids: list[int], node=None, hardware_manager=None, gesture_name=None):
        """
        Initialize the behavior

        Args:
            name: Behavior name
            motor_ids: List of motor IDs to control
            node: ROS2 node (optional, will create one if not provided)
            hardware_manager: Shared HardwareManager instance
            gesture_name: Initial gesture to execute (optional)
        """
        super().__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.own_node = False
        self.hardware_manager = hardware_manager
        self.gesture_name = gesture_name
        self.pending_gesture = None

        # Motor IDs mapping (consistent with gesture_executor_hardware)
        # Right arm: motors 1,2,3,4 | Left arm: motors 5,6,7,8
        self.right_motor_ids = [1, 2, 3, 4]
        self.left_motor_ids = [5, 6, 7, 8]

        # IK solver and trajectory planner
        self.ik_solver = None
        self.trajectory_planner = None

        # Gestures directory
        self.gestures_directory = None

        # Publishers for state machine integration
        self.status_pub = None
        self.executing_pub = None
        self.gesture_command_sub = None

        # Execution state
        self.execution_started = False
        self.execution_complete = False
        self.execution_success = False
        self.running = False

        # Behavior activation state (separate from gesture execution state)
        self.is_active = False  # True when behavior is in active state, False otherwise

        # Motor state for control
        self.current_positions = [0.0] * 8
        self.target_positions = [0.0] * 8

        # PD Control parameters (from gesture_executor_hardware)
        self.kp = 1.0
        self.kp_motor7 = 0.5
        self.kd = 0.1
        self.max_current = 5.0
        self.Kt = 0.35

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        """Setup the behavior - initialize IK solver and publishers"""
        # Create or use provided node
        if self.node is None:
            self.node = rclpy.create_node('autonomous_gesture_execution')
            self.own_node = True
        else:
            self.own_node = False

        # Get robot parameters file
        pkg_share = get_package_share_directory('smilei_dual_arm_ik')
        robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')

        # Initialize IK solver
        try:
            self.ik_solver = InverseKinematicsDualArm(robot_params_file)
            self.node.get_logger().info('✅ IK Solver initialized')
        except Exception as e:
            self.node.get_logger().error(f'Failed to initialize IK solver: {e}')
            return True  # Still return True to continue like other behaviors

        # Initialize trajectory planner
        self.trajectory_planner = TrajectoryPlannerDualArm()

        # Set gestures directory
        self.gestures_directory = os.path.join(pkg_share, 'config', 'gestures')

        # Subscribe to gesture command topic
        self.gesture_command_sub = self.node.create_subscription(
            String,
            '/gesture_command',
            self.gesture_command_callback,
            10
        )

        # Create publishers for state machine integration
        self.status_pub = self.node.create_publisher(
            String,
            '/gesture_execution_status',
            10
        )

        self.executing_pub = self.node.create_publisher(
            Bool,
            '/gesture_executing',
            10
        )

        # Check hardware connection
        if self.hardware_manager is not None:
            available_motors = self.hardware_manager.get_available_motors()
            self.available_motors = [m for m in self.motor_ids if m in available_motors]
            if self.available_motors:
                self.node.get_logger().info(f'Hardware connected - motors: {self.available_motors}')
            else:
                self.node.get_logger().warning('No motors available')
        else:
            self.node.get_logger().warning('Hardware manager not available - simulation mode')

        self.node.get_logger().info('Autonomous Gesture Execution behavior setup complete')
        return True

    def gesture_command_callback(self, msg):
        """Callback for receiving gesture commands via topic"""
        self.node.get_logger().info(f'📨 Received gesture command: {msg.data}')

        # Only process gestures if behavior is in active state
        if self.is_active:
            self.node.get_logger().info(f'⚡ Behavior active - executing gesture immediately')
            self.gesture_name = msg.data
            self.execution_started = False
            self.execution_complete = False
            self.execution_success = False
            self.running = True
        else:
            self.node.get_logger().info(f'💤 Behavior inactive - gesture ignored')

    def initialise(self) -> None:
        """Called when behavior is activated"""
        # Mark behavior as active (can now receive gesture commands)
        self.is_active = True

        # Use pending gesture if available
        if self.pending_gesture:
            self.gesture_name = self.pending_gesture
            self.pending_gesture = None

        if not self.gesture_name:
            self.node.get_logger().warning('No gesture name provided')
            self.publish_status("idle", False)
            self.running = False
            return

        self.node.get_logger().info(f'🎭 Initializing gesture execution: {self.gesture_name}')

        # Reset state
        self.execution_started = False
        self.execution_complete = False
        self.execution_success = False
        self.running = True

        # Publish initial status
        self.publish_status(f"initializing_{self.gesture_name}", False)

    def update(self) -> py_trees.common.Status:
        """Main update loop"""
        # Check if we have a gesture to execute
        if not self.gesture_name or not self.running:
            self.publish_status("idle", False)
            return py_trees.common.Status.RUNNING  # Stay active waiting for commands

        # Start execution if not started
        if not self.execution_started:
            return self.start_gesture_execution()

        # Check execution status
        if not self.execution_complete:
            return py_trees.common.Status.RUNNING

        # Execution complete - return result and reset for next gesture
        if self.execution_success:
            self.node.get_logger().info(f'✅ Gesture "{self.gesture_name}" executed successfully!')
            self.publish_status(f"completed_{self.gesture_name}_success", False)
            # Reset state to be ready for next gesture
            self.gesture_name = None
            self.running = False
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error(f'❌ Gesture "{self.gesture_name}" failed')
            self.publish_status(f"completed_{self.gesture_name}_failed", False)
            # Reset state to be ready for next gesture
            self.gesture_name = None
            self.running = False
            return py_trees.common.Status.FAILURE

    def start_gesture_execution(self):
        """Start gesture execution"""
        self.node.get_logger().info(f'🚀 Starting gesture: {self.gesture_name}')
        self.publish_status(f"starting_{self.gesture_name}", True)

        try:
            # Load gesture from YAML
            gesture_config = self.load_gesture_config(self.gesture_name)
            if gesture_config is None:
                self.execution_started = True
                self.execution_complete = True
                self.execution_success = False
                return py_trees.common.Status.RUNNING

            # Solve IK for waypoints
            right_angles, left_angles = self.solve_ik_for_gesture(gesture_config)

            if not right_angles and not left_angles:
                self.node.get_logger().error('IK solving failed for all waypoints')
                self.execution_started = True
                self.execution_complete = True
                self.execution_success = False
                return py_trees.common.Status.RUNNING

            # Plan trajectory
            trajectory = self.plan_dual_arm_trajectory(
                right_angles,
                left_angles,
                gesture_config.get('steps_per_segment', 50),
                gesture_config.get('synchronized', False),
                gesture_config.get('interpolation_method', 'cubic')
            )

            # Execute trajectory
            success = self.execute_trajectory(trajectory)

            self.execution_started = True
            self.execution_complete = True
            self.execution_success = success

        except Exception as e:
            self.node.get_logger().error(f'Error during gesture execution: {e}')
            self.execution_started = True
            self.execution_complete = True
            self.execution_success = False

        return py_trees.common.Status.RUNNING

    def load_gesture_config(self, gesture_name):
        """Load gesture configuration from YAML file"""
        gesture_file = os.path.join(self.gestures_directory, f'{gesture_name}.yaml')

        if not os.path.exists(gesture_file):
            self.node.get_logger().error(f'Gesture file not found: {gesture_file}')
            return None

        try:
            with open(gesture_file, 'r') as f:
                config = yaml.safe_load(f)
            self.node.get_logger().info(f'✅ Loaded gesture: {gesture_name}')
            return config
        except Exception as e:
            self.node.get_logger().error(f'Error loading gesture: {e}')
            return None

    def solve_ik_for_gesture(self, gesture_config):
        """Solve IK for all waypoints in the gesture"""
        right_waypoints = gesture_config.get('right_arm_waypoints', [])
        left_waypoints = gesture_config.get('left_arm_waypoints', [])

        # DEBUG: Log loaded waypoints
        self.node.get_logger().info(f'🔍 DEBUG: Right waypoints loaded: {right_waypoints}')
        self.node.get_logger().info(f'🔍 DEBUG: Left waypoints loaded: {left_waypoints}')

        right_joint_angles = []
        left_joint_angles = []

        # Solve for right arm
        for waypoint in right_waypoints:
            target_pos = np.array([waypoint['x'], waypoint['y'], waypoint['z']])
            solution = self.ik_solver.solve_ik_right_arm_multiple_attempts(target_pos)

            if solution['success']:
                right_joint_angles.append(solution['joint_angles'])
            else:
                self.node.get_logger().warning(f'IK failed for right waypoint: {target_pos}')

        # Solve for left arm
        for waypoint in left_waypoints:
            target_pos = np.array([waypoint['x'], waypoint['y'], waypoint['z']])
            solution = self.ik_solver.solve_ik_left_arm_multiple_attempts(target_pos)

            if solution['success']:
                left_joint_angles.append(solution['joint_angles'])
            else:
                self.node.get_logger().warning(f'IK failed for left waypoint: {target_pos}')

        return right_joint_angles, left_joint_angles

    def plan_dual_arm_trajectory(self, right_angles, left_angles, steps_per_segment,
                                  synchronized, interpolation_method):
        """Plan smooth trajectory for both arms"""
        planner = TrajectoryPlannerDualArm(interpolation_method=interpolation_method)

        result = planner.plan_dual_arm_trajectory(
            right_angles,
            left_angles,
            num_steps_per_segment=steps_per_segment,
            synchronized=synchronized
        )

        self.node.get_logger().info(f'✅ Trajectory planned: {result["right_arm"]["num_points"]} points')
        return result

    def execute_trajectory(self, trajectory):
        """Execute the planned trajectory by sending position commands"""
        if not self.hardware_manager:
            self.node.get_logger().warning('[SIM] Would execute trajectory')
            time.sleep(2.0)  # Simulate execution
            return True

        self.publish_status(f"executing_{self.gesture_name}", True)

        right_traj = trajectory['right_arm']['trajectory']
        left_traj = trajectory['left_arm']['trajectory']
        total_points = max(len(right_traj), len(left_traj))

        self.node.get_logger().info(f'▶️ Executing trajectory: {total_points} points')

        for i in range(total_points):
            # Get current joint angles
            right_angles = right_traj[min(i, len(right_traj) - 1)]
            left_angles = left_traj[min(i, len(left_traj) - 1)]

            # Send position commands to motors
            position_pairs = []

            for j in range(4):
                # Right arm
                if j < len(right_angles):
                    right_motor_id = self.right_motor_ids[j]
                    position_pairs.append((right_motor_id, right_angles[j]))

                # Left arm
                if j < len(left_angles):
                    left_motor_id = self.left_motor_ids[j]
                    position_pairs.append((left_motor_id, left_angles[j]))

            # Send to hardware
            try:
                self.hardware_manager.set_goal_position(*position_pairs)
            except Exception as e:
                self.node.get_logger().error(f'Error sending positions: {e}')
                return False

            # Log progress
            if i % 10 == 0:
                progress = (i / total_points) * 100
                self.node.get_logger().info(f'📊 Progress: {progress:.1f}%')

            # Small delay between points
            time.sleep(0.05)  # 50ms between waypoints

        self.node.get_logger().info('✅ Trajectory execution complete')
        return True

    def publish_status(self, status_str: str, is_executing: bool):
        """Publish current execution status to topics"""
        if self.status_pub:
            status_msg = String()
            status_msg.data = status_str
            self.status_pub.publish(status_msg)

        if self.executing_pub:
            executing_msg = Bool()
            executing_msg.data = is_executing
            self.executing_pub.publish(executing_msg)

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Cleanup when behavior terminates"""
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info(f'✅ Gesture execution completed: {self.gesture_name}')
        elif new_status == py_trees.common.Status.FAILURE:
            self.node.get_logger().warning(f'⚠️ Gesture execution failed: {self.gesture_name}')
        else:
            self.node.get_logger().info(f'🛑 Gesture execution interrupted: {self.gesture_name}')

        # Publish idle status
        self.publish_status("idle", False)

        # Clear all state to prevent executing old gestures on next activation
        self.pending_gesture = None
        self.gesture_name = None
        self.running = False
        self.is_active = False  # Mark behavior as inactive

        # Only destroy node if we created it
        if self.own_node and self.node:
            self.node.destroy_node()
