#!/usr/bin/env python3
"""
Autonomous Gesture Execution with Current Control

Behavior that executes predefined gestures using PD current control instead of position control.
Similar to autonomous_gesture_execution.py but uses the same control law as remote_teleoperation.py.
All hardware access is done in a single sequential function to avoid USB serial port racing.
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


class AutonomousGestureCurrentControl(py_trees.behaviour.Behaviour):
    """
    Executes gestures using PD current control with sequential hardware access.
    Uses the same control law as remote_teleoperation.py for consistency.
    """

    def __init__(self, name: str, gesture_name=None, node=None, hardware_manager=None):
        super().__init__(name)
        self.node = node
        self.own_node = False
        self.hardware_manager = hardware_manager

        # Motor IDs mapping (consistent with other behaviors)
        # Right arm: motors 1,2,3,4 | Left arm: motors 5,6,7,8
        self.right_motor_ids = [1, 2, 3, 4]
        self.left_motor_ids = [5, 6, 7, 8]
        self.motor_ids = self.right_motor_ids + self.left_motor_ids
        self.available_motors = []

        # Gesture management
        self.gesture_name = gesture_name
        self.pending_gesture = None

        # IK solver and trajectory planner
        self.ik_solver = None
        self.trajectory_planner = None

        # Directories
        self.gestures_directory = None
        self.trajectories_directory = None

        # Publishers and subscribers
        self.status_pub = None
        self.executing_pub = None
        self.gesture_command_sub = None
        self.gesture_control_sub = None

        # Execution state
        self.execution_started = False
        self.execution_complete = False
        self.execution_success = False
        self.running = False
        self.is_active = False
        self._success_logged = False  # Flag to log success only once

        # Loop control
        self.loop_mode = False
        self.loop_count = 0
        self.loop_start_time = None
        self.loop_max_count = None
        self.loop_timeout = 300.0
        self._loop_set_by_topic = False

        # PD Control parameters (from remote_teleoperation.py)
        self.kp = 1.0              # Proportional gain
        self.kp_motor7 = 0.5       # Specific gain for motor 7
        self.kd = 0.1              # Damping gain
        self.Kt = 0.35             # Torque constant
        self.max_current = 5.0     # Maximum current limit (A)

        # Nonlinear PD parameters
        self.r1 = 0.4
        self.r2 = 0.3
        self.p1 = (2*self.r2 - self.r1) / self.r1

        # Velocity estimator parameters
        self.Fc = 35               # Frequency cutoff
        self.Tl = 0.010            # Loop period (100Hz) - matches min_control_period

        # Velocity estimator variables (one per motor)
        self.theta_estimators = [0.0] * 8
        self.vel_estimators = [0.0] * 8

        # Motor state
        self.current_positions = [0.0] * 8
        self.current_velocities = [0.0] * 8
        self.target_positions = [0.0] * 8

        # Control loop timing
        self.default_total_duration = 5.0  # default total gesture duration in seconds
        self.min_control_period = 0.010  # minimum time between control iterations (100Hz max) - reduced to avoid USB overload
        self.time_per_point = 0.025  # will be calculated dynamically based on total_duration

        # Smooth transition parameters
        self.transition_threshold = 0.05  # radians (~2.8 degrees) - threshold to trigger transition
        self.transition_points = 30  # number of interpolation points for smooth transition
        self.transition_duration = 0.5  # seconds for transition between gestures

        # Loop mode hold duration (time to hold position between loop iterations)
        self.loop_hold_duration = 0.1  # seconds to hold final position before restarting in loop mode

    def setup(self):
        """Initialize the behavior"""
        if self.node is None:
            self.node = rclpy.create_node('autonomous_gesture_current_control')
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
            return True

        # Initialize trajectory planner
        self.trajectory_planner = TrajectoryPlannerDualArm()

        # Set gestures and trajectories directories
        # For gestures: prioritize source directory for fast development, fallback to install
        source_gestures_dir = os.path.expanduser('~/smilei_ws/src/smilei_dual_arm_ik/config/gestures')
        if os.path.exists(source_gestures_dir):
            self.gestures_directory = source_gestures_dir
            self.node.get_logger().info(f'📂 Using SOURCE gestures directory: {source_gestures_dir}')
        else:
            self.gestures_directory = os.path.join(pkg_share, 'config', 'gestures')
            self.node.get_logger().info(f'📂 Using INSTALL gestures directory: {self.gestures_directory}')

        # For trajectories: use install directory (cache is temporary)
        self.trajectories_directory = os.path.join(pkg_share, 'config', 'trajectories')
        os.makedirs(self.trajectories_directory, exist_ok=True)

        # Subscribe to gesture command topic
        self.gesture_command_sub = self.node.create_subscription(
            String,
            '/gesture_command_current',  # Different topic to avoid conflict
            self.gesture_command_callback,
            10
        )

        # Subscribe to gesture control topic (loop control)
        self.gesture_control_sub = self.node.create_subscription(
            String,
            '/gesture_control_current',  # Different topic to avoid conflict
            self.gesture_control_callback,
            10
        )

        # Create publishers
        self.status_pub = self.node.create_publisher(
            String,
            '/gesture_execution_status_current',
            10
        )

        self.executing_pub = self.node.create_publisher(
            Bool,
            '/gesture_executing_current',
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

        self.node.get_logger().info('Autonomous Gesture Current Control behavior setup complete')
        return True

    def gesture_command_callback(self, msg):
        """Callback for receiving gesture commands via topic"""
        self.node.get_logger().info(f'📨 Received gesture command: {msg.data}')

        if self.is_active:
            self.node.get_logger().info(f'⚡ Behavior active - executing gesture immediately')
            self.gesture_name = msg.data
            self.execution_started = False
            self.execution_complete = False
            self.execution_success = False
            self._success_logged = False
            self.running = True
        else:
            self.node.get_logger().info(f'💤 Behavior inactive - gesture ignored')

    def gesture_control_callback(self, msg):
        """Callback for gesture control commands (loop control)"""
        command = msg.data.lower()

        if command == 'loop':
            self.loop_mode = True
            self._loop_set_by_topic = True
            if self.loop_start_time is None:
                self.loop_start_time = time.time()
                self.loop_count = 0
            self.node.get_logger().info('🔄 Loop mode ENABLED via topic - gesture will repeat')

        elif command == 'once':
            self.loop_mode = False
            self._loop_set_by_topic = True
            self.loop_count = 0
            self.loop_start_time = None
            self.node.get_logger().info('▶️ Single execution mode via topic - gesture will execute once')

        elif command == 'stop':
            self.loop_mode = False
            self.running = False
            self._loop_set_by_topic = False
            self.loop_count = 0
            self.loop_start_time = None
            self.node.get_logger().info('⏹️ Stop command received - stopping gesture execution')

        else:
            self.node.get_logger().warning(f'⚠️ Unknown control command: {command} (use: loop/once/stop)')

    def initialise(self):
        """Called when behavior is activated"""
        self.node.get_logger().info(f'Initializing Autonomous Gesture Current Control: {self.gesture_name}')
        self.is_active = True
        self.running = False
        self.execution_started = False
        self.execution_complete = False
        self.execution_success = False
        self._success_logged = False

    def update(self):
        """Main behavior update loop"""
        # If no gesture is set, return RUNNING
        if not self.gesture_name:
            return py_trees.common.Status.RUNNING

        # Start gesture execution
        if not self.execution_started and not self.execution_complete:
            self.start_gesture_execution()

        # Check if execution is complete
        if self.execution_complete:
            # Execution complete - check if we should loop or finish
            if self.execution_success:
                # Log success only once
                if not self._success_logged:
                    self.node.get_logger().info(f'✅ Gesture "{self.gesture_name}" executed successfully!')
                    self.publish_status(f"completed_{self.gesture_name}_success", False)
                    self._success_logged = True

                # Check if we should loop
                if self.loop_mode:
                    self.loop_count += 1

                    # Safety check: max iteration count
                    if self.loop_max_count is not None and self.loop_count >= self.loop_max_count:
                        self.node.get_logger().info(f'🏁 Max loop count reached ({self.loop_max_count}) - stopping')
                        return py_trees.common.Status.SUCCESS

                    # Safety check: timeout
                    if self.loop_start_time is not None:
                        elapsed_time = time.time() - self.loop_start_time
                        if elapsed_time >= self.loop_timeout:
                            self.node.get_logger().warning(f'⏱️ Loop timeout reached ({self.loop_timeout}s) - stopping for safety')
                            return py_trees.common.Status.SUCCESS

                    self.node.get_logger().info(f'🔄 Loop mode active - repeating gesture (iteration {self.loop_count})')

                    # Reset execution flags to restart the gesture
                    self.execution_started = False
                    self.execution_complete = False
                    self.execution_success = False
                    self._success_logged = False  # Reset for next iteration

                    # Return RUNNING to continue the loop
                    return py_trees.common.Status.RUNNING

                return py_trees.common.Status.SUCCESS
            else:
                self.node.get_logger().error(f'❌ Gesture "{self.gesture_name}" execution failed')
                self.publish_status(f"completed_{self.gesture_name}_failed", False)
                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Called when behavior is terminated"""
        self.node.get_logger().info(f'Terminating Autonomous Gesture Current Control: {new_status}')
        self.is_active = False
        self.running = False  # This will stop the holding loop

        if self.hardware_manager:
            # Send zero currents to stop motors
            self.node.get_logger().info('⏹️ Stopping current control - sending zero currents')
            try:
                zero_currents = [(motor_id, 0.0) for motor_id in self.motor_ids]
                self.hardware_manager.set_goal_iq(*zero_currents)
            except Exception as e:
                self.node.get_logger().error(f'Error sending zero currents: {e}')

            # Restore position control mode
            self.restore_position_control()

    def publish_status(self, status, is_executing):
        """Publish execution status"""
        if self.status_pub:
            msg = String()
            msg.data = status
            self.status_pub.publish(msg)

        if self.executing_pub:
            msg = Bool()
            msg.data = is_executing
            self.executing_pub.publish(msg)

    def start_gesture_execution(self):
        """Load and execute the gesture"""
        try:
            self.node.get_logger().info(f'🚀 Starting gesture: {self.gesture_name}')
            self.running = True
            self.execution_started = True  # Mark as started at the beginning
            self.publish_status(f"starting_{self.gesture_name}", True)

            # Setup motors for current control mode
            if not self.setup_current_control():
                self.node.get_logger().error('Failed to setup current control mode')
                self.execution_started = True
                self.execution_complete = True
                self.execution_success = False
                return py_trees.common.Status.RUNNING

            # Load gesture configuration
            gesture_config = self.load_gesture_config(self.gesture_name)
            if gesture_config is None:
                self.execution_started = True
                self.execution_complete = True
                self.execution_success = False
                return py_trees.common.Status.RUNNING

            # Read total duration for smooth timing
            self.gesture_total_duration = gesture_config.get('total_duration', self.default_total_duration)

            # Read loop parameters from YAML (only if not already controlled via topic)
            if not hasattr(self, '_loop_set_by_topic') or not self._loop_set_by_topic:
                yaml_loop_mode = gesture_config.get('loop', False)
                yaml_loop_count = gesture_config.get('loop_count', None)
                yaml_loop_timeout = gesture_config.get('loop_timeout', 300.0)

                if yaml_loop_mode and not self.loop_mode:
                    self.loop_mode = True
                    self.loop_max_count = yaml_loop_count
                    self.loop_timeout = yaml_loop_timeout
                    if self.loop_start_time is None:
                        self.loop_start_time = time.time()
                        self.loop_count = 0
                    self.node.get_logger().info(f'📄 YAML loop config: enabled, max_count={yaml_loop_count}, timeout={yaml_loop_timeout}s')

            # Try to load cached trajectory first
            trajectory = self.load_trajectory_cache(self.gesture_name)

            if trajectory is not None:
                # Cached trajectory found - use it directly!
                self.node.get_logger().info(f'⚡ Using cached trajectory - skipping IK/planning')
            else:
                # No cached trajectory - compute it
                self.node.get_logger().info(f'🔧 Computing new trajectory...')

                # Check waypoint format
                control_mode = gesture_config.get('control_mode', 'cartesian')
                self.node.get_logger().info(f'📋 Waypoint Format: {control_mode.upper()}')

                # Branch based on waypoint format
                if control_mode == 'joint':
                    self.node.get_logger().info('📐 Extracting joint angles directly from waypoints')
                    right_angles, left_angles = self.extract_joint_angles_from_gesture(gesture_config)

                elif control_mode == 'cartesian':
                    self.node.get_logger().info('🗺️  Solving IK to convert cartesian waypoints to joint angles')
                    right_angles, left_angles = self.solve_ik_for_gesture(gesture_config)

                else:
                    self.node.get_logger().error(f'Unknown control mode: {control_mode}')
                    self.execution_started = True
                    self.execution_complete = True
                    self.execution_success = False
                    return py_trees.common.Status.RUNNING

                # Check if we have at least 1 valid waypoint for either arm
                if len(right_angles) < 1 and len(left_angles) < 1:
                    self.node.get_logger().error('No valid IK solutions found')
                    self.execution_started = True
                    self.execution_complete = True
                    self.execution_success = False
                    return py_trees.common.Status.RUNNING

                # Handle single waypoint case
                if len(right_angles) == 1:
                    self.node.get_logger().info('Right arm has 1 waypoint - duplicating for smooth motion')
                    right_angles.append(right_angles[0])

                if len(left_angles) == 1:
                    self.node.get_logger().info('Left arm has 1 waypoint - duplicating for smooth motion')
                    left_angles.append(left_angles[0])

                # If one arm has no waypoints, maintain current position
                if len(right_angles) == 0:
                    self.node.get_logger().info('Right arm has no waypoints - holding current position')
                    current_right = np.zeros(4)
                    if self.hardware_manager is not None:
                        try:
                            positions = self.hardware_manager.get_present_position(*self.right_motor_ids)
                            if len(positions) == 4:
                                current_right = np.array(positions)
                        except Exception as e:
                            self.node.get_logger().warning(f'Could not read current right arm position: {e}')
                    right_angles = [current_right, current_right]

                if len(left_angles) == 0:
                    self.node.get_logger().info('Left arm has no waypoints - holding current position')
                    current_left = np.zeros(4)
                    if self.hardware_manager is not None:
                        try:
                            positions = self.hardware_manager.get_present_position(*self.left_motor_ids)
                            if len(positions) == 4:
                                current_left = np.array(positions)
                        except Exception as e:
                            self.node.get_logger().warning(f'Could not read current left arm position: {e}')
                    left_angles = [current_left, current_left]

                # Plan trajectory
                trajectory = self.plan_dual_arm_trajectory(
                    right_angles,
                    left_angles,
                    gesture_config.get('steps_per_segment', 50),
                    gesture_config.get('synchronized', False),
                    gesture_config.get('interpolation_method', 'cubic')
                )

                # Save trajectory to cache for future use
                self.save_trajectory_cache(self.gesture_name, trajectory)

            # Execute trajectory using current control
            success = self.execute_trajectory_with_current_control(trajectory)

            # Check if execution was interrupted by new gesture
            # (callback would have set execution_started = False)
            if not success and not self.execution_started:
                # New gesture pending - don't mark as complete, allow restart
                self.node.get_logger().info('🔄 Holding interrupted by new gesture - ready to restart')
                return py_trees.common.Status.RUNNING

            # Normal completion (success or failure) - mark as complete
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

    def load_trajectory_cache(self, gesture_name):
        """Load cached trajectory from file if it exists"""
        trajectory_file = os.path.join(self.trajectories_directory, f'{gesture_name}_trajectory.yaml')

        if not os.path.exists(trajectory_file):
            return None

        try:
            with open(trajectory_file, 'r') as f:
                cached_data = yaml.safe_load(f)

            # Convert lists back to numpy arrays
            trajectory = {
                'right_arm': {
                    'trajectory': [np.array(point) for point in cached_data['right_arm_trajectory']],
                    'num_points': cached_data['num_points']
                },
                'left_arm': {
                    'trajectory': [np.array(point) for point in cached_data['left_arm_trajectory']],
                    'num_points': cached_data['num_points']
                }
            }

            self.node.get_logger().info(f'📦 Loaded cached trajectory for: {gesture_name} ({cached_data["num_points"]} points)')
            return trajectory
        except Exception as e:
            self.node.get_logger().warning(f'Failed to load cached trajectory: {e}')
            return None

    def save_trajectory_cache(self, gesture_name, trajectory):
        """Save computed trajectory to cache file"""
        cache_data = {
            'gesture_name': gesture_name,
            'generated': time.strftime('%Y-%m-%d %H:%M:%S'),
            'num_points': trajectory['right_arm']['num_points'],
            'right_arm_trajectory': [point.tolist() for point in trajectory['right_arm']['trajectory']],
            'left_arm_trajectory': [point.tolist() for point in trajectory['left_arm']['trajectory']]
        }

        saved_count = 0

        # Save to install directory
        install_file = os.path.join(self.trajectories_directory, f'{gesture_name}_trajectory.yaml')
        try:
            with open(install_file, 'w') as f:
                yaml.dump(cache_data, f, default_flow_style=False)
            self.node.get_logger().info(f'💾 Saved to install: {install_file}')
            saved_count += 1
        except Exception as e:
            self.node.get_logger().error(f'Failed to save to install directory: {e}')

        # Save to source directory (for version control)
        if '/install/' in self.trajectories_directory:
            source_trajectories_dir = self.trajectories_directory.replace('/install/', '/src/').replace('/share/smilei_dual_arm_ik/', '/')
            source_file = os.path.join(source_trajectories_dir, f'{gesture_name}_trajectory.yaml')

            try:
                os.makedirs(source_trajectories_dir, exist_ok=True)
                with open(source_file, 'w') as f:
                    yaml.dump(cache_data, f, default_flow_style=False)
                self.node.get_logger().info(f'💾 Saved to source: {source_file}')
                saved_count += 1
            except Exception as e:
                self.node.get_logger().warning(f'Failed to save to source directory: {e}')

        return saved_count > 0

    def solve_ik_for_gesture(self, gesture_config):
        """Solve IK for all waypoints in the gesture"""
        right_waypoints = gesture_config.get('right_arm_waypoints', [])
        left_waypoints = gesture_config.get('left_arm_waypoints', [])

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

    def extract_joint_angles_from_gesture(self, gesture_config):
        """Extract joint angles directly from joint mode gesture (no IK needed)"""
        right_waypoints = gesture_config.get('right_arm_waypoints', [])
        left_waypoints = gesture_config.get('left_arm_waypoints', [])

        right_joint_angles = []
        left_joint_angles = []

        # Extract right arm joint angles
        for waypoint in right_waypoints:
            joints = waypoint.get('joints', [])
            if len(joints) == 4:
                right_joint_angles.append(np.array(joints))
            else:
                self.node.get_logger().warning(f'Invalid right arm waypoint (expected 4 joints, got {len(joints)})')

        # Extract left arm joint angles
        for waypoint in left_waypoints:
            joints = waypoint.get('joints', [])
            if len(joints) == 4:
                left_joint_angles.append(np.array(joints))
            else:
                self.node.get_logger().warning(f'Invalid left arm waypoint (expected 4 joints, got {len(joints)})')

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

    def execute_trajectory_with_current_control(self, trajectory):
        """
        Execute trajectory using PD current control.
        ALL hardware access is done sequentially in this function to avoid USB serial racing.
        Based on remote_teleoperation.py control loop behavior.
        """
        if not self.hardware_manager:
            self.node.get_logger().warning('[SIM] Would execute trajectory with current control')
            time.sleep(2.0)
            return True

        self.publish_status(f"executing_{self.gesture_name}", True)

        right_traj = trajectory['right_arm']['trajectory']
        left_traj = trajectory['left_arm']['trajectory']
        total_points = max(len(right_traj), len(left_traj))

        # Calculate time per point based on total gesture duration
        total_duration = getattr(self, 'gesture_total_duration', self.default_total_duration)
        self.time_per_point = total_duration / max(total_points, 1)  # Avoid division by zero

        self.node.get_logger().info(f'▶️ Starting trajectory execution with CURRENT CONTROL')

        # Initialize velocity estimators and get initial motor state
        try:
            positions = self.hardware_manager.get_present_position(*self.motor_ids)
            velocities = self.hardware_manager.get_present_velocity(*self.motor_ids)

            if len(positions) == 8 and len(velocities) == 8:
                self.current_positions = positions[:]
                self.current_velocities = velocities[:]
                # Initialize estimators with current state
                self.theta_estimators = [0.0] * 8
                self.vel_estimators = velocities[:]  # Start with actual velocities
                self.node.get_logger().info(f'📍 Initial positions: {[f"{p:.3f}" for p in positions]}')
            else:
                self.node.get_logger().error('Failed to get initial motor states')
                return False
        except Exception as e:
            self.node.get_logger().error(f'Error getting initial state: {e}')
            return False

        # Check if smooth transition is needed from current position to first trajectory point
        first_right = right_traj[0] if len(right_traj) > 0 else None
        first_left = left_traj[0] if len(left_traj) > 0 else None

        if first_right is not None and first_left is not None:
            # Calculate maximum position error between current and first target
            max_position_error = 0.0
            for i in range(4):
                if i < len(first_right):
                    error_right = abs(self.current_positions[i] - first_right[i])
                    max_position_error = max(max_position_error, error_right)
                if i < len(first_left):
                    error_left = abs(self.current_positions[i + 4] - first_left[i])
                    max_position_error = max(max_position_error, error_left)

            # If error exceeds threshold, add smooth transition
            if max_position_error > self.transition_threshold:
                self.node.get_logger().info(
                    f'🔄 Transition needed: max error {max_position_error:.3f} rad ({max_position_error*57.3:.1f}°) '
                    f'> threshold {self.transition_threshold:.3f} rad'
                )

                # Current positions as numpy arrays
                current_right = np.array(self.current_positions[0:4])
                current_left = np.array(self.current_positions[4:8])

                # Generate smooth transition points using linear interpolation
                transition_right = []
                transition_left = []

                for i in range(1, self.transition_points + 1):
                    alpha = i / self.transition_points  # 0 to 1
                    trans_right = current_right + alpha * (first_right - current_right)
                    trans_left = current_left + alpha * (first_left - current_left)
                    transition_right.append(trans_right)
                    transition_left.append(trans_left)

                # Insert transition at beginning of trajectory
                right_traj = transition_right + right_traj
                left_traj = transition_left + left_traj
                total_points = max(len(right_traj), len(left_traj))

                # Recalculate time per point accounting for transition time
                # Transition gets its own time budget, rest gets gesture time
                gesture_points = total_points - self.transition_points
                total_time = self.transition_duration + total_duration

                self.node.get_logger().info(
                    f'✅ Added {self.transition_points} transition points '
                    f'({self.transition_duration}s transition + {total_duration}s gesture = {total_time}s total)'
                )

                # Update timing: we'll handle transition and gesture timing separately in the loop
                self.time_per_point = total_time / total_points
            else:
                self.node.get_logger().info(
                    f'✓ No transition needed: max error {max_position_error:.3f} rad ({max_position_error*57.3:.1f}°) '
                    f'< threshold {self.transition_threshold:.3f} rad'
                )

        # Log final trajectory info after potential transition additions
        self.node.get_logger().info(
            f'📊 Executing {total_points} trajectory points | '
            f'Time per point: {self.time_per_point*1000:.1f}ms'
        )

        try:
            for i in range(total_points):
                # Check if execution was stopped
                if not self.running:
                    self.node.get_logger().warning('⏹️ Execution stopped by user command')
                    self.send_zero_currents()
                    return False

                # Get target joint angles for this trajectory point
                right_angles = right_traj[min(i, len(right_traj) - 1)]
                left_angles = left_traj[min(i, len(left_traj) - 1)]

                # Update target positions (8 motors total)
                for j in range(4):
                    if j < len(right_angles):
                        self.target_positions[j] = right_angles[j]  # Motors 1-4
                    if j < len(left_angles):
                        self.target_positions[j + 4] = left_angles[j]  # Motors 5-8

                # Log target for first and every 10th trajectory point
                if i == 0 or i % 10 == 0:
                    self.node.get_logger().info(f'🎯 Point {i}/{total_points}: targets = {[f"{t:.3f}" for t in self.target_positions]}')

                # Control loop for this trajectory point - keep trying until time expires
                point_start_time = time.time()
                iteration_count = 0

                while (time.time() - point_start_time) < self.time_per_point:
                    if not self.running:
                        break

                    iteration_start = time.time()

                    # SEQUENTIAL HARDWARE ACCESS - all in one place to avoid racing
                    success = self.hardware_control_step()
                    if not success:
                        self.node.get_logger().warning(f'Hardware control step failed at point {i}, iteration {iteration_count}')
                        # Continue instead of failing completely

                    iteration_count += 1

                    # Allow ROS2 to process callbacks
                    rclpy.spin_once(self.node, timeout_sec=0.0001)

                    # Minimal delay to avoid overwhelming USB
                    elapsed = time.time() - iteration_start
                    if elapsed < self.min_control_period:
                        time.sleep(self.min_control_period - elapsed)

                # Log progress with iteration count
                if i % 5 == 0:
                    progress = (i / total_points) * 100
                    self.node.get_logger().info(f'📊 Progress: {progress:.1f}% (point {i}/{total_points}, {iteration_count} control iterations)')

        except Exception as e:
            self.node.get_logger().error(f'Error during trajectory execution: {e}')
            import traceback
            self.node.get_logger().error(traceback.format_exc())
            self.send_zero_currents()
            return False

        # Hold final position with CURRENT CONTROL (don't switch to position control)
        if self.loop_mode:
            self.node.get_logger().info(
                f'🔒 Holding final position with current control for {self.loop_hold_duration}s before loop restart: {[f"{p:.3f}" for p in self.target_positions]}'
            )
        else:
            self.node.get_logger().info(
                f'🔒 Holding final position with current control: {[f"{p:.3f}" for p in self.target_positions]}'
            )

        # Keep the current control loop active with fixed targets until:
        # - New gesture arrives (execution_started becomes False via callback)
        # - Behavior is terminated (running becomes False)
        # - Loop mode: hold duration elapsed (time to restart gesture)
        hold_iteration = 0
        hold_start_time = time.time()
        new_gesture_pending = False
        loop_restart_needed = False

        try:
            # Hold while no new gesture is pending and still running
            while self.running and self.execution_started:
                iteration_start = time.time()

                # Check if loop mode and hold duration elapsed
                if self.loop_mode:
                    hold_elapsed = time.time() - hold_start_time
                    if hold_elapsed >= self.loop_hold_duration:
                        self.node.get_logger().info(f'🔄 Loop hold duration ({self.loop_hold_duration}s) elapsed - restarting gesture')
                        loop_restart_needed = True
                        break

                # Continue hardware control with fixed target positions
                success = self.hardware_control_step()
                if not success:
                    # Continue even if control step fails occasionally
                    pass

                hold_iteration += 1

                # Log holding status every 5 seconds
                if hold_iteration % 500 == 0:  # 500 iterations × 10ms = 5 seconds
                    hold_elapsed = time.time() - hold_start_time
                    if self.loop_mode:
                        self.node.get_logger().info(
                            f'💤 Holding position in loop mode ({hold_elapsed:.1f}/{self.loop_hold_duration}s, {hold_iteration} iterations)'
                        )
                    else:
                        self.node.get_logger().info(
                            f'💤 Still holding position (current control active, {hold_iteration} iterations)'
                        )

                # Allow ROS2 to process callbacks (including new gesture commands)
                rclpy.spin_once(self.node, timeout_sec=0.0001)

                # Maintain control loop timing
                elapsed = time.time() - iteration_start
                if elapsed < self.min_control_period:
                    time.sleep(self.min_control_period - elapsed)

        except Exception as e:
            self.node.get_logger().error(f'Error during position holding: {e}')
            import traceback
            self.node.get_logger().error(traceback.format_exc())
            return False

        # Check why we exited holding mode
        if loop_restart_needed:
            # Loop mode: hold duration elapsed
            # Return True to mark as complete - update() will handle loop restart with safety checks
            self.node.get_logger().info(f'🔁 Loop hold complete - returning to update() for restart')
            return True
        elif not self.execution_started:
            # New gesture arrived (callback set execution_started = False)
            self.node.get_logger().info(f'🔄 New gesture "{self.gesture_name}" pending - exiting hold mode')
            new_gesture_pending = True
        elif not self.running:
            # Termination requested (could be stop command)
            self.node.get_logger().info('⏹️ Termination requested - exiting hold mode')

        # Return False if new gesture is pending (don't mark current gesture as complete)
        # This allows update() to restart execution with the new gesture
        if new_gesture_pending:
            return False  # Don't mark as complete - new gesture needs to execute
        else:
            return True  # Normal completion or termination

    def hardware_control_step(self):
        """
        Single sequential hardware control step.
        Reads motor states, calculates control currents, and sends commands.
        ALL hardware access happens here to avoid USB serial port racing.
        """
        try:
            # STEP 1: Read current motor positions and velocities (1 USB transaction)
            positions = self.hardware_manager.get_present_position(*self.motor_ids)
            velocities = self.hardware_manager.get_present_velocity(*self.motor_ids)

            if len(positions) != 8 or len(velocities) != 8:
                self.node.get_logger().warning(f'Invalid state: got {len(positions)} positions, {len(velocities)} velocities')
                return False

            self.current_positions = positions[:]
            self.current_velocities = velocities[:]

            # STEP 2: Calculate control currents using PD control law
            currents = self.calculate_control_currents()

            # Debug logging every 50 iterations
            if not hasattr(self, '_control_step_count'):
                self._control_step_count = 0
            self._control_step_count += 1

            if self._control_step_count % 50 == 0:
                # Log state for first 2 motors as example
                for i in range(2):
                    motor_id = self.motor_ids[i]
                    pos = positions[i]
                    target = self.target_positions[i]
                    error = pos - target
                    current = currents[i]
                    self.node.get_logger().info(
                        f'M{motor_id}: pos={pos:.3f}, target={target:.3f}, '
                        f'error={error:.3f}, current={current:.3f}A'
                    )

            # STEP 3: Send current commands (1 USB transaction)
            current_pairs = [(motor_id, currents[idx]) for idx, motor_id in enumerate(self.motor_ids)]
            self.hardware_manager.set_goal_iq(*current_pairs)

            return True

        except Exception as e:
            self.node.get_logger().error(f'Hardware control step error: {e}')
            import traceback
            self.node.get_logger().error(traceback.format_exc())
            return False

    def calculate_control_currents(self):
        """
        Calculate PD control currents for all motors.
        Uses the same control law as remote_teleoperation.py.
        """
        currents = []

        for i, motor_id in enumerate(self.motor_ids):
            # Get current and target positions
            current_pos = self.current_positions[i]
            target_pos = self.target_positions[i]

            # Error calculation
            error = current_pos - target_pos

            # Velocity estimation (from remote_teleoperation.py)
            self.vel_estimators[i] = self.Fc * (self.theta_estimators[i] + current_pos)
            self.theta_estimators[i] = self.theta_estimators[i] - self.Tl * self.vel_estimators[i]
            vel_estimate = self.vel_estimators[i]

            # Select kp value (motor 7 has different gain)
            kp_value = self.kp_motor7 if motor_id == 7 else self.kp

            # Nonlinear PD control law (same as remote_teleoperation.py)
            tau = -kp_value * ((abs(error)**self.p1) * np.sign(error)) - self.kd * vel_estimate

            # Convert torque to current
            current = tau / self.Kt

            # Apply current limits
            current = max(-self.max_current, min(self.max_current, current))

            currents.append(current)

        return currents

    def setup_current_control(self):
        """
        Setup motors for current control mode.
        Based on remote_teleoperation.py setup_current_control().
        """
        try:
            if not self.hardware_manager:
                self.node.get_logger().info('[SIM] Setting up current control mode')
                return True

            self.node.get_logger().info('⚙️ Configuring motors for CURRENT CONTROL mode...')

            for motor_id in self.motor_ids:
                # Set PID gains for current control (from remote_teleoperation.py)
                self.hardware_manager.set_p_gain_iq((motor_id, 0.277))
                self.hardware_manager.set_i_gain_iq((motor_id, 0.061))
                self.hardware_manager.set_d_gain_iq((motor_id, 0))
                self.hardware_manager.set_p_gain_id((motor_id, 0.277))
                self.hardware_manager.set_i_gain_id((motor_id, 0.061))
                self.hardware_manager.set_d_gain_id((motor_id, 0))

                # Set to current mode (mode 0)
                self.hardware_manager.set_mode((motor_id, 0))

                # Enable torque
                self.hardware_manager.set_torque_enable((motor_id, 1))

            self.node.get_logger().info('✅ Motors configured for current control (mode 0)')
            return True

        except Exception as e:
            self.node.get_logger().error(f'Error setting up current control: {e}')
            return False

    def restore_position_control(self):
        """
        Restore position control mode after current control execution.
        Based on remote_teleoperation.py restore_position_control().
        """
        try:
            if not self.hardware_manager:
                self.node.get_logger().info('[SIM] Restoring position control mode')
                return

            self.node.get_logger().info('⚙️ Restoring POSITION CONTROL mode...')

            # Restore PID gains for position control
            if hasattr(self.hardware_manager, 'configure_pid_gains'):
                self.hardware_manager.configure_pid_gains(self.motor_ids, p_gain=5.0, i_gain=0.0, d_gain=0.2)

            # Change to position mode (mode 2)
            self.hardware_manager.set_mode(*[(motor_id, 2) for motor_id in self.motor_ids])

            self.node.get_logger().info('✅ Motors restored to position control (mode 2)')

        except Exception as e:
            self.node.get_logger().error(f'Error restoring position control: {e}')

    def send_zero_currents(self):
        """Send zero currents to all motors"""
        try:
            zero_currents = [(motor_id, 0.0) for motor_id in self.motor_ids]
            self.hardware_manager.set_goal_iq(*zero_currents)
        except Exception as e:
            self.node.get_logger().error(f'Error sending zero currents: {e}')
