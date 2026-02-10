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
from datetime import datetime

# Import IK and trajectory planning
from smilei_dual_arm_ik.inverse_kinematics_dual_arm import InverseKinematicsDualArm
from smilei_dual_arm_ik.trajectory_planner_dual_arm import TrajectoryPlannerDualArm


class PerformanceMetrics:
    """Class to track and store performance metrics for gesture execution"""

    def __init__(self):
        self.reset()

    def reset(self):
        """Reset all metrics for a new execution"""
        # Common metrics
        self.gesture_name = None
        self.control_mode = None  # 'joint' or 'cartesian'
        self.num_waypoints = 0
        self.num_interpolated_points = 0
        self.execution_timestamp = None
        self.from_cache = False

        # Timing metrics
        self.ik_solve_time = 0.0  # Only for cartesian mode
        self.trajectory_planning_time = 0.0
        self.cache_load_time = 0.0
        self.total_initialization_time = 0.0
        self.trajectory_execution_time = 0.0

        # Cartesian mode specific
        self.ik_success_count = 0
        self.ik_failed_count = 0
        self.end_effector_position_error = None  # Will be computed if hardware available
        self.dual_arm_sync_error = None

        # Joint mode specific
        self.max_joint_velocity = 0.0
        self.max_joint_acceleration = 0.0
        self.joint_tracking_error_rms = 0.0
        self.joint_tracking_error_max = 0.0

        # Smoothness metrics
        self.velocity_discontinuities = 0

    def to_dict(self):
        """Convert metrics to dictionary for YAML serialization"""
        # Helper function to convert numpy types to native Python types
        def convert_value(val):
            if val is None:
                return None
            # Check if it's a numpy type
            if hasattr(val, 'item'):  # numpy scalar
                return float(val.item())
            elif isinstance(val, np.ndarray):
                return val.tolist()
            elif isinstance(val, (np.integer, np.floating)):
                return float(val)
            else:
                return val

        return {
            'gesture_name': self.gesture_name,
            'control_mode': self.control_mode,
            'execution_timestamp': self.execution_timestamp,
            'from_cache': self.from_cache,
            'waypoints': {
                'num_waypoints': int(convert_value(self.num_waypoints)),
                'num_interpolated_points': int(convert_value(self.num_interpolated_points))
            },
            'timing_ms': {
                'ik_solve_time': round(float(convert_value(self.ik_solve_time)) * 1000, 2),
                'trajectory_planning_time': round(float(convert_value(self.trajectory_planning_time)) * 1000, 2),
                'cache_load_time': round(float(convert_value(self.cache_load_time)) * 1000, 2),
                'total_initialization_time': round(float(convert_value(self.total_initialization_time)) * 1000, 2),
                'trajectory_execution_time': round(float(convert_value(self.trajectory_execution_time)) * 1000, 2)
            },
            'cartesian_metrics': {
                'ik_success_count': int(convert_value(self.ik_success_count)),
                'ik_failed_count': int(convert_value(self.ik_failed_count)),
                'end_effector_position_error_cm': round(float(convert_value(self.end_effector_position_error)), 2) if self.end_effector_position_error else None,
                'dual_arm_sync_error_ms': round(float(convert_value(self.dual_arm_sync_error)) * 1000, 2) if self.dual_arm_sync_error else None
            } if self.control_mode == 'cartesian' else None,
            'joint_metrics': {
                'max_joint_velocity_rad_s': round(float(convert_value(self.max_joint_velocity)), 3),
                'max_joint_acceleration_rad_s2': round(float(convert_value(self.max_joint_acceleration)), 3),
                'joint_tracking_error_rms_percent': round(float(convert_value(self.joint_tracking_error_rms)), 2),
                'joint_tracking_error_max_percent': round(float(convert_value(self.joint_tracking_error_max)), 2),
                'velocity_discontinuities': int(convert_value(self.velocity_discontinuities))
            } if self.control_mode == 'joint' else None
        }

    def compute_speedup_factor(self, cached_time, uncached_time):
        """Compute speedup factor when using cache"""
        if cached_time > 0:
            return uncached_time / cached_time
        return 0.0


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
        self.trajectories_directory = None
        self.metrics_directory = None

        # Performance tracking
        self.metrics = PerformanceMetrics()
        self.enable_metrics = True  # Set to False to disable metrics collection

        # Publishers for state machine integration
        self.status_pub = None
        self.executing_pub = None
        self.gesture_command_sub = None

        # Execution state
        self.execution_started = False
        self.execution_complete = False
        self.execution_success = False
        self.running = False
        self._success_logged = False  # Flag to log success only once

        # Behavior activation state (separate from gesture execution state)
        self.is_active = False  # True when behavior is in active state, False otherwise

        # Loop control
        self.loop_mode = False  # True = loop gesture, False = execute once
        self.loop_count = 0  # Number of times the gesture has looped
        self.loop_start_time = None  # When the loop started
        self.loop_max_count = None  # Maximum loop iterations (None = infinite)
        self.loop_timeout = 300.0  # Maximum loop duration in seconds (default 5 min)
        self._loop_set_by_topic = False  # Flag to track if loop was set via topic
        self.gesture_control_sub = None  # Subscriber for loop control

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

        # Set gestures and trajectories directories
        # For gestures: prioritize source directory for fast development, fallback to install
        source_gestures_dir = os.path.expanduser('~/smilei_ws/src/smilei_dual_arm_ik/config/gestures')
        if os.path.exists(source_gestures_dir):
            self.gestures_directory = source_gestures_dir
            self.node.get_logger().info(f'📂 Using SOURCE gestures directory: {source_gestures_dir}')
        else:
            self.gestures_directory = os.path.join(pkg_share, 'config', 'gestures')
            self.node.get_logger().info(f'📂 Using INSTALL gestures directory: {self.gestures_directory}')

        # For trajectories: use install directory (cache is temporary, doesn't need version control)
        self.trajectories_directory = os.path.join(pkg_share, 'config', 'trajectories')

        # Create trajectories directory if it doesn't exist
        os.makedirs(self.trajectories_directory, exist_ok=True)

        # For metrics: use source directory for persistence
        source_metrics_dir = os.path.expanduser('~/smilei_ws/src/smilei_state_machine/performance_metrics')
        self.metrics_directory = source_metrics_dir
        os.makedirs(self.metrics_directory, exist_ok=True)
        self.node.get_logger().info(f'📊 Performance metrics directory: {self.metrics_directory}')

        # Subscribe to gesture command topic
        self.gesture_command_sub = self.node.create_subscription(
            String,
            '/gesture_command',
            self.gesture_command_callback,
            10
        )

        # Subscribe to gesture control topic (loop control)
        self.gesture_control_sub = self.node.create_subscription(
            String,
            '/gesture_control',
            self.gesture_control_callback,
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
        self._success_logged = False
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
                    self.gesture_name = None
                    self.running = False
                    self.loop_mode = False
                    self.loop_count = 0
                    self.loop_start_time = None
                    return py_trees.common.Status.SUCCESS

                # Safety check: timeout
                if self.loop_start_time is not None:
                    elapsed_time = time.time() - self.loop_start_time
                    if elapsed_time >= self.loop_timeout:
                        self.node.get_logger().warning(f'⏱️ Loop timeout reached ({self.loop_timeout}s) - stopping for safety')
                        self.gesture_name = None
                        self.running = False
                        self.loop_mode = False
                        self.loop_count = 0
                        self.loop_start_time = None
                        return py_trees.common.Status.SUCCESS

                self.node.get_logger().info(f'🔄 Loop mode active - repeating gesture (iteration {self.loop_count})')

                # Reset execution flags to restart the gesture
                self.execution_started = False
                self.execution_complete = False
                self.execution_success = False
                self._success_logged = False  # Reset for next iteration

                # Return RUNNING to continue the loop
                return py_trees.common.Status.RUNNING
            else:
                # Single execution mode - reset state and finish
                self.gesture_name = None
                self.running = False
                self.loop_count = 0
                self.loop_start_time = None
                return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().error(f'❌ Gesture "{self.gesture_name}" failed')
            self.publish_status(f"completed_{self.gesture_name}_failed", False)
            # Always stop on failure
            self.gesture_name = None
            self.running = False
            self.loop_mode = False
            self.loop_count = 0
            self.loop_start_time = None
            return py_trees.common.Status.FAILURE

    def start_gesture_execution(self):
        """Start gesture execution"""
        self.node.get_logger().info(f'🚀 Starting gesture: {self.gesture_name}')
        self.publish_status(f"starting_{self.gesture_name}", True)

        # Reset metrics for new execution
        if self.enable_metrics:
            self.metrics.reset()
            self.metrics.gesture_name = self.gesture_name
            self.metrics.execution_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

        # Start timing total initialization
        init_start_time = time.time()

        try:
            # Load gesture config to check for loop parameters
            gesture_config = self.load_gesture_config(self.gesture_name)
            if gesture_config is None:
                self.execution_started = True
                self.execution_complete = True
                self.execution_success = False
                return py_trees.common.Status.RUNNING

            # Store control mode for metrics
            control_mode = gesture_config.get('control_mode', 'cartesian')
            if self.enable_metrics:
                self.metrics.control_mode = control_mode

            # Read loop parameters from YAML (only if not already controlled via topic)
            if not hasattr(self, '_loop_set_by_topic') or not self._loop_set_by_topic:
                yaml_loop_mode = gesture_config.get('loop', False)
                yaml_loop_count = gesture_config.get('loop_count', None)
                yaml_loop_timeout = gesture_config.get('loop_timeout', 300.0)  # Default 5 minutes

                if yaml_loop_mode and not self.loop_mode:
                    self.loop_mode = True
                    self.loop_max_count = yaml_loop_count  # Can be None (infinite)
                    self.loop_timeout = yaml_loop_timeout
                    if self.loop_start_time is None:
                        self.loop_start_time = time.time()
                        self.loop_count = 0
                    self.node.get_logger().info(f'📄 YAML loop config: enabled, max_count={yaml_loop_count}, timeout={yaml_loop_timeout}s')

            # Try to load cached trajectory first
            cache_start_time = time.time()
            trajectory = self.load_trajectory_cache(self.gesture_name)
            cache_load_time = time.time() - cache_start_time

            if trajectory is not None:
                # Cached trajectory found - use it directly!
                self.node.get_logger().info(f'⚡ Using cached trajectory - skipping IK/planning')
                if self.enable_metrics:
                    self.metrics.from_cache = True
                    self.metrics.cache_load_time = cache_load_time
                    self.metrics.num_interpolated_points = trajectory['right_arm']['num_points']
            else:
                # No cached trajectory - compute it
                self.node.get_logger().info(f'🔧 Computing new trajectory...')

                # Check waypoint format (default to 'cartesian' for backward compatibility)
                # Note: control_mode refers to WAYPOINT FORMAT, not motor control method
                # Both modes ultimately generate joint angle setpoints and use motor's internal PID control
                control_mode = gesture_config.get('control_mode', 'cartesian')
                self.node.get_logger().info(f'📋 Waypoint Format: {control_mode.upper()}')

                # Branch based on waypoint format
                if control_mode == 'joint':
                    # JOINT FORMAT: Waypoints contain joint angles directly (no IK needed)
                    self.node.get_logger().info('📐 Extracting joint angles directly from waypoints')

                    # Extract joint angles from gesture config
                    right_angles, left_angles = self.extract_joint_angles_from_gesture(gesture_config)

                    # Store waypoint count for metrics
                    if self.enable_metrics:
                        self.metrics.num_waypoints = max(len(right_angles), len(left_angles))

                elif control_mode == 'cartesian':
                    # CARTESIAN FORMAT: Waypoints contain cartesian positions (need IK to get joint angles)
                    self.node.get_logger().info('🗺️  Solving IK to convert cartesian waypoints to joint angles')

                    # Solve IK for waypoints and measure time
                    ik_start_time = time.time()
                    right_angles, left_angles = self.solve_ik_for_gesture(gesture_config)
                    ik_solve_time = time.time() - ik_start_time

                    # Store IK metrics
                    if self.enable_metrics:
                        self.metrics.ik_solve_time = ik_solve_time
                        self.metrics.num_waypoints = max(
                            len(gesture_config.get('right_arm_waypoints', [])),
                            len(gesture_config.get('left_arm_waypoints', []))
                        )

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

                # Handle single waypoint case - duplicate waypoint so planner has 2 points
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

                    # Try to get current position from hardware
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

                    # Try to get current position from hardware
                    if self.hardware_manager is not None:
                        try:
                            positions = self.hardware_manager.get_present_position(*self.left_motor_ids)
                            if len(positions) == 4:
                                current_left = np.array(positions)
                        except Exception as e:
                            self.node.get_logger().warning(f'Could not read current left arm position: {e}')

                    left_angles = [current_left, current_left]

                # Plan trajectory and measure time
                planning_start_time = time.time()
                trajectory = self.plan_dual_arm_trajectory(
                    right_angles,
                    left_angles,
                    gesture_config.get('steps_per_segment', 50),
                    gesture_config.get('synchronized', False),
                    gesture_config.get('interpolation_method', 'cubic')
                )
                planning_time = time.time() - planning_start_time

                if self.enable_metrics:
                    self.metrics.trajectory_planning_time = planning_time
                    self.metrics.num_interpolated_points = trajectory['right_arm']['num_points']

                # Save trajectory to cache for future use
                self.save_trajectory_cache(self.gesture_name, trajectory)

            # Store total initialization time
            if self.enable_metrics:
                self.metrics.total_initialization_time = time.time() - init_start_time

            # Compute smoothness metrics for joint mode
            if self.enable_metrics and self.metrics.control_mode == 'joint':
                self.compute_trajectory_smoothness_metrics(trajectory)

            # Execute trajectory (whether cached or newly computed)
            exec_start_time = time.time()
            success = self.execute_trajectory(trajectory)
            exec_time = time.time() - exec_start_time

            if self.enable_metrics:
                self.metrics.trajectory_execution_time = exec_time

            self.execution_started = True
            self.execution_complete = True
            self.execution_success = success

            # Save metrics if execution was successful
            if self.enable_metrics and success:
                self.save_performance_metrics()

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

            # Convert lists back to numpy arrays for consistency
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
        """Save computed trajectory to cache file (both install and source directories)"""
        # Convert numpy arrays to lists for YAML serialization
        cache_data = {
            'gesture_name': gesture_name,
            'generated': time.strftime('%Y-%m-%d %H:%M:%S'),
            'num_points': trajectory['right_arm']['num_points'],
            'right_arm_trajectory': [point.tolist() for point in trajectory['right_arm']['trajectory']],
            'left_arm_trajectory': [point.tolist() for point in trajectory['left_arm']['trajectory']]
        }

        saved_count = 0

        # Save to install directory (for immediate use)
        install_file = os.path.join(self.trajectories_directory, f'{gesture_name}_trajectory.yaml')
        try:
            with open(install_file, 'w') as f:
                yaml.dump(cache_data, f, default_flow_style=False)
            self.node.get_logger().info(f'💾 Saved to install: {install_file}')
            saved_count += 1
        except Exception as e:
            self.node.get_logger().error(f'Failed to save to install directory: {e}')

        # Save to source directory (for version control and persistence)
        # Convert install path to source path
        if '/install/' in self.trajectories_directory:
            source_trajectories_dir = self.trajectories_directory.replace('/install/', '/src/').replace('/share/smilei_dual_arm_ik/', '/')
            source_file = os.path.join(source_trajectories_dir, f'{gesture_name}_trajectory.yaml')

            try:
                # Ensure source directory exists
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

        ik_success = 0
        ik_failed = 0

        # Solve for right arm
        for waypoint in right_waypoints:
            target_pos = np.array([waypoint['x'], waypoint['y'], waypoint['z']])
            solution = self.ik_solver.solve_ik_right_arm_multiple_attempts(target_pos)

            if solution['success']:
                right_joint_angles.append(solution['joint_angles'])
                ik_success += 1
            else:
                self.node.get_logger().warning(f'IK failed for right waypoint: {target_pos}')
                ik_failed += 1

        # Solve for left arm
        for waypoint in left_waypoints:
            target_pos = np.array([waypoint['x'], waypoint['y'], waypoint['z']])
            solution = self.ik_solver.solve_ik_left_arm_multiple_attempts(target_pos)

            if solution['success']:
                left_joint_angles.append(solution['joint_angles'])
                ik_success += 1
            else:
                self.node.get_logger().warning(f'IK failed for left waypoint: {target_pos}')
                ik_failed += 1

        # Store IK success/fail counts for metrics
        if self.enable_metrics:
            self.metrics.ik_success_count = ik_success
            self.metrics.ik_failed_count = ik_failed

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

        self.node.get_logger().info(f'📐 Extracted joint angles: {len(right_joint_angles)} right, {len(left_joint_angles)} left')
        return right_joint_angles, left_joint_angles

    def configure_motors_for_position_mode(self):
        """Configure motors for position control mode (mode 2) - for joint mode gestures"""
        if not self.hardware_manager:
            self.node.get_logger().info('[SIM] Would configure motors for position mode')
            return True

        try:
            self.node.get_logger().info('⚙️  Configuring motors for POSITION CONTROL (mode 2)')

            # Configure PID gains for position mode (same as enable_robot)
            if not self.hardware_manager.configure_pid_gains(self.motor_ids, p_gain=5.0, d_gain=0.2, i_gain=0.0):
                self.node.get_logger().error('Failed to configure PID gains')
                return False

            # Set position mode and limits
            if not self.hardware_manager.set_position_mode_and_limits(self.motor_ids, iq_max=3.0):
                self.node.get_logger().error('Failed to set position mode')
                return False

            # Enable torque
            enable_pairs = [(motor_id, 1) for motor_id in self.motor_ids]
            self.hardware_manager.set_torque_enable(*enable_pairs)

            self.node.get_logger().info('✅ Motors configured for position mode')
            return True

        except Exception as e:
            self.node.get_logger().error(f'Error configuring motors for position mode: {e}')
            return False

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
            # Check if execution was stopped via topic
            if not self.running:
                self.node.get_logger().warning('⏹️ Execution stopped by user command')
                return False

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

            # Timing: allow ROS2 to process callbacks while waiting
            # This maintains responsiveness to new gesture commands
            start_time = time.time()
            while time.time() - start_time < 0.05:  # 50ms between waypoints
                rclpy.spin_once(self.node, timeout_sec=0.001)  # Process callbacks for 1ms
                time.sleep(0.001)  # Small sleep to prevent CPU spinning

        self.node.get_logger().info('✅ Trajectory execution complete')
        return True

    def compute_trajectory_smoothness_metrics(self, trajectory):
        """Compute smoothness metrics for joint-space trajectory"""
        try:
            right_traj = trajectory['right_arm']['trajectory']
            left_traj = trajectory['left_arm']['trajectory']

            # Combine both arms for analysis
            all_trajectories = right_traj + left_traj

            max_velocity = 0.0
            max_acceleration = 0.0
            discontinuities = 0

            # Compute velocities and accelerations
            for traj_points in [right_traj, left_traj]:
                if len(traj_points) < 2:
                    continue

                # Convert to numpy array for easier computation
                traj_array = np.array(traj_points)  # Shape: (n_points, 4 joints)

                # Compute velocities (finite differences)
                dt = 0.05  # 50ms between points (from execute_trajectory)
                velocities = np.diff(traj_array, axis=0) / dt  # rad/s

                # Compute accelerations
                if len(velocities) > 1:
                    accelerations = np.diff(velocities, axis=0) / dt  # rad/s^2
                    max_acceleration = max(max_acceleration, np.max(np.abs(accelerations)))

                # Find max velocity
                max_velocity = max(max_velocity, np.max(np.abs(velocities)))

                # Check for velocity discontinuities (large jumps in velocity)
                # A discontinuity is when acceleration exceeds a threshold
                if len(velocities) > 1:
                    velocity_jumps = np.abs(np.diff(velocities, axis=0))
                    # Count discontinuities where velocity changes by > 1.0 rad/s in 50ms
                    discontinuities += np.sum(velocity_jumps > 1.0)

            # Store metrics
            self.metrics.max_joint_velocity = max_velocity
            self.metrics.max_joint_acceleration = max_acceleration
            self.metrics.velocity_discontinuities = int(discontinuities)

            self.node.get_logger().info(
                f'📈 Smoothness: max_vel={max_velocity:.3f} rad/s, '
                f'max_accel={max_acceleration:.3f} rad/s², '
                f'discontinuities={discontinuities}'
            )

        except Exception as e:
            self.node.get_logger().warning(f'Failed to compute smoothness metrics: {e}')

    def save_performance_metrics(self):
        """Save performance metrics to YAML file"""
        try:
            # Create filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'{self.metrics.gesture_name}_{self.metrics.control_mode}_{timestamp}.yaml'
            filepath = os.path.join(self.metrics_directory, filename)

            # Convert metrics to dict
            metrics_dict = self.metrics.to_dict()

            # Add comparison info if both cached and uncached runs exist
            if self.metrics.from_cache:
                metrics_dict['performance_note'] = 'Loaded from cache - faster initialization'
            else:
                metrics_dict['performance_note'] = 'First run - computed IK/trajectory'

            # Save to file
            with open(filepath, 'w') as f:
                yaml.dump(metrics_dict, f, default_flow_style=False, sort_keys=False)

            self.node.get_logger().info(f'💾 Performance metrics saved: {filepath}')

            # Also save to a "latest" file for easy access
            latest_filename = f'{self.metrics.gesture_name}_{self.metrics.control_mode}_latest.yaml'
            latest_filepath = os.path.join(self.metrics_directory, latest_filename)
            with open(latest_filepath, 'w') as f:
                yaml.dump(metrics_dict, f, default_flow_style=False, sort_keys=False)

            # Append to summary file for aggregated statistics
            self.append_to_summary_file(metrics_dict)

        except Exception as e:
            self.node.get_logger().error(f'Failed to save performance metrics: {e}')

    def append_to_summary_file(self, metrics_dict):
        """Append metrics to a summary file for statistical analysis"""
        try:
            summary_file = os.path.join(self.metrics_directory, 'performance_summary.yaml')

            # Load existing summary if it exists
            if os.path.exists(summary_file):
                with open(summary_file, 'r') as f:
                    summary_data = yaml.safe_load(f) or {'executions': []}
            else:
                summary_data = {'executions': []}

            # Append new execution
            summary_data['executions'].append(metrics_dict)

            # Compute aggregated statistics
            summary_data['total_executions'] = len(summary_data['executions'])
            summary_data['last_updated'] = datetime.now().strftime('%Y-%m-%d %H:%M:%S')

            # Group by control mode
            cartesian_execs = [e for e in summary_data['executions'] if e.get('control_mode') == 'cartesian']
            joint_execs = [e for e in summary_data['executions'] if e.get('control_mode') == 'joint']

            summary_data['statistics'] = {
                'cartesian_count': len(cartesian_execs),
                'joint_count': len(joint_execs)
            }

            # Save summary
            with open(summary_file, 'w') as f:
                yaml.dump(summary_data, f, default_flow_style=False, sort_keys=False)

            self.node.get_logger().info(f'📊 Updated performance summary ({summary_data["total_executions"]} total executions)')

        except Exception as e:
            self.node.get_logger().warning(f'Failed to update summary file: {e}')

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
