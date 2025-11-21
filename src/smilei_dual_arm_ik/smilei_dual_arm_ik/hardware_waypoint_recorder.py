#!/usr/bin/env python3
"""
Hardware Waypoint Recorder - Keyboard-controlled joint angle waypoint capture

Records joint positions from /joint_states topic and allows saving them as waypoints.
Works in conjunction with hardware_joint_state_publisher which reads from hardware.

You can physically move the robot arms to desired positions and capture them.

Keyboard Controls:
  Space: Capture current joint positions as waypoint
  P: Print current joint positions
  L: List all saved waypoints
  C: Clear all saved waypoints
  E: Export waypoints to YAML file
  ESC: Exit

The waypoints are saved as joint angles for each arm, ready to be used
for trajectory planning or gesture execution.

Usage:
  Run alongside hardware_visualization.launch.py:

  Terminal 1: ros2 launch smilei_dual_arm_ik hardware_visualization.launch.py
  Terminal 2: ros2 run smilei_dual_arm_ik hardware_waypoint_recorder.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
import sys
import termios
import tty
import threading
import numpy as np
import yaml
import os
from datetime import datetime
from scipy.interpolate import CubicSpline
from ament_index_python.packages import get_package_share_directory

# Import forward kinematics for visualization
from smilei_dual_arm_ik.inverse_kinematics_dual_arm import InverseKinematicsDualArm


class HardwareWaypointRecorder(Node):
    """
    ROS2 Node that records joint positions from /joint_states topic as waypoints
    """

    def __init__(self):
        super().__init__('hardware_waypoint_recorder')

        # Declare output directory for YAML exports
        self.declare_parameter('output_directory', '')

        # Output directory
        output_dir = self.get_parameter('output_directory').value
        if not output_dir:
            # Default to package config/gestures directory
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            self.output_directory = os.path.join(pkg_share, 'config', 'gestures')
        else:
            self.output_directory = output_dir

        # Current joint state
        self.current_joint_state = None
        self.last_joint_state_time = None

        # Joint angles (4 per arm)
        self.current_right_angles = np.zeros(4)
        self.current_left_angles = np.zeros(4)

        # Joint names (must match hardware_joint_state_publisher)
        self.right_joint_names = [
            'right_joint_0', 'right_joint_1', 'right_joint_2', 'right_joint_3'
        ]
        self.left_joint_names = [
            'left_joint_0', 'left_joint_1', 'left_joint_2', 'left_joint_3'
        ]

        # Saved waypoints (list of dicts with 'right' and 'left' joint angles)
        self.waypoints = []

        # Persisted end-effector positions for visualization
        self.persisted_right_ee_positions = []  # Cartesian positions of right end effector
        self.persisted_left_ee_positions = []   # Cartesian positions of left end effector

        # Interpolated trajectory points for visualization
        self.interpolated_right_trajectory = []
        self.interpolated_left_trajectory = []

        # Initialize FK solver for end-effector position calculation
        pkg_share = get_package_share_directory('smilei_dual_arm_ik')
        robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')
        self.fk_solver = InverseKinematicsDualArm(robot_params_file)

        # Subscribe to joint states topic
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for visualization markers
        self.markers_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )

        # Keyboard input thread
        self.running = True
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()

        # Print instructions
        self.print_instructions()

        self.get_logger().info('Waiting for joint states on /joint_states...')
        self.get_logger().info('Make sure hardware_joint_state_publisher is running!')

    def print_instructions(self):
        """Print keyboard control instructions"""
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('Hardware Waypoint Recorder')
        self.get_logger().info('='*60)
        self.get_logger().info('Subscribes to /joint_states topic to read joint positions.')
        self.get_logger().info('')
        self.get_logger().info('Usage:')
        self.get_logger().info('  Terminal 1: ros2 launch smilei_dual_arm_ik \\')
        self.get_logger().info('              hardware_visualization.launch.py')
        self.get_logger().info('  Terminal 2: ros2 run smilei_dual_arm_ik \\')
        self.get_logger().info('              hardware_waypoint_recorder.py')
        self.get_logger().info('')
        self.get_logger().info('Physically move the robot arms to desired positions,')
        self.get_logger().info('then press Space to capture waypoint.')
        self.get_logger().info('')
        self.get_logger().info('Keyboard Controls:')
        self.get_logger().info('  Space: Capture current joint positions as waypoint')
        self.get_logger().info('  P: Print current joint positions')
        self.get_logger().info('  L: List all saved waypoints')
        self.get_logger().info('  V: Clear waypoint visualization')
        self.get_logger().info('  I: Interpolate and visualize trajectory')
        self.get_logger().info('  E: Export waypoints to YAML file')
        self.get_logger().info('  ESC: Exit')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Output directory: {self.output_directory}')
        self.get_logger().info('='*60)
        self.get_logger().info('')

    def joint_state_callback(self, msg: JointState):
        """Callback for /joint_states topic"""
        self.current_joint_state = msg
        self.last_joint_state_time = self.get_clock().now()

        # Extract joint positions by name
        try:
            # Right arm
            for i, joint_name in enumerate(self.right_joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_right_angles[i] = msg.position[idx]

            # Left arm
            for i, joint_name in enumerate(self.left_joint_names):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    self.current_left_angles[i] = msg.position[idx]

        except Exception as e:
            self.get_logger().error(f'Error parsing joint state: {e}')

    def get_current_joint_angles(self):
        """Get current joint angles for both arms"""
        return self.current_right_angles.copy(), self.current_left_angles.copy()

    def get_key(self):
        """Get a single keypress from terminal"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def keyboard_input_loop(self):
        """Loop to handle keyboard input"""
        while self.running and rclpy.ok():
            try:
                key = self.get_key()
                self.handle_key(key)
            except Exception as e:
                self.get_logger().error(f'Error reading key: {e}')
                break

    def handle_key(self, key):
        """Handle keyboard input"""
        if key == ' ':
            self.capture_waypoint()
        elif key == 'p' or key == 'P':
            self.print_current_position()
        elif key == 'l' or key == 'L':
            self.list_waypoints()
        elif key == 'v' or key == 'V':
            self.clear_visualization()
        elif key == 'i' or key == 'I':
            self.interpolate_and_visualize()
        elif key == 'e' or key == 'E':
            self.export_waypoints()
        elif key == '\x1b':  # ESC
            self.get_logger().info('Exiting...')
            if len(self.waypoints) > 0:
                self.list_waypoints()
            self.running = False
            rclpy.shutdown()
        elif key == '\x03':  # Ctrl+C
            self.get_logger().info('Interrupted')
            self.running = False
            rclpy.shutdown()

    def print_current_position(self):
        """Print current motor positions"""
        right_angles, left_angles = self.get_current_joint_angles()

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('Current Motor Positions (Joint Angles):')
        self.get_logger().info('='*60)
        self.get_logger().info('Right Arm:')
        for i in range(4):
            self.get_logger().info(f'  Joint {i}: {right_angles[i]:+.4f} rad ({np.degrees(right_angles[i]):+.2f}°)')
        self.get_logger().info('')
        self.get_logger().info('Left Arm:')
        for i in range(4):
            self.get_logger().info(f'  Joint {i}: {left_angles[i]:+.4f} rad ({np.degrees(left_angles[i]):+.2f}°)')
        self.get_logger().info('='*60)
        self.get_logger().info('')

    def capture_waypoint(self):
        """Capture current motor positions as a waypoint and visualize end-effector positions"""
        right_angles, left_angles = self.get_current_joint_angles()

        # Save joint angles
        waypoint = {
            'right': right_angles.tolist(),
            'left': left_angles.tolist()
        }
        self.waypoints.append(waypoint)

        # Calculate and save end-effector positions for visualization
        try:
            # Right arm FK
            T_right = self.fk_solver.forward_kinematics_right_arm(right_angles)
            right_ee_pos = T_right[:3, 3]  # Extract [x, y, z]
            self.persisted_right_ee_positions.append(right_ee_pos.tolist())

            # Left arm FK
            T_left = self.fk_solver.forward_kinematics_left_arm(left_angles)
            left_ee_pos = T_left[:3, 3]  # Extract [x, y, z]
            self.persisted_left_ee_positions.append(left_ee_pos.tolist())

        except Exception as e:
            self.get_logger().error(f'Error calculating FK: {e}')

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'✅ Waypoint {len(self.waypoints)} captured & visualized!')
        self.get_logger().info('='*60)
        self.get_logger().info('Right Arm:')
        for i in range(4):
            self.get_logger().info(f'  Joint {i}: {right_angles[i]:+.4f} rad')
        if len(self.persisted_right_ee_positions) > 0:
            pos = self.persisted_right_ee_positions[-1]
            self.get_logger().info(f'  End Effector: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')

        self.get_logger().info('Left Arm:')
        for i in range(4):
            self.get_logger().info(f'  Joint {i}: {left_angles[i]:+.4f} rad')
        if len(self.persisted_left_ee_positions) > 0:
            pos = self.persisted_left_ee_positions[-1]
            self.get_logger().info(f'  End Effector: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]')

        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        self.get_logger().info('='*60)
        self.get_logger().info('')

        # Publish visualization markers
        self.publish_waypoint_markers()

    def list_waypoints(self):
        """List all saved waypoints"""
        if len(self.waypoints) == 0:
            self.get_logger().info('No waypoints saved.')
            return

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Saved {len(self.waypoints)} waypoints:')
        self.get_logger().info('='*60)

        for i, waypoint in enumerate(self.waypoints):
            self.get_logger().info(f'Waypoint {i+1}:')
            self.get_logger().info('  Right: ' + str([f'{a:+.4f}' for a in waypoint['right']]))
            self.get_logger().info('  Left:  ' + str([f'{a:+.4f}' for a in waypoint['left']]))

        self.get_logger().info('='*60)
        self.get_logger().info('')

    def clear_visualization(self):
        """Clear waypoint visualization (but keep waypoints)"""
        num_right = len(self.persisted_right_ee_positions)
        num_left = len(self.persisted_left_ee_positions)
        num_interp_right = len(self.interpolated_right_trajectory)
        num_interp_left = len(self.interpolated_left_trajectory)

        self.persisted_right_ee_positions = []
        self.persisted_left_ee_positions = []
        self.interpolated_right_trajectory = []
        self.interpolated_left_trajectory = []

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'🧹 Cleared visualization:')
        self.get_logger().info(f'   Right markers: {num_right}')
        self.get_logger().info(f'   Left markers: {num_left}')
        self.get_logger().info(f'   Right trajectory: {num_interp_right} points')
        self.get_logger().info(f'   Left trajectory: {num_interp_left} points')
        self.get_logger().info(f'   (Waypoints still saved: {len(self.waypoints)})')
        self.get_logger().info('='*60)
        self.get_logger().info('')

        # Publish empty marker array to clear visualization
        self.publish_waypoint_markers()

    def export_waypoints(self):
        """Export waypoints to YAML file in joint mode format"""
        if len(self.waypoints) == 0:
            self.get_logger().warn('No waypoints to export!')
            return

        # Ask for gesture name
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('Export Waypoints to YAML (Joint Mode)')
        self.get_logger().info('='*60)

        # For now, use timestamp as default name
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        gesture_name = f'recorded_gesture_{timestamp}'

        # Prepare YAML data structure for JOINT MODE
        # This format is compatible with the autonomous_gesture_execution behavior
        yaml_data = {
            'gesture_name': gesture_name,
            'control_mode': 'joint',  # NEW: Specify joint mode
            'recorded': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'num_waypoints': len(self.waypoints),
            'synchronized': True,  # Execute both arms in sync
            'interpolation_method': 'cubic',  # Smooth interpolation
            'steps_per_segment': 20,  # Number of interpolated steps between waypoints
            'right_arm_waypoints': [],
            'left_arm_waypoints': []
        }

        # Add each waypoint - extract right and left arm joint angles
        for waypoint in self.waypoints:
            # Right arm waypoint (4 joint angles)
            right_entry = {
                'joints': [float(a) for a in waypoint['right']]
            }
            yaml_data['right_arm_waypoints'].append(right_entry)

            # Left arm waypoint (4 joint angles)
            left_entry = {
                'joints': [float(a) for a in waypoint['left']]
            }
            yaml_data['left_arm_waypoints'].append(left_entry)

        # Create output file path
        output_file = os.path.join(self.output_directory, f'{gesture_name}.yaml')

        # Ensure output directory exists
        os.makedirs(self.output_directory, exist_ok=True)

        # Write to file
        try:
            with open(output_file, 'w') as f:
                yaml.dump(yaml_data, f, default_flow_style=False, sort_keys=False)

            self.get_logger().info(f'✅ Exported {len(self.waypoints)} waypoints in JOINT MODE to:')
            self.get_logger().info(f'   {output_file}')
            self.get_logger().info(f'   Control Mode: joint (direct joint angle control)')
            self.get_logger().info(f'   Motor Mode: Position control (mode 2)')
            self.get_logger().info('='*60)
            self.get_logger().info('')

        except Exception as e:
            self.get_logger().error(f'Failed to export waypoints: {e}')

    def interpolate_and_visualize(self):
        """Interpolate trajectories between waypoints and visualize"""
        if len(self.waypoints) < 2:
            self.get_logger().warn('⚠️  Need at least 2 waypoints to interpolate!')
            self.get_logger().warn(f'   Currently saved: {len(self.waypoints)} waypoint(s)')
            return

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'🔄 Interpolating trajectories for {len(self.waypoints)} waypoints...')

        try:
            # Extract joint angles for interpolation
            right_joint_waypoints = []
            left_joint_waypoints = []

            for waypoint in self.waypoints:
                right_joint_waypoints.append(waypoint['right'])
                left_joint_waypoints.append(waypoint['left'])

            # Convert to numpy arrays
            right_joints = np.array(right_joint_waypoints)  # Shape: (num_waypoints, 4)
            left_joints = np.array(left_joint_waypoints)    # Shape: (num_waypoints, 4)

            # Create parameter t for interpolation
            num_waypoints = len(self.waypoints)
            t = np.linspace(0, 1, num_waypoints)

            # Interpolate each joint independently using cubic splines
            num_interp_points = (num_waypoints - 1) * 50
            t_interp = np.linspace(0, 1, num_interp_points)

            # Interpolate right arm joints
            right_interp_joints = np.zeros((num_interp_points, 4))
            for joint_idx in range(4):
                cs = CubicSpline(t, right_joints[:, joint_idx])
                right_interp_joints[:, joint_idx] = cs(t_interp)

            # Interpolate left arm joints
            left_interp_joints = np.zeros((num_interp_points, 4))
            for joint_idx in range(4):
                cs = CubicSpline(t, left_joints[:, joint_idx])
                left_interp_joints[:, joint_idx] = cs(t_interp)

            # Calculate FK for interpolated joint angles to get cartesian positions
            self.interpolated_right_trajectory = []
            self.interpolated_left_trajectory = []

            for i in range(num_interp_points):
                # Right arm FK
                T_right = self.fk_solver.forward_kinematics_right_arm(right_interp_joints[i])
                right_pos = T_right[:3, 3].tolist()
                self.interpolated_right_trajectory.append(right_pos)

                # Left arm FK
                T_left = self.fk_solver.forward_kinematics_left_arm(left_interp_joints[i])
                left_pos = T_left[:3, 3].tolist()
                self.interpolated_left_trajectory.append(left_pos)

            self.get_logger().info(f'✅ Interpolation complete!')
            self.get_logger().info(f'   Right trajectory: {len(self.interpolated_right_trajectory)} points')
            self.get_logger().info(f'   Left trajectory: {len(self.interpolated_left_trajectory)} points')
            self.get_logger().info(f'   Using cubic spline interpolation')
            self.get_logger().info('='*60)
            self.get_logger().info('')

            # Publish visualization
            self.publish_waypoint_markers()

        except Exception as e:
            self.get_logger().error(f'❌ Interpolation failed: {e}')
            self.get_logger().info('='*60)
            self.get_logger().info('')

    def publish_waypoint_markers(self):
        """Publish visualization markers for waypoints and trajectories"""
        marker_array = MarkerArray()

        # Delete all previous markers first
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.markers_pub.publish(marker_array)

        # Clear and rebuild
        marker_array = MarkerArray()

        # Publish right arm waypoint markers (RED spheres)
        for i, pos in enumerate(self.persisted_right_ee_positions):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'right_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.015
            marker.scale.y = 0.015
            marker.scale.z = 0.015

            # RED for right arm
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)

            marker_array.markers.append(marker)

        # Publish left arm waypoint markers (BLUE spheres)
        for i, pos in enumerate(self.persisted_left_ee_positions):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'left_waypoints'
            marker.id = i + 1000
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.015
            marker.scale.y = 0.015
            marker.scale.z = 0.015

            # BLUE for left arm
            marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.9)

            marker_array.markers.append(marker)

        # Publish right arm interpolated trajectory (ORANGE small spheres)
        for i, pos in enumerate(self.interpolated_right_trajectory):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'right_trajectory'
            marker.id = i + 2000
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.004
            marker.scale.y = 0.004
            marker.scale.z = 0.004

            # ORANGE for right trajectory
            marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)

            marker_array.markers.append(marker)

        # Publish left arm interpolated trajectory (CYAN small spheres)
        for i, pos in enumerate(self.interpolated_left_trajectory):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'left_trajectory'
            marker.id = i + 10000
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.004
            marker.scale.y = 0.004
            marker.scale.z = 0.004

            # CYAN for left trajectory
            marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=0.6)

            marker_array.markers.append(marker)

        # Publish all markers
        self.markers_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = HardwareWaypointRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        try:
            node.running = False
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
