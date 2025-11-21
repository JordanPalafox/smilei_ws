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
import sys
import termios
import tty
import threading
import numpy as np
import yaml
import os
from datetime import datetime


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

        # Subscribe to joint states topic
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
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
        self.get_logger().info('  C: Clear all saved waypoints')
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
        elif key == 'c' or key == 'C':
            self.clear_waypoints()
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
        """Capture current motor positions as a waypoint"""
        right_angles, left_angles = self.get_current_joint_angles()

        waypoint = {
            'right': right_angles.tolist(),
            'left': left_angles.tolist()
        }

        self.waypoints.append(waypoint)

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'✅ Waypoint {len(self.waypoints)} captured!')
        self.get_logger().info('='*60)
        self.get_logger().info('Right Arm:')
        for i in range(4):
            self.get_logger().info(f'  Joint {i}: {right_angles[i]:+.4f} rad')
        self.get_logger().info('Left Arm:')
        for i in range(4):
            self.get_logger().info(f'  Joint {i}: {left_angles[i]:+.4f} rad')
        self.get_logger().info(f'Total waypoints: {len(self.waypoints)}')
        self.get_logger().info('='*60)
        self.get_logger().info('')

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

    def clear_waypoints(self):
        """Clear all saved waypoints"""
        num_waypoints = len(self.waypoints)
        self.waypoints = []

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'🗑️  Cleared {num_waypoints} waypoints')
        self.get_logger().info('='*60)
        self.get_logger().info('')

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
