#!/usr/bin/env python3
"""
Hardware Waypoint Recorder - Keyboard-controlled joint angle waypoint capture

Records motor joint positions from hardware and allows saving them as waypoints.
You can physically move the robot arms to desired positions and capture them.

Keyboard Controls:
  Space: Capture current motor positions as waypoint
  P: Print current motor positions
  L: List all saved waypoints
  C: Clear all saved waypoints
  E: Export waypoints to YAML file
  ESC: Exit

The waypoints are saved as joint angles for each arm, ready to be used
for trajectory planning or gesture execution.
"""

import rclpy
from rclpy.node import Node
import sys
import termios
import tty
import threading
import numpy as np
import yaml
import os
from datetime import datetime

# Import HardwareManager from smilei_state_machine
sys.path.append('/home/rovestrada/smilei_ws/src/smilei_state_machine')
from smilei_state_machine.hardware_manager import HardwareManager


class HardwareWaypointRecorder(Node):
    """
    ROS2 Node that records motor positions as waypoints using keyboard control
    """

    def __init__(self):
        super().__init__('hardware_waypoint_recorder')

        # Declare hardware manager parameters
        self.declare_parameter('hardware_manager.usb_ports', ['/dev/ttyUSB0', '/dev/ttyUSB1'])
        self.declare_parameter('hardware_manager.baudrate', 8000000)
        self.declare_parameter('hardware_manager.auto_detect', True)
        self.declare_parameter('hardware_manager.debug', False)

        # Declare motor configuration parameters
        self.declare_parameter('motors.right_arm_ids', [1, 2, 3, 4])
        self.declare_parameter('motors.left_arm_ids', [5, 6, 7, 8])

        # Declare output directory for YAML exports
        self.declare_parameter('output_directory', '')

        # Load parameters
        usb_ports = self.get_parameter('hardware_manager.usb_ports').value
        baudrate = self.get_parameter('hardware_manager.baudrate').value
        auto_detect = self.get_parameter('hardware_manager.auto_detect').value
        debug = self.get_parameter('hardware_manager.debug').value

        # Motor ID mappings
        self.right_motor_ids = self.get_parameter('motors.right_arm_ids').value
        self.left_motor_ids = self.get_parameter('motors.left_arm_ids').value

        # Output directory
        output_dir = self.get_parameter('output_directory').value
        if not output_dir:
            # Default to package config/gestures directory
            from ament_index_python.packages import get_package_share_directory
            pkg_share = get_package_share_directory('smilei_dual_arm_ik')
            self.output_directory = os.path.join(pkg_share, 'config', 'gestures')
        else:
            self.output_directory = output_dir

        # Initialize hardware manager
        self.get_logger().info('Initializing hardware manager...')
        self.hardware_manager = HardwareManager(
            node=self,
            usb_ports=usb_ports,
            baudrate=baudrate,
            auto_detect=auto_detect,
            debug=debug
        )

        # Get available motors
        self.motor_ids = self.hardware_manager.get_available_motors()
        if not self.motor_ids:
            self.get_logger().error('No motors detected!')
            raise RuntimeError('Failed to detect any motors')

        self.get_logger().info(f'Detected motors: {self.motor_ids}')
        self.get_logger().info(f'Motor mapping - Right: {self.right_motor_ids}, Left: {self.left_motor_ids}')

        # Motor state variables
        self.current_positions = [0.0] * 8

        # Saved waypoints (list of dicts with 'right' and 'left' joint angles)
        self.waypoints = []

        # Keyboard input thread
        self.running = True
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()

        # Timer to continuously read motor positions
        self.read_timer = self.create_timer(0.1, self.read_motor_positions)

        # Print instructions
        self.print_instructions()

        # Read initial positions
        self.read_motor_positions()

    def print_instructions(self):
        """Print keyboard control instructions"""
        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info('Hardware Waypoint Recorder')
        self.get_logger().info('='*60)
        self.get_logger().info('Physically move the robot arms to desired positions,')
        self.get_logger().info('then press Space to capture waypoint.')
        self.get_logger().info('')
        self.get_logger().info('Keyboard Controls:')
        self.get_logger().info('  Space: Capture current motor positions as waypoint')
        self.get_logger().info('  P: Print current motor positions')
        self.get_logger().info('  L: List all saved waypoints')
        self.get_logger().info('  C: Clear all saved waypoints')
        self.get_logger().info('  E: Export waypoints to YAML file')
        self.get_logger().info('  ESC: Exit')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Output directory: {self.output_directory}')
        self.get_logger().info('='*60)
        self.get_logger().info('')

    def read_motor_positions(self):
        """Read current positions from all motors"""
        try:
            # Get positions for all motors
            positions = self.hardware_manager.get_present_position(*self.motor_ids)

            if len(positions) == len(self.motor_ids):
                self.current_positions = positions[:]
                return True
            else:
                return False

        except Exception as e:
            self.get_logger().error(f'Error reading motor positions: {e}')
            return False

    def get_current_joint_angles(self):
        """Get current joint angles for both arms"""
        # Map motor positions to arm joint angles
        right_angles = np.zeros(4)
        left_angles = np.zeros(4)

        for i in range(4):
            # Right arm
            right_motor_id = self.right_motor_ids[i]
            if right_motor_id in self.motor_ids:
                idx = self.motor_ids.index(right_motor_id)
                right_angles[i] = self.current_positions[idx]

            # Left arm
            left_motor_id = self.left_motor_ids[i]
            if left_motor_id in self.motor_ids:
                idx = self.motor_ids.index(left_motor_id)
                left_angles[i] = self.current_positions[idx]

        return right_angles, left_angles

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
