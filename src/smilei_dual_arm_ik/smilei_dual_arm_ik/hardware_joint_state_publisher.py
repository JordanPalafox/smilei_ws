#!/usr/bin/env python3
"""
Hardware Joint State Publisher - Real-time URDF Visualization from BEAR Motors

Reads motor positions from hardware and publishes to /joint_states for real-time
URDF visualization in RViz. This allows you to see the robot move in RViz as you
physically manipulate the motors.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys

# Import HardwareManager from smilei_state_machine
sys.path.append('/home/rovestrada/smilei_ws/src/smilei_state_machine')
from smilei_state_machine.hardware_manager import HardwareManager


class HardwareJointStatePublisher(Node):
    """
    ROS2 Node that continuously reads motor positions and publishes joint states
    """

    def __init__(self):
        super().__init__('hardware_joint_state_publisher')

        # Declare hardware manager parameters
        self.declare_parameter('hardware_manager.usb_ports', ['/dev/ttyUSB0', '/dev/ttyUSB1'])
        self.declare_parameter('hardware_manager.baudrate', 8000000)
        self.declare_parameter('hardware_manager.auto_detect', True)
        self.declare_parameter('hardware_manager.debug', False)

        # Declare motor configuration parameters
        self.declare_parameter('motors.right_arm_ids', [1, 2, 3, 4])
        self.declare_parameter('motors.left_arm_ids', [5, 6, 7, 8])

        # Declare publishing frequency
        self.declare_parameter('publish_frequency', 50.0)  # Hz

        # Load parameters
        usb_ports = self.get_parameter('hardware_manager.usb_ports').value
        baudrate = self.get_parameter('hardware_manager.baudrate').value
        auto_detect = self.get_parameter('hardware_manager.auto_detect').value
        debug = self.get_parameter('hardware_manager.debug').value

        # Motor ID mappings
        self.right_motor_ids = self.get_parameter('motors.right_arm_ids').value
        self.left_motor_ids = self.get_parameter('motors.left_arm_ids').value

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

        # Publisher for joint states
        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Joint names for URDF (matching the convention from gesture_executor_hardware.py)
        self.joint_names = [
            'right_joint_0', 'right_joint_1', 'right_joint_2', 'right_joint_3',
            'left_joint_0', 'left_joint_1', 'left_joint_2', 'left_joint_3'
        ]

        # Create timer for continuous publishing
        publish_freq = self.get_parameter('publish_frequency').value
        timer_period = 1.0 / publish_freq
        self.publish_timer = self.create_timer(
            timer_period,
            self.publish_callback
        )

        self.get_logger().info('Hardware Joint State Publisher initialized')
        self.get_logger().info(f'  Publishing frequency: {publish_freq} Hz')
        self.get_logger().info(f'  Publishing to: /joint_states')
        self.get_logger().info(f'  Motors: {self.motor_ids}')

        # Read and publish initial state
        self.read_motor_positions()
        self.publish_joint_state()

    def read_motor_positions(self):
        """Read current positions from all motors"""
        try:
            # Get positions for all motors
            positions = self.hardware_manager.get_present_position(*self.motor_ids)

            if len(positions) == len(self.motor_ids):
                self.current_positions = positions[:]
                return True
            else:
                self.get_logger().warning(f'Expected {len(self.motor_ids)} positions, got {len(positions)}')
                return False

        except Exception as e:
            self.get_logger().error(f'Error reading motor positions: {e}')
            return False

    def publish_joint_state(self):
        """Publish current joint state for URDF visualization"""
        # Map motor positions to joint positions
        # Right arm: motors self.right_motor_ids -> joints 0,1,2,3
        # Left arm: motors self.left_motor_ids -> joints 0,1,2,3
        right_positions = []
        left_positions = []

        for i in range(4):
            # Right arm
            right_motor_id = self.right_motor_ids[i]
            if right_motor_id in self.motor_ids:
                idx = self.motor_ids.index(right_motor_id)
                right_positions.append(self.current_positions[idx])
            else:
                right_positions.append(0.0)

            # Left arm
            left_motor_id = self.left_motor_ids[i]
            if left_motor_id in self.motor_ids:
                idx = self.motor_ids.index(left_motor_id)
                left_positions.append(self.current_positions[idx])
            else:
                left_positions.append(0.0)

        # Create and publish joint state message
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = self.joint_names
        joint_msg.position = right_positions + left_positions
        joint_msg.velocity = [0.0] * len(joint_msg.position)

        self.joint_pub.publish(joint_msg)

    def publish_callback(self):
        """Timer callback - read positions and publish joint state"""
        if self.read_motor_positions():
            self.publish_joint_state()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = HardwareJointStatePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
