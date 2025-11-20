#!/usr/bin/env python3
"""
Interactive Marker Node with IK Validation

Permite mover un marker con el teclado y publica su posición.
El marker cambia de color según si la posición tiene solución de cinemática inversa:
  🟢 VERDE: Posición alcanzable (IK solution existe)
  🔴 ROJO: Posición no alcanzable (no hay IK solution)
  🔵 CYAN: Validación IK deshabilitada

Controles de Movimiento:
  W/S: Mover en X (adelante/atrás)
  A/D: Mover en Y (izquierda/derecha)
  Q/E: Mover en Z (arriba/abajo)
  R: Reset a posición inicial
  H: Mover a posición actual del End Effector

Controles de Waypoints:
  Space: Guardar posición actual como waypoint
  P: Imprimir posición actual (incluye status IK)
  L: Listar waypoints guardados

Controles de Visualización:
  V: Persistir punto actual en RViz (esfera verde oscura)
  C: Limpiar todos los puntos persistidos
  I: Interpolar y mostrar trayectoria entre waypoints (esferas azules)

  ESC: Salir

Parámetros:
  enable_ik_validation: true/false (default: true)
  step_size: tamaño del paso en metros (default: 0.01)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import JointState
import sys
import termios
import tty
import threading
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory
from smilei_dual_arm_ik.inverse_kinematics_solver import InverseKinematicsSolver
from scipy.interpolate import CubicSpline


class InteractiveMarkerNode(Node):
    """Nodo que permite mover un marker con el teclado"""

    def __init__(self):
        super().__init__('interactive_marker_node')

        # Declare parameters
        self.declare_parameter('step_size', 0.01)  # meters
        self.declare_parameter('initial_x', 0.10)
        self.declare_parameter('initial_y', 0.00)
        self.declare_parameter('initial_z', 0.16)
        self.declare_parameter('enable_ik_validation', True)  # Validate IK and change color

        # Get parameters
        self.step_size = self.get_parameter('step_size').value
        self.enable_ik_validation = self.get_parameter('enable_ik_validation').value
        self.initial_pos = [
            self.get_parameter('initial_x').value,
            self.get_parameter('initial_y').value,
            self.get_parameter('initial_z').value
        ]

        # Current position
        self.position = list(self.initial_pos)

        # Saved waypoints
        self.waypoints = []

        # Persisted visualization points
        self.persisted_points = []  # List of 3D points to visualize
        self.interpolated_points = []  # Interpolated trajectory points

        # IK validation state
        self.last_validated_position = None
        self.is_reachable = None  # None=unknown, True=reachable, False=unreachable
        self.last_ik_error = None

        # IK solver for FK calculations
        pkg_share = get_package_share_directory('smilei_dual_arm_ik')
        robot_params_file = os.path.join(pkg_share, 'config', 'robot_parameters.yaml')
        self.ik_solver = InverseKinematicsSolver(robot_params_file)

        # Store current end effector position
        self.current_ee_position = None
        self.current_joint_state = None

        # Subscriber for joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Publisher for marker
        self.marker_pub = self.create_publisher(
            Marker,
            '/interactive_marker',
            10
        )

        # Publisher for position
        self.position_pub = self.create_publisher(
            PointStamped,
            '/marker_position',
            10
        )

        # Publisher for persisted markers
        self.persisted_markers_pub = self.create_publisher(
            MarkerArray,
            '/persisted_markers',
            10
        )

        # Timer to publish marker
        self.timer = self.create_timer(0.1, self.publish_marker)

        # Keyboard input thread
        self.running = True
        self.input_thread = threading.Thread(target=self.keyboard_input_loop, daemon=True)
        self.input_thread.start()

        self.get_logger().info('Interactive Marker Node started')
        self.get_logger().info('='*60)
        self.get_logger().info('Movement Controls:')
        self.get_logger().info('  W/S: Move in X (forward/backward)')
        self.get_logger().info('  A/D: Move in Y (left/right)')
        self.get_logger().info('  Q/E: Move in Z (up/down)')
        self.get_logger().info('  R: Reset to initial position')
        self.get_logger().info('  H: Move to current End Effector position')
        self.get_logger().info('')
        self.get_logger().info('Waypoint Controls:')
        self.get_logger().info('  Space: Save current position as waypoint')
        self.get_logger().info('  P: Print current position')
        self.get_logger().info('  L: List all saved waypoints')
        self.get_logger().info('')
        self.get_logger().info('Visualization Controls:')
        self.get_logger().info('  V: Persist current point in RViz (green sphere)')
        self.get_logger().info('  C: Clear all persisted points')
        self.get_logger().info('  I: Interpolate & show trajectory between waypoints')
        self.get_logger().info('')
        self.get_logger().info('  ESC: Exit')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Step size: {self.step_size} m')
        self.get_logger().info(f'Initial position: {self.position}')
        self.get_logger().info(f'IK Validation: {"ENABLED" if self.enable_ik_validation else "DISABLED"}')
        if self.enable_ik_validation:
            self.get_logger().info('  🟢 Green = Reachable   🔴 Red = Unreachable')

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
        moved = False

        if key == 'w' or key == 'W':
            self.position[0] += self.step_size
            moved = True
        elif key == 's' or key == 'S':
            self.position[0] -= self.step_size
            moved = True
        elif key == 'a' or key == 'A':
            self.position[1] += self.step_size
            moved = True
        elif key == 'd' or key == 'D':
            self.position[1] -= self.step_size
            moved = True
        elif key == 'q' or key == 'Q':
            self.position[2] += self.step_size
            moved = True
        elif key == 'e' or key == 'E':
            self.position[2] -= self.step_size
            moved = True
        elif key == 'r' or key == 'R':
            self.position = list(self.initial_pos)
            self.get_logger().info(f'Reset to initial position: {self.position}')
        elif key == 'h' or key == 'H':
            self.move_to_end_effector()
        elif key == 'p' or key == 'P':
            self.print_position()
        elif key == 'l' or key == 'L':
            self.print_saved_waypoints()
        elif key == 'v' or key == 'V':
            self.persist_current_point()
        elif key == 'c' or key == 'C':
            self.clear_persisted_points()
        elif key == 'i' or key == 'I':
            self.interpolate_and_visualize()
        elif key == ' ':
            self.save_waypoint()
        elif key == '\x1b':  # ESC
            self.get_logger().info('Exiting...')
            self.print_saved_waypoints()
            self.running = False
            rclpy.shutdown()
        elif key == '\x03':  # Ctrl+C
            self.get_logger().info('Interrupted')
            self.running = False
            rclpy.shutdown()

        if moved:
            self.get_logger().info(f'Position: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}]')

    def print_position(self):
        """Print current position in detail"""
        self.get_logger().info('='*60)
        self.get_logger().info('Current Position:')
        self.get_logger().info(f'  x: {self.position[0]:.4f} m')
        self.get_logger().info(f'  y: {self.position[1]:.4f} m')
        self.get_logger().info(f'  z: {self.position[2]:.4f} m')

        # Show IK validation status if enabled
        if self.enable_ik_validation and self.is_reachable is not None:
            if self.is_reachable:
                self.get_logger().info(f'  Status: ✅ REACHABLE (error: {self.last_ik_error*100:.2f}cm)')
            else:
                self.get_logger().info(f'  Status: ❌ UNREACHABLE (error: {self.last_ik_error*100:.2f}cm)')

        self.get_logger().info('')
        self.get_logger().info('YAML format:')
        self.get_logger().info(f'  - x: {self.position[0]:.3f}')
        self.get_logger().info(f'    y: {self.position[1]:.3f}')
        self.get_logger().info(f'    z: {self.position[2]:.3f}')
        self.get_logger().info('='*60)

    def save_waypoint(self):
        """Save current position as waypoint AND persist in visualization"""
        waypoint = list(self.position)

        # Save as waypoint
        self.waypoints.append(waypoint)

        # Also persist for visualization (only if reachable)
        if self.enable_ik_validation and self.is_reachable is False:
            # Unreachable - save waypoint but don't visualize
            self.get_logger().info('='*60)
            self.get_logger().warn(f'⚠️  Waypoint {len(self.waypoints)} saved BUT it is UNREACHABLE!')
            self.get_logger().warn(f'   Position: [{waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f}]')
            self.get_logger().warn(f'   IK Error: {self.last_ik_error*100:.2f}cm')
            self.get_logger().warn(f'   Not added to visualization')
            self.get_logger().info('='*60)
        else:
            # Reachable - save waypoint AND visualize
            self.persisted_points.append(waypoint)

            self.get_logger().info('='*60)
            self.get_logger().info(f'✅ Waypoint {len(self.waypoints)} saved & persisted!')
            self.get_logger().info(f'   Position: [{waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f}]')
            self.get_logger().info(f'   Persisted points: {len(self.persisted_points)}')
            self.get_logger().info(f'   Saved waypoints: {len(self.waypoints)}')
            if self.enable_ik_validation and self.is_reachable is True:
                self.get_logger().info(f'   IK Verified: REACHABLE ✅')
            self.get_logger().info('='*60)

            # Publish updated markers
            self.publish_persisted_markers()

    def print_saved_waypoints(self):
        """Print all saved waypoints in YAML format"""
        if len(self.waypoints) == 0:
            self.get_logger().info('No waypoints saved.')
            return

        self.get_logger().info('')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Saved {len(self.waypoints)} waypoints:')
        self.get_logger().info('='*60)
        self.get_logger().info('target_waypoints:')
        for i, wp in enumerate(self.waypoints):
            self.get_logger().info(f'  # Waypoint {i+1}')
            self.get_logger().info(f'  - x: {wp[0]:.3f}')
            self.get_logger().info(f'    y: {wp[1]:.3f}')
            self.get_logger().info(f'    z: {wp[2]:.3f}')
            self.get_logger().info('')
        self.get_logger().info('='*60)

    def joint_state_callback(self, msg):
        """Callback to receive joint states and calculate end effector position"""
        try:
            # Store joint state
            self.current_joint_state = msg

            # Extract joint angles (assuming joints are named joint_0, joint_1, joint_2, joint_3)
            joint_angles = np.zeros(4)
            for i in range(4):
                joint_name = f'joint_{i}'
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    joint_angles[i] = msg.position[idx]

            # Calculate forward kinematics
            T = self.ik_solver.forward_kinematics(joint_angles)
            self.current_ee_position = T[:3, 3]

        except Exception as e:
            self.get_logger().error(f'Error calculating FK: {e}')

    def move_to_end_effector(self):
        """Move marker to current end effector position"""
        if self.current_ee_position is None:
            self.get_logger().warn('No end effector position available yet. Waiting for joint states...')
            return

        self.position = list(self.current_ee_position)
        self.get_logger().info('='*60)
        self.get_logger().info('Moved to End Effector position:')
        self.get_logger().info(f'  x: {self.position[0]:.4f} m')
        self.get_logger().info(f'  y: {self.position[1]:.4f} m')
        self.get_logger().info(f'  z: {self.position[2]:.4f} m')
        self.get_logger().info('='*60)

    def validate_ik(self):
        """Validate if current position has IK solution"""
        current_pos = np.array(self.position)

        # Check if position changed since last validation
        if self.last_validated_position is not None:
            pos_diff = np.linalg.norm(current_pos - self.last_validated_position)
            if pos_diff < 0.001:  # Less than 1mm change, skip validation
                return self.is_reachable

        # Validate IK for current position
        target = np.array(self.position)
        solution = self.ik_solver.solve_ik_multiple_attempts(
            target,
            num_attempts=5,  # Fewer attempts for real-time performance
            use_global_if_needed=False  # Skip global optimizer for speed
        )

        # Update validation state
        previous_reachable = self.is_reachable
        self.is_reachable = solution['success']
        self.last_ik_error = solution['position_error']
        self.last_validated_position = current_pos.copy()

        # Log if reachability status changed
        if previous_reachable is not None and previous_reachable != self.is_reachable:
            if self.is_reachable:
                self.get_logger().info(f'✅ Position is now REACHABLE (error: {self.last_ik_error*100:.2f}cm)')
            else:
                self.get_logger().warn(f'⚠️  Position is now UNREACHABLE (error: {self.last_ik_error*100:.2f}cm)')

        return self.is_reachable

    def persist_current_point(self):
        """Persist current point in visualization AND save as waypoint (only if IK valid)"""
        if self.enable_ik_validation and not self.is_reachable:
            self.get_logger().warn('⚠️  Cannot persist UNREACHABLE point!')
            self.get_logger().warn(f'   Position: [{self.position[0]:.3f}, {self.position[1]:.3f}, {self.position[2]:.3f}]')
            self.get_logger().warn(f'   IK Error: {self.last_ik_error*100:.2f}cm')
            return

        # Add point to persisted list (for visualization)
        point = list(self.position)
        self.persisted_points.append(point)

        # ALSO save as waypoint (for interpolation and export)
        self.waypoints.append(list(self.position))

        self.get_logger().info('='*60)
        self.get_logger().info(f'✅ Point {len(self.persisted_points)} persisted & saved as waypoint!')
        self.get_logger().info(f'   Position: [{point[0]:.3f}, {point[1]:.3f}, {point[2]:.3f}]')
        self.get_logger().info(f'   Persisted points: {len(self.persisted_points)}')
        self.get_logger().info(f'   Saved waypoints: {len(self.waypoints)}')
        self.get_logger().info('='*60)

        # Publish updated markers
        self.publish_persisted_markers()

    def clear_persisted_points(self):
        """Clear all persisted points and interpolated trajectory"""
        num_persisted = len(self.persisted_points)
        num_interpolated = len(self.interpolated_points)

        self.persisted_points.clear()
        self.interpolated_points.clear()

        self.get_logger().info('='*60)
        self.get_logger().info(f'🧹 Cleared visualization:')
        self.get_logger().info(f'   Persisted points: {num_persisted}')
        self.get_logger().info(f'   Interpolated points: {num_interpolated}')
        self.get_logger().info('='*60)

        # Publish empty marker array to clear visualization
        self.publish_persisted_markers()

    def interpolate_and_visualize(self):
        """Interpolate between saved waypoints and visualize trajectory"""
        if len(self.waypoints) < 2:
            self.get_logger().warn('⚠️  Need at least 2 waypoints to interpolate!')
            self.get_logger().warn(f'   Currently saved: {len(self.waypoints)} waypoint(s)')
            return

        self.get_logger().info('='*60)
        self.get_logger().info(f'🔄 Interpolating trajectory between {len(self.waypoints)} waypoints...')

        try:
            # Convert waypoints to numpy array
            waypoints_array = np.array(self.waypoints)

            # Create parameter t (0 to 1) for interpolation
            t = np.linspace(0, 1, len(self.waypoints))

            # Create cubic splines for x, y, z
            cs_x = CubicSpline(t, waypoints_array[:, 0])
            cs_y = CubicSpline(t, waypoints_array[:, 1])
            cs_z = CubicSpline(t, waypoints_array[:, 2])

            # Generate interpolated points (50 points per segment)
            num_points = (len(self.waypoints) - 1) * 50
            t_interp = np.linspace(0, 1, num_points)

            x_interp = cs_x(t_interp)
            y_interp = cs_y(t_interp)
            z_interp = cs_z(t_interp)

            # Store interpolated points
            self.interpolated_points = []
            for i in range(num_points):
                self.interpolated_points.append([x_interp[i], y_interp[i], z_interp[i]])

            self.get_logger().info(f'✅ Interpolation complete!')
            self.get_logger().info(f'   Generated {len(self.interpolated_points)} interpolated points')
            self.get_logger().info(f'   Using cubic spline interpolation')
            self.get_logger().info('='*60)

            # Publish updated markers
            self.publish_persisted_markers()

        except Exception as e:
            self.get_logger().error(f'❌ Interpolation failed: {e}')
            self.get_logger().info('='*60)

    def publish_persisted_markers(self):
        """Publish all persisted points and interpolated trajectory as markers"""
        marker_array = MarkerArray()

        # Delete all previous markers first
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.persisted_markers_pub.publish(marker_array)

        # Clear array and rebuild
        marker_array = MarkerArray()

        # Add persisted points (green spheres, slightly smaller than main marker)
        for i, point in enumerate(self.persisted_points):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'persisted_points'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.008
            marker.scale.y = 0.008
            marker.scale.z = 0.008

            # Dark green color for persisted points
            marker.color = ColorRGBA(r=0.0, g=0.6, b=0.0, a=0.8)

            marker_array.markers.append(marker)

        # Add interpolated trajectory points (blue spheres, very small)
        for i, point in enumerate(self.interpolated_points):
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'interpolated_trajectory'
            marker.id = i + 1000  # Offset ID to avoid collision
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = point[2]
            marker.pose.orientation.w = 1.0

            marker.scale.x = 0.003
            marker.scale.y = 0.003
            marker.scale.z = 0.003

            # Blue color for interpolated trajectory
            marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.6)

            marker_array.markers.append(marker)

        # Publish all markers
        if len(marker_array.markers) > 0:
            self.persisted_markers_pub.publish(marker_array)

    def publish_marker(self):
        """Publish marker visualization"""
        # Validate IK if enabled
        if self.enable_ik_validation:
            is_reachable = self.validate_ik()
        else:
            is_reachable = None

        # Create marker
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'interactive_marker'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = self.position[0]
        marker.pose.position.y = self.position[1]
        marker.pose.position.z = self.position[2]
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        # Set color based on IK validation
        if is_reachable is None:
            # IK validation disabled - use cyan/turquoise
            marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        elif is_reachable:
            # Position is reachable - GREEN
            marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        else:
            # Position is unreachable - RED
            marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)

        self.marker_pub.publish(marker)

        # Publish position as PointStamped
        point_msg = PointStamped()
        point_msg.header.frame_id = 'base_link'
        point_msg.header.stamp = self.get_clock().now().to_msg()
        point_msg.point.x = self.position[0]
        point_msg.point.y = self.position[1]
        point_msg.point.z = self.position[2]

        self.position_pub.publish(point_msg)


def main(args=None):
    rclpy.init(args=args)
    node = InteractiveMarkerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
