#!/usr/bin/env python3
"""
SMILEi Robot Dashboard
Dashboard with IMGUI for robot visualization and state control
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import time

# IMGUI imports
try:
    import imgui
    from imgui.integrations.glfw import GlfwRenderer
    import glfw
    import OpenGL.GL as gl
    IMGUI_AVAILABLE = True
except ImportError:
    IMGUI_AVAILABLE = False
    print("WARNING: ImGui dependencies not available. Install with:")
    print("pip install imgui[glfw] PyOpenGL")


class DashboardNode(Node):
    """ROS 2 Node for SMILEi Dashboard"""

    def __init__(self):
        super().__init__('dashboard_node')

        # Robot selection: "operador" or "seguidor"
        self.selected_robot = "operador"

        # State management (protegido con lock para thread-safety)
        self._state_lock = threading.Lock()
        self._current_state = "idle"
        self.last_completed_state = None
        self._last_state_update_time = None  # None = nunca ha recibido mensaje

        # IP Configuration for remote teleoperation
        self.operador_ip = "192.168.0.100"
        self.seguidor_ip = "192.168.0.2"

        # Publishers and subscribers (will be updated when robot changes)
        self.state_command_pub = None
        self.state_sub = None

        # Publisher for IP configuration
        self.ip_config_pub = self.create_publisher(
            String,
            '/teleoperation_ip_config',
            10
        )

        self.update_robot_topics()

        # Available states from state machine
        self.available_states = [
            "enable",
            "home",
            "zero",
            "say_hello",
            "remote_teleoperation",
            "disable"
        ]

        # GUI state
        self.gui_running = True

        self.get_logger().info('Dashboard node initialized')

        # Publicar configuración de IPs inicial
        time.sleep(0.5)  # Esperar a que los subscribers se conecten
        self.update_ip_config(self.operador_ip, self.seguidor_ip)

    @property
    def current_state(self):
        """Thread-safe getter para el estado actual"""
        with self._state_lock:
            return self._current_state

    @current_state.setter
    def current_state(self, value):
        """Thread-safe setter para el estado actual"""
        with self._state_lock:
            self._current_state = value
            self._last_state_update_time = time.time()

    def state_callback(self, msg):
        """Callback for state updates (thread-safe)"""
        with self._state_lock:
            self._current_state = msg.data
            self._last_state_update_time = time.time()

    def is_state_machine_responsive(self, timeout_sec=2.0):
        """Verifica si la state machine está respondiendo (último update reciente)"""
        with self._state_lock:
            # Si nunca ha recibido un mensaje, no está respondiendo
            if self._last_state_update_time is None:
                return False
            time_since_update = time.time() - self._last_state_update_time
            return time_since_update < timeout_sec

    def update_robot_topics(self):
        """Update publishers and subscribers based on selected robot"""
        # Destroy old publishers/subscribers if they exist
        if self.state_command_pub:
            self.destroy_publisher(self.state_command_pub)
        if self.state_sub:
            self.destroy_subscription(self.state_sub)

        # Create new publisher for state commands
        state_command_topic = f'/{self.selected_robot}/state_command'
        self.state_command_pub = self.create_publisher(
            String,
            state_command_topic,
            10
        )

        # Create new subscriber for state feedback
        current_state_topic = f'/{self.selected_robot}/current_state'
        self.state_sub = self.create_subscription(
            String,
            current_state_topic,
            self.state_callback,
            10
        )

        self.get_logger().info(f'Updated topics for robot: {self.selected_robot}')
        self.get_logger().info(f'  - Publishing to: {state_command_topic}')
        self.get_logger().info(f'  - Subscribing to: {current_state_topic}')

    def set_robot(self, robot_name):
        """Change the selected robot (operador or seguidor)"""
        if robot_name in ["operador", "seguidor"]:
            self.selected_robot = robot_name
            self.update_robot_topics()
            # Reset state when switching robots - poner None hasta recibir mensaje del nuevo robot
            with self._state_lock:
                self._current_state = "idle"
                self._last_state_update_time = None  # Esperar primer mensaje del nuevo robot
            return True
        else:
            self.get_logger().error(f'Invalid robot name: {robot_name}')
            return False

    def publish_state_command(self, state_name):
        """Publish a state change command"""
        msg = String()
        msg.data = state_name
        self.state_command_pub.publish(msg)
        self.get_logger().info(f'Published state command to /{self.selected_robot}/state_command: {state_name}')

    def update_ip_config(self, operador_ip, seguidor_ip):
        """Update IP configuration for teleoperation"""
        self.operador_ip = operador_ip
        self.seguidor_ip = seguidor_ip

        # Publish configuration as JSON string
        import json
        config = {
            "operador_ip": operador_ip,
            "seguidor_ip": seguidor_ip
        }
        msg = String()
        msg.data = json.dumps(config)
        self.ip_config_pub.publish(msg)

        self.get_logger().info(f'Updated IP config - Operador: {operador_ip}, Seguidor: {seguidor_ip}')


class DashboardGUI:
    """IMGUI Dashboard GUI"""

    def __init__(self, node):
        self.node = node

        # Temporary buffers for IP input
        self.operador_ip_buffer = self.node.operador_ip
        self.seguidor_ip_buffer = self.node.seguidor_ip

        if not IMGUI_AVAILABLE:
            self.node.get_logger().error("ImGui not available!")
            return

        # Initialize GLFW
        if not glfw.init():
            self.node.get_logger().error("Failed to initialize GLFW")
            return

        # Create window
        glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
        glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
        glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
        glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, gl.GL_TRUE)

        self.window = glfw.create_window(
            1280, 720, "SMILEi Robot Dashboard", None, None
        )

        if not self.window:
            glfw.terminate()
            self.node.get_logger().error("Failed to create GLFW window")
            return

        glfw.make_context_current(self.window)
        glfw.swap_interval(1)  # Enable vsync

        # Initialize ImGui
        imgui.create_context()
        self.impl = GlfwRenderer(self.window)

        # Style configuration
        self.configure_style()

        self.node.get_logger().info("Dashboard GUI initialized")

    def configure_style(self):
        """Configure ImGui style"""
        style = imgui.get_style()
        style.window_rounding = 5.0
        style.frame_rounding = 4.0
        style.grab_rounding = 3.0

        # Remove window padding to allow positioning at (0,0)
        style.window_padding = (8, 8)
        style.window_border_size = 1.0

        # Dark theme colors
        imgui.style_colors_dark()

    def render_robot_configuration_panel(self):
        """Render robot configuration panel with selection and IP config"""
        # Set fixed window position and size
        imgui.set_next_window_position(0, 0)
        imgui.set_next_window_size(300, 220)

        imgui.begin("Robot Configuration", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_COLLAPSE)

        # Robot selection
        imgui.text("Selected Robot:")
        imgui.spacing()

        # Radio buttons for robot selection (always enabled)
        if imgui.radio_button("Operador", self.node.selected_robot == "operador"):
            self.node.set_robot("operador")

        imgui.same_line()

        if imgui.radio_button("Seguidor", self.node.selected_robot == "seguidor"):
            self.node.set_robot("seguidor")

        imgui.separator()

        # IP Configuration section
        imgui.text("Teleoperation IPs:")
        imgui.spacing()

        # Operador IP
        imgui.text("Operador IP:")
        changed, self.operador_ip_buffer = imgui.input_text(
            "##operador_ip",
            self.operador_ip_buffer,
            256
        )

        imgui.spacing()

        # Seguidor IP
        imgui.text("Seguidor IP:")
        changed, self.seguidor_ip_buffer = imgui.input_text(
            "##seguidor_ip",
            self.seguidor_ip_buffer,
            256
        )

        imgui.spacing()

        # Apply button
        if imgui.button("Apply IPs", 280, 30):
            self.node.update_ip_config(self.operador_ip_buffer, self.seguidor_ip_buffer)

        imgui.end()

    def render_state_control_panel(self):
        """Render the state control panel with buttons"""
        # Set fixed window position and size - next to robot configuration
        imgui.set_next_window_position(0, 220)
        imgui.set_next_window_size(300, 280)

        imgui.begin("Robot State Control", flags=imgui.WINDOW_NO_RESIZE | imgui.WINDOW_NO_MOVE | imgui.WINDOW_NO_COLLAPSE)

        imgui.text(f"Robot: {self.node.selected_robot.upper()}")
        imgui.text(f"Current State: {self.node.current_state}")

        # Indicador de heartbeat (si la state machine está respondiendo)
        with self.node._state_lock:
            last_update = self.node._last_state_update_time

        if last_update is None:
            # Nunca ha recibido mensajes
            imgui.push_style_color(imgui.COLOR_TEXT, 0.8, 0.6, 0.2, 1.0)  # Naranja
            imgui.text("State Machine: WAITING...")
            imgui.pop_style_color(1)
        elif self.node.is_state_machine_responsive(timeout_sec=2.0):
            # Respondiendo normalmente
            imgui.push_style_color(imgui.COLOR_TEXT, 0.2, 0.8, 0.2, 1.0)  # Verde
            imgui.text("State Machine: RESPONSIVE")
            imgui.pop_style_color(1)
        else:
            # Dejó de responder
            time_since = time.time() - last_update
            imgui.push_style_color(imgui.COLOR_TEXT, 0.8, 0.2, 0.2, 1.0)  # Rojo
            imgui.text(f"State Machine: TIMEOUT ({time_since:.1f}s)")
            imgui.pop_style_color(1)

        imgui.separator()

        imgui.text("State Commands:")
        imgui.spacing()

        # Create buttons in a grid layout (2 columns)
        button_width = 135
        button_height = 40

        states_per_row = 2

        # Set smaller font scale for buttons only
        io = imgui.get_io()
        original_font_scale = io.font_global_scale
        io.font_global_scale = 0.5

        for i, state in enumerate(self.node.available_states):
            # Color coding for different state types
            if state == "idle":
                imgui.push_style_color(imgui.COLOR_BUTTON, 0.4, 0.4, 0.4, 1.0)
            elif state == "enable":
                imgui.push_style_color(imgui.COLOR_BUTTON, 0.2, 0.7, 0.2, 1.0)
            elif state == "disable":
                imgui.push_style_color(imgui.COLOR_BUTTON, 0.7, 0.2, 0.2, 1.0)
            elif state in ["home", "zero"]:
                imgui.push_style_color(imgui.COLOR_BUTTON, 0.2, 0.5, 0.7, 1.0)
            elif "remote_teleoperation" in state:
                imgui.push_style_color(imgui.COLOR_BUTTON, 0.7, 0.5, 0.2, 1.0)
            else:
                imgui.push_style_color(imgui.COLOR_BUTTON, 0.3, 0.3, 0.6, 1.0)

            # Check if button is active (current state)
            is_active = (state == self.node.current_state)

            if imgui.button(f"{state.upper()}", button_width, button_height):
                self.node.publish_state_command(state)

            imgui.pop_style_color(1)

            # Layout: 2 buttons per row
            if (i + 1) % states_per_row != 0:
                imgui.same_line()

        # Restore original font scale
        io.font_global_scale = original_font_scale

        imgui.end()

    def render(self):
        """Main render loop"""
        while not glfw.window_should_close(self.window) and self.node.gui_running:
            # Poll events
            glfw.poll_events()
            self.impl.process_inputs()

            # Start new frame
            imgui.new_frame()

            # Render dashboard panels
            self.render_robot_configuration_panel()
            self.render_state_control_panel()

            # Rendering
            gl.glClearColor(0.1, 0.1, 0.1, 1.0)
            gl.glClear(gl.GL_COLOR_BUFFER_BIT)

            imgui.render()
            self.impl.render(imgui.get_draw_data())

            glfw.swap_buffers(self.window)

        # Cleanup
        self.impl.shutdown()
        glfw.terminate()
        self.node.gui_running = False
        self.node.get_logger().info("Dashboard GUI closed")


def main(args=None):
    """Main entry point"""

    if not IMGUI_AVAILABLE:
        print("ERROR: ImGui dependencies not available!")
        print("Install with: pip install imgui[glfw] PyOpenGL")
        return

    # Initialize ROS 2
    rclpy.init(args=args)

    # Create node
    node = DashboardNode()

    # Create GUI
    gui = DashboardGUI(node)

    # Run ROS 2 spinning in a separate thread
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    # Run GUI in main thread
    try:
        gui.render()
    except KeyboardInterrupt:
        pass
    finally:
        node.gui_running = False
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
