import time
import rclpy
from rclpy.node import Node
import py_trees
from std_msgs.msg import String
from smilei_state_machine.behaviors.home_position import HomePosition
from smilei_state_machine.behaviors.zero_position import ZeroPosition
from smilei_state_machine.behaviors.enable_robot import EnableRobot
from smilei_state_machine.behaviors.say_hello import SayHello
from smilei_state_machine.behaviors.local_teleoperation import LocalTeleoperation
from smilei_state_machine.behaviors.remote_teleoperation import RemoteTeleoperation
from smilei_state_machine.behaviors.disable_robot import DisableRobot
from smilei_state_machine.behaviors.autonomous_gesture_execution import AutonomousGestureExecution
from smilei_state_machine.behaviors.autonomous_gesture_current_control import AutonomousGestureCurrentControl
from smilei_state_machine.hardware_manager import HardwareManager

# Variable global para almacenar el comando actual
current_state_command = "idle"
last_completed_state = None

# Comportamiento personalizado que selecciona directamente el comportamiento según el estado
class StateMachineRoot(py_trees.behaviour.Behaviour):
    def __init__(self, name="StateMachine"):
        super().__init__(name)
        self.state_behaviors = {}
        self.current_behavior = None
        self.node = None
        self.behaviors_setup_done = False
        # Atributos para depuración de tiempo
        self.sm_debug = False
        self.last_log_time = 0.0
        self.tick_count = 0
        self.last_tick_time = 0.0
        # Atributo para limitar logs de éxito
        self.last_success_log_time = {}
    
    def setup_with_node(self, node):
        self.node = node
        # No configuramos los comportamientos aquí, lo haremos en la primera actualización
        # cuando ROS esté completamente inicializado

        # Leer el parámetro de depuración
        self.node.declare_parameter('debug', False)
        self.sm_debug = self.node.get_parameter('debug').value
        if self.sm_debug:
            self.node.get_logger().info("Depuración de tiempo de la máquina de estados HABILITADA.")
            self.last_log_time = time.time()
            self.last_tick_time = time.time()
    
    def add_state(self, state_name, behavior):
        self.state_behaviors[state_name] = behavior
    
    def setup_all_behaviors(self):
        """Configura todos los comportamientos una vez que ROS está inicializado"""
        if self.behaviors_setup_done:
            return
        
        if self.node:
            self.node.get_logger().info("Configurando todos los comportamientos...")
            
        # Configurar todos los comportamientos
        for state_name, behavior in self.state_behaviors.items():
            if hasattr(behavior, 'setup'):
                try:
                    # Primero verificamos si el comportamiento tiene un método setup con argumentos
                    import inspect
                    sig = inspect.signature(behavior.setup)
                    if len(sig.parameters) > 0 and 'timeout_sec' in sig.parameters:
                        # Llamada con timeout muy corto ya que usamos Pybear directamente
                        if self.node:
                            self.node.get_logger().info(f"Configurando comportamiento {state_name} con Pybear")
                        result = behavior.setup(timeout_sec=0.1)  # Timeout muy corto
                        if not result:
                            if self.node:
                                self.node.get_logger().warning(f"Setup fallido para {state_name}, pero continuando...")
                    else:
                        # Llamada sin argumentos
                        if self.node:
                            self.node.get_logger().info(f"Configurando comportamiento {state_name}")
                        behavior.setup()
                except Exception as e:
                    if self.node:
                        self.node.get_logger().warning(f"Error al configurar comportamiento {state_name}: {str(e)} - Continuando de todos modos...")
                        # No fallar completamente, solo advertir
        
        self.behaviors_setup_done = True
        if self.node:
            self.node.get_logger().info("Configuración de comportamientos completada (con Pybear directo)")
    
    def initialise(self):
        global current_state_command, last_completed_state
        # No hay inicialización específica necesaria
        if self.node:
            self.node.get_logger().info(f"State machine inicializada, estado actual: {current_state_command}")
    
    def update(self):
        global current_state_command, last_completed_state
        
        start_time = time.time()

        # Configurar todos los comportamientos si no lo hemos hecho ya
        if not self.behaviors_setup_done:
            self.setup_all_behaviors()
        
        # Si el estado solicitado no existe, devolver RUNNING (esperar a que se solicite un estado válido)
        if current_state_command not in self.state_behaviors:
            if self.node:
                self.node.get_logger().warn(f"Estado solicitado '{current_state_command}' no existe")
            return py_trees.common.Status.RUNNING
        
        # Si cambiamos de comportamiento, finalizar el anterior e inicializar el nuevo
        if self.current_behavior is None or self.state_behaviors[current_state_command] != self.current_behavior:
            # Finalizar el comportamiento anterior si existe
            if self.current_behavior is not None:
                try:
                    self.current_behavior.terminate(py_trees.common.Status.INVALID)
                except Exception as e:
                    if self.node:
                        self.node.get_logger().error(f"Error al terminar comportamiento: {str(e)}")
            
            # Actualizar el comportamiento actual
            self.current_behavior = self.state_behaviors[current_state_command]
            
            # Inicializar el nuevo comportamiento
            try:
                self.current_behavior.initialise()
                if self.node:
                    self.node.get_logger().info(f"Iniciando comportamiento {self.current_behavior.name}")
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Error al inicializar comportamiento: {str(e)}")
                return py_trees.common.Status.RUNNING
        
        # Ejecutar el comportamiento actual
        try:
            status = self.current_behavior.update()
            
            # Si el comportamiento completó con éxito, registrarlo
            if status == py_trees.common.Status.SUCCESS:
                last_completed_state = current_state_command
                
                current_time = time.time()
                last_log_time_for_state = self.last_success_log_time.get(current_state_command, 0.0)

                # Limitar el log a una vez cada 0.5 segundos por estado
                if self.node and (current_time - last_log_time_for_state > 0.5):
                    self.node.get_logger().info(f"Estado {current_state_command} completado con éxito")
                    self.last_success_log_time[current_state_command] = current_time

            # Lógica de depuración de tiempo
            if self.sm_debug and self.node:
                iteration_time_ms = (time.time() - start_time) * 1000
                self.tick_count += 1
                current_time = time.time()
                delta_time = current_time - self.last_log_time
                
                if delta_time >= 1.0:
                    frequency = self.tick_count / delta_time
                    self.node.get_logger().info(
                        f"[DEBUG] Frecuencia SM: {frequency:.2f} Hz | Tiempo de iteración: {iteration_time_ms:.3f} ms"
                    )
                    self.tick_count = 0
                    self.last_log_time = current_time
            
            return py_trees.common.Status.RUNNING  # La máquina de estados siempre está ejecutándose
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error al actualizar comportamiento: {str(e)}")
            return py_trees.common.Status.RUNNING
    
    def terminate(self, new_status):
        # Terminar el comportamiento actual si existe
        if self.current_behavior is not None:
            try:
                self.current_behavior.terminate(new_status)
            except Exception as e:
                if self.node:
                    self.node.get_logger().error(f"Error al terminar comportamiento: {str(e)}")

def state_command_callback(msg, node):
    """Callback para recibir comandos de estado"""
    global current_state_command, last_completed_state
    node.get_logger().info(f"📡 Callback recibido! Comando: {msg.data}, Estado actual: {current_state_command}")
    if current_state_command != msg.data:
        node.get_logger().info(f"🔄 Cambiando a estado: {msg.data}")
        current_state_command = msg.data
        # Reset último estado completado cuando cambiamos de comando
        last_completed_state = None
    else:
        node.get_logger().info(f"⚠️ Mismo estado solicitado: {msg.data}")

# Función eliminada - ahora usamos spin_once en el loop principal

def main():
    rclpy.init()
    
    # El nombre del nodo se establece en el launch file.
    # El namespace se aplica automáticamente desde la línea de comandos (ej: __ns:=/operador)
    node = Node('state_machine_node')

    # El namespace del nodo se aplica automáticamente a los topics.
    state_command_sub = node.create_subscription(
        String,
        'state_command',
        lambda msg: state_command_callback(msg, node),
        10
    )

    # Añadir una pausa para asegurar que ROS está inicializado
    time.sleep(2.0)

    # Declarar y obtener los parámetros del hardware manager.
    # Los valores por defecto se usan si no se encuentran en el yaml.
    node.declare_parameter('hardware_manager.usb_ports', ['/dev/ttyUSB0'])
    node.declare_parameter('hardware_manager.baudrate', 8000000)
    node.declare_parameter('hardware_manager.auto_detect', True)
    node.declare_parameter('hardware_manager.debug', False)

    usb_ports = node.get_parameter('hardware_manager.usb_ports').value
    baudrate = node.get_parameter('hardware_manager.baudrate').value
    auto_detect = node.get_parameter('hardware_manager.auto_detect').value
    debug = node.get_parameter('hardware_manager.debug').value
    
    # Inicializar hardware manager robusto
    node.get_logger().info("Inicializando hardware manager robusto...")
    hardware_manager = HardwareManager(
        node=node,
        usb_ports=usb_ports,
        baudrate=baudrate,
        auto_detect=auto_detect,
        debug=debug
    )

    # Obtener motor IDs dinámicamente desde el hardware manager
    motor_ids = hardware_manager.get_available_motors()
    
    # Si no hay motores detectados, usar [1] como fallback para modo simulación
    if not motor_ids:
        motor_ids = [1]
        node.get_logger().warning("No se detectaron motores, usando motor ID [1] como fallback")
    
    node.get_logger().info(f"Iniciando la máquina de estados con motores detectados: {motor_ids}")

    # Crear comportamiento idle personalizado
    class IdleBehavior(py_trees.behaviour.Behaviour):
        def __init__(self, name="IdleBehavior"):
            super().__init__(name)
        def setup(self, timeout_sec=None, **kwargs):
            return True
        def initialise(self):
            pass
        def update(self):
            return py_trees.common.Status.RUNNING
        def terminate(self, new_status):
            pass
    
    idle = IdleBehavior()  # Comportamiento simple para estado idle
    enable = EnableRobot(name="EnableRobot", motor_ids=motor_ids, node=node, hardware_manager=hardware_manager)
    home = HomePosition(name="GoHome", motor_ids=motor_ids, node=node, hardware_manager=hardware_manager)
    zero = ZeroPosition(name="GoZero", motor_ids=motor_ids, node=node, hardware_manager=hardware_manager)
    say_hello = SayHello(name="SayHello", motor_ids=motor_ids, node=node, hardware_manager=hardware_manager)
    teleoperation = LocalTeleoperation(name="LocalTeleoperation", motor_ids=motor_ids, node=node, hardware_manager=hardware_manager)
    remote_teleoperation = RemoteTeleoperation(name="RemoteTeleoperation", motor_ids=motor_ids, node=node, hardware_manager=hardware_manager)
    disable = DisableRobot(name="DisableRobot", motor_ids=motor_ids, node=node, hardware_manager=hardware_manager)
    autonomous_gesture = AutonomousGestureExecution(name="AutonomousGestureExecution", motor_ids=motor_ids, node=node, hardware_manager=hardware_manager)
    autonomous_gesture_current = AutonomousGestureCurrentControl(name="AutonomousGestureCurrentControl", node=node, hardware_manager=hardware_manager)

    # Crear comportamiento raíz personalizado
    root = StateMachineRoot()
    root.setup_with_node(node)
    
    # Añadir estados a la máquina
    root.add_state("idle", idle)
    root.add_state("enable", enable)
    root.add_state("home", home)
    root.add_state("zero", zero)
    root.add_state("say_hello", say_hello)
    root.add_state("teleoperation", teleoperation)
    root.add_state("remote_teleoperation", remote_teleoperation)
    root.add_state("disable", disable)
    root.add_state("autonomous_gesture", autonomous_gesture)
    root.add_state("autonomous_gesture_current", autonomous_gesture_current)

    # Crear el árbol de comportamiento
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup()

    node.get_logger().info("Starting state machine...")
    try:
        while rclpy.ok():
            # Procesar callbacks de ROS primero
            rclpy.spin_once(node, timeout_sec=0.001)
            
            # Luego ejecutar el árbol de comportamiento
            tree.tick()
            node.get_logger().debug(f"Estado actual: {current_state_command}, Completado: {last_completed_state}")
            
            # Pequeña pausa para no saturar el CPU
            time.sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down state machine")
        # No destruir el nodo aquí, se hará después de salir del bucle
        rclpy.shutdown()
        # Ahora podemos destruir el nodo con seguridad
        node.destroy_node()

if __name__ == '__main__':
    main()