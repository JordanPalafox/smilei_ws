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
    
    def setup_with_node(self, node):
        self.node = node
        # No configuramos los comportamientos aquí, lo haremos en la primera actualización
        # cuando ROS esté completamente inicializado
    
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
                if self.node:
                    self.node.get_logger().info(f"Estado {current_state_command} completado con éxito")
            
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
    
    # Temporary node to get robot_name
    temp_node = rclpy.create_node('temp_state_machine_parser')
    temp_node.declare_parameter('robot_name', '')
    robot_name = temp_node.get_parameter('robot_name').value
    temp_node.destroy_node()

    node_name = 'state_machine_node'
    if robot_name:
        node_name = f'{robot_name}_{node_name}'

    node = Node(node_name)

    node.declare_parameter('robot_name', '')
    robot_name = node.get_parameter('robot_name').value

    def get_topic_name(topic_name):
        if robot_name:
            return f'/{robot_name}/{topic_name.lstrip("/")}'
        return topic_name

    # Crear suscriptor para recibir comandos de estado
    state_command_sub = node.create_subscription(
        String,
        get_topic_name('state_command'),
        lambda msg: state_command_callback(msg, node),
        10
    )

    # Añadir una pausa para asegurar que ROS está inicializado
    time.sleep(2.0)
    
    # Inicializar hardware manager robusto
    node.get_logger().info("Inicializando hardware manager robusto...")
    hardware_manager = HardwareManager(
        node=node,
        usb_ports=['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3'],
        baudrate=8000000,
        auto_detect=True,
        debug=False
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
    enable = EnableRobot(name="EnableRobot", motor_ids=motor_ids, node=node, robot_name=robot_name, hardware_manager=hardware_manager)
    home = HomePosition(name="GoHome", motor_ids=motor_ids, node=node, robot_name=robot_name, hardware_manager=hardware_manager)
    zero = ZeroPosition(name="GoZero", motor_ids=motor_ids, node=node, robot_name=robot_name, hardware_manager=hardware_manager)
    say_hello = SayHello(name="SayHello", motor_ids=motor_ids, node=node, robot_name=robot_name, hardware_manager=hardware_manager)
    teleoperation = LocalTeleoperation(name="LocalTeleoperation", motor_ids=motor_ids, node=node, robot_name=robot_name, hardware_manager=hardware_manager)
    remote_teleoperation = RemoteTeleoperation(name="RemoteTeleoperation", motor_ids=motor_ids, node=node, robot_name=robot_name, hardware_manager=hardware_manager)
    disable = DisableRobot(name="DisableRobot", motor_ids=motor_ids, node=node, robot_name=robot_name, hardware_manager=hardware_manager)

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