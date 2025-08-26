import time
import rclpy
from rclpy.node import Node
import py_trees
import threading
from std_msgs.msg import String
from smilei_state_machine.behaviors.home_position import HomePosition
from smilei_state_machine.behaviors.zero_position import ZeroPosition
from smilei_state_machine.behaviors.enable_robot import EnableRobot
from smilei_state_machine.behaviors.say_hello import SayHello
from smilei_state_machine.behaviors.local_teleoperation import LocalTeleoperation
from smilei_state_machine.behaviors.disable_robot import DisableRobot

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
                        # Llamada con timeout explícito de 5.0 segundos
                        if self.node:
                            self.node.get_logger().info(f"Configurando comportamiento {state_name} con timeout")
                        behavior.setup(timeout_sec=5.0)
                    else:
                        # Llamada sin argumentos
                        if self.node:
                            self.node.get_logger().info(f"Configurando comportamiento {state_name}")
                        behavior.setup()
                except Exception as e:
                    if self.node:
                        self.node.get_logger().error(f"Error al configurar comportamiento {state_name}: {str(e)}")
        
        self.behaviors_setup_done = True
        if self.node:
            self.node.get_logger().info("Todos los comportamientos configurados")
    
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

def state_command_callback(msg):
    """Callback para recibir comandos de estado"""
    global current_state_command, last_completed_state
    if current_state_command != msg.data:
        print(f"Cambiando a estado: {msg.data}")
        current_state_command = msg.data
        # Reset último estado completado cuando cambiamos de comando
        last_completed_state = None

def spin_ros(node):
    """Funcion para ejecutar el spin de ROS en un hilo separado"""
    rclpy.spin(node)

def main():
    rclpy.init()
    node = Node('state_machine_node')

    # Crear suscriptor para recibir comandos de estado
    state_command_sub = node.create_subscription(
        String,
        'state_command',
        state_command_callback,
        10
    )

    # Define the motor IDs to control
    motor_ids = [6, 1]

    # Añadir estado idle para evitar advertencias
    node.get_logger().info(f"Iniciando la máquina de estados con motores: {motor_ids}")
    
    # Añadir una pausa para asegurar que ROS está inicializado
    time.sleep(2.0)
    
    # Comprobar si los servicios están disponibles
    try:
        node.get_logger().info("Comprobando servicios disponibles...")
        from rclpy.qos import QoSProfile
        from rclpy.duration import Duration
        
        # Intentar obtener la lista de servicios disponibles
        services = node.get_service_names_and_types()
        
        available_services = [s[0] for s in services]
        node.get_logger().info(f"Servicios disponibles: {available_services}")
        
        # Advertir si faltan servicios importantes
        expected_services = [
            '/westwood_motor/set_motor_id_and_target',
            '/westwood_motor/get_motor_positions',
            '/westwood_motor/get_available_motors',
            '/westwood_motor/set_position_gains',
            '/westwood_motor/set_mode',
            '/westwood_motor/set_torque_enable',
            '/westwood_motor/set_goal_iq'
        ]
        
        for service in expected_services:
            if service not in available_services:
                node.get_logger().warn(f"Servicio esperado no disponible: {service}")
    except Exception as e:
        node.get_logger().error(f"Error al comprobar servicios: {str(e)}")

    # Crear comportamientos (pasando el nodo a cada uno)
    idle = py_trees.behaviours.Running(name="IdleBehavior")  # Comportamiento simple para estado idle
    enable = EnableRobot(name="EnableRobot", motor_ids=motor_ids, node=node)
    home = HomePosition(name="GoHome", motor_ids=motor_ids, node=node)
    zero = ZeroPosition(name="GoZero", motor_ids=motor_ids, node=node)
    say_hello = SayHello(name="SayHello", motor_ids=motor_ids, node=node)
    teleoperation = LocalTeleoperation(name="LocalTeleoperation", node=node)
    disable = DisableRobot(name="DisableRobot", motor_ids=motor_ids, node=node)

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
    root.add_state("disable", disable)

    # Crear el árbol de comportamiento
    tree = py_trees.trees.BehaviourTree(root)
    tree.setup()

    # Iniciar spin de ROS en un hilo separado
    spin_thread = threading.Thread(target=spin_ros, args=(node,))
    spin_thread.daemon = True
    spin_thread.start()

    node.get_logger().info("Starting state machine...")
    try:
        while rclpy.ok():
            tree.tick()
            node.get_logger().debug(f"Estado actual: {current_state_command}, Completado: {last_completed_state}")
            time.sleep(0.1)  # Actualización más frecuente
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