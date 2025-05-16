import time
import rclpy
from rclpy.node import Node
import py_trees
import threading
from std_msgs.msg import String
from smilei_state_machine.behaviors.home_position import HomePosition
from smilei_state_machine.behaviors.zero_position import ZeroPosition
from smilei_state_machine.behaviors.enable_robot import EnableRobot

# Variable global para almacenar el comando actual
current_state_command = "idle"
last_completed_state = None

# Clase para controlar estados basados en un comando
class StateControl(py_trees.decorators.Decorator):
    def __init__(self, name: str, state_name: str, child: py_trees.behaviour.Behaviour):
        super().__init__(name=name, child=child)
        self.state_name = state_name
        self.completed = False
    
    def update(self) -> py_trees.common.Status:
        global current_state_command, last_completed_state
        
        # Si este estado ya completó su ejecución y seguimos en el mismo comando,
        # devolver SUCCESS sin volver a ejecutar el comportamiento
        if self.completed and current_state_command == self.state_name:
            return py_trees.common.Status.SUCCESS
        
        # Si cambiamos a un nuevo comando, resetear el estado de completado
        if last_completed_state != current_state_command:
            self.completed = False
        
        # Si este es el estado actual solicitado y no ha completado
        if current_state_command == self.state_name and not self.completed:
            # Ejecutar el comportamiento decorado
            status = self.decorated.status
            
            # Si el comportamiento terminó con éxito, marcar como completado
            if status == py_trees.common.Status.SUCCESS:
                self.completed = True
                last_completed_state = current_state_command
                print(f"Estado {self.state_name} completado con éxito")
                
            return status
        else:
            return py_trees.common.Status.FAILURE

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
    motor_ids = [6]

    # Crear comportamientos (pasando el nodo a cada uno)
    enable = EnableRobot(name="EnableRobot", motor_ids=motor_ids, node=node)
    home = HomePosition(name="GoHome", motor_ids=motor_ids, node=node)
    zero = ZeroPosition(name="GoZero", motor_ids=motor_ids, node=node)

    # Envolver comportamientos en controles de estado
    enable_state = StateControl(name="EnableState", state_name="enable", child=enable)
    home_state = StateControl(name="HomeState", state_name="home", child=home)
    zero_state = StateControl(name="ZeroState", state_name="zero", child=zero)

    # Build the behavior tree (usando Selector en lugar de Sequence)
    # Un Selector ejecutará cada hijo hasta que uno tenga éxito
    root = py_trees.composites.Selector(name="RobotStateMachine", memory=False)
    root.add_children([enable_state, home_state, zero_state])

    tree = py_trees.trees.BehaviourTree(root)
    
    # Configuración del árbol sin pasar timeout
    tree.setup()

    # Iniciar spin de ROS en un hilo separado
    spin_thread = threading.Thread(target=spin_ros, args=(node,))
    spin_thread.daemon = True
    spin_thread.start()

    node.get_logger().info("Starting state machine...")
    try:
        while rclpy.ok():
            tree.tick()
            node.get_logger().info(f"Estado actual: {current_state_command}, Completado: {last_completed_state}")
            time.sleep(1.0)
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