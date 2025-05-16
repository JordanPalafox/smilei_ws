import time
import rclpy
from rclpy.node import Node
import py_trees
import threading
from std_msgs.msg import String
from smilei_state_machine.behaviors.home_position import HomePosition
from smilei_state_machine.behaviors.zero_position import ZeroPosition

# Variable global para almacenar el comando actual
current_state_command = "idle"

# Clase para controlar estados basados en un comando
class StateControl(py_trees.decorators.Decorator):
    def __init__(self, name: str, state_name: str, child: py_trees.behaviour.Behaviour):
        super().__init__(name=name, child=child)
        self.state_name = state_name
    
    def update(self) -> py_trees.common.Status:
        global current_state_command
        # Solo ejecutar el comportamiento si el comando coincide con este estado
        if current_state_command == self.state_name:
            return self.decorated.status
        else:
            return py_trees.common.Status.FAILURE

def state_command_callback(msg):
    """Callback para recibir comandos de estado"""
    global current_state_command
    current_state_command = msg.data
    print(f"Cambiando a estado: {current_state_command}")

def spin_ros(node):
    """Función para ejecutar el spin de ROS en un hilo separado"""
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
    home = HomePosition(name="GoHome", motor_ids=motor_ids, node=node)
    zero = ZeroPosition(name="GoZero", motor_ids=motor_ids, node=node)

    # Envolver comportamientos en controles de estado
    home_state = StateControl(name="HomeState", state_name="home", child=home)
    zero_state = StateControl(name="ZeroState", state_name="zero", child=zero)

    # Build the behavior tree (usando Selector en lugar de Sequence)
    # Un Selector ejecutará cada hijo hasta que uno tenga éxito
    root = py_trees.composites.Selector(name="RobotStateMachine", memory=False)
    root.add_children([home_state, zero_state])

    tree = py_trees.trees.BehaviourTree(root)
    
    # Setup the tree with timeout
    timeout_sec = 1000.0
    tree.setup(timeout_sec=timeout_sec)

    # Iniciar spin de ROS en un hilo separado
    spin_thread = threading.Thread(target=spin_ros, args=(node,))
    spin_thread.daemon = True
    spin_thread.start()

    node.get_logger().info("Starting state machine...")
    try:
        while rclpy.ok():
            tree.tick()
            node.get_logger().info(f"Estado actual: {current_state_command}")
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