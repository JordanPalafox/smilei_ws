#!/usr/bin/env python3

"""
Nodo principal para la máquina de estados del robot SMILEi usando py_trees.
Implementa los estados definidos en la descripción de la aplicación original y
utiliza los servicios de westwood_motor_server.
"""

import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from py_trees.common import Status
from py_trees_ros.trees import BehaviourTree
import threading

from smilei_state_machine.behaviors.robot_behaviors import (
    EnableRobot,
    HomePosition,
    ZeroPosition,
    SayHello,
    LocalTeleoperation,
    DisableRobot
)

class SmileiStateMachine(Node):
    """
    Nodo principal que gestiona la máquina de estados del robot SMILEi.
    """
    def __init__(self):
        super().__init__('smilei_state_machine')
        self.get_logger().info('Iniciando SMILEi State Machine')
        
        # Declarar parámetros
        self.declare_parameter('use_sim_time', False)
        
        # Crear la máquina de estados
        self.tree = self.create_behavior_tree()
        
        # Configurar el árbol de comportamiento ROS
        self.behaviour_tree = BehaviourTree(
            self.tree,
            self,
            unicode_tree_debug=True
        )
        
        self.get_logger().info('SMILEi State Machine inicializada y lista')
    
    def create_behavior_tree(self):
        """
        Crea la estructura de árbol de comportamiento para la máquina de estados.
        """
        # Nodos de comportamiento
        enable_robot = EnableRobot(self, name="Enable Robot")
        home_position = HomePosition(self, name="Home Position")
        zero_position = ZeroPosition(self, name="Zero Position")
        say_hello = SayHello(self, name="Say Hello")
        local_teleoperation = LocalTeleoperation(self, name="Local Teleoperation")
        disable_robot = DisableRobot(self, name="Disable Robot")
        
        # Crear estado de selección de menú interactivo
        menu = py_trees.composites.Selector(name="Menu de Estado", memory=False)
        
        # Añadir todos los comportamientos al menú
        menu.add_children([
            enable_robot,
            home_position,
            zero_position,
            say_hello,
            local_teleoperation,
            disable_robot
        ])
        
        # Crear raíz del árbol
        root = py_trees.composites.Sequence(name="SMILEi State Machine", memory=False)
        root.add_child(menu)
        
        return root

def main(args=None):
    rclpy.init(args=args)
    
    node = SmileiStateMachine()
    
    # Ejecutar el nodo en un hilo separado
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    
    try:
        # Ejecutar la máquina de estados
        node.behaviour_tree.setup()
        
        while rclpy.ok():
            node.behaviour_tree.tick()
            py_trees.console.read_single_keypress()
    except KeyboardInterrupt:
        pass
    finally:
        node.behaviour_tree.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main() 