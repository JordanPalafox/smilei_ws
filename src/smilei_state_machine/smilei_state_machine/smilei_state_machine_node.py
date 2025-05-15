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
import sys

# Importar comportamientos específicos del robot
from smilei_state_machine.behaviors import (
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
        # Crear el blackboard para compartir información entre comportamientos
        blackboard = py_trees.blackboard.Blackboard()
        
        # Nodos de comportamiento
        enable_robot = EnableRobot(self, name="Enable Robot")
        home_position = HomePosition(self, name="Home Position")
        zero_position = ZeroPosition(self, name="Zero Position")
        say_hello = SayHello(self, name="Say Hello")
        local_teleoperation = LocalTeleoperation(self, name="Local Teleoperation")
        disable_robot = DisableRobot(self, name="Disable Robot")
        
        # Crear fallbacks (selectores) para cada opción del menú
        enable_fallback = py_trees.composites.Selector(name="Enable Robot Option", memory=True)
        home_fallback = py_trees.composites.Selector(name="Home Position Option", memory=True)
        zero_fallback = py_trees.composites.Selector(name="Zero Position Option", memory=True)
        hello_fallback = py_trees.composites.Selector(name="Say Hello Option", memory=True)
        teleop_fallback = py_trees.composites.Selector(name="Local Teleoperation Option", memory=True)
        disable_fallback = py_trees.composites.Selector(name="Disable Robot Option", memory=True)
        
        # Crear condiciones para las opciones del menú
        is_option_enable = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Is Option Enable",
            check=py_trees.common.ComparisonExpression(
                variable="menu_option",
                value="1",
                operator=lambda x, y: x == y
            )
        )
        
        is_option_home = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Is Option Home",
            check=py_trees.common.ComparisonExpression(
                variable="menu_option",
                value="2",
                operator=lambda x, y: x == y
            )
        )
        
        is_option_zero = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Is Option Zero",
            check=py_trees.common.ComparisonExpression(
                variable="menu_option",
                value="3",
                operator=lambda x, y: x == y
            )
        )
        
        is_option_hello = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Is Option Hello",
            check=py_trees.common.ComparisonExpression(
                variable="menu_option",
                value="4",
                operator=lambda x, y: x == y
            )
        )
        
        is_option_teleop = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Is Option Teleop",
            check=py_trees.common.ComparisonExpression(
                variable="menu_option",
                value="5",
                operator=lambda x, y: x == y
            )
        )
        
        is_option_disable = py_trees.behaviours.CheckBlackboardVariableValue(
            name="Is Option Disable",
            check=py_trees.common.ComparisonExpression(
                variable="menu_option",
                value="6",
                operator=lambda x, y: x == y
            )
        )
        
        # Comportamiento para leer la selección de menú del usuario
        read_menu_option = ReadMenuOption(self, name="Read Menu Option")
        
        # Añadir condiciones y comportamientos a los fallbacks
        enable_fallback.add_children([is_option_enable, py_trees.behaviours.Failure(name="Skip Enable")])
        home_fallback.add_children([is_option_home, py_trees.behaviours.Failure(name="Skip Home")])
        zero_fallback.add_children([is_option_zero, py_trees.behaviours.Failure(name="Skip Zero")])
        hello_fallback.add_children([is_option_hello, py_trees.behaviours.Failure(name="Skip Hello")])
        teleop_fallback.add_children([is_option_teleop, py_trees.behaviours.Failure(name="Skip Teleop")])
        disable_fallback.add_children([is_option_disable, py_trees.behaviours.Failure(name="Skip Disable")])
        
        # Secuencia condicional para cada opción
        enable_sequence = py_trees.composites.Sequence(name="Enable Robot Sequence", memory=True)
        enable_sequence.add_children([enable_fallback, enable_robot])
        
        home_sequence = py_trees.composites.Sequence(name="Home Position Sequence", memory=True)
        home_sequence.add_children([home_fallback, home_position])
        
        zero_sequence = py_trees.composites.Sequence(name="Zero Position Sequence", memory=True)
        zero_sequence.add_children([zero_fallback, zero_position])
        
        hello_sequence = py_trees.composites.Sequence(name="Say Hello Sequence", memory=True)
        hello_sequence.add_children([hello_fallback, say_hello])
        
        teleop_sequence = py_trees.composites.Sequence(name="Local Teleoperation Sequence", memory=True)
        teleop_sequence.add_children([teleop_fallback, local_teleoperation])
        
        disable_sequence = py_trees.composites.Sequence(name="Disable Robot Sequence", memory=True)
        disable_sequence.add_children([disable_fallback, disable_robot])
        
        # Crear paralelo para ejecutar todas las opciones al mismo tiempo
        options_parallel = py_trees.composites.Parallel(
            name="Menu Options",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne()
        )
        
        options_parallel.add_children([
            enable_sequence,
            home_sequence,
            zero_sequence,
            hello_sequence,
            teleop_sequence,
            disable_sequence
        ])
        
        # Crear raíz del árbol
        root = py_trees.composites.Sequence(name="SMILEi State Machine", memory=False)
        root.add_children([read_menu_option, options_parallel])
        
        return root

class ReadMenuOption(py_trees.behaviour.Behaviour):
    """
    Comportamiento que lee la opción del menú seleccionada por el usuario.
    """
    def __init__(self, node, name="Read Menu Option"):
        super().__init__(name)
        self.node = node
        
    def initialise(self):
        """Inicializa el comportamiento."""
        self.node.get_logger().info("Seleccione una opción del menú:")
        self.node.get_logger().info("1. Enable Robot")
        self.node.get_logger().info("2. Go to Home Position")
        self.node.get_logger().info("3. Go to Zero Position")
        self.node.get_logger().info("4. Say Hello")
        self.node.get_logger().info("5. Local Teleoperation")
        self.node.get_logger().info("6. Disable Robot")
        self.node.get_logger().info("7. Salir")
        
    def update(self):
        """Lee la selección del usuario y la almacena en el blackboard."""
        # Leer entrada del usuario (simulación con keypress)
        key = py_trees.console.read_single_keypress()
        
        if key in ['1', '2', '3', '4', '5', '6', '7']:
            # Escribir la selección en el blackboard
            self.blackboard = py_trees.blackboard.Blackboard()
            self.blackboard.set("menu_option", key)
            
            if key == '7':
                # Salir del programa
                self.node.get_logger().info("Saliendo del programa...")
                return Status.FAILURE
                
            return Status.SUCCESS
        else:
            self.node.get_logger().info(f"Opción inválida: {key}")
            return Status.RUNNING
        
    def terminate(self, new_status):
        """Finaliza el comportamiento."""
        self.node.get_logger().info(f"Opción seleccionada: {self.blackboard.get('menu_option')}")

def main(args=None):
    rclpy.init(args=args)
    
    node = SmileiStateMachine()
    
    # Ejecutar el nodo en un hilo separado
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()
    
    try:
        # Ejecutar la máquina de estados
        node.behaviour_tree.setup()
        
        while rclpy.ok():
            node.behaviour_tree.tick()
            if node.behaviour_tree.root.status == Status.FAILURE:
                break
    except KeyboardInterrupt:
        node.get_logger().info("Programa interrumpido por el usuario")
    finally:
        node.get_logger().info("Cerrando la máquina de estados...")
        node.behaviour_tree.shutdown()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main() 