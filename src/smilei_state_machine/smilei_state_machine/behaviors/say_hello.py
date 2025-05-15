#!/usr/bin/env python3

"""
Behavior para que el robot SMILEi salude.
"""

import py_trees
import time
import math
from py_trees.common import Status
from smilei_state_machine.westwood_motor_client import WestwoodMotorClient
from smilei_state_machine.behaviors.zero_position import ZeroPosition

class SayHello(py_trees.behaviour.Behaviour):
    """
    Comportamiento para hacer que el robot salude.
    """
    def __init__(self, node, name="Say Hello"):
        super().__init__(name)
        self.node = node
        self.bear = None
        self.running = False
        
    def initialise(self):
        self.node.get_logger().info(f"{self.name}: inicializando")
        self.blackboard = py_trees.blackboard.Blackboard()
        self.bear = self.blackboard.get("bear")
        
        if self.bear is None:
            self.bear = WestwoodMotorClient(self.node)
            
        self.running = True
        
    def update(self):
        if not self.blackboard.get("robot_enabled", False):
            self.node.get_logger().warn("Robot no habilitado. Por favor habilítelo primero.")
            return Status.FAILURE
            
        # Primero mover a posición cero
        zero_behavior = ZeroPosition(self.node)
        zero_behavior.initialise()
        status = zero_behavior.update()
        
        if status != Status.SUCCESS:
            return Status.FAILURE
            
        # Realizar movimiento de saludo
        start_time = time.time()
        timeout = 10.0  # 10 segundos de saludar
        
        while self.running and (time.time() - start_time < timeout):
            seconds = time.time()
            qd = 0.5 * math.sin(10.0 * seconds) - 1
            
            # Mover articulaciones 4 y 8 para saludar
            self.bear.set_goal_position((4, -qd), (8, qd))
            time.sleep(0.01)
            
            # Verificar si hay interrupción (este mecanismo podría necesitar ajustes)
            if hasattr(self, "feedback_message") and self.feedback_message == "stop":
                self.running = False
                break
                
        self.node.get_logger().info("Saludo completado")
        return Status.SUCCESS if not self.running else Status.RUNNING
            
    def terminate(self, new_status):
        self.running = False
        self.node.get_logger().info(f"{self.name}: terminando con estado {new_status}") 