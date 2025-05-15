#!/usr/bin/env python3

"""
Behavior para mover el robot SMILEi a la posición cero.
"""

import py_trees
import numpy as np
import time
from py_trees.common import Status
from smilei_state_machine.westwood_motor_client import WestwoodMotorClient

class ZeroPosition(py_trees.behaviour.Behaviour):
    """
    Comportamiento para mover el robot a la posición cero.
    """
    def __init__(self, node, name="Zero Position"):
        super().__init__(name)
        self.node = node
        self.bear = None
        self.motor_ids = [1, 2, 3, 4, 5, 6, 7, 8]
        
    def initialise(self):
        self.node.get_logger().info(f"{self.name}: inicializando")
        self.blackboard = py_trees.blackboard.Blackboard()
        self.bear = self.blackboard.get("bear")
        
        if self.bear is None:
            self.bear = WestwoodMotorClient(self.node)
            
    def update(self):
        if not self.blackboard.get("robot_enabled", False):
            self.node.get_logger().warn("Robot no habilitado. Por favor habilítelo primero.")
            return Status.FAILURE
            
        # Posición cero para todos los motores
        zero_pos = np.zeros(8)
        
        # Leer posiciones actuales
        init = np.array([
            self.bear.get_present_position(1)[0][0][0],
            self.bear.get_present_position(2)[0][0][0],
            self.bear.get_present_position(3)[0][0][0],
            self.bear.get_present_position(4)[0][0][0],
            self.bear.get_present_position(5)[0][0][0],
            self.bear.get_present_position(6)[0][0][0],
            self.bear.get_present_position(7)[0][0][0],
            self.bear.get_present_position(8)[0][0][0]
        ])
        
        # Número de pasos para suavizar el movimiento
        num = 100
        delta_angle = (zero_pos - init) / num
        
        # Mover gradualmente a posición cero
        for i in range(num):
            goal_pos = init + delta_angle * (i + 1)
            
            self.bear.set_goal_position(
                (1, goal_pos[0]), (2, goal_pos[1]), 
                (3, goal_pos[2]), (4, goal_pos[3]),
                (5, goal_pos[4]), (6, goal_pos[5]), 
                (7, goal_pos[6]), (8, goal_pos[7])
            )
            time.sleep(0.01)
            
        self.node.get_logger().info("Robot en posición cero")
        return Status.SUCCESS
            
    def terminate(self, new_status):
        self.node.get_logger().info(f"{self.name}: terminando con estado {new_status}") 