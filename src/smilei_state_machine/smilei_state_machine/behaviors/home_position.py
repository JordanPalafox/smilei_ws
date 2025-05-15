#!/usr/bin/env python3

"""
Behavior para mover el robot SMILEi a la posición home.
"""

import py_trees
import numpy as np
import time
from py_trees.common import Status
from smilei_state_machine.westwood_motor_client import WestwoodMotorClient

class HomePosition(py_trees.behaviour.Behaviour):
    """
    Comportamiento para mover el robot a la posición home.
    """
    def __init__(self, node, name="Home Position"):
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
            
        # Posiciones home para brazo derecho e izquierdo
        home_pos_r = np.array([0, 1.5707, -1.5707, 1.5707])
        home_pos_l = np.array([0, -1.5707, 1.5707, -1.5707])
        
        # Leer posiciones actuales
        init_r = np.array([
            self.bear.get_present_position(1)[0][0][0],
            self.bear.get_present_position(2)[0][0][0],
            self.bear.get_present_position(3)[0][0][0],
            self.bear.get_present_position(4)[0][0][0]
        ])
        
        init_l = np.array([
            self.bear.get_present_position(5)[0][0][0],
            self.bear.get_present_position(6)[0][0][0],
            self.bear.get_present_position(7)[0][0][0],
            self.bear.get_present_position(8)[0][0][0]
        ])
        
        # Número de pasos para suavizar el movimiento
        num = 100
        delta_angle_r = (home_pos_r - init_r) / num
        delta_angle_l = (home_pos_l - init_l) / num
        
        # Mover gradualmente a posición home
        for i in range(num):
            goal_pos_r = init_r + delta_angle_r * (i + 1)
            goal_pos_l = init_l + delta_angle_l * (i + 1)
            
            self.bear.set_goal_position(
                (1, goal_pos_r[0]), (2, goal_pos_r[1]), 
                (3, goal_pos_r[2]), (4, goal_pos_r[3]),
                (5, goal_pos_l[0]), (6, goal_pos_l[1]), 
                (7, goal_pos_l[2]), (8, goal_pos_l[3])
            )
            time.sleep(0.01)
            
        self.node.get_logger().info("Robot en posición home")
        return Status.SUCCESS
            
    def terminate(self, new_status):
        self.node.get_logger().info(f"{self.name}: terminando con estado {new_status}") 