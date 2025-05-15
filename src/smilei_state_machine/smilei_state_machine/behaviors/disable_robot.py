#!/usr/bin/env python3

"""
Behavior para desactivar el robot SMILEi.
"""

import py_trees
from py_trees.common import Status
from smilei_state_machine.westwood_motor_client import WestwoodMotorClient
from smilei_state_machine.behaviors.home_position import HomePosition

class DisableRobot(py_trees.behaviour.Behaviour):
    """
    Comportamiento para desactivar el robot.
    """
    def __init__(self, node, name="Disable Robot"):
        super().__init__(name)
        self.node = node
        self.bear = None
        
    def initialise(self):
        self.node.get_logger().info(f"{self.name}: inicializando")
        self.blackboard = py_trees.blackboard.Blackboard()
        self.bear = self.blackboard.get("bear")
        
        if self.bear is None:
            self.bear = WestwoodMotorClient(self.node)
            
    def update(self):
        if self.blackboard.get("robot_enabled", False):
            # Primero mover a posición home
            home_behavior = HomePosition(self.node)
            home_behavior.initialise()
            home_behavior.update()
            
            # Desactivar el torque de los motores
            self.bear.set_torque_enable(*[(m_id, 0) for m_id in range(1, 9)])
            
            # Actualizar estado en el blackboard
            self.blackboard.set("robot_enabled", False)
            
            self.node.get_logger().info("Robot desactivado")
        else:
            self.node.get_logger().info("El robot ya está desactivado")
            
        return Status.SUCCESS
            
    def terminate(self, new_status):
        self.node.get_logger().info(f"{self.name}: terminando con estado {new_status}") 