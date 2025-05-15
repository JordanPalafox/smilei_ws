#!/usr/bin/env python3

"""
Behavior para activar el robot SMILEi y configurar sus parámetros.
"""

import py_trees
import time
from py_trees.common import Status
from smilei_state_machine.westwood_motor_client import WestwoodMotorClient

class EnableRobot(py_trees.behaviour.Behaviour):
    """
    Comportamiento para activar el robot y configurar sus parámetros iniciales.
    """
    def __init__(self, node, name="Enable Robot"):
        super().__init__(name)
        self.node = node
        self.bear = None
        self.motor_ids = [1, 2, 3, 4, 5, 6, 7, 8]
        self.enabled = False
        # Obtener parámetros de ROS
        self.position_control_gains = self.node.get_parameter("position_control_gains").value
        if self.position_control_gains is None:
            self.node.get_logger().warn("No se pudieron cargar los parámetros de control de posición, usando valores predeterminados")
            # Valores predeterminados en caso de que no se puedan cargar
            self.position_control_gains = {
                "p_gain_position": 5.0,
                "d_gain_position": 0.2,
                "i_gain_position": 0.0,
                "iq_max": 1.5,
                "p_gain_iq": 0.02,
                "i_gain_iq": 0.02,
                "d_gain_iq": 0.0,
                "p_gain_id": 0.02,
                "i_gain_id": 0.02,
                "d_gain_id": 0.0,
                "kt": 0.35
            }
        
    def initialise(self):
        self.node.get_logger().info(f"{self.name}: inicializando")
        self.bear = WestwoodMotorClient(self.node)
        self.enabled = False
        
    def update(self):
        if not self.enabled:
            # Comprobar si todos los motores están disponibles
            motors_connected = self.bear.ping(*self.motor_ids)[0]
            
            if not motors_connected:
                self.node.get_logger().error("Motores no conectados o no disponibles.")
                return Status.FAILURE
                
            # Configurar parámetros de control de posición desde los parámetros ROS
            gains = self.position_control_gains
            
            # Configurar PID para modo corriente (iq/id)
            self.bear.set_p_gain_iq(*[(m_id, gains["p_gain_iq"]) for m_id in self.motor_ids])
            self.bear.set_i_gain_iq(*[(m_id, gains["i_gain_iq"]) for m_id in self.motor_ids])
            self.bear.set_d_gain_iq(*[(m_id, gains["d_gain_iq"]) for m_id in self.motor_ids])
            self.bear.set_p_gain_id(*[(m_id, gains["p_gain_id"]) for m_id in self.motor_ids])
            self.bear.set_i_gain_id(*[(m_id, gains["i_gain_id"]) for m_id in self.motor_ids])
            self.bear.set_d_gain_id(*[(m_id, gains["d_gain_id"]) for m_id in self.motor_ids])
            
            # Configurar PID para modo posición
            self.bear.set_p_gain_position(*[(m_id, gains["p_gain_position"]) for m_id in self.motor_ids])
            self.bear.set_i_gain_position(*[(m_id, gains["i_gain_position"]) for m_id in self.motor_ids])
            self.bear.set_d_gain_position(*[(m_id, gains["d_gain_position"]) for m_id in self.motor_ids])
            
            # Configurar modo de operación a posición (2)
            self.bear.set_mode(*[(m_id, 2) for m_id in self.motor_ids])
            
            # Establecer límites de iq
            self.bear.set_limit_iq_max(*[(m_id, gains["iq_max"]) for m_id in self.motor_ids])
            
            # Habilitar torque en los motores
            self.bear.set_torque_enable(*[(m_id, 1) for m_id in self.motor_ids])
            
            self.node.get_logger().info("¡Robot habilitado!")
            self.enabled = True
            time.sleep(1)
            
            # Guardar estado global en el blackboard para compartir con otros comportamientos
            self.blackboard = py_trees.blackboard.Blackboard()
            self.blackboard.set("bear", self.bear)
            self.blackboard.set("robot_enabled", True)
            
            return Status.SUCCESS
        else:
            self.node.get_logger().info("El robot ya está habilitado")
            return Status.SUCCESS
            
    def terminate(self, new_status):
        self.node.get_logger().info(f"{self.name}: terminando con estado {new_status}") 