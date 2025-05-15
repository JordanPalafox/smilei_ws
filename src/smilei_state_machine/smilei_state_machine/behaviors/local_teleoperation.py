#!/usr/bin/env python3

"""
Behavior para teleoperación local del robot SMILEi.
"""

import py_trees
import numpy as np
import time
from py_trees.common import Status
from smilei_state_machine.westwood_motor_client import WestwoodMotorClient
from smilei_state_machine.behaviors.zero_position import ZeroPosition
from smilei_state_machine.behaviors.home_position import HomePosition
from smilei_state_machine.behaviors.gravity_utils import right_gravity_vector, left_gravity_vector

class LocalTeleoperation(py_trees.behaviour.Behaviour):
    """
    Comportamiento para teleoperación local del robot.
    """
    def __init__(self, node, name="Local Teleoperation"):
        super().__init__(name)
        self.node = node
        self.bear = None
        self.running = False
        
        # Obtener parámetros de ROS
        self.current_control_gains = self.node.get_parameter("current_control_gains").value
        self.position_control_gains = self.node.get_parameter("position_control_gains").value
        
        # Valores predeterminados en caso de que no se puedan cargar
        if self.current_control_gains is None:
            self.node.get_logger().warn("No se pudieron cargar los parámetros de control de corriente, usando valores predeterminados")
            self.current_control_gains = {
                "p_gain_position": 0.0,
                "d_gain_position": 0.0,
                "i_gain_position": 0.0,
                "p_gain_force": 0.0,
                "d_gain_force": 0.0,
                "i_gain_force": 0.0,
                "iq_max": 3.0,
                "p_gain_iq": 0.277,
                "i_gain_iq": 0.061,
                "d_gain_iq": 0.0,
                "p_gain_id": 0.277,
                "i_gain_id": 0.061,
                "d_gain_id": 0.0,
                "kt": 0.35
            }
            
        if self.position_control_gains is None:
            self.node.get_logger().warn("No se pudieron cargar los parámetros de control de posición, usando valores predeterminados")
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
            
        # Cambiar a modo de control de corriente
        gains = self.current_control_gains
            
        # Configurar PID para modo corriente (iq/id)
        self.bear.set_p_gain_iq(*[(m_id, gains["p_gain_iq"]) for m_id in range(1, 9)])
        self.bear.set_i_gain_iq(*[(m_id, gains["i_gain_iq"]) for m_id in range(1, 9)])
        self.bear.set_d_gain_iq(*[(m_id, gains["d_gain_iq"]) for m_id in range(1, 9)])
        self.bear.set_p_gain_id(*[(m_id, gains["p_gain_id"]) for m_id in range(1, 9)])
        self.bear.set_i_gain_id(*[(m_id, gains["i_gain_id"]) for m_id in range(1, 9)])
        self.bear.set_d_gain_id(*[(m_id, gains["d_gain_id"]) for m_id in range(1, 9)])
            
        # Configurar PID posición a cero
        self.bear.set_p_gain_position(*[(m_id, 0) for m_id in range(1, 9)])
        self.bear.set_i_gain_position(*[(m_id, 0) for m_id in range(1, 9)])
        self.bear.set_d_gain_position(*[(m_id, 0) for m_id in range(1, 9)])
            
        # Borrar PID de fuerza directa
        self.bear.set_p_gain_force(*[(m_id, 0) for m_id in range(1, 9)])
        self.bear.set_i_gain_force(*[(m_id, 0) for m_id in range(1, 9)])
        self.bear.set_d_gain_force(*[(m_id, 0) for m_id in range(1, 9)])
            
        # Cambiar a modo corriente
        self.bear.set_mode(*[(m_id, 0) for m_id in range(1, 9)])
        self.bear.set_torque_enable(*[(m_id, 1) for m_id in range(1, 9)])
        
        # Parámetros para control bilateral
        kp = 1
        kd = 0.001
        kt = gains["kt"]
        
        # Bucle de teleoperación
        start_time = time.time()
        timeout = 60.0  # 60 segundos de operación
        
        while self.running and (time.time() - start_time < timeout):
            # Leer posiciones y velocidades actuales
            q_l = np.array([
                self.bear.get_present_position(5)[0][0][0],
                self.bear.get_present_position(6)[0][0][0],
                self.bear.get_present_position(7)[0][0][0],
                self.bear.get_present_position(8)[0][0][0]
            ])
            
            dq_l = np.array([
                self.bear.get_present_velocity(5)[0][0][0],
                self.bear.get_present_velocity(6)[0][0][0],
                self.bear.get_present_velocity(7)[0][0][0],
                self.bear.get_present_velocity(8)[0][0][0]
            ])
            
            q_r = np.array([
                self.bear.get_present_position(1)[0][0][0],
                self.bear.get_present_position(2)[0][0][0],
                self.bear.get_present_position(3)[0][0][0],
                self.bear.get_present_position(4)[0][0][0]
            ])
            
            dq_r = np.array([
                self.bear.get_present_velocity(1)[0][0][0],
                self.bear.get_present_velocity(2)[0][0][0],
                self.bear.get_present_velocity(3)[0][0][0],
                self.bear.get_present_velocity(4)[0][0][0]
            ])
            
            # Calcular vectores de gravedad
            G_L = left_gravity_vector(q_l, self.node)
            G_R = right_gravity_vector(q_r, self.node)
            
            # Calcular corrientes para teleoperación
            iq_1 = (-kp * (q_r[0] - (-1.0 * q_l[0])) - kd * dq_r[0] + G_R[0]) / kt
            iq_2 = (-kp * (q_r[1] - (-1.0 * q_l[1])) - kd * dq_r[1] + G_R[1]) / kt
            iq_3 = (-kp * (q_r[2] - (-1.0 * q_l[2])) - kd * dq_r[2]) / kt
            iq_4 = (-kp * (self.bear.get_present_position(4)[0][0][0] - (-1.0 * self.bear.get_present_position(8)[0][0][0]))) / kt
            
            iq_5 = (-kp * (q_l[0] - (-1.0 * q_r[0])) - kd * dq_l[0] + G_L[0]) / kt
            iq_6 = (-kp * (q_l[1] - (-1.0 * q_r[1])) - kd * dq_l[1] + G_L[1]) / kt
            iq_7 = (-kp * (q_l[2] - (-1.0 * q_r[2])) - kd * dq_l[2]) / kt
            iq_8 = (-kp * (self.bear.get_present_position(8)[0][0][0] - (-1.0 * self.bear.get_present_position(4)[0][0][0]))) / kt
            
            # Enviar corrientes a los motores
            self.bear.set_goal_iq(
                (1, iq_1), (2, iq_2), (3, iq_3), (4, iq_4),
                (5, iq_5), (6, iq_6), (7, iq_7), (8, iq_8)
            )
            
            time.sleep(0.001)
            
            # Verificar si hay interrupción
            if hasattr(self, "feedback_message") and self.feedback_message == "stop":
                self.running = False
                break
                
        # Restaurar ganancias y modo de posición
        gains = self.position_control_gains
            
        # Configurar PID para modo corriente (iq/id)
        self.bear.set_p_gain_iq(*[(m_id, gains["p_gain_iq"]) for m_id in range(1, 9)])
        self.bear.set_i_gain_iq(*[(m_id, gains["i_gain_iq"]) for m_id in range(1, 9)])
        self.bear.set_d_gain_iq(*[(m_id, gains["d_gain_iq"]) for m_id in range(1, 9)])
        self.bear.set_p_gain_id(*[(m_id, gains["p_gain_id"]) for m_id in range(1, 9)])
        self.bear.set_i_gain_id(*[(m_id, gains["i_gain_id"]) for m_id in range(1, 9)])
        self.bear.set_d_gain_id(*[(m_id, gains["d_gain_id"]) for m_id in range(1, 9)])
            
        # Configurar PID posición
        self.bear.set_p_gain_position(*[(m_id, gains["p_gain_position"]) for m_id in range(1, 9)])
        self.bear.set_i_gain_position(*[(m_id, gains["i_gain_position"]) for m_id in range(1, 9)])
        self.bear.set_d_gain_position(*[(m_id, gains["d_gain_position"]) for m_id in range(1, 9)])
            
        # Cambiar a modo posición
        self.bear.set_mode(*[(m_id, 2) for m_id in range(1, 9)])
        
        # Volver a posición home
        home_behavior = HomePosition(self.node)
        home_behavior.initialise()
        home_behavior.update()
        
        self.node.get_logger().info("Teleoperación finalizada")
        return Status.SUCCESS
            
    def terminate(self, new_status):
        self.running = False
        self.node.get_logger().info(f"{self.name}: terminando con estado {new_status}") 