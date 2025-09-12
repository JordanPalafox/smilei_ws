#!/usr/bin/env python3

"""
Nodo de control PD completamente asíncrono
"""

import rclpy
from rclpy.node import Node
import time
import math
import numpy as np

from westwood_motor_interfaces.srv import (
    GetMotorPositions,
    GetMotorVelocities, 
    SetGoalIq,
    SetTorqueEnable,
    SetMode,
    GetAvailableMotors
)


class PDControlNode(Node):
    def __init__(self):
        super().__init__('pd_control_node')
        
        # Parámetros exactos del código original
        self.kp = 1
        self.kd = 0.1  
        self.Kt = 0.35
        
        # Parámetros estimación velocidad
        self.r1 = 0.4
        self.r2 = 0.3
        self.p1 = (2*self.r2-self.r1)/self.r1
        self.Fc = 35
        self.Tl = 0.002
        self.theta_1 = 0
        self.theta_2 = 0
        self.vel_stim1 = 0
        self.vel_stim2 = 0
        
        # Estado del control
        self.pos1 = 0.0
        self.pos2 = 0.0
        self.vel1 = 0.0
        self.vel2 = 0.0
        self.control_active = False
        
        self.get_logger().info(f"Parámetros: kp={self.kp}, kd={self.kd}, Kt={self.Kt}, p1={self.p1:.3f}")
        
        # Clientes
        self.pos_client = self.create_client(GetMotorPositions, '/westwood_motor/get_motor_positions')
        self.vel_client = self.create_client(GetMotorVelocities, '/westwood_motor/get_motor_velocities')
        self.iq_client = self.create_client(SetGoalIq, '/westwood_motor/set_goal_iq')
        self.torque_client = self.create_client(SetTorqueEnable, '/westwood_motor/set_torque_enable')
        self.mode_client = self.create_client(SetMode, '/westwood_motor/set_mode')
        self.available_client = self.create_client(GetAvailableMotors, '/westwood_motor/get_available_motors')
        
        # Inicializar sistema de forma síncrona
        self.initialize_system()
        
        # Iniciar control asíncrono a máxima frecuencia
        self.count = 0
        self.timer = self.create_timer(0.001, self.control_callback)
        
    def initialize_system(self):
        self.get_logger().info("Inicializando sistema...")
        
        # Esperar servicios
        self.pos_client.wait_for_service()
        self.vel_client.wait_for_service()
        self.iq_client.wait_for_service()
        self.torque_client.wait_for_service()
        
        # Verificar motores
        req = GetAvailableMotors.Request()
        future = self.available_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not (future.result().success and 1 in future.result().motor_ids and 2 in future.result().motor_ids):
            self.get_logger().error("Motores no disponibles")
            return
            
        # Configurar sistema una sola vez
        self.configure_mode()
        
        # No habilitar torque aquí - se hace en el bucle como el código original
        
        self.control_active = True
        self.get_logger().info("Sistema inicializado - Control PD activo")
        
    def configure_mode(self):
        # Configurar modo corriente una sola vez
        req = SetMode.Request()
        req.motor_ids = [1, 2]
        req.modes = [0, 0]
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Motores configurados en modo corriente")
        
    def control_callback(self):
        if not self.control_active:
            return
            
        self.count += 1
        
        # Seguir la lógica exacta del código original: posiciones primero
        pos_req = GetMotorPositions.Request()
        pos_req.motor_ids = [1, 2]
        pos_future = self.pos_client.call_async(pos_req)
        
        # Solo posiciones, después velocidades como en el original
        pos_future.add_done_callback(self.positions_received)
        
    def positions_received(self, future):
        try:
            if not future.done():
                return
                
            pos_resp = future.result()
            if not (pos_resp and pos_resp.success and len(pos_resp.positions) >= 2):
                return
                
            self.pos1, self.pos2 = pos_resp.positions[0], pos_resp.positions[1]
            
            # Ahora solicitar velocidades como en el código original
            vel_req = GetMotorVelocities.Request()
            vel_req.motor_ids = [1, 2]
            vel_future = self.vel_client.call_async(vel_req)
            vel_future.add_done_callback(self.velocities_received)
            
        except Exception as e:
            self.get_logger().error(f"Error en positions_received: {e}")
            
    def velocities_received(self, future):
        try:
            if not future.done():
                return
                
            vel_resp = future.result()
            if not (vel_resp and vel_resp.success and len(vel_resp.velocities) >= 2):
                return
                
            self.vel1, self.vel2 = vel_resp.velocities[0], vel_resp.velocities[1]
            
            # Ejecutar control PD como en el código original
            self.execute_pd_control()
            
        except Exception as e:
            self.get_logger().error(f"Error en velocities_received: {e}")
            
    def execute_pd_control(self):
        try:
            # Habilitar torque como en el código original (línea 92)
            torque_req = SetTorqueEnable.Request()
            torque_req.motor_ids = [1, 2]
            torque_req.enable_torque = [True, True]
            self.torque_client.call_async(torque_req)
            
            # Calcular errores (targets por defecto a 0.0)
            e_1 = self.pos1 - 0.0
            e_2 = self.pos2 - 0.0
            
            # Estimación velocidad exacta del código original
            self.vel_stim1 = self.Fc * (self.theta_1 + self.pos1)
            self.theta_1 = self.theta_1 - self.Tl * self.vel_stim1
            
            self.vel_stim2 = self.Fc * (self.theta_2 + self.pos2)
            self.theta_2 = self.theta_2 - self.Tl * self.vel_stim2
            
            # Control PD exacto del código original
            tau1 = -self.kp * ((abs(e_1)**self.p1) * np.sign(e_1)) - self.kd * self.vel_stim1
            tau2 = -self.kp * ((abs(e_2)**self.p1) * np.sign(e_2)) - self.kd * self.vel_stim2
            
            # Convertir a corriente
            i_g_1 = tau1 / self.Kt
            i_g_2 = tau2 / self.Kt
            
            # Log cada 1000 iteraciones
            if self.count % 1000 == 0:
                self.get_logger().info(f"Ciclo {self.count}: pos=[{self.pos1:.4f}, {self.pos2:.4f}] err=[{e_1:.4f}, {e_2:.4f}] tau=[{tau1:.4f}, {tau2:.4f}] iq=[{i_g_1:.4f}, {i_g_2:.4f}]")
            
            # Enviar corrientes por separado como en el código original (líneas 113-114)
            iq1_req = SetGoalIq.Request()
            iq1_req.motor_ids = [1]
            iq1_req.goal_iq = [float(i_g_1)]
            self.iq_client.call_async(iq1_req)
            
            iq2_req = SetGoalIq.Request()
            iq2_req.motor_ids = [2]
            iq2_req.goal_iq = [float(i_g_2)]
            self.iq_client.call_async(iq2_req)
            
        except Exception as e:
            self.get_logger().error(f"Error en execute_pd_control: {e}")
            
    def destroy_node(self):
        self.get_logger().info("Deshabilitando motores...")
        self.control_active = False
        
        try:
            req = SetTorqueEnable.Request()
            req.motor_ids = [1, 2]
            req.enable_torque = [False, False]
            future = self.torque_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        except:
            pass
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = PDControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Control terminado por usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()