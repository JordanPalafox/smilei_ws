#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
import math
import numpy as np

from westwood_motor_interfaces.srv import GetMotorPositions, GetMotorVelocities, SetMode, SetMotorIdAndTargetCurrent


class MotorPositionClient(Node):
    def __init__(self):
        super().__init__('motor_position_client')
        
        # Clientes para obtener posiciones y velocidades
        self.pos_client = self.create_client(GetMotorPositions, '/westwood_motor/get_motor_positions')
        self.vel_client = self.create_client(GetMotorVelocities, '/westwood_motor/get_motor_velocities')
        self.mode_client = self.create_client(SetMode, '/westwood_motor/set_mode')
        self.current_client = self.create_client(SetMotorIdAndTargetCurrent, '/westwood_motor/set_motor_id_and_target_current')
        
        # Variables para almacenar datos
        self.pos1, self.pos2 = 0.0, 0.0
        self.vel1, self.vel2 = 0.0, 0.0
        self.request_position = True  # Alternar entre pos y vel
        
        # PD Control parameters (from reference code)
        self.kp = 1.0        # Proportional gain
        self.kd = 0.1        # Damping gain
        self.Kt = 0.35       # N.m/A
        self.r1 = 0.4
        self.r2 = 0.3
        self.p1 = (2*self.r2 - self.r1) / self.r1
        self.p2 = (2*self.r2 - self.r1) / self.r2
        self.Fc = 35         # Frequency cutoff
        self.Tl = 0.002      # Loop frequency
        
        # Velocity estimator variables
        self.theta_1 = 0.0
        self.vel_stim1 = 0.0
        self.theta_2 = 0.0
        self.vel_stim2 = 0.0
        
        # Control start time for trajectory generation
        self.start_time = time.time()
        
        # Variables para calcular frecuencia
        self.last_time = time.time()
        self.response_count = 0
        self.freq_timer = self.create_timer(1.0, self.print_frequency)
        
        # Esperar servicios
        self.get_logger().info("Esperando servicios...")
        if not self.pos_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Servicio de posiciones no disponible!")
            return
        if not self.vel_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Servicio de velocidades no disponible!")
            return
        if not self.mode_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Servicio de modo no disponible!")
            return
        if not self.current_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Servicio de corriente no disponible!")
            return
        self.get_logger().info("Servicios conectados exitosamente")
        
        # Configurar motores en modo corriente antes de iniciar el bucle
        self.set_current_mode()
        
        # Timer para leer a máxima frecuencia posible
        self.timer = self.create_timer(0.001, self.get_data)  # 1ms = 1000Hz
        
    def get_data(self):
        try:
            if self.request_position:
                # Solicitar posiciones
                pos_req = GetMotorPositions.Request()
                pos_req.motor_ids = [1, 2]
                pos_future = self.pos_client.call_async(pos_req)
                pos_future.add_done_callback(self.position_response_callback)
            else:
                # Solicitar velocidades
                vel_req = GetMotorVelocities.Request()
                vel_req.motor_ids = [1, 2]
                vel_future = self.vel_client.call_async(vel_req)
                vel_future.add_done_callback(self.velocity_response_callback)
            
            # Alternar para la próxima vez
            self.request_position = not self.request_position
        except Exception as e:
            self.get_logger().error(f"Error en llamada al servicio: {e}")
    
    def position_response_callback(self, future):
        try:
            result = future.result()
            if result and result.success and len(result.positions) >= 2:
                self.pos1, self.pos2 = result.positions[0], result.positions[1]
                self.response_count += 1
                self.compute_pd_control()
                self.get_logger().info(f"Motor 1: pos={self.pos1:.4f}, vel={self.vel1:.4f} | Motor 2: pos={self.pos2:.4f}, vel={self.vel2:.4f}")
            else:
                self.get_logger().error(f"Error posiciones: {result.message if result else 'Sin respuesta'}")
        except Exception as e:
            self.get_logger().error(f"Error en respuesta de posiciones: {e}")
    
    def velocity_response_callback(self, future):
        try:
            result = future.result()
            if result and result.success and len(result.velocities) >= 2:
                self.vel1, self.vel2 = result.velocities[0], result.velocities[1]
                self.response_count += 1
                self.get_logger().info(f"Motor 1: pos={self.pos1:.4f}, vel={self.vel1:.4f} | Motor 2: pos={self.pos2:.4f}, vel={self.vel2:.4f}")
            else:
                self.get_logger().error(f"Error velocidades: {result.message if result else 'Sin respuesta'}")
        except Exception as e:
            self.get_logger().error(f"Error en respuesta de velocidades: {e}")
    
    def print_frequency(self):
        current_time = time.time()
        elapsed_time = current_time - self.last_time
        frequency = self.response_count / elapsed_time if elapsed_time > 0 else 0
        self.get_logger().info(f"==================== Frecuencia: {frequency:.1f} Hz ({self.response_count} respuestas en {elapsed_time:.1f}s) ====================")
        self.response_count = 0
        self.last_time = current_time
    
    def set_current_mode(self):
        """Configurar los motores 1 y 2 en modo corriente (modo 0)"""
        try:
            self.get_logger().info("Configurando motores en modo corriente...")
            mode_req = SetMode.Request()
            mode_req.motor_ids = [1, 2]
            mode_req.modes = [0, 0]  # Modo 0 = corriente
            
            future = self.mode_client.call_async(mode_req)
            rclpy.spin_until_future_complete(self, future)
            
            result = future.result()
            if result and result.success:
                self.get_logger().info("Motores configurados en modo corriente exitosamente")
            else:
                self.get_logger().error(f"Error configurando modo: {result.message if result else 'Sin respuesta'}")
                
        except Exception as e:
            self.get_logger().error(f"Error al configurar modo corriente: {e}")
    
    def compute_pd_control(self):
        """Compute PD control and send current commands"""
        try:
            # Generate desired trajectory (sinusoidal reference like in original code)
            current_time = time.time()
            elapsed_time = current_time - self.start_time
            qd = 0.5 * math.pi * math.sin(elapsed_time)
            
            # Compute position errors (desired position = 0.0 like in original)
            e_1 = self.pos1 - 0.0
            e_2 = self.pos2 - 0.0
            
            # Velocity estimator (low-pass filter implementation from original)
            self.vel_stim1 = self.Fc * (self.theta_1 + self.pos1)
            self.theta_1 = self.theta_1 - self.Tl * self.vel_stim1
            
            self.vel_stim2 = self.Fc * (self.theta_2 + self.pos2)
            self.theta_2 = self.theta_2 - self.Tl * self.vel_stim2
            
            # Compute PD control torques (non-linear proportional term)
            tau1 = -self.kp * ((abs(e_1)**self.p1) * np.sign(e_1)) - self.kd * self.vel_stim1
            tau2 = -self.kp * ((abs(e_2)**self.p1) * np.sign(e_2)) - self.kd * self.vel_stim2
            
            # Convert torque to current
            i_g_1 = tau1 / self.Kt
            i_g_2 = tau2 / self.Kt
            
            # Send current commands
            self.send_current_commands([i_g_1, i_g_2])
            
        except Exception as e:
            self.get_logger().error(f"Error en cálculo PD: {e}")
    
    def send_current_commands(self, currents):
        """Send current commands to motors"""
        try:
            current_req = SetMotorIdAndTargetCurrent.Request()
            current_req.motor_ids = [1, 2]
            current_req.target_currents = currents
            
            # Send asynchronously to avoid blocking
            self.current_client.call_async(current_req)
            
        except Exception as e:
            self.get_logger().error(f"Error enviando comandos de corriente: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = MotorPositionClient()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Cliente terminado por usuario")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()