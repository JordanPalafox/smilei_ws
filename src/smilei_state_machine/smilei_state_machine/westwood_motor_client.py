#!/usr/bin/env python3

"""
Cliente para comunicarse con el servidor Westwood Motor.
Este módulo proporciona una interfaz para interactuar con los motores del robot
utilizando los servicios ROS expuestos por westwood_motor_server.py.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import time
import math
from threading import Lock

from westwood_motor_interfaces.srv import SetMotorIdAndTarget, GetMotorPositions, GetAvailableMotors

class WestwoodMotorClient:
    """
    Cliente para comunicarse con los servicios del servidor de motores Westwood.
    """
    def __init__(self, node):
        """
        Inicializa el cliente con el nodo ROS proporcionado.
        
        Args:
            node: Nodo ROS que contiene los clientes de servicio
        """
        self.node = node
        self.lock = Lock()
        
        # Crear clientes para los servicios
        self.set_motor_client = self.node.create_client(
            SetMotorIdAndTarget,
            'westwood_motor/set_motor_id_and_target'
        )
        
        self.get_positions_client = self.node.create_client(
            GetMotorPositions,
            'westwood_motor/get_motor_positions'
        )
        
        self.get_motors_client = self.node.create_client(
            GetAvailableMotors,
            'westwood_motor/get_available_motors'
        )
        
        # Esperar a que los servicios estén disponibles
        for client, name in [
            (self.set_motor_client, 'set_motor_id_and_target'),
            (self.get_positions_client, 'get_motor_positions'),
            (self.get_motors_client, 'get_available_motors')
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.node.get_logger().warn(f'Esperando al servicio {name}...')
        
        self.node.get_logger().info('Todos los servicios de motor disponibles')
    
    def get_available_motors(self):
        """
        Obtiene la lista de IDs de motores disponibles.
        
        Returns:
            list: Lista de IDs de motores disponibles
        """
        with self.lock:
            request = GetAvailableMotors.Request()
            future = self.get_motors_client.call_async(request)
            
            rclpy.spin_until_future_complete(self.node, future)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    return response.motor_ids
                else:
                    self.node.get_logger().error(f'Error: {response.message}')
                    return [1, 2, 3, 4, 5, 6, 7, 8]  # Valores por defecto
            else:
                self.node.get_logger().error('Servicio de obtención de motores disponibles falló')
                return [1, 2, 3, 4, 5, 6, 7, 8]  # Valores por defecto
    
    def ping(self, *motor_ids):
        """
        Verifica si los motores especificados están disponibles.
        
        Args:
            motor_ids: IDs de los motores a verificar
            
        Returns:
            bool: True si todos los motores respondieron, False en caso contrario
        """
        available_motors = self.get_available_motors()
        for motor_id in motor_ids:
            if motor_id not in available_motors:
                return False
        return True
    
    def get_present_position(self, motor_id):
        """
        Obtiene la posición actual del motor especificado.
        
        Args:
            motor_id: ID del motor
            
        Returns:
            float: Posición actual del motor, o 0.0 si hay error
        """
        with self.lock:
            request = GetMotorPositions.Request()
            request.motor_ids = [motor_id]
            
            future = self.get_positions_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            
            if future.result() is not None:
                response = future.result()
                if response.success and response.positions:
                    # Convertir al formato esperado para mantener compatibilidad
                    return [[[response.positions[0]]]]
                else:
                    self.node.get_logger().error(f'Error al obtener posición: {response.message}')
                    return [[[0.0]]]
            else:
                self.node.get_logger().error('Servicio de obtención de posición falló')
                return [[[0.0]]]
    
    def get_present_velocity(self, motor_id):
        """
        Simula obtener la velocidad actual (no disponible en el servidor actual).
        
        Args:
            motor_id: ID del motor
            
        Returns:
            float: Velocidad simulada (0.0)
        """
        # Como el servicio no proporciona velocidad, devolvemos 0.0
        return [[[0.0]]]
    
    def set_goal_position(self, *args):
        """
        Establece las posiciones objetivo para los motores especificados.
        
        Args:
            args: Tuplas de (motor_id, position)
        """
        with self.lock:
            motor_ids = []
            positions = []
            
            for arg in args:
                motor_ids.append(arg[0])
                positions.append(arg[1])
            
            request = SetMotorIdAndTarget.Request()
            request.motor_ids = motor_ids
            request.target_positions = positions
            
            future = self.set_motor_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            
            if future.result() is not None:
                response = future.result()
                if not response.success:
                    self.node.get_logger().error(f'Error: {response.message}')
            else:
                self.node.get_logger().error('Servicio de control de posición falló')
    
    def set_goal_iq(self, *args):
        """
        Establece las corrientes objetivo para los motores especificados.
        Nota: Este método simula la API original pero no está disponible en el servidor actual.
        
        Args:
            args: Tuplas de (motor_id, iq)
        """
        self.node.get_logger().warn('set_goal_iq: Esta función no está implementada en el servidor actual')
    
    # Funciones para configurar los parámetros de control
    def set_p_gain_position(self, *args):
        self.node.get_logger().debug('set_p_gain_position: Simulado')
        
    def set_i_gain_position(self, *args):
        self.node.get_logger().debug('set_i_gain_position: Simulado')
        
    def set_d_gain_position(self, *args):
        self.node.get_logger().debug('set_d_gain_position: Simulado')
        
    def set_p_gain_iq(self, *args):
        self.node.get_logger().debug('set_p_gain_iq: Simulado')
        
    def set_i_gain_iq(self, *args):
        self.node.get_logger().debug('set_i_gain_iq: Simulado')
        
    def set_d_gain_iq(self, *args):
        self.node.get_logger().debug('set_d_gain_iq: Simulado')
        
    def set_p_gain_id(self, *args):
        self.node.get_logger().debug('set_p_gain_id: Simulado')
        
    def set_i_gain_id(self, *args):
        self.node.get_logger().debug('set_i_gain_id: Simulado')
        
    def set_d_gain_id(self, *args):
        self.node.get_logger().debug('set_d_gain_id: Simulado')
    
    def set_p_gain_force(self, *args):
        self.node.get_logger().debug('set_p_gain_force: Simulado')
        
    def set_i_gain_force(self, *args):
        self.node.get_logger().debug('set_i_gain_force: Simulado')
        
    def set_d_gain_force(self, *args):
        self.node.get_logger().debug('set_d_gain_force: Simulado')
        
    def set_mode(self, *args):
        self.node.get_logger().debug('set_mode: Simulado')
        
    def set_limit_iq_max(self, *args):
        self.node.get_logger().debug('set_limit_iq_max: Simulado')
        
    def set_torque_enable(self, *args):
        self.node.get_logger().debug('set_torque_enable: Simulado')
        
    def close(self):
        """
        Cierra la conexión con el servidor.
        """
        self.node.get_logger().info('Cerrando conexión con el servidor de motores') 