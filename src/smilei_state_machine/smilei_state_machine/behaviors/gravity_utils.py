#!/usr/bin/env python3

"""
Utilidades para el cálculo de vectores de gravedad de los brazos robot SMILEi.
"""

import numpy as np
import math
import rclpy
import threading

# Clase para almacenar los parámetros del robot
class RobotParameters:
    _instance = None
    _lock = threading.Lock()
    
    @classmethod
    def get_instance(cls, node=None):
        with cls._lock:
            if cls._instance is None:
                cls._instance = cls(node)
            elif node is not None and cls._instance.node is None:
                # Actualizar la instancia con el nodo proporcionado
                cls._instance.set_node(node)
            return cls._instance
        
    def __init__(self, node=None):
        self.right_arm_params = None
        self.left_arm_params = None
        self.node = node
        
        if self.node is not None:
            self._load_parameters()
        else:
            # Si no se proporciona un nodo, usar valores predeterminados
            self._set_default_parameters()
        
    def _load_parameters(self):
        # Cargar parámetros desde el servidor ROS
        try:
            self.right_arm_params = self.node.get_parameter("right_arm").value
            self.left_arm_params = self.node.get_parameter("left_arm").value
            
            # Si alguno es None, usar valores predeterminados
            if self.right_arm_params is None or self.left_arm_params is None:
                self.node.get_logger().warn("No se pudieron cargar los parámetros de los brazos, usando valores predeterminados")
                self._set_default_parameters()
        except Exception as e:
            if self.node:
                self.node.get_logger().error(f"Error al cargar los parámetros: {str(e)}")
            self._set_default_parameters()
    
    def _set_default_parameters(self):
        # Parámetros predeterminados
        self.right_arm_params = {
            "m1": 0.361,
            "m2": 0.400,
            "m3": 0.452,
            "m4": 0.181,
            "d2": 0.113,
            "a3": 0.123,
            "rcx1": -0.0371,
            "rcx2": -0.0154,
            "rcy2": 0.0812,
            "rcz2": -0.2084,
            "rcy3": 0.1585,
            "rcz3": 0.1687,
            "rcx4": -0.1282,
            "rcy4": -0.0039,
            "rcz4": -0.4192,
            "g": 9.81
        }
        
        self.left_arm_params = {
            "m1": 0.361,
            "m2": 0.400,
            "m3": 0.452,
            "m4": 0.181,
            "d2": 0.113,
            "a3": 0.123,
            "rcx1": -0.0264,
            "rcx2": -0.0235,
            "rcy2": -0.0609,
            "rcz2": -0.0386,
            "rcy3": 0.0150,
            "rcz3": 0.0022,
            "rcx4": -0.0228,
            "rcy4": 0.0568,
            "rcz4": -0.0279,
            "g": 9.81
        }
        
    def set_node(self, node):
        if node is not None:
            self.node = node
            self._load_parameters()

def right_gravity_vector(q, node=None):
    """
    Calcula el vector de gravedad para el brazo derecho.
    
    Args:
        q: Vector de posiciones articulares [q1, q2, q3, q4]
        node: Nodo ROS para acceder a los parámetros
        
    Returns:
        Vector de gravedad de 4 componentes
    """
    q = q.copy()
    q[3] = -1.0 * q[3]
    
    # Obtener parámetros
    robot_params = RobotParameters.get_instance(node)
    params = robot_params.right_arm_params
    
    g = params["g"]
    m1 = params["m1"]
    m2 = params["m2"]
    m3 = params["m3"]
    m4 = params["m4"]
    a3 = params["a3"]
    d2 = params["d2"]
    rcx1 = params["rcx1"]
    rcx2 = params["rcx2"]
    rcy2 = params["rcy2"]
    rcz2 = params["rcz2"]
    rcy3 = params["rcy3"]
    rcz3 = params["rcz3"]
    rcx4 = params["rcx4"]
    rcy4 = params["rcy4"]
    rcz4 = params["rcz4"]
    
    G = np.zeros(4)
    sinq = np.zeros(4)
    cosq = np.zeros(4)

    for i in range(4):
        sinq[i] = math.sin(q[i])
        cosq[i] = math.cos(q[i])
        
    g1 = g*m4*(rcz4*(cosq[0]*cosq[2] - cosq[1]*sinq[0]*sinq[2]) + rcx4*(sinq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) + cosq[3]*sinq[0]*sinq[1]) + rcy4*(cosq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) - sinq[0]*sinq[1]*sinq[3]) + a3*sinq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) + d2*sinq[0]*sinq[1] + a3*cosq[3]*sinq[0]*sinq[1]) - g*m2*(rcy2*cosq[0] + rcx2*cosq[1]*sinq[0] - rcz2*sinq[0]*sinq[1]) + g*m3*(rcz3*(cosq[0]*cosq[2] - cosq[1]*sinq[0]*sinq[2]) + d2*sinq[0]*sinq[1] + rcy3*sinq[0]*sinq[1]) - g*m1*rcx1*sinq[0]
    g2 = -g*cosq[0]*(d2*m3*cosq[1] + d2*m4*cosq[1] + m3*rcy3*cosq[1] + m2*rcz2*cosq[1] + m2*rcx2*sinq[1] + m3*rcz3*sinq[1]*sinq[2] + m4*rcz4*sinq[1]*sinq[2] + a3*m4*cosq[1]*cosq[3] + m4*rcx4*cosq[1]*cosq[3] - m4*rcy4*cosq[1]*sinq[3] - a3*m4*cosq[2]*sinq[1]*sinq[3] - m4*rcy4*cosq[2]*cosq[3]*sinq[1] - m4*rcx4*cosq[2]*sinq[1]*sinq[3])
    g3 = g*m4*(a3*sinq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]) - rcz4*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + rcy4*cosq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]) + rcx4*sinq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2])) - g*m3*rcz3*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2])    
    g4 = g*m4*(rcx4*(cosq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + cosq[0]*sinq[1]*sinq[3]) - rcy4*(sinq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) - cosq[0]*cosq[3]*sinq[1]) + a3*cosq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + a3*cosq[0]*sinq[1]*sinq[3])

    G[0] = g1
    G[1] = g2
    G[2] = g3 * 0
    G[3] = g4 * 0
    return G

def left_gravity_vector(q, node=None):
    """
    Calcula el vector de gravedad para el brazo izquierdo.
    
    Args:
        q: Vector de posiciones articulares [q1, q2, q3, q4]
        node: Nodo ROS para acceder a los parámetros
        
    Returns:
        Vector de gravedad de 4 componentes
    """
    q = q.copy()
    q[3] = -1.0 * q[3]
    
    # Obtener parámetros
    robot_params = RobotParameters.get_instance(node)
    params = robot_params.left_arm_params
    
    g = params["g"]
    m1 = params["m1"]
    m2 = params["m2"]
    m3 = params["m3"]
    m4 = params["m4"]
    a3 = params["a3"]
    d2 = params["d2"]
    rcx1 = params["rcx1"]
    rcx2 = params["rcx2"]
    rcy2 = params["rcy2"]
    rcz2 = params["rcz2"]
    rcy3 = params["rcy3"]
    rcz3 = params["rcz3"]
    rcx4 = params["rcx4"]
    rcy4 = params["rcy4"]
    rcz4 = params["rcz4"]

    G = np.zeros(4)
    sinq = np.zeros(4)
    cosq = np.zeros(4)

    for i in range(4):
        sinq[i] = math.sin(q[i])
        cosq[i] = math.cos(q[i])
        
    g1 = -g*m2*(rcy2*cosq[0] + rcx2*cosq[1]*sinq[0] + rcz2*sinq[0]*sinq[1]) - g*m4*(rcz4*(cosq[0]*cosq[2] - cosq[1]*sinq[0]*sinq[2]) + rcx4*(sinq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) + cosq[3]*sinq[0]*sinq[1]) + rcy4*(cosq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) - sinq[0]*sinq[1]*sinq[3]) + a3*sinq[3]*(cosq[0]*sinq[2] + cosq[1]*cosq[2]*sinq[0]) + d2*sinq[0]*sinq[1] + a3*cosq[3]*sinq[0]*sinq[1]) - g*m3*(rcz3*(cosq[0]*cosq[2] - cosq[1]*sinq[0]*sinq[2]) + d2*sinq[0]*sinq[1] - rcy3*sinq[0]*sinq[1]) - g*m1*rcx1*sinq[0]
    g2 = g*cosq[0]*(d2*m3*cosq[1] + d2*m4*cosq[1] - m3*rcy3*cosq[1] + m2*rcz2*cosq[1] - m2*rcx2*sinq[1] + m3*rcz3*sinq[1]*sinq[2] + m4*rcz4*sinq[1]*sinq[2] + a3*m4*cosq[1]*cosq[3] + m4*rcx4*cosq[1]*cosq[3] - m4*rcy4*cosq[1]*sinq[3] - a3*m4*cosq[2]*sinq[1]*sinq[3] - m4*rcy4*cosq[2]*cosq[3]*sinq[1] - m4*rcx4*cosq[2]*sinq[1]*sinq[3])
    g3 = g*m3*rcz3*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) - g*m4*(a3*sinq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]) - rcz4*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + rcy4*cosq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]) + rcx4*sinq[3]*(cosq[2]*sinq[0] + cosq[0]*cosq[1]*sinq[2]))
    g4 = -g*m4*(rcx4*(cosq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + cosq[0]*sinq[1]*sinq[3]) - rcy4*(sinq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) - cosq[0]*cosq[3]*sinq[1]) + a3*cosq[3]*(sinq[0]*sinq[2] - cosq[0]*cosq[1]*cosq[2]) + a3*cosq[0]*sinq[1]*sinq[3])
    
    G[0] = g1
    G[1] = g2
    G[2] = g3 * 0
    G[3] = g4 * 0
    return G

def right_arm_fk(q, node=None):
    """
    Calcula la cinemática directa del brazo derecho.
    
    Args:
        q: Vector de posiciones articulares [q1, q2, q3, q4]
        node: Nodo ROS para acceder a los parámetros
        
    Returns:
        Posición del efector final [x, y, z]
    """
    q = q.copy()
    q[3] = -1.0 * q[3]
    
    # Obtener parámetros
    robot_params = RobotParameters.get_instance(node)
    params = robot_params.right_arm_params
    
    a3 = params["a3"]
    d2 = params["d2"]
    
    K = np.zeros(3)
    sinq = np.zeros(4)
    cosq = np.zeros(4)

    for i in range(4):
        sinq[i] = math.sin(q[i])
        cosq[i] = math.cos(q[i])

    Px = sinq[3]*cosq[2]*sinq[0]*cosq[1]*a3 + sinq[3]*sinq[2]*cosq[0]*a3 + sinq[0]*sinq[1]*a3*cosq[3] + sinq[0]*sinq[1]*d2
    Py = sinq[1]*cosq[2]*a3*sinq[3] - cosq[1]*a3*cosq[3] - cosq[1]*d2
    Pz = -sinq[3]*cosq[2]*cosq[1]*cosq[0]*a3 + sinq[3]*sinq[2]*sinq[0]*a3 - cosq[0]*sinq[1]*a3*cosq[3] - cosq[0]*sinq[1]*d2
    K = np.array([Px, Py, Pz])
    return K

def left_arm_fk(q, node=None):
    """
    Calcula la cinemática directa del brazo izquierdo.
    
    Args:
        q: Vector de posiciones articulares [q1, q2, q3, q4]
        node: Nodo ROS para acceder a los parámetros
        
    Returns:
        Posición del efector final [x, y, z]
    """
    q = q.copy()
    q[3] = -1.0 * q[3]
    
    # Obtener parámetros
    robot_params = RobotParameters.get_instance(node)
    params = robot_params.left_arm_params
    
    a3 = params["a3"]
    d2 = params["d2"]
    
    K = np.zeros(3)
    sinq = np.zeros(4)
    cosq = np.zeros(4)

    for i in range(4):
        sinq[i] = math.sin(q[i])
        cosq[i] = math.cos(q[i])

    Px = sinq[3]*cosq[2]*sinq[0]*cosq[1]*a3 + sinq[3]*sinq[2]*cosq[0]*a3 + sinq[0]*sinq[1]*a3*cosq[3] + sinq[0]*sinq[1]*d2
    Py = -sinq[1]*cosq[2]*a3*sinq[3] + cosq[1]*a3*cosq[3] + cosq[1]*d2
    Pz = sinq[3]*cosq[2]*cosq[1]*cosq[0]*a3 - sinq[3]*sinq[2]*sinq[0]*a3 + cosq[0]*sinq[1]*a3*cosq[3] + cosq[0]*sinq[1]*d2
    K = np.array([Px, Py, Pz])
    return K 