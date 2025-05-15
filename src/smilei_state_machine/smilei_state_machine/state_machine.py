#!/usr/bin/env python3

"""
Máquina de estados principal para el robot SMILEi.
"""

import py_trees
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from smilei_state_machine.behaviors import (
    EnableRobot,
    HomePosition,
    ZeroPosition,
    SayHello,
    LocalTeleoperation,
    DisableRobot
)

class SMILEiStateNode(Node):
    def __init__(self):
        super().__init__("smilei_state_node")
        
        # Declarar parámetros de configuración
        self.declare_robot_parameters()
        
        # Cargar parámetros de configuración
        self.load_parameters()
        
        # Crear árbol de comportamiento
        self.create_root()
        
    def declare_robot_parameters(self):
        """
        Declarar todos los parámetros del robot que se cargarán del archivo YAML
        """
        # Parámetros para límites de articulaciones
        joint_limits_desc = ParameterDescriptor(
            name="joint_limits",
            type=ParameterType.PARAMETER_DOUBLE_ARRAY,
            description="Límites de las articulaciones en radianes"
        )
        self.declare_parameter("joint_limits.q_l1", [-1.5708, 1.5708], joint_limits_desc)
        self.declare_parameter("joint_limits.q_l2", [-1.5708, 0.7854], joint_limits_desc)
        self.declare_parameter("joint_limits.q_l3", [-1.5708, 2.3562], joint_limits_desc)
        self.declare_parameter("joint_limits.q_l4", [-1.5708, 1.5708], joint_limits_desc)
        self.declare_parameter("joint_limits.q_r1", [-1.5708, 1.5708], joint_limits_desc)
        self.declare_parameter("joint_limits.q_r2", [-0.7854, 1.5708], joint_limits_desc)
        self.declare_parameter("joint_limits.q_r3", [-2.3562, 1.5708], joint_limits_desc)
        self.declare_parameter("joint_limits.q_r4", [-1.5708, 1.5708], joint_limits_desc)
        
        # Parámetros para control de posición
        position_gains_desc = ParameterDescriptor(
            name="position_control_gains",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Ganancias para el control de posición"
        )
        self.declare_parameter("position_control_gains.p_gain_position", 5.0, position_gains_desc)
        self.declare_parameter("position_control_gains.d_gain_position", 0.2, position_gains_desc)
        self.declare_parameter("position_control_gains.i_gain_position", 0.0, position_gains_desc)
        self.declare_parameter("position_control_gains.iq_max", 1.5, position_gains_desc)
        self.declare_parameter("position_control_gains.p_gain_iq", 0.02, position_gains_desc)
        self.declare_parameter("position_control_gains.i_gain_iq", 0.02, position_gains_desc)
        self.declare_parameter("position_control_gains.d_gain_iq", 0.0, position_gains_desc)
        self.declare_parameter("position_control_gains.p_gain_id", 0.02, position_gains_desc)
        self.declare_parameter("position_control_gains.i_gain_id", 0.02, position_gains_desc)
        self.declare_parameter("position_control_gains.d_gain_id", 0.0, position_gains_desc)
        self.declare_parameter("position_control_gains.kt", 0.35, position_gains_desc)
        
        # Parámetros para control de corriente
        current_gains_desc = ParameterDescriptor(
            name="current_control_gains",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Ganancias para el control de corriente"
        )
        self.declare_parameter("current_control_gains.p_gain_position", 0.0, current_gains_desc)
        self.declare_parameter("current_control_gains.d_gain_position", 0.0, current_gains_desc)
        self.declare_parameter("current_control_gains.i_gain_position", 0.0, current_gains_desc)
        self.declare_parameter("current_control_gains.p_gain_force", 0.0, current_gains_desc)
        self.declare_parameter("current_control_gains.d_gain_force", 0.0, current_gains_desc)
        self.declare_parameter("current_control_gains.i_gain_force", 0.0, current_gains_desc)
        self.declare_parameter("current_control_gains.iq_max", 3.0, current_gains_desc)
        self.declare_parameter("current_control_gains.p_gain_iq", 0.277, current_gains_desc)
        self.declare_parameter("current_control_gains.i_gain_iq", 0.061, current_gains_desc)
        self.declare_parameter("current_control_gains.d_gain_iq", 0.0, current_gains_desc)
        self.declare_parameter("current_control_gains.p_gain_id", 0.277, current_gains_desc)
        self.declare_parameter("current_control_gains.i_gain_id", 0.061, current_gains_desc)
        self.declare_parameter("current_control_gains.d_gain_id", 0.0, current_gains_desc)
        self.declare_parameter("current_control_gains.kt", 0.35, current_gains_desc)
        
        # Parámetros del brazo derecho
        right_arm_desc = ParameterDescriptor(
            name="right_arm",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Parámetros físicos del brazo derecho"
        )
        self.declare_parameter("right_arm.m1", 0.361, right_arm_desc)
        self.declare_parameter("right_arm.m2", 0.400, right_arm_desc)
        self.declare_parameter("right_arm.m3", 0.452, right_arm_desc)
        self.declare_parameter("right_arm.m4", 0.181, right_arm_desc)
        self.declare_parameter("right_arm.d2", 0.113, right_arm_desc)
        self.declare_parameter("right_arm.a3", 0.123, right_arm_desc)
        self.declare_parameter("right_arm.rcx1", -0.0371, right_arm_desc)
        self.declare_parameter("right_arm.rcx2", -0.0154, right_arm_desc)
        self.declare_parameter("right_arm.rcy2", 0.0812, right_arm_desc)
        self.declare_parameter("right_arm.rcz2", -0.2084, right_arm_desc)
        self.declare_parameter("right_arm.rcy3", 0.1585, right_arm_desc)
        self.declare_parameter("right_arm.rcz3", 0.1687, right_arm_desc)
        self.declare_parameter("right_arm.rcx4", -0.1282, right_arm_desc)
        self.declare_parameter("right_arm.rcy4", -0.0039, right_arm_desc)
        self.declare_parameter("right_arm.rcz4", -0.4192, right_arm_desc)
        self.declare_parameter("right_arm.g", 9.81, right_arm_desc)
        
        # Parámetros del brazo izquierdo
        left_arm_desc = ParameterDescriptor(
            name="left_arm",
            type=ParameterType.PARAMETER_DOUBLE,
            description="Parámetros físicos del brazo izquierdo"
        )
        self.declare_parameter("left_arm.m1", 0.361, left_arm_desc)
        self.declare_parameter("left_arm.m2", 0.400, left_arm_desc)
        self.declare_parameter("left_arm.m3", 0.452, left_arm_desc)
        self.declare_parameter("left_arm.m4", 0.181, left_arm_desc)
        self.declare_parameter("left_arm.d2", 0.113, left_arm_desc)
        self.declare_parameter("left_arm.a3", 0.123, left_arm_desc)
        self.declare_parameter("left_arm.rcx1", -0.0264, left_arm_desc)
        self.declare_parameter("left_arm.rcx2", -0.0235, left_arm_desc)
        self.declare_parameter("left_arm.rcy2", -0.0609, left_arm_desc)
        self.declare_parameter("left_arm.rcz2", -0.0386, left_arm_desc)
        self.declare_parameter("left_arm.rcy3", 0.0150, left_arm_desc)
        self.declare_parameter("left_arm.rcz3", 0.0022, left_arm_desc)
        self.declare_parameter("left_arm.rcx4", -0.0228, left_arm_desc)
        self.declare_parameter("left_arm.rcy4", 0.0568, left_arm_desc)
        self.declare_parameter("left_arm.rcz4", -0.0279, left_arm_desc)
        self.declare_parameter("left_arm.g", 9.81, left_arm_desc)
    
    def load_parameters(self):
        """
        Cargar parámetros en estructuras de diccionarios para su uso en los comportamientos
        """
        # Límites de articulaciones
        self.joint_limits = {
            "q_l1": self.get_parameter("joint_limits.q_l1").value,
            "q_l2": self.get_parameter("joint_limits.q_l2").value,
            "q_l3": self.get_parameter("joint_limits.q_l3").value,
            "q_l4": self.get_parameter("joint_limits.q_l4").value,
            "q_r1": self.get_parameter("joint_limits.q_r1").value,
            "q_r2": self.get_parameter("joint_limits.q_r2").value,
            "q_r3": self.get_parameter("joint_limits.q_r3").value,
            "q_r4": self.get_parameter("joint_limits.q_r4").value,
        }
        
        # Ganancias para control de posición
        self.position_control_gains = {
            "p_gain_position": self.get_parameter("position_control_gains.p_gain_position").value,
            "d_gain_position": self.get_parameter("position_control_gains.d_gain_position").value,
            "i_gain_position": self.get_parameter("position_control_gains.i_gain_position").value,
            "iq_max": self.get_parameter("position_control_gains.iq_max").value,
            "p_gain_iq": self.get_parameter("position_control_gains.p_gain_iq").value,
            "i_gain_iq": self.get_parameter("position_control_gains.i_gain_iq").value,
            "d_gain_iq": self.get_parameter("position_control_gains.d_gain_iq").value,
            "p_gain_id": self.get_parameter("position_control_gains.p_gain_id").value,
            "i_gain_id": self.get_parameter("position_control_gains.i_gain_id").value,
            "d_gain_id": self.get_parameter("position_control_gains.d_gain_id").value,
            "kt": self.get_parameter("position_control_gains.kt").value
        }
        
        # Ganancias para control de corriente
        self.current_control_gains = {
            "p_gain_position": self.get_parameter("current_control_gains.p_gain_position").value,
            "d_gain_position": self.get_parameter("current_control_gains.d_gain_position").value,
            "i_gain_position": self.get_parameter("current_control_gains.i_gain_position").value,
            "p_gain_force": self.get_parameter("current_control_gains.p_gain_force").value,
            "d_gain_force": self.get_parameter("current_control_gains.d_gain_force").value,
            "i_gain_force": self.get_parameter("current_control_gains.i_gain_force").value,
            "iq_max": self.get_parameter("current_control_gains.iq_max").value,
            "p_gain_iq": self.get_parameter("current_control_gains.p_gain_iq").value,
            "i_gain_iq": self.get_parameter("current_control_gains.i_gain_iq").value,
            "d_gain_iq": self.get_parameter("current_control_gains.d_gain_iq").value,
            "p_gain_id": self.get_parameter("current_control_gains.p_gain_id").value,
            "i_gain_id": self.get_parameter("current_control_gains.i_gain_id").value,
            "d_gain_id": self.get_parameter("current_control_gains.d_gain_id").value,
            "kt": self.get_parameter("current_control_gains.kt").value
        }
        
        # Parámetros del brazo derecho
        self.right_arm = {
            "m1": self.get_parameter("right_arm.m1").value,
            "m2": self.get_parameter("right_arm.m2").value,
            "m3": self.get_parameter("right_arm.m3").value,
            "m4": self.get_parameter("right_arm.m4").value,
            "d2": self.get_parameter("right_arm.d2").value,
            "a3": self.get_parameter("right_arm.a3").value,
            "rcx1": self.get_parameter("right_arm.rcx1").value,
            "rcx2": self.get_parameter("right_arm.rcx2").value,
            "rcy2": self.get_parameter("right_arm.rcy2").value,
            "rcz2": self.get_parameter("right_arm.rcz2").value,
            "rcy3": self.get_parameter("right_arm.rcy3").value,
            "rcz3": self.get_parameter("right_arm.rcz3").value,
            "rcx4": self.get_parameter("right_arm.rcx4").value,
            "rcy4": self.get_parameter("right_arm.rcy4").value,
            "rcz4": self.get_parameter("right_arm.rcz4").value,
            "g": self.get_parameter("right_arm.g").value
        }
        
        # Parámetros del brazo izquierdo
        self.left_arm = {
            "m1": self.get_parameter("left_arm.m1").value,
            "m2": self.get_parameter("left_arm.m2").value,
            "m3": self.get_parameter("left_arm.m3").value,
            "m4": self.get_parameter("left_arm.m4").value,
            "d2": self.get_parameter("left_arm.d2").value,
            "a3": self.get_parameter("left_arm.a3").value,
            "rcx1": self.get_parameter("left_arm.rcx1").value,
            "rcx2": self.get_parameter("left_arm.rcx2").value,
            "rcy2": self.get_parameter("left_arm.rcy2").value,
            "rcz2": self.get_parameter("left_arm.rcz2").value,
            "rcy3": self.get_parameter("left_arm.rcy3").value,
            "rcz3": self.get_parameter("left_arm.rcz3").value,
            "rcx4": self.get_parameter("left_arm.rcx4").value,
            "rcy4": self.get_parameter("left_arm.rcy4").value,
            "rcz4": self.get_parameter("left_arm.rcz4").value,
            "g": self.get_parameter("left_arm.g").value
        }
        
        # Establecer parámetros como valores del nodo para que los comportamientos puedan acceder a ellos
        self.set_parameters([
            Parameter("joint_limits", Parameter.Type.STRING, self.joint_limits),
            Parameter("position_control_gains", Parameter.Type.STRING, self.position_control_gains),
            Parameter("current_control_gains", Parameter.Type.STRING, self.current_control_gains),
            Parameter("right_arm", Parameter.Type.STRING, self.right_arm),
            Parameter("left_arm", Parameter.Type.STRING, self.left_arm)
        ])
        
    def create_root(self):
        # Crear comportamientos
        enable_robot = EnableRobot(self)
        home_position = HomePosition(self)
        zero_position = ZeroPosition(self)
        say_hello = SayHello(self)
        local_teleoperation = LocalTeleoperation(self)
        disable_robot = DisableRobot(self)
        
        # Crear árbol raíz
        self.root = py_trees.composites.Sequence("Root")
        self.root.add_child(enable_robot)
        
        # Secuencia de comportamientos de prueba
        test_sequence = py_trees.composites.Sequence("Test Sequence")
        test_sequence.add_child(home_position)
        test_sequence.add_child(zero_position)
        test_sequence.add_child(say_hello)
        test_sequence.add_child(local_teleoperation)
        test_sequence.add_child(disable_robot)
        
        self.root.add_child(test_sequence)
        
def main(args=None):
    rclpy.init(args=args)
    node = SMILEiStateNode()
    
    # Crear y ejecutar el árbol de comportamiento
    behavior_tree = py_trees.trees.BehaviourTree(node.root)
    behavior_tree.setup(timeout=15)
    
    try:
        behavior_tree.tick_tock(
            period_ms=500,
            number_of_iterations=py_trees.trees.CONTINUOUS_TICK_TOCK,
            pre_tick_handler=None,
            post_tick_handler=None
        )
    except KeyboardInterrupt:
        node.get_logger().info('Árbol de comportamiento interrumpido por el usuario')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 