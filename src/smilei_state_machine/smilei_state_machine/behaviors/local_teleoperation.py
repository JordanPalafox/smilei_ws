import py_trees
import numpy as np
import rclpy
import math
import time
import sys
import select
from westwood_motor_interfaces.srv import SetMotorIdAndTarget, GetMotorPositions
from westwood_motor_interfaces.srv import SetGains, SetMode, SetTorqueEnable
from westwood_motor_interfaces.srv import SetGoalIq

# Definiciones de límites y parámetros
class Joint_Limits:
    q_l1=[-90.0, 90.0]
    q_l2=[-90.0, 45.0]
    q_l3=[-90.0, 135.0]
    q_l4=[-90.0, 90.0]
    q_r1=[-90.0, 90.0]
    q_r2=[-45.0, 90.0]
    q_r3=[-135.0, 90.0]
    q_r4=[-90.0, 90.0]

class Current_Control_Mode_Table_Gains:
    """
    Ganancias para el modo de control de corriente
    """
    p_gain_position = 0.0
    d_gain_position = 0.0
    i_gain_position = 0.0
    p_gain_force = 0.0
    d_gain_force = 0.0
    i_gain_force = 0.0
    iq_max = 3.0
    p_gain_iq = 0.277
    i_gain_iq = 0.061
    d_gain_iq = 0.0
    p_gain_id = 0.277
    i_gain_id = 0.061
    d_gain_id = 0.0
    kt = 0.35

class Position_Control_Mode_Table_Gains:
    """
    Ganancias para el modo de control de posición
    """
    p_gain_position = 5.0
    d_gain_position = 0.2
    i_gain_position = 0.0
    iq_max = 1.5
    p_gain_iq = 0.02
    i_gain_iq = 0.02
    d_gain_iq = 0.0
    p_gain_id = 0.02
    i_gain_id = 0.02
    d_gain_id = 0.0
    kt = 0.35

class RightArmParams:
    """Parámetros físicos del brazo derecho"""
    # Masas
    m1 = 0.361
    m2 = 0.400
    m3 = 0.452
    m4 = 0.181
    # Longitudes
    d2 = 0.113
    a3 = 0.123
    # Parámetros de inercia
    rcx1 = -0.0371
    rcx2 = -0.0154
    rcy2 = 0.0812
    rcz2 = -0.2084
    rcy3 = 0.1585
    rcz3 = 0.1687
    rcx4 = -0.1282
    rcy4 = -0.0039
    rcz4 = -0.4192
    # Gravedad
    g = 9.81

class LeftArmParams:
    """Parámetros físicos del brazo izquierdo"""
    # Masas
    m1 = 0.361
    m2 = 0.400
    m3 = 0.452
    m4 = 0.181
    # Longitudes
    d2 = 0.113
    a3 = 0.123
    # Parámetros de inercia
    rcx1 = -0.0264
    rcx2 = -0.0235
    rcy2 = -0.0609
    rcz2 = -0.0386
    rcy3 = 0.0150
    rcz3 = 0.0022
    rcx4 = -0.0228
    rcy4 = 0.0568
    rcz4 = -0.0279
    # Gravedad
    g = 9.81

class LocalTeleoperation(py_trees.behaviour.Behaviour):
    """Comportamiento que implementa teleoperación local con control de gravedad entre los brazos del robot"""
    def __init__(self, name: str, motor_ids: list[int], node=None):
        super().__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.running = False
        self.own_node = False
        
        # Clientes para servicios
        self.set_position_client = None
        self.get_position_client = None
        self.set_gains_client = None
        self.set_mode_client = None
        self.set_torque_client = None
        self.set_goal_iq_client = None
        
        # Parámetros de control
        self.kp = 1.5  # Aumentado para mejor seguimiento
        self.kd = 0.05  # Aumentado para mejor amortiguamiento
        self.Kt = Current_Control_Mode_Table_Gains.kt
        
        # Comprobar que tenemos al menos 2 motores
        if len(motor_ids) < 2:
            raise ValueError("Se requieren al menos 2 motores para la teleoperación")
            
        # Modo de teleoperación (8 motores = completo, 2 motores = simple)
        self.full_teleoperation = len(motor_ids) >= 8
        self.node_created = False
        
        # Inicializar detección de movimiento
        self.movement_threshold = 0.01  # Umbral reducido para mayor sensibilidad
        self.max_current = 1.2  # Corriente máxima aumentada para mejor respuesta

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        # Usar el nodo proporcionado en lugar de crear uno nuevo
        if self.node is None:
            self.node = rclpy.create_node('local_teleoperation_client')
            self.own_node = True
        else:
            self.own_node = False
        
        # Crear clientes para los servicios
        self.set_position_client = self.node.create_client(
            SetMotorIdAndTarget,
            'westwood_motor/set_motor_id_and_target'
        )
        
        self.get_position_client = self.node.create_client(
            GetMotorPositions,
            'westwood_motor/get_motor_positions'
        )
        
        self.set_gains_client = self.node.create_client(
            SetGains,
            'westwood_motor/set_position_gains'
        )
        
        self.set_mode_client = self.node.create_client(
            SetMode,
            'westwood_motor/set_mode'
        )
        
        self.set_torque_client = self.node.create_client(
            SetTorqueEnable,
            'westwood_motor/set_torque_enable'
        )
        
        self.set_goal_iq_client = self.node.create_client(
            SetGoalIq,
            'westwood_motor/set_goal_iq'
        )
        
        if timeout_sec is None:
            timeout_sec = 1.0
        
        # Intentar esperar por los servicios principales
        if not self.set_position_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("Servicio set_motor_id_and_target no disponible, continuando en modo simulación")
        
        if not self.get_position_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("Servicio get_motor_positions no disponible, continuando en modo simulación")
        
        return True  # Siempre retorna True para permitir que continúe

    def zero_position(self):
        """Envía todos los motores a posición cero"""
        self.node.get_logger().info("Enviando motores a posición cero")
        
        # Crear solicitud para establecer posiciones a cero
        req = SetMotorIdAndTarget.Request()
        req.motor_ids = self.motor_ids
        req.target_positions = [0.0] * len(self.motor_ids)
        
        # Llamar al servicio
        try:
            future = self.set_position_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.done():
                result = future.result()
                if not result.success:
                    self.node.get_logger().warning(f"Error al establecer posición cero: {result.message}")
                    return False
            else:
                self.node.get_logger().warning("Timeout al establecer posición cero")
                return False
            
            # Esperar a que los motores lleguen a la posición
            time.sleep(2.0)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al establecer posición cero: {str(e)}")
            return False

    def setup_current_control(self):
        """Configura los motores para control de corriente"""
        self.node.get_logger().info("Configurando motores para control de corriente")
        
        try:
            # Usar los valores de la clase Current_Control_Mode_Table_Gains
            # Asegurarse de que todos los valores sean float
            req_gains = SetGains.Request()
            req_gains.motor_ids = self.motor_ids
            req_gains.p_gain_position = float(Current_Control_Mode_Table_Gains.p_gain_position)
            req_gains.i_gain_position = float(Current_Control_Mode_Table_Gains.i_gain_position)
            req_gains.d_gain_position = float(Current_Control_Mode_Table_Gains.d_gain_position)
            req_gains.p_gain_iq = float(Current_Control_Mode_Table_Gains.p_gain_iq)
            req_gains.i_gain_iq = float(Current_Control_Mode_Table_Gains.i_gain_iq)
            req_gains.d_gain_iq = float(Current_Control_Mode_Table_Gains.d_gain_iq)
            req_gains.p_gain_id = float(Current_Control_Mode_Table_Gains.p_gain_id)
            req_gains.i_gain_id = float(Current_Control_Mode_Table_Gains.i_gain_id)
            req_gains.d_gain_id = float(Current_Control_Mode_Table_Gains.d_gain_id)
            req_gains.p_gain_force = float(Current_Control_Mode_Table_Gains.p_gain_force)
            req_gains.i_gain_force = float(Current_Control_Mode_Table_Gains.i_gain_force)
            req_gains.d_gain_force = float(Current_Control_Mode_Table_Gains.d_gain_force)
            req_gains.iq_max = float(Current_Control_Mode_Table_Gains.iq_max)
            req_gains.kt = float(Current_Control_Mode_Table_Gains.kt)
            
            future = self.set_gains_client.call_async(req_gains)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            # Configurar modo corriente (modo 0)
            req_mode = SetMode.Request()
            req_mode.motor_ids = self.motor_ids
            req_mode.modes = [0] * len(self.motor_ids)
            
            future = self.set_mode_client.call_async(req_mode)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            # Habilitar torque
            req_torque = SetTorqueEnable.Request()
            req_torque.motor_ids = self.motor_ids
            req_torque.enable_torque = [True] * len(self.motor_ids)
            
            future = self.set_torque_client.call_async(req_torque)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al configurar motores para control de corriente: {str(e)}")
            return False

    def left_gravity_vector(self, q):
        """Vector de gravedad para el brazo izquierdo usando parámetros físicos"""
        # Implementación mejorada - usando los parámetros físicos
        gravity = np.zeros(len(q))
        if len(q) >= 4:
            # Aproximación simplificada del modelo de gravedad
            gravity[0] = LeftArmParams.m1 * LeftArmParams.g * np.sin(q[0])
            gravity[1] = LeftArmParams.m2 * LeftArmParams.g * np.sin(q[1])
            gravity[2] = LeftArmParams.m3 * LeftArmParams.g * np.sin(q[2])
            gravity[3] = LeftArmParams.m4 * LeftArmParams.g * np.sin(q[3])
        return gravity * 0.1  # Factor de escala para ajustar la compensación

    def right_gravity_vector(self, q):
        """Vector de gravedad para el brazo derecho usando parámetros físicos"""
        # Implementación mejorada - usando los parámetros físicos
        gravity = np.zeros(len(q))
        if len(q) >= 4:
            # Aproximación simplificada del modelo de gravedad
            gravity[0] = RightArmParams.m1 * RightArmParams.g * np.sin(q[0])
            gravity[1] = RightArmParams.m2 * RightArmParams.g * np.sin(q[1])
            gravity[2] = RightArmParams.m3 * RightArmParams.g * np.sin(q[2])
            gravity[3] = RightArmParams.m4 * RightArmParams.g * np.sin(q[3])
        return gravity * 0.1  # Factor de escala para ajustar la compensación

    def get_motor_position(self, motor_id):
        """Obtiene la posición actual de un motor"""
        req = GetMotorPositions.Request()
        req.motor_ids = [motor_id]
        
        try:
            future = self.get_position_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
            
            if future.done():
                result = future.result()
                if result.success and len(result.positions) > 0:
                    return result.positions[0]
            
            return 0.0
        except Exception as e:
            self.node.get_logger().error(f"Error al obtener posición del motor {motor_id}: {str(e)}")
            return 0.0

    def get_motor_velocity(self, motor_id):
        """Estimación de velocidad basada en diferencias de posición"""
        # En un sistema real, usaríamos un servicio específico para velocidad
        if not hasattr(self, 'last_positions'):
            self.last_positions = {}
            self.last_velocity_time = {}
            return 0.0
            
        current_pos = self.get_motor_position(motor_id)
        current_time = time.time()
        
        if motor_id in self.last_positions and motor_id in self.last_velocity_time:
            dt = current_time - self.last_velocity_time[motor_id]
            if dt > 0:
                velocity = (current_pos - self.last_positions[motor_id]) / dt
                self.last_positions[motor_id] = current_pos
                self.last_velocity_time[motor_id] = current_time
                return velocity
        
        # Si es la primera vez o no hay datos previos
        self.last_positions[motor_id] = current_pos
        self.last_velocity_time[motor_id] = current_time
        return 0.0

    def initialise(self) -> None:
        self.node.get_logger().info("Iniciando teleoperación local")
        self.running = True
        
        # Inicializar diccionarios para velocidades
        self.last_positions = {}
        self.last_velocity_time = {}
        
        # Para detección de movimiento en modo simple
        self.prev_positions = {motor_id: None for motor_id in self.motor_ids}
        
        # Configuración inicial
        self.zero_position()
        self.setup_current_control()

    def update(self) -> py_trees.common.Status:
        # Si no está corriendo, devolver SUCCESS para permitir la transición al siguiente estado
        if not self.running:
            return py_trees.common.Status.SUCCESS
        
        # Verificar si hay entrada del usuario para terminar
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            if line:
                self.node.get_logger().info("Terminando teleoperación local por entrada del usuario")
                self.running = False
                return py_trees.common.Status.SUCCESS
        
        try:
            # Modo teleoperación completa (8 motores)
            if self.full_teleoperation and len(self.motor_ids) >= 8:
                self._update_full_teleoperation()
            else:
                # Modo simple (2 o más motores)
                self._update_simple_teleoperation()
            
            # Tiempo de pausa para estabilidad
            time.sleep(0.001)
            
        except Exception as e:
            self.node.get_logger().error(f"Error en teleoperación local: {str(e)}")
        
        return py_trees.common.Status.RUNNING

    def _update_simple_teleoperation(self):
        """Implementa teleoperación simple entre pares de motores con respuesta mejorada"""
        # Variables para almacenar posiciones actuales y anteriores
        if len(self.motor_ids) < 2:
            return
        
        # En modo simple, trabajamos con los primeros dos motores
        motor_1 = self.motor_ids[0]  # Primer motor
        motor_2 = self.motor_ids[1]  # Segundo motor
        
        # Obtener posiciones actuales
        q1 = self.get_motor_position(motor_1)
        q2 = self.get_motor_position(motor_2)
        
        # Obtener velocidades para amortiguamiento
        dq1 = self.get_motor_velocity(motor_1)
        dq2 = self.get_motor_velocity(motor_2)
        
        # Determinar cuál motor se ha movido más desde la lectura anterior
        moved_motor = None
        target_position = None
        
        if self.prev_positions[motor_1] is not None and self.prev_positions[motor_2] is not None:
            delta1 = abs(q1 - self.prev_positions[motor_1]) if self.prev_positions[motor_1] is not None else 0
            delta2 = abs(q2 - self.prev_positions[motor_2]) if self.prev_positions[motor_2] is not None else 0
            
            # Si el motor 1 se ha movido más que el motor 2 y supera el umbral
            if delta1 > delta2 and delta1 > self.movement_threshold:
                moved_motor = motor_1
                target_position = q1
            
            # Si el motor 2 se ha movido más que el motor 1 y supera el umbral
            elif delta2 > delta1 and delta2 > self.movement_threshold:
                moved_motor = motor_2
                target_position = q2
        
        # Guardar posiciones actuales para la próxima iteración
        self.prev_positions[motor_1] = q1
        self.prev_positions[motor_2] = q2
        
        # Control de corriente más directo: ambos motores siempre siguen al otro
        # Esto evita la detección de "líder/seguidor" que puede causar retardos
        
        # Calcular error de posición entre motores (espejo)
        error_1_to_2 = q1 - (-q2)  # Motor 1 debe seguir al motor 2 en espejo
        error_2_to_1 = q2 - (-q1)  # Motor 2 debe seguir al motor 1 en espejo
        
        # Calcular corrientes con control proporcional-derivativo
        iq1 = -self.kp * error_1_to_2 - self.kd * dq1
        iq2 = -self.kp * error_2_to_1 - self.kd * dq2
        
        # Limitar corrientes
        iq1 = max(min(iq1, self.max_current), -self.max_current)
        iq2 = max(min(iq2, self.max_current), -self.max_current)
        
        # Si se detectó un movimiento específico, dar prioridad a ese motor
        if moved_motor == motor_1:
            iq1 = 0.0  # No aplicar corriente al motor que se mueve manualmente
            self.node.get_logger().debug(f"Motor {motor_1} en movimiento, iq={iq1:.3f}, iq2={iq2:.3f}")
        elif moved_motor == motor_2:
            iq2 = 0.0  # No aplicar corriente al motor que se mueve manualmente
            self.node.get_logger().debug(f"Motor {motor_2} en movimiento, iq={iq1:.3f}, iq2={iq2:.3f}")
        
        # Enviar corrientes a los motores
        req = SetGoalIq.Request()
        req.motor_ids = [motor_1, motor_2]
        req.goal_iq = [iq1, iq2]
        
        future = self.set_goal_iq_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.1)

    def _update_full_teleoperation(self):
        """Implementa teleoperación completa con 8 motores (ambos brazos)"""
        # Obtener referencias a los motores
        m_id_1 = self.motor_ids[0]  # Brazo derecho, motor 1
        m_id_2 = self.motor_ids[1]  # Brazo derecho, motor 2
        m_id_3 = self.motor_ids[2]  # Brazo derecho, motor 3
        m_id_4 = self.motor_ids[3]  # Brazo derecho, motor 4
        m_id_5 = self.motor_ids[4]  # Brazo izquierdo, motor 1
        m_id_6 = self.motor_ids[5]  # Brazo izquierdo, motor 2
        m_id_7 = self.motor_ids[6]  # Brazo izquierdo, motor 3
        m_id_8 = self.motor_ids[7]  # Brazo izquierdo, motor 4
        
        # Obtener posiciones actuales
        q_r = np.array([
            self.get_motor_position(m_id_1),
            self.get_motor_position(m_id_2),
            self.get_motor_position(m_id_3),
            self.get_motor_position(m_id_4)
        ])
        
        q_l = np.array([
            self.get_motor_position(m_id_5),
            self.get_motor_position(m_id_6),
            self.get_motor_position(m_id_7),
            self.get_motor_position(m_id_8)
        ])
        
        # Obtener velocidades
        dq_r = np.array([
            self.get_motor_velocity(m_id_1),
            self.get_motor_velocity(m_id_2),
            self.get_motor_velocity(m_id_3),
            self.get_motor_velocity(m_id_4)
        ])
        
        dq_l = np.array([
            self.get_motor_velocity(m_id_5),
            self.get_motor_velocity(m_id_6),
            self.get_motor_velocity(m_id_7),
            self.get_motor_velocity(m_id_8)
        ])
        
        # Calcular vectores de gravedad
        G_L = self.left_gravity_vector(q_l)
        G_R = self.right_gravity_vector(q_r)
        
        # Detección de movimiento - cuál brazo se está moviendo
        if not hasattr(self, 'prev_q_r') or not hasattr(self, 'prev_q_l'):
            self.prev_q_r = q_r.copy()
            self.prev_q_l = q_l.copy()
            right_motion = np.zeros(4)
            left_motion = np.zeros(4)
        else:
            right_motion = np.abs(q_r - self.prev_q_r)
            left_motion = np.abs(q_l - self.prev_q_l)
            self.prev_q_r = q_r.copy()
            self.prev_q_l = q_l.copy()
        
        # Determinar cuál brazo se está moviendo más
        right_movement = np.sum(right_motion)
        left_movement = np.sum(left_motion)
        
        # Calcular corrientes para control de motores (espejo entre brazos)
        # Aplicar control adaptativo basado en detección de movimiento
        
        # Inicializar corrientes
        iq_values = np.zeros(8)
        
        # Si el brazo derecho se mueve más que el izquierdo
        if right_movement > left_movement and right_movement > self.movement_threshold * 4:
            # El brazo derecho es el líder, el izquierdo sigue
            self.node.get_logger().debug(f"Brazo DERECHO en movimiento: {right_motion}")
            
            # No aplicar corriente al brazo derecho (dejarlo moverse libremente)
            iq_values[0:4] = np.zeros(4)
            
            # El brazo izquierdo sigue al derecho con signo invertido
            for i in range(4):
                iq_values[i+4] = (-self.kp * (q_l[i] - (-1.0 * q_r[i])) - self.kd * dq_l[i] + G_L[i]) / self.Kt
        
        # Si el brazo izquierdo se mueve más que el derecho
        elif left_movement > right_movement and left_movement > self.movement_threshold * 4:
            # El brazo izquierdo es el líder, el derecho sigue
            self.node.get_logger().debug(f"Brazo IZQUIERDO en movimiento: {left_motion}")
            
            # El brazo derecho sigue al izquierdo con signo invertido
            for i in range(4):
                iq_values[i] = (-self.kp * (q_r[i] - (-1.0 * q_l[i])) - self.kd * dq_r[i] + G_R[i]) / self.Kt
                
            # No aplicar corriente al brazo izquierdo (dejarlo moverse libremente)
            iq_values[4:8] = np.zeros(4)
        
        # Si ambos brazos se mueven poco o de forma similar
        else:
            # Aplicar un control suave a ambos brazos para mantener simetría
            # Brazo derecho
            for i in range(4):
                iq_values[i] = (-0.5 * self.kp * (q_r[i] - (-1.0 * q_l[i])) - 0.5 * self.kd * dq_r[i] + G_R[i]) / self.Kt
            
            # Brazo izquierdo
            for i in range(4):
                iq_values[i+4] = (-0.5 * self.kp * (q_l[i] - (-1.0 * q_r[i])) - 0.5 * self.kd * dq_l[i] + G_L[i]) / self.Kt
        
        # Limitar corrientes para seguridad
        limited_iq = [max(min(iq, self.max_current), -self.max_current) for iq in iq_values]
        
        # Enviar corrientes a los motores
        req = SetGoalIq.Request()
        req.motor_ids = self.motor_ids[:8]
        req.goal_iq = limited_iq
        
        future = self.set_goal_iq_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.1)

    def restore_position_control(self):
        """Restaura el control de posición de los motores"""
        self.node.get_logger().info("Restaurando control de posición de los motores")
        
        try:
            # Usar valores de Position_Control_Mode_Table_Gains
            # Asegurarnos de que todos los valores sean float
            req_gains = SetGains.Request()
            req_gains.motor_ids = self.motor_ids
            req_gains.p_gain_position = float(Position_Control_Mode_Table_Gains.p_gain_position)
            req_gains.i_gain_position = float(Position_Control_Mode_Table_Gains.i_gain_position)
            req_gains.d_gain_position = float(Position_Control_Mode_Table_Gains.d_gain_position)
            req_gains.p_gain_iq = float(Position_Control_Mode_Table_Gains.p_gain_iq)
            req_gains.i_gain_iq = float(Position_Control_Mode_Table_Gains.i_gain_iq)
            req_gains.d_gain_iq = float(Position_Control_Mode_Table_Gains.d_gain_iq)
            req_gains.p_gain_id = float(Position_Control_Mode_Table_Gains.p_gain_id)
            req_gains.i_gain_id = float(Position_Control_Mode_Table_Gains.i_gain_id)
            req_gains.d_gain_id = float(Position_Control_Mode_Table_Gains.d_gain_id)
            req_gains.iq_max = float(Position_Control_Mode_Table_Gains.iq_max)
            req_gains.kt = float(Position_Control_Mode_Table_Gains.kt)
            
            future = self.set_gains_client.call_async(req_gains)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            # Configurar modo posición (modo 2)
            req_mode = SetMode.Request()
            req_mode.motor_ids = self.motor_ids
            req_mode.modes = [2] * len(self.motor_ids)
            
            future = self.set_mode_client.call_async(req_mode)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            time.sleep(2.0)
            
            # Enviar a posición home (1.5707 rad ≈ 90 grados)
            home_req = SetMotorIdAndTarget.Request()
            home_req.motor_ids = self.motor_ids
            home_req.target_positions = [1.5707] * len(self.motor_ids)
            
            future = self.set_position_client.call_async(home_req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al restaurar control de posición: {str(e)}")
            return False

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.node.get_logger().info(f"Terminando teleoperación local con estado {new_status}")
        self.running = False
        
        # Restaurar control de posición antes de terminar
        self.restore_position_control()
        
        # Solo destruir el nodo si lo creamos nosotros mismos
        if self.own_node and self.node:
            self.node.destroy_node() 