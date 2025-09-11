import py_trees
import rclpy
import time
import sys
import select
import socket
import struct
import threading
import numpy as np
from westwood_motor_interfaces.srv import (
    SetMotorIdAndTarget, GetMotorPositions, GetMotorVelocities,
    SetMode, SetTorqueEnable, SetGoalIq
)


class RemoteTeleoperation(py_trees.behaviour.Behaviour):
    """
    Comportamiento de teleoperación remota que permite que motores en diferentes 
    computadoras se repliquen mutuamente a través de comunicación UDP.
    
    Basado en el código original de teleoperación remota de main.py
    """
    def __init__(self, name: str, motor_ids=None, node=None):
        super().__init__(name)
        self.node = node
        self.running = False
        self.own_node = False
        
        # Configuración desde parámetros ROS2 (se carga en setup)
        self.motor_ids = None
        self.local_ip = None
        self.remote_ip = None  
        self.send_port = None
        self.receive_port = None
        self.socket_timeout = 0.001
        self.max_communication_errors = 10
        self.control_frequency = 1000
        
        
        # Sockets UDP
        self.send_socket = None
        self.receive_socket = None
        
        # Datos de comunicación
        self.received_data = []
        self.data_lock = threading.Lock()
        
        # Clientes para servicios ROS2
        self.set_position_client = None
        self.get_position_client = None
        self.get_velocity_client = None
        self.set_mode_client = None
        self.set_torque_client = None
        self.set_iq_client = None
        
        # Variables para control (se inicializan después de cargar parámetros)
        self.current_positions = []
        self.current_velocities = []
        self.target_positions = []
        
        # Variables para manejo de errores
        self.communication_error_count = 0
        
        # Parámetros para control de transparencia (TL)
        self.r1 = 0.5
        self.r2 = 0.4
        # Verificar constraint: 2r2 > r1 > r2 > 0
        assert 2*self.r2 > self.r1 > self.r2 > 0, f"Constraint violated: 2*{self.r2} > {self.r1} > {self.r2} > 0"
        self.p1 = (2*self.r2 - self.r1) / self.r1  # ((2r2-r1)/r1)
        self.p2 = (2*self.r2 - self.r1) / self.r1  # ((2r2-r1)/r1)

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        """Configurar el comportamiento"""
        if self.node is None:
            self.node = rclpy.create_node('remote_teleoperation_client')
            self.own_node = True
        else:
            self.own_node = False
        
        # Declarar y cargar parámetros desde ROS2
        try:
            # Declarar parámetros de teleoperación remota
            self.node.declare_parameter('remote_teleoperation.motor_ids', [1])
            self.node.declare_parameter('remote_teleoperation.is_machine_a', True)
            self.node.declare_parameter('remote_teleoperation.machine_a_ip', '192.168.4.241')
            self.node.declare_parameter('remote_teleoperation.machine_b_ip', '192.168.4.238')
            self.node.declare_parameter('remote_teleoperation.socket_timeout', 0.001)
            self.node.declare_parameter('remote_teleoperation.max_communication_errors', 10)
            self.node.declare_parameter('remote_teleoperation.control_frequency', 1000)
            self.node.declare_parameter('remote_teleoperation.control_gains.kp', 1.75)
            self.node.declare_parameter('remote_teleoperation.control_gains.kd', 0.1)
            self.node.declare_parameter('remote_teleoperation.control_gains.kt', 0.35)
            
            
            # Cargar valores de parámetros
            self.motor_ids = self.node.get_parameter('remote_teleoperation.motor_ids').value
            self.is_machine_a = self.node.get_parameter('remote_teleoperation.is_machine_a').value
            is_machine_a = self.is_machine_a
            machine_a_ip = self.node.get_parameter('remote_teleoperation.machine_a_ip').value
            machine_b_ip = self.node.get_parameter('remote_teleoperation.machine_b_ip').value
            self.socket_timeout = self.node.get_parameter('remote_teleoperation.socket_timeout').value
            self.max_communication_errors = self.node.get_parameter('remote_teleoperation.max_communication_errors').value
            self.control_frequency = self.node.get_parameter('remote_teleoperation.control_frequency').value
            self.kp = self.node.get_parameter('remote_teleoperation.control_gains.kp').value
            self.kd = self.node.get_parameter('remote_teleoperation.control_gains.kd').value
            self.kt = self.node.get_parameter('remote_teleoperation.control_gains.kt').value
            
            
            
            # Configurar IPs y puertos basado en qué máquina somos
            if is_machine_a:
                self.local_ip = machine_a_ip
                self.remote_ip = machine_b_ip
                self.send_port = 4000      # Máquina A envía al puerto 4000
                self.receive_port = 5001   # Máquina A recibe en puerto 5001
            else:
                self.local_ip = machine_b_ip
                self.remote_ip = machine_a_ip
                self.send_port = 5001      # Máquina B envía al puerto 5001
                self.receive_port = 4000   # Máquina B recibe en puerto 4000
            
            # Inicializar variables de control
            self.current_positions = [0.0] * len(self.motor_ids)
            self.current_velocities = [0.0] * len(self.motor_ids)
            self.target_positions = [0.0] * len(self.motor_ids)
            
            pass
            
        except Exception as e:
            self.node.get_logger().error(f"Error cargando parámetros: {str(e)}")
            return False
        
        # Crear clientes para servicios
        self.set_position_client = self.node.create_client(
            SetMotorIdAndTarget, 'westwood_motor/set_motor_id_and_target')
        self.get_position_client = self.node.create_client(
            GetMotorPositions, 'westwood_motor/get_motor_positions')
        self.get_velocity_client = self.node.create_client(
            GetMotorVelocities, 'westwood_motor/get_motor_velocities')
        self.set_mode_client = self.node.create_client(
            SetMode, 'westwood_motor/set_mode')
        self.set_torque_client = self.node.create_client(
            SetTorqueEnable, 'westwood_motor/set_torque_enable')
        self.set_iq_client = self.node.create_client(
            SetGoalIq, 'westwood_motor/set_goal_iq')
        
        if timeout_sec is None:
            timeout_sec = 1.0
        
        # Esperar por servicios
        services_ready = True
        for client, name in [
            (self.set_position_client, 'set_motor_id_and_target'),
            (self.get_position_client, 'get_motor_positions'),
            (self.get_velocity_client, 'get_motor_velocities'),
            (self.set_iq_client, 'set_goal_iq')
        ]:
            if not client.wait_for_service(timeout_sec=timeout_sec):
                pass
                services_ready = False
        
        return services_ready

    def setup_udp_communication(self):
        """Configurar sockets UDP para comunicación remota"""
        try:
            # Socket para enviar datos (no necesita bind específico)
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_socket.settimeout(self.socket_timeout)
            
            # Socket para recibir datos (bind en IP local en puerto de recepción)
            self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.receive_socket.bind((self.local_ip, self.receive_port))
            self.receive_socket.settimeout(self.socket_timeout)
            
            pass
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error configurando UDP: {str(e)}")
            return False

    def zero_position(self):
        """Envía todos los motores a posición cero"""
        pass
        
        req = SetMotorIdAndTarget.Request()
        req.motor_ids = self.motor_ids
        req.target_positions = [0.0] * len(self.motor_ids)
        
        try:
            future = self.set_position_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.done():
                result = future.result()
                if result.success:
                    time.sleep(2.0)  # Esperar a que lleguen a posición
                    return True
                else:
                    pass
            return False
        except Exception as e:
            self.node.get_logger().error(f"Error en zero_position: {str(e)}")
            return False

    def setup_current_control(self):
        """Configura los motores para control de corriente usando SetMotorIdAndTargetCurrent"""
        pass
        
        try:
            # Las ganancias PID se configuran automáticamente al usar SetMotorIdAndTargetCurrent
            # Solo configurar modo corriente (modo 0) y habilitar torque
            req_mode = SetMode.Request()
            req_mode.motor_ids = self.motor_ids
            req_mode.modes = [0] * len(self.motor_ids)  # Modo corriente
            
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
            self.node.get_logger().error(f"Error configurando control de corriente: {str(e)}")
            return False

    def get_motor_states(self):
        """Obtiene posiciones y velocidades actuales de todos los motores"""
        try:
            # Obtener posiciones
            req_pos = GetMotorPositions.Request()
            req_pos.motor_ids = self.motor_ids
            
            future_pos = self.get_position_client.call_async(req_pos)
            rclpy.spin_until_future_complete(self.node, future_pos, timeout_sec=0.1)
            
            if future_pos.done():
                result_pos = future_pos.result()
                if result_pos.success:
                    old_pos = self.current_positions[0] if len(self.current_positions) > 0 else 0.0
                    self.current_positions = list(result_pos.positions)
                    
                    # Debug si cambió la posición
                    if abs(self.current_positions[0] - old_pos) > 0.01:
                        pass
            
            # Obtener velocidades
            req_vel = GetMotorVelocities.Request()
            req_vel.motor_ids = self.motor_ids
            
            future_vel = self.get_velocity_client.call_async(req_vel)
            rclpy.spin_until_future_complete(self.node, future_vel, timeout_sec=0.1)
            
            if future_vel.done():
                result_vel = future_vel.result()
                if result_vel.success:
                    self.current_velocities = list(result_vel.velocities)
            
            return True
        except Exception as e:
            self.node.get_logger().debug(f"Error obteniendo estados: {str(e)}")
            return False

    def send_positions(self):
        """Envía posiciones actuales vía UDP"""
        if self.send_socket is None:
            return
        
        try:
            # Usar las posiciones actuales tal como están
            pos_to_send = self.current_positions.copy()
            
            # Crear formato dinámico basado en número de motores
            num_motors = len(self.motor_ids)
            format_str = f'{num_motors}f'
            
            # Empaquetar solo los datos necesarios
            struct_data = struct.pack(format_str, *pos_to_send[:num_motors])
            # Enviar al puerto correcto de la máquina remota
            self.send_socket.sendto(struct_data, (self.remote_ip, self.send_port))
            
            # Debug cada 10 envíos
            if not hasattr(self, '_send_count'):
                self._send_count = 0
            self._send_count += 1
            
            if self._send_count % 1 == 0:
                pos_str = ", ".join([f"{pos:.3f}" for pos in pos_to_send[:num_motors]])
                self.node.get_logger().info(f"SEND: [{pos_str}]")
            
        except Exception as e:
            self.node.get_logger().warning(f"Error enviando posiciones: {str(e)}")

    def receive_positions(self):
        """Recibe posiciones remotas vía UDP"""
        if self.receive_socket is None:
            return
        
        try:
            data, addr = self.receive_socket.recvfrom(1024)
            
            # Crear formato dinámico basado en número de motores
            num_motors = len(self.motor_ids)
            format_str = f'<{num_motors}f'
            
            struct_data = struct.unpack(format_str, data)
            
            with self.data_lock:
                self.received_data.append(struct_data)
            
            # Debug cada 10 recepciones
            if not hasattr(self, '_receive_count'):
                self._receive_count = 0
            self._receive_count += 1
            
            if self._receive_count % 1 == 0:
                pos_str = ", ".join([f"{pos:.3f}" for pos in struct_data[:num_motors]])
                self.node.get_logger().info(f"RECV: [{pos_str}]")
                
        except socket.timeout:
            pass  # Timeout normal
        except Exception as e:
            self.node.get_logger().warning(f"Error recibiendo posiciones: {str(e)}")

    def update_target_positions(self):
        """Actualiza posiciones objetivo desde datos recibidos"""
        with self.data_lock:
            if not self.received_data:
                return False
            
            # Procesar el último dato recibido
            for entry in self.received_data:
                # Actualizar posiciones objetivo para todos los motores disponibles
                for i in range(min(len(entry), len(self.motor_ids))):
                    self.target_positions[i] = entry[i]
            
            # Limpiar datos procesados
            self.received_data.clear()
            return True

    def calculate_control_currents(self):
        """Calcula corrientes de control basado en error de posición, velocidad y compensación de gravedad"""
        currents = []
        
        # Verificar que tenemos al menos 8 motores (4 right + 4 left)
        if len(self.motor_ids) != 8:
            # Fallback al control simple para casos con menos motores con TL
            for i in range(len(self.motor_ids)):
                if i < len(self.current_positions) and i < len(self.target_positions):
                    # Obtener velocidad del motor (0.0 si no está disponible)
                    motor_velocity = self.current_velocities[i] if i < len(self.current_velocities) else 0.0
                    
                    # Calcular TL para este motor
                    TL = self.calculate_transparency_level(
                        self.current_positions[i], 
                        self.target_positions[i], 
                        motor_velocity,
                        self.is_machine_a
                    )
                    
                    # Aplicar TL dividido entre Kt
                    iq = TL / self.kt
                    currents.append(iq)
                else:
                    currents.append(0.0)
            return currents
        
        # Control avanzado para 8 motores con compensación de gravedad
        try:
            # Separar posiciones actuales en derecho e izquierdo
            qr1, qr2, qr3, qr4 = self.current_positions[:4]  # Right arm
            ql1, ql2, ql3, ql4 = self.current_positions[4:]  # Left arm
            
            # Separar velocidades actuales
            dqr1, dqr2, dqr3, dqr4 = self.current_velocities[:4] if len(self.current_velocities) >= 4 else [0.0, 0.0, 0.0, 0.0]
            dql1, dql2, dql3, dql4 = self.current_velocities[4:] if len(self.current_velocities) >= 8 else [0.0, 0.0, 0.0, 0.0]
            
            # Posiciones objetivo (q_r y q_l del código original)
            q_r1, q_r2, q_r3, q_r4 = self.target_positions[:4]
            q_l1, q_l2, q_l3, q_l4 = self.target_positions[4:]
            
            # Crear vectores para gravedad (simulando el cálculo original)
            # Nota: Necesitarás implementar right_gravity_vector y left_gravity_vector
            # Por ahora uso un placeholder
            g_qr = [q_l1, q_l2, q_l3, q_l4]  # Del código original: g_qr=q_l[:4]
            g_ql = [self.target_positions[4], self.target_positions[5], self.target_positions[6], -1.0 * self.target_positions[7]]  # g_ql[3]=-1.0*g_ql[3]
            
            # Vectores de gravedad
            GR = self.right_gravity_vector(g_qr)
            GL = self.left_gravity_vector(g_ql)
            
            # Ganancias (pueden ser arrays para cada articulación)
            kp = [self.kp] * 4 if not hasattr(self, 'kp_array') else self.kp_array
            kd = [self.kd] * 4 if not hasattr(self, 'kd_array') else self.kd_array
            Kt = self.kt
            
            # Calcular TL para cada motor y aplicarlo junto con compensación de gravedad
            TL_1 = self.calculate_transparency_level(qr1, q_r1, dqr1, self.is_machine_a)
            TL_2 = self.calculate_transparency_level(qr2, q_r2, dqr2, self.is_machine_a)
            TL_3 = self.calculate_transparency_level(qr3, q_r3, dqr3, self.is_machine_a)
            TL_4 = self.calculate_transparency_level(qr4, q_r4, dqr4, self.is_machine_a)
            
            TL_5 = self.calculate_transparency_level(ql1, q_l1, dql1, self.is_machine_a)
            TL_6 = self.calculate_transparency_level(ql2, q_l2, dql2, self.is_machine_a)
            TL_7 = self.calculate_transparency_level(ql3, q_l3, dql3, self.is_machine_a)
            TL_8 = self.calculate_transparency_level(ql4, q_l4, dql4, self.is_machine_a)
            
            # Calcular corrientes combinando TL con compensación de gravedad
            i_g_1 = (TL_1) / Kt
            i_g_2 = (TL_2) / Kt
            i_g_3 = TL_3 / Kt  # Sin compensación de gravedad
            i_g_4 = TL_4 / Kt  # Sin compensación de gravedad
            
            i_g_5 = (TL_5) / Kt
            i_g_6 = (TL_6) / Kt
            i_g_7 = TL_7 / Kt  # Sin compensación de gravedad
            i_g_8 = TL_8 / Kt  # Sin compensación de gravedad
            
            currents = [i_g_1, i_g_2, i_g_3, i_g_4, i_g_5, i_g_6, i_g_7, i_g_8]
            
            # Debug logging cada 100 cálculos
            if not hasattr(self, '_calc_debug_count'):
                self._calc_debug_count = 0
            self._calc_debug_count += 1
            
            if self._calc_debug_count % 100 == 0:
                self.node.get_logger().info(f"Corrientes calculadas: R[{i_g_1:.3f}, {i_g_2:.3f}, {i_g_3:.3f}, {i_g_4:.3f}] L[{i_g_5:.3f}, {i_g_6:.3f}, {i_g_7:.3f}, {i_g_8:.3f}]")
            
            return currents
            
        except Exception as e:
            self.node.get_logger().error(f"Error calculando corrientes avanzadas: {str(e)}")
            # Fallback al control simple con TL
            simple_currents = []
            for i in range(len(self.motor_ids)):
                if i < len(self.current_positions) and i < len(self.target_positions):
                    # Obtener velocidad del motor (0.0 si no está disponible)
                    motor_velocity = self.current_velocities[i] if i < len(self.current_velocities) else 0.0
                    
                    # Calcular TL para este motor
                    TL = self.calculate_transparency_level(
                        self.current_positions[i], 
                        self.target_positions[i], 
                        motor_velocity,
                        self.is_machine_a
                    )
                    
                    # Aplicar TL dividido entre Kt
                    iq = TL / self.kt
                    simple_currents.append(iq)
                else:
                    simple_currents.append(0.0)
            return simple_currents

    def right_gravity_vector(self, q):
        """Calcula el vector de compensación de gravedad para el brazo derecho"""
        # Implementación simplificada de compensación de gravedad
        # Basada en el modelo dinámico del brazo robótico
        # q = [q1, q2, q3, q4] - ángulos de las articulaciones
        
        import math
        
        # Parámetros del brazo (valores típicos para un brazo robótico)
        m1, m2 = 2.0, 1.5  # masas de los eslabones (kg)
        l1, l2 = 0.3, 0.25  # longitudes de los eslabones (m)
        lc1, lc2 = 0.15, 0.125  # centros de masa (m)
        g = 9.81  # gravedad (m/s²)
        
        q1, q2, q3, q4 = q
        
        # Compensación de gravedad para las primeras dos articulaciones
        # Solo las primeras dos articulaciones necesitan compensación (como se ve en el código original)
        tau1 = (m1 * lc1 + m2 * l1) * g * math.cos(q1) + m2 * lc2 * g * math.cos(q1 + q2)
        tau2 = m2 * lc2 * g * math.cos(q1 + q2)
        
        # Las articulaciones 3 y 4 no tienen compensación de gravedad en el código original
        tau3 = 0.0
        tau4 = 0.0
        
        return [tau1, tau2, tau3, tau4]
    
    def left_gravity_vector(self, q):
        """Calcula el vector de compensación de gravedad para el brazo izquierdo"""
        # Implementación similar al brazo derecho pero con orientación espejada
        import math
        
        # Parámetros del brazo (iguales al derecho)
        m1, m2 = 2.0, 1.5  # masas de los eslabones (kg)
        l1, l2 = 0.3, 0.25  # longitudes de los eslabones (m)
        lc1, lc2 = 0.15, 0.125  # centros de masa (m)
        g = 9.81  # gravedad (m/s²)
        
        q1, q2, q3, q4 = q
        
        # Compensación de gravedad para las primeras dos articulaciones
        # El brazo izquierdo puede tener orientación espejada
        tau1 = (m1 * lc1 + m2 * l1) * g * math.cos(q1) + m2 * lc2 * g * math.cos(q1 + q2)
        tau2 = m2 * lc2 * g * math.cos(q1 + q2)
        
        # Las articulaciones 3 y 4 no tienen compensación de gravedad
        tau3 = 0.0
        tau4 = 0.0
        
        return [tau1, tau2, tau3, tau4]

    def calculate_transparency_level(self, local_motor_position, remote_motor_position, local_motor_velocity, is_machine_a=True):
        """
        Calcula el nivel de transparencia (TL) basado en error local y velocidad del motor
        
        Args:
            local_motor_position: Posición del motor local
            remote_motor_position: Posición del motor remoto  
            local_motor_velocity: Velocidad del motor local
            is_machine_a: True si es máquina A, False si es máquina B
            
        Returns:
            TL: Nivel de transparencia calculado
        """
        import math
        
        # Calcular error local
        if is_machine_a:
            # Máquina A: error_local = posición_motor_local - posición_motor_remoto
            error_local = local_motor_position - remote_motor_position
        else:
            # Máquina B: error_local = posición_motor_remoto - posición_motor_local
            error_local = remote_motor_position - local_motor_position
        
        # Calcular TL según la fórmula:
        # TL = -kp * |error_local|^p1 * sign(error_local) - kd * |motor_velocity|^p2 * sign(motor_velocity)
        
        # Primer término: -kp * |error_local|^p1 * sign(error_local)
        error_magnitude = abs(error_local)
        error_sign = math.copysign(1, error_local) if error_local != 0 else 0
        first_term = -self.kp * (error_magnitude ** 0.75) * error_sign
        
        # Segundo término: -kd * |motor_velocity|^p2 * sign(motor_velocity)  
        velocity_magnitude = abs(local_motor_velocity)
        velocity_sign = math.copysign(1, local_motor_velocity) if local_motor_velocity != 0 else 0
        second_term = -self.kd * (velocity_magnitude ** 1.0) * velocity_sign
        
        # TL total
        TL = first_term + second_term
        
        return TL

    def send_current_commands(self, currents):
        """Envía comandos de corriente a los motores"""
        try:
            req = SetGoalIq.Request()
            req.motor_ids = self.motor_ids
            req.goal_iq = currents
            
            # Debug de corrientes cada 100 comandos
            if not hasattr(self, '_current_send_count'):
                self._current_send_count = 0
            self._current_send_count += 1
            
            pass
            
            future = self.set_iq_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.05)
            
            if future.done():
                result = future.result()
                pass
                return result.success
            
            return False
        except Exception as e:
            self.node.get_logger().warning(f"Error enviando corrientes: {str(e)}")
            return False

    def initialise(self) -> None:
        """Inicializar teleoperación remota"""
        pass
        
        self.running = True
        
        # Configuración inicial
        pass
        if not self.zero_position():
            pass
        else:
            pass
        
        pass
        time.sleep(2)
        
        pass
        if not self.setup_current_control():
            pass
            self.running = False
            return
        else:
            pass
        
        pass
        if not self.setup_udp_communication():
            pass
            self.running = False
            return
        else:
            pass
            
        pass

    def update(self) -> py_trees.common.Status:
        """Bucle principal de teleoperación remota"""
        if not self.running:
            return py_trees.common.Status.SUCCESS
        
        # Verificar errores de comunicación
        if self.communication_error_count >= self.max_communication_errors:
            pass
            self.running = False
            return py_trees.common.Status.FAILURE
        
        # Verificar entrada del usuario
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            pass
            self.running = False
            return py_trees.common.Status.SUCCESS
        
        try:
            # Obtener estados actuales
            if not self.get_motor_states():
                self.communication_error_count += 1
                if hasattr(self, '_error_counter'):
                    self._error_counter += 1
                else:
                    self._error_counter = 1
                    
                if self._error_counter % 100 == 0:
                    pass
                return py_trees.common.Status.RUNNING
            
            # Comunicación UDP en hilos separados
            send_thread = threading.Thread(target=self.send_positions, daemon=True)
            receive_thread = threading.Thread(target=self.receive_positions, daemon=True)
            
            send_thread.start()
            receive_thread.start()
            
            send_thread.join()
            receive_thread.join()
            
            # Actualizar objetivos desde datos recibidos
            if self.update_target_positions():
                # Calcular corrientes de control
                currents = self.calculate_control_currents()
                
                # Enviar comandos
                if self.send_current_commands(currents):
                    self.communication_error_count = max(0, self.communication_error_count - 1)
                else:
                    self.communication_error_count += 1
            
            # Debug cada 50 iteraciones
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 0
            
            pass
            
        except Exception as e:
            self.node.get_logger().error(f"Error en teleoperación remota: {str(e)}")
            self.communication_error_count += 1
        
        time.sleep(1.0 / self.control_frequency)  # Control de alta frecuencia configurable
        return py_trees.common.Status.RUNNING

    def restore_position_control(self):
        """Restaura control de posición antes de terminar"""
        pass
        
        try:
            # Cambiar a modo posición
            req_mode = SetMode.Request()
            req_mode.motor_ids = self.motor_ids
            req_mode.modes = [2] * len(self.motor_ids)
            
            future = self.set_mode_client.call_async(req_mode)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            time.sleep(1.0)
            
            # Ir a posición home
            req_home = SetMotorIdAndTarget.Request()
            req_home.motor_ids = self.motor_ids
            req_home.target_positions = [1.5707] * len(self.motor_ids)  # 90 grados
            
            future = self.set_position_client.call_async(req_home)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
        except Exception as e:
            self.node.get_logger().error(f"Error restaurando control: {str(e)}")

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Terminar teleoperación remota"""
        pass
        self.running = False
        
        # Cerrar sockets UDP
        try:
            if self.send_socket:
                self.send_socket.close()
            if self.receive_socket:
                self.receive_socket.close()
        except:
            pass
        
        # Restaurar control de posición
        self.restore_position_control()
        
        # Solo destruir nodo si lo creamos nosotros
        if self.own_node and self.node:
            self.node.destroy_node()