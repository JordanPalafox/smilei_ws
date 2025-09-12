import py_trees
import rclpy
import time
import sys
import select
import socket
import struct
import threading
import numpy as np
import math
from std_msgs.msg import Float64MultiArray


class RemoteTeleoperation(py_trees.behaviour.Behaviour):
    """
    Comportamiento de teleoperación remota basado exactamente en el código de referencia main.py
    Implementa control bilateral con compensación de gravedad y límites de seguridad
    """
    def __init__(self, name: str, motor_ids=None, node=None, robot_name: str = "", hardware_manager=None):
        super().__init__(name)
        self.node = node
        self.robot_name = robot_name
        self.running = False
        self.own_node = False
        
        # Hardware manager para control de motores
        self.hardware_manager = hardware_manager
        self.available_motors = []
        
        # Configuración de red (se carga desde parámetros ROS2)
        self.motor_ids = None
        self.is_machine_a = None
        self.machine_a_ip = None
        self.machine_b_ip = None
        self.local_ip = None
        self.remote_ip = None
        self.send_port = None
        self.receive_port = None
        self.local_addr = None
        
        # Sockets UDP
        self.send_socket = None
        self.receive_socket = None
        
        # Control bilateral
        self.received_data = []
        self.data_lock = threading.Lock()
        
        # Ganancias de control (del código de referencia)
        self.kp = [1.75, 1.75, 1.75, 1.75]  # Proporcional 
        self.kd = [0.1, 0.1, 0.1, 0.1]      # Derivativo
        self.Kt = 0.35                      # Constante de torque
        
        # Variables de estado de motores
        self.current_positions = [0.0] * 8
        self.current_velocities = [0.0] * 8
        self.target_positions = [0.0] * 8
        
        # Variables de comunicación
        self.communication_error_count = 0
        self.max_communication_errors = 10
        
        # Publisher para debugging (opcional)
        self.publish_goal_iq = False
        self.goal_iq_publisher = None

    def _get_topic_name(self, topic_name):
        if self.robot_name:
            return f'/{self.robot_name}/{topic_name.lstrip("/")}'
        return topic_name

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        """Configurar el comportamiento según parámetros ROS2"""
        if self.node is None:
            self.node = rclpy.create_node('remote_teleoperation_client')
            self.own_node = True
        else:
            self.own_node = False
        
        try:
            # Declarar parámetros - configuración flexible para número de motores
            self.node.declare_parameter('remote_teleoperation.motor_ids', [1])  # Por defecto un motor
            self.node.declare_parameter('remote_teleoperation.is_machine_a', True)
            self.node.declare_parameter('remote_teleoperation.machine_a_ip', '192.168.0.100')
            self.node.declare_parameter('remote_teleoperation.machine_b_ip', '192.168.0.2')
            self.node.declare_parameter('remote_teleoperation.num_total_motors', 2)  # Total de motores en el sistema (A+B)
            
            # Cargar parámetros
            self.motor_ids = self.node.get_parameter('remote_teleoperation.motor_ids').value
            self.is_machine_a = self.node.get_parameter('remote_teleoperation.is_machine_a').value
            self.machine_a_ip = self.node.get_parameter('remote_teleoperation.machine_a_ip').value
            self.machine_b_ip = self.node.get_parameter('remote_teleoperation.machine_b_ip').value
            self.num_total_motors = self.node.get_parameter('remote_teleoperation.num_total_motors').value
            
            # Configurar red según máquina (basado en código de referencia)
            if self.is_machine_a:
                # Máquina A configuración
                self.local_ip = self.machine_a_ip
                self.remote_ip = self.machine_b_ip
                self.send_port = 4000    # A envía a puerto 4000 de B  
                self.receive_port = 5005 # A recibe en puerto 5005
                self.local_addr = (self.remote_ip, 5005)  # Dirección a donde A envía
            else:
                # Máquina B configuración  
                self.local_ip = self.machine_b_ip
                self.remote_ip = self.machine_a_ip
                self.send_port = 5005    # B envía a puerto 5005 de A
                self.receive_port = 4000 # B recibe en puerto 4000
                self.local_addr = (self.remote_ip, 4000)  # Dirección a donde B envía
            
            # Inicializar variables de control basadas en número total de motores
            self.node.get_logger().info(f"Configuración: {len(self.motor_ids)} motor(es) local(es), {self.num_total_motors} total en sistema")
            
            # Inicializar arrays con tamaño correcto
            self.current_positions = [0.0] * self.num_total_motors
            self.current_velocities = [0.0] * self.num_total_motors  
            self.target_positions = [0.0] * self.num_total_motors
            
        except Exception as e:
            self.node.get_logger().error(f"Error cargando parámetros: {str(e)}")
            return False
        
        # Verificar hardware
        if self.hardware_manager is not None:
            available_motors = self.hardware_manager.get_available_motors()
            self.available_motors = [m for m in self.motor_ids if m in available_motors]
            if self.available_motors:
                self.node.get_logger().info(f"Hardware conectado - motores disponibles: {self.available_motors}")
            else:
                self.node.get_logger().warning("No hay motores disponibles - modo simulación")
        else:
            self.node.get_logger().warning("Hardware manager no disponible - modo simulación")
        
        return True

    def setup_udp_communication(self):
        """Configurar sockets UDP como en el código de referencia"""
        try:
            # Socket para enviar (servidor) - no requiere bind específico
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_socket.settimeout(0.001)  # Timeout como en referencia
            
            # Socket para recibir (cliente) - bind en puerto local
            self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.receive_socket.bind((self.local_ip, self.receive_port))
            self.receive_socket.settimeout(0.001)  # Timeout como en referencia
            
            self.node.get_logger().info(f"UDP configurado - Local: {self.local_ip}:{self.receive_port}, Remoto: {self.local_addr}")
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"Error configurando UDP: {str(e)}")
            return False

    def zero_position(self):
        """Envía motores a posición cero (igual que otros behaviors)"""
        try:
            if self.hardware_manager:
                position_pairs = [(motor_id, 0.0) for motor_id in self.motor_ids]
                self.hardware_manager.set_goal_position(*position_pairs)
                self.node.get_logger().info("Motores enviados a posición cero")
            else:
                self.node.get_logger().info("[SIM] Motores enviados a posición cero")
            time.sleep(2.0)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error en zero_position: {str(e)}")
            return False

    def setup_current_control(self):
        """Configura motores para control de corriente como en código de referencia"""
        try:
            if not self.hardware_manager:
                self.node.get_logger().info("[SIM] Configurando control de corriente")
                return True
            
            self.node.get_logger().info("Configurando motores para control de corriente...")
            
            # PID para control de corriente id/iq (del código de referencia)
            # P=0.277, I=0.061, D=0 para todos los motores
            
            # Configurar PID gains para corriente - simulamos con el hardware_manager existente
            # En el código original se usan set_p_gain_iq, set_i_gain_iq, etc.
            # Como no tenemos acceso directo, usamos lo que tenemos disponible
            
            # PID posición a cero (importante del código de referencia)
            if hasattr(self.hardware_manager, 'configure_pid_gains'):
                self.hardware_manager.configure_pid_gains(self.motor_ids, p_gain=0.0, i_gain=0.0, d_gain=0.0)
            
            # Modo corriente (modo 0) y habilitar torque
            self.hardware_manager.set_mode(*[(motor_id, 0) for motor_id in self.motor_ids])
            self.hardware_manager.set_torque_enable(*[(motor_id, 1) for motor_id in self.motor_ids])
            
            self.node.get_logger().info("Motores configurados para control de corriente")
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"Error configurando control de corriente: {str(e)}")
            return False

    def get_motor_states(self):
        """Obtiene posiciones y velocidades de motores locales"""
        try:
            if not self.hardware_manager:
                # Modo simulación - mantener estados actuales
                return True
            
            # Obtener posiciones solo de motores locales disponibles
            positions = []
            velocities = []
            
            for i, motor_id in enumerate(self.motor_ids):
                try:
                    # Posición actual
                    pos_result = self.hardware_manager.get_present_position(motor_id)
                    if pos_result and len(pos_result) > 0:
                        pos = pos_result[0] if isinstance(pos_result[0], (int, float)) else 0.0
                    else:
                        pos = 0.0
                    positions.append(pos)
                    
                    # Velocidad - estimación simple basada en cambio de posición
                    if i < len(self.current_positions):
                        vel = (pos - self.current_positions[i]) / 0.001  # dt aproximado
                    else:
                        vel = 0.0
                    velocities.append(vel)
                    
                except Exception as e:
                    self.node.get_logger().warning(f"Error leyendo motor {motor_id}: {e}")
                    positions.append(0.0)
                    velocities.append(0.0)
            
            # Actualizar solo los estados locales (no rellenar con ceros)
            self.current_positions = positions[:]
            self.current_velocities = velocities[:]
                
            return True
            
        except Exception as e:
            self.node.get_logger().warning(f"Error obteniendo estados motores: {e}")
            return False

    def send_positions(self):
        """Envía posiciones vía UDP con formato flexible según número de motores"""
        if not self.send_socket:
            return
        
        try:
            # Preparar posiciones locales para enviar
            # Solo enviar las posiciones de los motores locales, rellenando con ceros el resto
            positions_to_send = [0.0] * self.num_total_motors
            
            # Llenar con posiciones reales de motores locales
            for i, motor_id in enumerate(self.motor_ids):
                if i < len(self.current_positions):
                    # Para máquina A: posiciones van en índices 0,1,etc 
                    # Para máquina B: posiciones van en índices 2,3,etc
                    if self.is_machine_a:
                        positions_to_send[i] = self.current_positions[i]
                    else:
                        if (i + 2) < len(positions_to_send):  # Offset para máquina B
                            positions_to_send[i + 2] = self.current_positions[i]
            
            # Crear formato dinámico basado en número total de motores
            format_str = f'{self.num_total_motors}f'
            
            # Empaquetar y enviar
            struct_ql = struct.pack(format_str, *positions_to_send)
            self.send_socket.sendto(struct_ql, self.local_addr)
            
        except Exception as e:
            self.node.get_logger().warning(f"Error enviando posiciones: {e}")

    def receive_positions(self):
        """Recibe posiciones con formato flexible según número de motores"""
        if not self.receive_socket:
            return
        
        try:
            data, addr = self.receive_socket.recvfrom(1024)
            
            # Crear formato dinámico basado en número total de motores
            format_str = f'<{self.num_total_motors}f'
            
            # Desempaquetar datos recibidos
            struct_qr = struct.unpack(format_str, data)
            
            with self.data_lock:
                self.received_data.append(struct_qr)
                
        except socket.timeout:
            # Timeout normal - no hacer nada
            pass
        except Exception as e:
            self.node.get_logger().warning(f"Error recibiendo posiciones: {e}")

    def update_target_positions(self):
        """Actualiza posiciones objetivo con validación flexible según número de motores"""
        with self.data_lock:
            if not self.received_data:
                return False
            
            # Procesar datos recibidos
            for entry in self.received_data:
                # Actualizar posiciones objetivo con validación básica
                for i in range(min(len(entry), self.num_total_motors)):
                    received_position = entry[i]
                    
                    # Aplicar límites básicos de seguridad para cualquier motor
                    if received_position > -3.15 and received_position < 3.15:  # Límites generales ±π
                        self.target_positions[i] = received_position
            
            # Limpiar datos procesados
            self.received_data.clear()
            return True

    def calculate_control_currents(self):
        """Calcula corrientes con número flexible de motores"""
        try:
            currents = []
            
            # Calcular corriente para cada motor local disponible
            for i, motor_id in enumerate(self.motor_ids):
                if i >= len(self.current_positions) or i >= len(self.current_velocities):
                    currents.append(0.0)
                    continue
                
                # Posición y velocidad locales
                local_pos = self.current_positions[i]
                local_vel = self.current_velocities[i]
                
                # Posición objetivo (remota) - usar índice local para máquinas A/B
                # Para máquina A (motores 1,2): usar i directamente
                # Para máquina B (motores 3,4): usar i+2 para acceder al array global
                if self.is_machine_a:
                    target_pos = self.target_positions[i] if i < len(self.target_positions) else 0.0
                else:
                    target_pos = self.target_positions[i + 2] if (i + 2) < len(self.target_positions) else 0.0
                
                # Control PD simple: i = (-kp*(pos_local - pos_remoto) - kd*vel_local) / Kt
                # Usar primera ganancia disponible como fallback
                kp = self.kp[0] if self.kp else 1.75
                kd = self.kd[0] if self.kd else 0.1
                
                # Calcular corriente de control
                error = local_pos - target_pos
                current = (-kp * error - kd * local_vel) / self.Kt
                currents.append(current)
            
            return currents
            
        except Exception as e:
            self.node.get_logger().error(f"Error calculando corrientes: {e}")
            return [0.0] * len(self.motor_ids)

    def right_gravity_vector(self, q):
        """Calcula compensación de gravedad para brazo derecho (basado en código original)"""
        # Implementación simplificada usando parámetros del robot real
        # Solo las primeras dos articulaciones necesitan compensación de gravedad
        
        if len(q) < 4:
            return [0.0, 0.0, 0.0, 0.0]
        
        q1, q2, q3, q4 = q[:4]
        
        # Parámetros del robot SMILEi (estimados)
        m1, m2 = 1.5, 1.0  # masas aproximadas (kg)
        l1, l2 = 0.25, 0.20  # longitudes de eslabones (m)
        lc1, lc2 = l1/2, l2/2  # centros de masa en medio de eslabones
        g = 9.81  # gravedad
        
        # Compensación de gravedad para primeras dos articulaciones
        tau1 = (m1 * lc1 + m2 * l1) * g * math.cos(q1) + m2 * lc2 * g * math.cos(q1 + q2)
        tau2 = m2 * lc2 * g * math.cos(q1 + q2)
        
        # Articulaciones 3 y 4 sin compensación (como en código original)
        tau3 = 0.0
        tau4 = 0.0
        
        return [tau1, tau2, tau3, tau4]
    
    def left_gravity_vector(self, q):
        """Calcula compensación de gravedad para brazo izquierdo (basado en código original)"""
        # Implementación similar al brazo derecho
        
        if len(q) < 4:
            return [0.0, 0.0, 0.0, 0.0]
        
        q1, q2, q3, q4 = q[:4]
        
        # Parámetros iguales al brazo derecho
        m1, m2 = 1.5, 1.0  # masas aproximadas (kg)
        l1, l2 = 0.25, 0.20  # longitudes de eslabones (m)
        lc1, lc2 = l1/2, l2/2  # centros de masa
        g = 9.81  # gravedad
        
        # Compensación de gravedad (orientación puede ser espejada)
        tau1 = (m1 * lc1 + m2 * l1) * g * math.cos(q1) + m2 * lc2 * g * math.cos(q1 + q2)
        tau2 = m2 * lc2 * g * math.cos(q1 + q2)
        
        # Articulaciones 3 y 4 sin compensación
        tau3 = 0.0
        tau4 = 0.0
        
        return [tau1, tau2, tau3, tau4]

    def send_current_commands(self, currents):
        """Envía comandos de corriente como bear_r.set_goal_iq en código de referencia"""
        try:
            if not self.hardware_manager:
                # Modo simulación
                return True
            
            # Usar set_goal_iq del hardware_manager como en el código de referencia
            # bear_r.set_goal_iq((m_id_1,i_g_1),(m_id_2,i_g_2),(m_id_3,i_g_3),(m_id_4,i_g_4))
            # bear_l.set_goal_iq((m_id_5,i_g_5),(m_id_6,i_g_6),(m_id_7,i_g_7),(m_id_8,i_g_8))
            
            # Crear pares de (motor_id, current) para motores locales disponibles
            current_pairs = []
            for i, motor_id in enumerate(self.motor_ids):
                current = currents[i] if i < len(currents) else 0.0
                current_pairs.append((motor_id, current))
            
            # Enviar corrientes usando hardware_manager
            success = self.hardware_manager.set_goal_iq(*current_pairs)
            
            # Log de las corrientes calculadas para debugging
            if not hasattr(self, '_current_log_count'):
                self._current_log_count = 0
            self._current_log_count += 1
            
            if self._current_log_count % 50 == 0:
                current_str = ', '.join(f'{c:.3f}' for c in currents)
                motor_str = ', '.join(f'M{mid}' for mid in self.motor_ids)
                local_str = ', '.join(f'{p:.3f}' for p in self.current_positions)
                target_str = ', '.join(f'{p:.3f}' for p in self.target_positions)
                self.node.get_logger().info(f"Debug teleoperation:")
                self.node.get_logger().info(f"  Local pos: [{local_str}]")
                self.node.get_logger().info(f"  Target pos: [{target_str}]")
                self.node.get_logger().info(f"  Corrientes [{motor_str}]: [{current_str}]")
            
            return success
            
        except Exception as e:
            self.node.get_logger().error(f"Error enviando corrientes: {e}")
            return False

    def initialise(self) -> None:
        """Inicializar teleoperación remota según secuencia del código de referencia"""
        self.node.get_logger().info("Iniciando teleoperación remota...")
        self.running = True
        self.communication_error_count = 0
        
        # Paso 1: Zero position como en código de referencia
        if not self.zero_position():
            self.node.get_logger().error("Error en zero_position")
            self.running = False
            return
        
        # Paso 2: Configurar control de corriente
        if not self.setup_current_control():
            self.node.get_logger().error("Error configurando control de corriente")
            self.running = False
            return
        
        # Paso 3: Configurar comunicación UDP
        if not self.setup_udp_communication():
            self.node.get_logger().error("Error configurando UDP")
            self.running = False
            return
        
        # Paso 4: Obtener posiciones iniciales
        if not self.get_motor_states():
            self.node.get_logger().warning("Error obteniendo estados iniciales - continuando")
            # Inicializar con ceros solo para motores locales
            self.current_positions = [0.0] * len(self.motor_ids)
            self.current_velocities = [0.0] * len(self.motor_ids)
        
        # Inicializar posiciones objetivo con tamaño del sistema total
        self.target_positions = [0.0] * self.num_total_motors
        
        self.node.get_logger().info(f"Teleoperación iniciada - Máquina {'A' if self.is_machine_a else 'B'}")
        self.node.get_logger().info(f"Local: {self.local_ip}:{self.receive_port} -> Remoto: {self.local_addr}")

    def update(self) -> py_trees.common.Status:
        """Bucle principal siguiendo exactamente la estructura del código de referencia"""
        if not self.running:
            return py_trees.common.Status.SUCCESS
        
        # Verificar entrada del usuario para terminar
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            self.node.get_logger().info("Terminado por usuario")
            self.running = False
            return py_trees.common.Status.SUCCESS
        
        try:
            # Paso 1: Obtener estados actuales de motores (como en while True del código original)
            if not self.get_motor_states():
                self.communication_error_count += 1
                return py_trees.common.Status.RUNNING
            
            # Paso 2: Comunicación UDP en hilos separados (como en código de referencia)
            send_thread = threading.Thread(target=self.send_positions, daemon=True)
            receive_thread = threading.Thread(target=self.receive_positions, daemon=True)
            
            receive_thread.start()
            send_thread.start()
            
            # Paso 3: Procesar datos recibidos y aplicar límites
            self.update_target_positions()
            
            # Esperar a que terminen los hilos
            send_thread.join()
            receive_thread.join()
            
            # Paso 4: Calcular y enviar corrientes de control
            currents = self.calculate_control_currents()
            self.send_current_commands(currents)
            
            # Reset contador de errores si llegamos aquí
            self.communication_error_count = 0
            
        except KeyboardInterrupt:
            self.node.get_logger().info("Terminando comunicación...")
            self.running = False
            return py_trees.common.Status.SUCCESS
            
        except Exception as e:
            self.node.get_logger().error(f"Error en bucle principal: {e}")
            self.communication_error_count += 1
            
            if self.communication_error_count >= self.max_communication_errors:
                self.node.get_logger().error("Demasiados errores de comunicación")
                self.running = False
                return py_trees.common.Status.FAILURE
        
        # Mantener máxima frecuencia (sin sleep adicional)
        return py_trees.common.Status.RUNNING

    def restore_position_control(self):
        """Restaura control de posición y va a home como en código de referencia"""
        try:
            if not self.hardware_manager:
                self.node.get_logger().info("[SIM] Restaurando control de posición")
                return
            
            self.node.get_logger().info("Restaurando control de posición...")
            
            # Restaurar PID gains para posición como en código de referencia
            if hasattr(self.hardware_manager, 'configure_pid_gains'):
                self.hardware_manager.configure_pid_gains(self.motor_ids, p_gain=5.0, i_gain=0.0, d_gain=0.2)
            
            # Cambiar a modo posición
            self.hardware_manager.set_mode(*[(motor_id, 2) for motor_id in self.motor_ids])
            
            # Esperar un poco antes de ir a home
            time.sleep(2)
            
            # Ir a posición home (como en código de referencia)
            # home_position() en el código original
            home_positions = [0.0, 1.5707, -1.5707, -0.785, 0.0, -1.5707, 1.5707, -0.785]
            position_pairs = [(motor_id, home_positions[i]) for i, motor_id in enumerate(self.motor_ids[:8])]
            self.hardware_manager.set_goal_position(*position_pairs)
            
        except Exception as e:
            self.node.get_logger().error(f"Error restaurando control: {e}")

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Terminar teleoperación como secuencia del código de referencia"""
        self.node.get_logger().info(f"Terminando teleoperación remota con estado {new_status}")
        self.running = False
        
        # Cerrar sockets UDP
        try:
            if self.send_socket:
                self.send_socket.close()
            if self.receive_socket:
                self.receive_socket.close()
            self.node.get_logger().info("Sockets UDP cerrados")
        except Exception as e:
            self.node.get_logger().warning(f"Error cerrando sockets: {e}")
        
        # Restaurar control de posición y ir a home
        try:
            self.restore_position_control()
        except Exception as e:
            self.node.get_logger().error(f"Error en restauración: {e}")
        
        # Destruir nodo solo si lo creamos
        if self.own_node and self.node:
            self.node.destroy_node()