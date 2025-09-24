import py_trees
import rclpy
import time
import sys
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
    def __init__(self, name: str, motor_ids=None, node=None, hardware_manager=None):
        super().__init__(name)
        self.node = node
        self.running = False
        self.own_node = False
        
        # Hardware manager para control de motores
        self.hardware_manager = hardware_manager
        self.available_motors = []
        self.all_system_motors = []
        
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
        
        # Ganancias de control PD - usando los valores del nodo PD que funciona
        self.kp = 1.0        # Proportional gain (del pd_control_node.py) - default para todos excepto motor 7
        self.kp_motor7 = 0.5  # Proportional gain específica para motor 7
        self.kd = 0.1        # Damping gain (del pd_control_node.py)
        
        # Parámetros del control PD no lineal (del pd_control_node.py)
        self.r1 = 0.4
        self.r2 = 0.3
        self.p1 = (2*self.r2 - self.r1) / self.r1
        self.p2 = (2*self.r2 - self.r1) / self.r2
        
        # Estimador de velocidad (del pd_control_node.py)
        self.Fc = 35         # Frequency cutoff
        self.Tl = 0.002      # Loop frequency
        
        # Variables del estimador de velocidad
        self.theta_estimators = []  # Se inicializa por motor
        self.vel_estimators = []    # Se inicializa por motor
        self.Kt = 0.35                      # Constante de torque
        
        # Límites de seguridad
        self.max_current = 5.0              # Límite máximo de corriente (A)
        self.error_deadband = 0.05          # Zona muerta para errores pequeños (rad)
        self.max_error = 1.57               # Error máximo permitido (π/2 rad)
        
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

        # Debugging
        self.debug_udp_latency = False
        self.debug_pd_control = False
        self.last_packet_time = None
        
        # Atributos para resumen de latencia simplificado
        self.latency_sum = 0.0
        self.latency_packet_count = 0
        self.last_latency_log_time = 0.0

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        """Configurar el comportamiento según parámetros ROS2"""
        if self.node is None:
            self.node = rclpy.create_node('remote_teleoperation_client')
            self.own_node = True
        else:
            self.own_node = False
        
        try:
            # Declarar parámetros - configuración flexible para número de motores
            # Usar lista de enteros por defecto para evitar problemas de tipos
            self.node.declare_parameter('remote_teleoperation.motor_ids', [1, 2])  # Por defecto motores 1 y 2
            self.node.declare_parameter('remote_teleoperation.use_all_motors', False)  # True = usar todos los motores disponibles
            self.node.declare_parameter('remote_teleoperation.is_machine_a', True)
            self.node.declare_parameter('remote_teleoperation.machine_a_ip', '192.168.0.144')
            self.node.declare_parameter('remote_teleoperation.machine_b_ip', '192.168.0.2')
            self.node.declare_parameter('remote_teleoperation.max_total_motors', 8)  # Máximo de motores en el sistema
            self.node.declare_parameter('remote_teleoperation.debug_udp_latency', False)
            self.node.declare_parameter('remote_teleoperation.debug_pd_control', False)
            
            # Cargar parámetros
            param_motor_ids = self.node.get_parameter('remote_teleoperation.motor_ids').value
            use_all_motors = self.node.get_parameter('remote_teleoperation.use_all_motors').value
            self.is_machine_a = self.node.get_parameter('remote_teleoperation.is_machine_a').value
            self.machine_a_ip = self.node.get_parameter('remote_teleoperation.machine_a_ip').value
            self.machine_b_ip = self.node.get_parameter('remote_teleoperation.machine_b_ip').value
            self.max_total_motors = self.node.get_parameter('remote_teleoperation.max_total_motors').value
            self.debug_udp_latency = self.node.get_parameter('remote_teleoperation.debug_udp_latency').value
            self.debug_pd_control = self.node.get_parameter('remote_teleoperation.debug_pd_control').value
            if self.debug_udp_latency:
                self.node.get_logger().info("Depuración de latencia UDP ACTIVADA.")
            if self.debug_pd_control:
                self.node.get_logger().info("Depuración de control PD ACTIVADA.")
            
            # Obtener motores disponibles del hardware manager
            if self.hardware_manager:
                available_motors = self.hardware_manager.get_available_motors()
                if use_all_motors:
                    # Usar todos los motores disponibles
                    self.motor_ids = available_motors
                    self.node.get_logger().info(f"Usando TODOS los motores disponibles: {available_motors}")
                else:
                    # Usar solo los motores especificados que están disponibles
                    self.motor_ids = [m for m in param_motor_ids if m in available_motors]
                    self.node.get_logger().info(f"Usando motores especificados: {self.motor_ids} (disponibles: {available_motors})")
                
                self.all_system_motors = available_motors
                self.num_total_motors = len(available_motors)
            else:
                # Sin hardware manager, usar parámetros
                self.motor_ids = param_motor_ids
                self.all_system_motors = self.motor_ids
                self.num_total_motors = len(self.motor_ids)
                self.node.get_logger().warning(f"Sin hardware manager - usando motores de parámetros: {self.motor_ids}")
            
            # Configurar red según máquina (basado en código de referencia)
            if self.is_machine_a:
                # Máquina A configuración
                self.local_ip = self.machine_a_ip
                self.remote_ip = self.machine_b_ip
                self.send_port = 4000    # Puerto usado para enviar (no importa mucho)
                self.receive_port = 5005 # A recibe en puerto 5005
                self.local_addr = (self.remote_ip, 4000)  # A envía a puerto 4000 de B
            else:
                # Máquina B configuración  
                self.local_ip = self.machine_b_ip
                self.remote_ip = self.machine_a_ip
                self.send_port = 5005    # Puerto usado para enviar (no importa mucho)
                self.receive_port = 4000 # B recibe en puerto 4000
                self.local_addr = (self.remote_ip, 5005)  # B envía a puerto 5005 de A
            
            # Inicializar variables de control basadas en número total de motores
            self.node.get_logger().info(f"Configuración: {len(self.motor_ids)} motor(es) local(es), {self.num_total_motors} total en sistema")
            self.node.get_logger().info(f"Motores locales: {self.motor_ids}")
            self.node.get_logger().info(f"Todos los motores del sistema: {self.all_system_motors}")
            
            # Inicializar arrays con tamaño correcto
            # current_positions/velocities: solo para motores locales
            self.current_positions = [0.0] * len(self.motor_ids)
            self.current_velocities = [0.0] * len(self.motor_ids)
            # target_positions: para todo el sistema
            self.target_positions = [0.0] * self.num_total_motors
            
        except Exception as e:
            self.node.get_logger().error(f"Error cargando parámetros: {str(e)}")
            return False
        
        # Verificar hardware
        if self.hardware_manager is not None:
            # Los motores ya fueron configurados arriba, solo verificar conexión
            self.available_motors = [m for m in self.motor_ids if m in self.all_system_motors]
            if self.available_motors:
                self.node.get_logger().info(f"Hardware conectado - motores disponibles para teleoperación: {self.available_motors}")
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
            # Para máquina A, usar timeout más largo debido a problemas de red
            send_timeout = 0.5 if self.is_machine_a else 0.1
            self.send_socket.settimeout(send_timeout)  # Timeout ajustado por máquina
            
            # Socket para recibir (cliente) - bind en puerto local (usar 0.0.0.0 para cualquier interfaz)
            self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            # Configurar opciones de socket para permitir reutilización
            self.receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                self.receive_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except AttributeError:
                # SO_REUSEPORT no disponible en todos los sistemas
                pass
            self.receive_socket.bind(('0.0.0.0', self.receive_port))
            self.receive_socket.settimeout(0.01)  # Timeout para recepción UDP
            
            self.node.get_logger().info(f"=== UDP CONFIGURACIÓN ===")
            self.node.get_logger().info(f"Máquina: {'A' if self.is_machine_a else 'B'}")
            self.node.get_logger().info(f"Local IP: {self.local_ip}")
            self.node.get_logger().info(f"Remoto IP: {self.remote_ip}")
            self.node.get_logger().info(f"Puerto de recepción: {self.receive_port}")
            self.node.get_logger().info(f"Enviando a: {self.local_addr}")
            self.node.get_logger().info(f"Formato UDP: {self.num_total_motors} floats ({self.num_total_motors * 4} bytes)")
            self.node.get_logger().info(f"✅ Puerto {self.receive_port} bound exitosamente")
                
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
        """Configuración de motores usando parámetros exactos del pd_control_node.py"""
        try:
            if not self.hardware_manager:
                self.node.get_logger().info("[SIM] Configurando control simple")
                return True
            
            if self.is_machine_a:
                self.node.get_logger().info("Configurando MÁQUINA A: motores con corriente CERO (movimiento libre)")
                # Máquina A: configurar motores para movimiento libre (corriente cero)
                # Configurar PID gains para control de corriente (como pd_control_node.py)
                for motor_id in self.motor_ids:
                    # PID iq/id control (del pd_control_node.py líneas 98-103)
                    self.hardware_manager.set_p_gain_iq((motor_id, 0.277))
                    self.hardware_manager.set_i_gain_iq((motor_id, 0.061))
                    self.hardware_manager.set_d_gain_iq((motor_id, 0))
                    self.hardware_manager.set_p_gain_id((motor_id, 0.277))
                    self.hardware_manager.set_i_gain_id((motor_id, 0.061))
                    self.hardware_manager.set_d_gain_id((motor_id, 0))
                    
                    # Modo corriente (modo 0) como pd_control_node.py línea 116
                    self.hardware_manager.set_mode((motor_id, 0))
                    
                    # Habilitar torque
                    self.hardware_manager.set_torque_enable((motor_id, 1))
                    
                self.node.get_logger().info("Máquina A configurada - motores libres para teleoperar")
                
            else:
                self.node.get_logger().info("Configurando MÁQUINA B: control PD activo")
                # Máquina B: configurar para control PD activo
                for motor_id in self.motor_ids:
                    # PID gains exactos del pd_control_node.py
                    self.hardware_manager.set_p_gain_iq((motor_id, 0.277))
                    self.hardware_manager.set_i_gain_iq((motor_id, 0.061))
                    self.hardware_manager.set_d_gain_iq((motor_id, 0))
                    self.hardware_manager.set_p_gain_id((motor_id, 0.277))
                    self.hardware_manager.set_i_gain_id((motor_id, 0.061))
                    self.hardware_manager.set_d_gain_id((motor_id, 0))
                    
                    # Modo corriente (modo 0) como pd_control_node.py
                    self.hardware_manager.set_mode((motor_id, 0))
                    
                    # Habilitar torque  
                    self.hardware_manager.set_torque_enable((motor_id, 1))
                
                self.node.get_logger().info("Máquina B configurada - control PD activo")
            
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"Error configurando motor: {str(e)}")
            return False

    def get_motor_states(self):
        """Obtiene posiciones y velocidades usando hardware_manager mejorado con formato pd_control_node.py"""
        try:
            if not self.hardware_manager:
                # Modo simulación - mantener estados actuales
                return True
            
            # Obtener posiciones y velocidades usando los métodos mejorados del hardware_manager
            positions = self.hardware_manager.get_present_position(*self.motor_ids)
            velocities = self.hardware_manager.get_present_velocity(*self.motor_ids)
            
            # Validar que obtuvimos datos válidos
            if len(positions) == len(self.motor_ids) and len(velocities) == len(self.motor_ids):
                self.current_positions = positions[:]
                self.current_velocities = velocities[:]
                return True
            else:
                # Fallback a valores anteriores si hay problemas de comunicación
                self.node.get_logger().debug("Datos incompletos - usando valores anteriores")
                return True
                
        except Exception as e:
            self.node.get_logger().debug(f"Error obteniendo estados motores: {e}")
            # Mantener valores anteriores en caso de error
            return True

    def send_positions(self):
        """Envía posiciones y el timestamp apropiado para el cálculo RTT."""
        if not self.send_socket:
            return
        
        try:
            # Preparar posiciones locales para enviar
            positions_to_send = [0.0] * max(self.max_total_motors, self.num_total_motors)
            
            # Llenar con posiciones reales de motores locales
            for i, motor_id in enumerate(self.motor_ids):
                if i < len(self.current_positions) and motor_id <= len(positions_to_send):
                    motor_index = motor_id - 1
                    if motor_index >= 0 and motor_index < len(positions_to_send):
                        positions_to_send[motor_index] = self.current_positions[i]

            # Lógica de timestamp para RTT
            if self.is_machine_a:
                # Máquina A (master) envía su propio timestamp actual
                timestamp = self.node.get_clock().now().nanoseconds / 1e9
                data_to_send = positions_to_send + [timestamp]
            else:
                # Máquina B (esclavo) devuelve el timestamp que recibió
                data_to_send = positions_to_send + [self.timestamp_to_echo]
            
            # Crear formato dinámico y empaquetar
            format_str = f'{len(data_to_send)}f'
            struct_ql = struct.pack(format_str, *data_to_send)
            self.send_socket.sendto(struct_ql, self.local_addr)
            
            # Log ocasional para debug (sin mostrar el timestamp)
            if not hasattr(self, '_send_count'):
                self._send_count = 0
            self._send_count += 1
            if self._send_count % 100 == 0:
                active_positions = [(i+1, pos) for i, pos in enumerate(positions_to_send) if pos != 0.0]
                machine = 'A' if self.is_machine_a else 'B'
                if active_positions:
                    pos_str = ', '.join(f'M{motor_id}:{pos:.3f}' for motor_id, pos in active_positions)
                    self.node.get_logger().info(f"UDP TX [{machine}] -> {self.local_addr}: {pos_str}")
                else:
                    self.node.get_logger().info(f"UDP TX [{machine}] -> {self.local_addr}: todas posiciones en 0")
            
        except socket.timeout:
            if not hasattr(self, '_timeout_fallback_count'):
                self._timeout_fallback_count = 0
            self._timeout_fallback_count += 1
            
            if self._timeout_fallback_count % 100 == 0:
                self.node.get_logger().warning(f"UDP timeout count: {self._timeout_fallback_count}")
                
        except Exception as e:
            self.node.get_logger().warning(f"Error enviando posiciones: {e}")

    def receive_positions(self):
        """Recibe datos, calcula latencia RTT/2 si es master, o guarda timestamp si es esclavo."""
        if not self.receive_socket:
            return
        
        try:
            data, addr = self.receive_socket.recvfrom(1024)
            reception_time = self.node.get_clock().now()

            num_floats = len(data) // 4
            if num_floats < 2:
                return

            format_str = f'{num_floats}f'
            unpacked_data = struct.unpack(format_str, data)
            
            positions = unpacked_data[:-1]
            received_timestamp = unpacked_data[-1]

            if self.debug_udp_latency:
                if self.is_machine_a:
                    # Máquina A (master) calcula la latencia RTT
                    if received_timestamp > 0:  # Ignorar ecos iniciales con timestamp 0
                        rtt_s = (reception_time.nanoseconds / 1e9) - received_timestamp
                        latency_ms = (rtt_s / 2.0) * 1000.0

                        # Filtro de robustez para la latencia calculada
                        if 0 < latency_ms < 1000:
                            self.latency_sum += latency_ms
                            self.latency_packet_count += 1
                        else:
                            # Log de latencia anómala (podría ser el primer paquete)
                            if not hasattr(self, '_rtt_warn_count'): self._rtt_warn_count = 0
                            if self._rtt_warn_count % 100 == 0:
                                self.node.get_logger().warning(f"Latencia RTT/2 anómala o inicial: {latency_ms:.2f}ms")
                            self._rtt_warn_count += 1
                else:
                    # Máquina B (esclavo) guarda el timestamp para devolverlo
                    self.timestamp_to_echo = received_timestamp

                # Logueo de estadísticas (solo se actualiza en la máquina A)
                if self.is_machine_a and (reception_time.nanoseconds / 1e9) - self.last_latency_log_time >= 1.0:
                    if self.latency_packet_count > 0:
                        avg_latency = self.latency_sum / self.latency_packet_count
                        self.node.get_logger().info(
                            f"UDP Stats (último seg): "
                            f"Latencia RTT/2 avg={avg_latency:.2f}ms | "
                            f"Paquetes={self.latency_packet_count}/s"
                        )
                    self.latency_sum = 0.0
                    self.latency_packet_count = 0
                    self.last_latency_log_time = reception_time.nanoseconds / 1e9
            
            with self.data_lock:
                self.received_data.append(positions)
                
            # Log ocasional para debug (usando `positions`)
            if not hasattr(self, '_recv_count'):
                self._recv_count = 0
            self._recv_count += 1
            if self._recv_count % 100 == 0:
                machine = 'A' if self.is_machine_a else 'B'
                active_positions = [(i+1, pos) for i, pos in enumerate(positions) if pos != 0.0]
                if active_positions:
                    pos_str = ', '.join(f'M{motor_id}:{pos:.3f}' for motor_id, pos in active_positions)
                    self.node.get_logger().info(f"UDP RX [{machine}] <- {addr}: {pos_str}")
                else:
                    self.node.get_logger().info(f"UDP RX [{machine}] <- {addr}: todas posiciones en 0")
                
        except socket.timeout:
            if not hasattr(self, '_timeout_count'):
                self._timeout_count = 0
            self._timeout_count += 1
            
            if self._timeout_count % 1000 == 0:
                self.node.get_logger().debug(f"UDP RX timeouts: {self._timeout_count}")
                
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
                # Expandir target_positions si es necesario
                while len(self.target_positions) < len(entry):
                    self.target_positions.append(0.0)
                
                for i in range(len(entry)):
                    received_position = entry[i]
                    
                    # Aplicar límites básicos de seguridad para cualquier motor
                    if received_position > -3.15 and received_position < 3.15:  # Límites generales ±π
                        if i < len(self.target_positions):
                            self.target_positions[i] = received_position
            
            # Limpiar datos procesados
            self.received_data.clear()
            return True

    def calculate_control_currents(self):
        """Calcula corrientes de control usando algoritmo PD exacto del pd_control_node.py"""
        try:
            currents = []
            
            # Para cada motor local
            for i, motor_id in enumerate(self.motor_ids):
                if self.is_machine_a:
                    # Máquina A: corriente cero para movimiento libre
                    currents.append(0.0)
                else:
                    # Máquina B: usar PD control exacto del pd_control_node.py
                    # Obtener target desde datos UDP usando el ID del motor como índice
                    target_pos = 0.0  # Default
                    motor_index = motor_id - 1  # Convertir a índice base 0
                    if motor_index >= 0 and motor_index < len(self.target_positions):
                        target_pos = self.target_positions[motor_index]
                    
                    current_pos = self.current_positions[i] if i < len(self.current_positions) else 0.0
                    
                    # Error de posición (pd_control_node.py líneas 132-133)
                    error = current_pos - target_pos

                    # Estimador de velocidad (pd_control_node.py líneas 135-140)
                    if i < len(self.vel_estimators):
                        self.vel_estimators[i] = self.Fc * (self.theta_estimators[i] + current_pos)
                        self.theta_estimators[i] = self.theta_estimators[i] - self.Tl * self.vel_estimators[i]
                        vel_estimate = self.vel_estimators[i]
                    else:
                        vel_estimate = 0.0

                    # Usar kp específica para motor 7, kp normal para el resto
                    kp_value = self.kp_motor7 if motor_id == 7 else self.kp

                    # Control PD no lineal exacto (pd_control_node.py líneas 143-144)
                    tau = -kp_value * ((abs(error)**self.p1) * np.sign(error)) - self.kd * vel_estimate
                    
                    # Convertir torque a corriente (pd_control_node.py líneas 147-148)
                    current = tau / self.Kt
                    
                    # Límites de seguridad
                    current = max(-self.max_current, min(self.max_current, current))
                    
                    currents.append(current)
                    
                    # Debug cada 100 iteraciones - mostrar info para cada motor
                    if not hasattr(self, '_debug_counter'):
                        self._debug_counter = {}
                    if motor_id not in self._debug_counter:
                        self._debug_counter[motor_id] = 0
                    
                    self._debug_counter[motor_id] += 1
                    if self._debug_counter[motor_id] % 100 == 0:
                        self.node.get_logger().info(f"🎯 PD Control M{motor_id}: pos={current_pos:.3f}, target={target_pos:.3f}, error={error:.3f}, current={current:.3f}A")
            
            return currents
            
        except Exception as e:
            self.node.get_logger().error(f"Error calculando corrientes PD: {e}")
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
            
            if self.debug_pd_control and self._current_log_count % 50 == 0:
                machine = 'A' if self.is_machine_a else 'B'
                # Mostrar información de cada motor por separado
                for i, motor_id in enumerate(self.motor_ids):
                    if i < len(currents) and i < len(self.current_positions):
                        current = currents[i]
                        local_pos = self.current_positions[i]
                        motor_index = motor_id - 1
                        remote_pos = self.target_positions[motor_index] if motor_index < len(self.target_positions) else 0.0
                        error = local_pos - remote_pos
                        # Normalizar error para mostrar
                        while error > 3.14159:
                            error -= 2 * 3.14159
                        while error < -3.14159:
                            error += 2 * 3.14159
                        self.node.get_logger().info(f"Debug [{machine}] M{motor_id}: Local={local_pos:.3f}, Remote={remote_pos:.3f}, Error={error:.3f}, I={current:.3f}A")
            
            return success
            
        except Exception as e:
            self.node.get_logger().error(f"Error enviando corrientes: {e}")
            return False
    
    def send_position_commands(self, positions):
        """Envía comandos de POSICIÓN (mucho más estable que corriente)"""
        try:
            if not self.hardware_manager:
                # Modo simulación
                return True
            
            # Crear pares de (motor_id, position) para motores locales disponibles
            position_pairs = []
            for i, motor_id in enumerate(self.motor_ids):
                position = positions[i] if i < len(positions) else 0.0
                position_pairs.append((motor_id, position))
            
            # Enviar posiciones usando hardware_manager (modo posición)
            success = self.hardware_manager.set_goal_position(*position_pairs)
            
            # Log cada cierto tiempo para debugging
            if not hasattr(self, '_position_log_count'):
                self._position_log_count = 0
            self._position_log_count += 1
            
            if self._position_log_count % 200 == 0:  # Menos frecuente que corrientes
                position_str = ', '.join(f'{p:.3f}' for p in positions)
                motor_str = ', '.join(f'M{mid}' for mid in self.motor_ids)
                machine = 'A' if self.is_machine_a else 'B'
                self.node.get_logger().info(f"🎯 Posición [{machine}]: {motor_str}={position_str}")
            
            return success
            
        except Exception as e:
            self.node.get_logger().error(f"Error enviando posiciones: {e}")
            return False

    def initialise(self) -> None:
        """Inicializar teleoperación remota según secuencia del código de referencia"""
        self.node.get_logger().info("Iniciando teleoperación remota...")
        self.running = True
        self.communication_error_count = 0
        
        # Usar hardware_manager mejorado en lugar de conexión directa Pybear
        if self.hardware_manager:
            self.node.get_logger().info("✅ Usando hardware_manager mejorado con patrones pd_control_node.py")
        else:
            self.node.get_logger().warning("⚠️ Hardware manager no disponible - modo simulación")
        
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
        
        # Inicializar estimadores de velocidad (del pd_control_node.py)
        self.theta_estimators = [0.0] * len(self.motor_ids)
        self.vel_estimators = [0.0] * len(self.motor_ids)

        # Resetear estadísticas de latencia
        if self.debug_udp_latency:
            self.latency_sum = 0.0
            self.latency_packet_count = 0
            self.last_latency_log_time = self.node.get_clock().now().nanoseconds / 1e9
        
        self.timestamp_to_echo = 0.0
        
        self.node.get_logger().info(f"Teleoperación iniciada - Máquina {'A' if self.is_machine_a else 'B'}")
        self.node.get_logger().info(f"Local: {self.local_ip}:{self.receive_port} -> Remoto: {self.local_addr}")

    def update(self) -> py_trees.common.Status:
        """Bucle principal siguiendo exactamente la estructura del código de referencia"""
        if not self.running:
            return py_trees.common.Status.SUCCESS
        
        # Verificar entrada del usuario para terminar - COMENTADO para ROS
        # En ROS no necesitamos esta verificación de stdin ya que el comportamiento
        # se maneja a través de la state machine
        # if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
        #     line = sys.stdin.readline().strip()
        #     self.node.get_logger().info("Terminado por usuario")
        #     self.running = False
        #     return py_trees.common.Status.SUCCESS
        
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
            
            # Paso 4: Calcular y enviar corrientes PD (como pd_control_node.py)
            control_currents = self.calculate_control_currents()
            self.send_current_commands(control_currents)
            
            # Reset contador de errores gradualmente si llegamos aquí sin problemas
            if self.communication_error_count > 0:
                self.communication_error_count = max(0, self.communication_error_count - 2)  # Reducir más rápido
            
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
        
        # SIN THROTTLE: Permitir que py_trees maneje la frecuencia naturalmente
        # Igual que pd_control_node.py que funciona perfecto
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
            # home_position() en el código original - posiciones seguras para cada motor
            default_home_positions = [0.0, 1.5707, -1.5707, -0.785, 0.0, -1.5707, 1.5707, -0.785]
            
            position_pairs = []
            for i, motor_id in enumerate(self.motor_ids):
                # Usar posición home por defecto si está en la lista, sino usar 0.0
                if i < len(default_home_positions):
                    home_pos = default_home_positions[i]
                else:
                    home_pos = 0.0  # Posición segura por defecto
                position_pairs.append((motor_id, home_pos))
            
            if position_pairs:
                self.hardware_manager.set_goal_position(*position_pairs)
                self.node.get_logger().info(f"Enviando a home {len(position_pairs)} motores: {[f'M{mid}:{pos:.3f}' for mid, pos in position_pairs]}")
            
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
                self.send_socket = None
            if self.receive_socket:
                self.receive_socket.close()
                self.receive_socket = None
            self.node.get_logger().info("Sockets UDP cerrados correctamente")
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