import py_trees
import rclpy
import time
import sys
import select
import socket
import struct
import threading
import numpy as np
from pathlib import Path
from westwood_motor_interfaces.srv import (
    SetMotorIdAndTarget, GetMotorPositions, GetMotorVelocities,
    SetGains, SetMode, SetTorqueEnable, SetGoalIq
)

# Importar configuración
try:
    import sys
    sys.path.append(str(Path(__file__).parent.parent.parent / "config"))
    from remote_teleop_config import get_network_config, get_motor_config, get_communication_config
    CONFIG_AVAILABLE = True
except ImportError:
    CONFIG_AVAILABLE = False

class RemoteTeleoperationGains:
    """
    Ganancias para teleoperación remota (control por corriente)
    Basado en el código original de Python
    """
    # Ganancias PID para iq/id (control de corriente)
    p_gain_iq = 0.277
    i_gain_iq = 0.061
    d_gain_iq = 0.0
    p_gain_id = 0.277
    i_gain_id = 0.061
    d_gain_id = 0.0
    
    # Ganancias de posición (puestas a cero para control de corriente)
    p_gain_position = 0.0
    i_gain_position = 0.0
    d_gain_position = 0.0
    
    # Ganancias de control remoto
    kp = [1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75, 1.75]
    kd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    
    # Constantes físicas
    iq_max = 3.0
    kt = 0.35  # N.m/A

class RemoteTeleoperation(py_trees.behaviour.Behaviour):
    """
    Comportamiento de teleoperación remota que permite que motores en diferentes 
    computadoras se repliquen mutuamente a través de comunicación UDP.
    
    Basado en el código original de teleoperación remota de main.py
    """
    def __init__(self, name: str, motor_ids=None, node=None, 
                 remote_ip=None, local_ip=None, send_port=None, receive_port=None):
        super().__init__(name)
        self.node = node
        self.running = False
        self.own_node = False
        
        # Cargar configuración desde archivo o usar valores por defecto
        if CONFIG_AVAILABLE:
            net_config = get_network_config()
            motor_config = get_motor_config()
            comm_config = get_communication_config()
            
            self.motor_ids = motor_ids if motor_ids is not None else motor_config['motor_ids']
            self.remote_ip = remote_ip or net_config['remote_ip']
            self.local_ip = local_ip or net_config['local_ip']
            self.send_port = send_port or net_config['send_port']
            self.receive_port = receive_port or net_config['receive_port']
            self.max_communication_errors = comm_config['max_errors']
            
            # Ganancias desde configuración
            RemoteTeleoperationGains.kp = motor_config['control_gains']['kp'][:len(self.motor_ids)]
            RemoteTeleoperationGains.kd = motor_config['control_gains']['kd'][:len(self.motor_ids)]
        else:
            # Valores por defecto si no hay configuración
            self.motor_ids = motor_ids if motor_ids is not None else [1, 2, 3, 4, 5, 6, 7, 8]
            self.remote_ip = remote_ip or '192.168.0.100'
            self.local_ip = local_ip or '192.168.0.2'
            self.send_port = send_port or 5005
            self.receive_port = receive_port or 4000
            self.max_communication_errors = 10
        
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
        self.set_gains_client = None
        self.set_mode_client = None
        self.set_torque_client = None
        self.set_iq_client = None
        
        # Variables para control
        self.current_positions = [0.0] * len(self.motor_ids)
        self.current_velocities = [0.0] * len(self.motor_ids)
        self.target_positions = [0.0] * len(self.motor_ids)
        
        # Variables para manejo de errores
        self.communication_error_count = 0

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        """Configurar el comportamiento"""
        if self.node is None:
            self.node = rclpy.create_node('remote_teleoperation_client')
            self.own_node = True
        else:
            self.own_node = False
        
        # Crear clientes para servicios
        self.set_position_client = self.node.create_client(
            SetMotorIdAndTarget, 'westwood_motor/set_motor_id_and_target')
        self.get_position_client = self.node.create_client(
            GetMotorPositions, 'westwood_motor/get_motor_positions')
        self.get_velocity_client = self.node.create_client(
            GetMotorVelocities, 'westwood_motor/get_motor_velocities')
        self.set_gains_client = self.node.create_client(
            SetGains, 'westwood_motor/set_position_gains')
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
                self.node.get_logger().warning(f"Servicio {name} no disponible")
                services_ready = False
        
        return services_ready

    def setup_udp_communication(self):
        """Configurar sockets UDP para comunicación remota"""
        try:
            # Socket para enviar datos (no necesita bind específico)
            self.send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.send_socket.settimeout(0.001)
            
            # Socket para recibir datos (bind en IP local en puerto de recepción)
            self.receive_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.receive_socket.bind((self.local_ip, self.receive_port))
            self.receive_socket.settimeout(0.001)
            
            self.node.get_logger().info(f"UDP configurado: Envío hacia {self.remote_ip}:{self.receive_port}, "
                                      f"Recepción en {self.local_ip}:{self.receive_port}")
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error configurando UDP: {str(e)}")
            return False

    def zero_position(self):
        """Envía todos los motores a posición cero"""
        self.node.get_logger().info("Enviando motores a posición cero")
        
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
                    self.node.get_logger().warning(f"Error en zero_position: {result.message}")
            return False
        except Exception as e:
            self.node.get_logger().error(f"Error en zero_position: {str(e)}")
            return False

    def setup_current_control(self):
        """Configura los motores para control de corriente (modo 0)"""
        self.node.get_logger().info(f"Configurando motores {self.motor_ids} para control de corriente")
        
        try:
            # Configurar ganancias para control de corriente
            req_gains = SetGains.Request()
            req_gains.motor_ids = self.motor_ids
            req_gains.p_gain_iq = float(RemoteTeleoperationGains.p_gain_iq)
            req_gains.i_gain_iq = float(RemoteTeleoperationGains.i_gain_iq)
            req_gains.d_gain_iq = float(RemoteTeleoperationGains.d_gain_iq)
            req_gains.p_gain_id = float(RemoteTeleoperationGains.p_gain_id)
            req_gains.i_gain_id = float(RemoteTeleoperationGains.i_gain_id)
            req_gains.d_gain_id = float(RemoteTeleoperationGains.d_gain_id)
            req_gains.p_gain_position = float(RemoteTeleoperationGains.p_gain_position)
            req_gains.i_gain_position = float(RemoteTeleoperationGains.i_gain_position)
            req_gains.d_gain_position = float(RemoteTeleoperationGains.d_gain_position)
            req_gains.iq_max = float(RemoteTeleoperationGains.iq_max)
            req_gains.kt = float(RemoteTeleoperationGains.kt)
            
            future = self.set_gains_client.call_async(req_gains)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            # Configurar modo corriente (modo 0)
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
                    self.current_positions = list(result_pos.positions)
            
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
            # Ajustar el signo del motor 4 (índice 3) como en el código original
            pos_to_send = self.current_positions.copy()
            if len(pos_to_send) > 3:
                pos_to_send[3] = -1.0 * pos_to_send[3]
            
            # Empaquetar datos (8 floats)
            if len(pos_to_send) < 8:
                pos_to_send.extend([0.0] * (8 - len(pos_to_send)))
            
            struct_data = struct.pack('8f', *pos_to_send[:8])
            # Enviar al puerto de recepción de la máquina remota
            self.send_socket.sendto(struct_data, (self.remote_ip, self.receive_port))
            
            # Debug cada 100 envíos
            if not hasattr(self, '_send_count'):
                self._send_count = 0
            self._send_count += 1
            
            if self._send_count % 100 == 0:
                self.node.get_logger().info(f"📤 Enviando posición {pos_to_send[0]:.3f} a {self.remote_ip}:{self.receive_port}")
            
        except Exception as e:
            self.node.get_logger().warning(f"Error enviando posiciones: {str(e)}")

    def receive_positions(self):
        """Recibe posiciones remotas vía UDP"""
        if self.receive_socket is None:
            return
        
        try:
            data, addr = self.receive_socket.recvfrom(1024)
            struct_data = struct.unpack('<8f', data)
            
            with self.data_lock:
                self.received_data.append(struct_data)
            
            # Debug cada 100 recepciones
            if not hasattr(self, '_receive_count'):
                self._receive_count = 0
            self._receive_count += 1
            
            if self._receive_count % 100 == 0:
                self.node.get_logger().info(f"📥 Recibido posición {struct_data[0]:.3f} de {addr}")
                
        except socket.timeout:
            pass  # Timeout normal
        except Exception as e:
            self.node.get_logger().warning(f"Error recibiendo posiciones: {str(e)}")

    def update_target_positions(self):
        """Actualiza posiciones objetivo desde datos recibidos con límites de seguridad"""
        with self.data_lock:
            if not self.received_data:
                return False
            
            # Procesar el último dato recibido
            for entry in self.received_data:
                # Aplicar límites de seguridad como en el código original
                limits = [
                    (-1.58, 1.58),    # Motor 1
                    (-0.79, 1.58),    # Motor 2
                    (-3.1416, 1.58),  # Motor 3
                    (-0.18, 1.16),    # Motor 4
                    (-1.58, 1.58),    # Motor 5
                    (-1.58, 0.79),    # Motor 6
                    (-1.58, 3.1416),  # Motor 7
                    (-1.16, 0.18)     # Motor 8
                ]
                
                for i, (min_val, max_val) in enumerate(limits[:len(self.motor_ids)]):
                    if i < len(entry) and min_val <= entry[i] <= max_val:
                        self.target_positions[i] = entry[i]
            
            # Limpiar datos procesados
            self.received_data.clear()
            return True

    def calculate_control_currents(self):
        """Calcula corrientes de control basado en error de posición y velocidad"""
        currents = []
        
        for i in range(len(self.motor_ids)):
            if i < len(self.current_positions) and i < len(self.target_positions):
                # Error de posición
                pos_error = self.current_positions[i] - self.target_positions[i]
                
                # Error de velocidad (si está disponible)
                vel_error = 0.0
                if i < len(self.current_velocities):
                    vel_error = self.current_velocities[i]
                
                # Calcular corriente objetivo usando ganancias
                kp = RemoteTeleoperationGains.kp[i] if i < len(RemoteTeleoperationGains.kp) else 1.75
                kd = RemoteTeleoperationGains.kd[i] if i < len(RemoteTeleoperationGains.kd) else 0.1
                
                iq = (-kp * pos_error - kd * vel_error) / RemoteTeleoperationGains.kt
                currents.append(iq)
            else:
                currents.append(0.0)
        
        return currents

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
            
            if self._current_send_count % 100 == 0:
                curr_str = ", ".join([f"I{i}={c:.3f}" for i, c in enumerate(currents)])
                self.node.get_logger().info(f"⚡ Enviando corrientes: [{curr_str}]")
            
            future = self.set_iq_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.05)
            
            if future.done():
                result = future.result()
                if not result.success and self._current_send_count % 50 == 0:
                    self.node.get_logger().warning(f"Error en set_goal_iq: {result.message}")
                return result.success
            
            return False
        except Exception as e:
            self.node.get_logger().warning(f"Error enviando corrientes: {str(e)}")
            return False

    def initialise(self) -> None:
        """Inicializar teleoperación remota"""
        self.node.get_logger().info(f"Iniciando teleoperación remota para motores {self.motor_ids}")
        self.node.get_logger().info(f"🌐 Red: {self.remote_ip}:{self.send_port} → {self.local_ip}:{self.send_port}")
        self.node.get_logger().info("🎮 Control: [ENTER] para terminar")
        
        self.running = True
        
        # Configuración inicial
        if not self.zero_position():
            self.node.get_logger().warning("Error en posición cero, continuando...")
        
        time.sleep(2)
        
        if not self.setup_current_control():
            self.node.get_logger().error("Error configurando control de corriente")
            self.running = False
            return
        
        if not self.setup_udp_communication():
            self.node.get_logger().error("Error configurando comunicación UDP")
            self.running = False
            return

    def update(self) -> py_trees.common.Status:
        """Bucle principal de teleoperación remota"""
        if not self.running:
            return py_trees.common.Status.SUCCESS
        
        # Verificar errores de comunicación
        if self.communication_error_count >= self.max_communication_errors:
            self.node.get_logger().error(f"Demasiados errores de comunicación. Terminando.")
            self.running = False
            return py_trees.common.Status.FAILURE
        
        # Verificar entrada del usuario
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            self.node.get_logger().info("🔄 Terminando teleoperación remota")
            self.running = False
            return py_trees.common.Status.SUCCESS
        
        try:
            # Obtener estados actuales
            if not self.get_motor_states():
                self.communication_error_count += 1
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
            
            if self._debug_counter % 50 == 0:
                pos_str = ", ".join([f"M{mid}={pos:.3f}" for mid, pos in 
                                   zip(self.motor_ids, self.current_positions)])
                target_str = ", ".join([f"T{i}={t:.3f}" for i, t in 
                                      enumerate(self.target_positions)])
                error = self.current_positions[0] - self.target_positions[0] if len(self.current_positions) > 0 and len(self.target_positions) > 0 else 0.0
                self.node.get_logger().info(f"Pos: [{pos_str}] Target: [{target_str}] Error: {error:.3f}")
            
        except Exception as e:
            self.node.get_logger().error(f"Error en teleoperación remota: {str(e)}")
            self.communication_error_count += 1
        
        time.sleep(0.001)  # Control de alta frecuencia como en el original
        return py_trees.common.Status.RUNNING

    def restore_position_control(self):
        """Restaura control de posición antes de terminar"""
        self.node.get_logger().info("Restaurando control de posición")
        
        try:
            # Restaurar ganancias de posición
            req_gains = SetGains.Request()
            req_gains.motor_ids = self.motor_ids
            req_gains.p_gain_position = 5.0   # Del código original
            req_gains.i_gain_position = 0.0
            req_gains.d_gain_position = 0.2
            req_gains.p_gain_iq = 0.02
            req_gains.i_gain_iq = 0.02
            req_gains.d_gain_iq = 0.0
            req_gains.p_gain_id = 0.02
            req_gains.i_gain_id = 0.02
            req_gains.d_gain_id = 0.0
            req_gains.iq_max = 3.0
            req_gains.kt = 0.35
            
            future = self.set_gains_client.call_async(req_gains)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
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
        self.node.get_logger().info(f"Terminando teleoperación remota con estado {new_status}")
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