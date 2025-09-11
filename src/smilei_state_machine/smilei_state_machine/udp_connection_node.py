#!/usr/bin/env python3
"""
Nodo UDP independiente para comunicación entre máquinas A y B.
Máquina A: Solo envía datos
Máquina B: Solo recibe datos
"""

import rclpy
from rclpy.node import Node
import socket
import struct
import threading
import time
import yaml
import os
from std_msgs.msg import Float32MultiArray
from westwood_motor_interfaces.srv import GetMotorPositions, SetMode, SetTorqueEnable


class UDPConnectionNode(Node):
    def __init__(self):
        super().__init__('udp_connection_node')
        
        # Cargar parámetros desde robot_params.yaml
        self.load_parameters()
        
        # Variables de red
        self.socket = None
        self.running = False
        
        # Variables para datos
        self.send_data = [0.0] * len(self.motor_ids)
        self.received_data = [0.0] * len(self.motor_ids)
        self.data_lock = threading.Lock()
        
        # Clientes de servicio para obtener posiciones y configurar motores
        self.get_position_client = self.create_client(
            GetMotorPositions, 'westwood_motor/get_motor_positions')
        self.set_mode_client = self.create_client(
            SetMode, 'westwood_motor/set_mode')
        self.set_torque_client = self.create_client(
            SetTorqueEnable, 'westwood_motor/set_torque_enable')
        
        # Variables para control de lectura de posiciones
        self.current_positions = [0.0] * len(self.motor_ids)
        self.positions_lock = threading.Lock()
        
        # Publishers y Subscribers
        if self.is_machine_a:
            # Máquina A: Suscribirse a datos para enviar
            self.data_subscriber = self.create_subscription(
                Float32MultiArray,
                'udp_send_data',
                self.data_to_send_callback,
                10
            )
            self.get_logger().info("Máquina A configurada - Solo enviará datos")
        else:
            # Máquina B: Publicar datos recibidos
            self.data_publisher = self.create_publisher(
                Float32MultiArray,
                'udp_received_data',
                10
            )
            self.get_logger().info("Máquina B configurada - Solo recibirá datos")
        
        # Configuración inicial de motores para movimiento manual
        self.get_logger().info("=== INICIANDO CONFIGURACIÓN DE MOTORES ===")
        
        # Esperar por los servicios necesarios
        if not self.wait_for_torque_service():
            self.get_logger().error("No se pudo conectar al servicio de torque. Abortando inicialización.")
            return
            
        # Deshabilitar torque para permitir movimiento manual
        if not self.disable_motor_torque():
            self.get_logger().error("No se pudo deshabilitar torque. Abortando inicialización.")
            return
        
        self.get_logger().info("=== CONFIGURACIÓN DE MOTORES COMPLETADA - LISTOS PARA MOVIMIENTO MANUAL ===")
        
        # Inicializar conexión UDP
        self.setup_udp_connection()
        
        # Esperar por el servicio de posiciones antes de continuar
        self.wait_for_position_service()
        
        # Crear timer para leer posiciones periódicamente
        # Solo para máquina A que necesita enviar posiciones reales
        if self.is_machine_a:
            position_timer_period = 1.0 / self.control_frequency  # Mismo período que UDP
            self.position_timer = self.create_timer(
                position_timer_period, 
                self.read_positions_callback
            )
            self.get_logger().info(f"Timer de posiciones creado con período {position_timer_period:.6f}s")
        
        # Iniciar hilo de comunicación UDP
        self.start_udp_communication()

    def load_parameters(self):
        """Cargar parámetros desde robot_params.yaml"""
        try:
            # Declarar parámetros directamente con el namespace correcto
            self.declare_parameter('remote_teleoperation.is_machine_a', False)
            self.declare_parameter('remote_teleoperation.machine_a_ip', '192.168.1.50')
            self.declare_parameter('remote_teleoperation.machine_b_ip', '192.168.0.2')
            self.declare_parameter('remote_teleoperation.motor_ids', [1])
            self.declare_parameter('remote_teleoperation.socket_timeout', 0.001)
            self.declare_parameter('remote_teleoperation.control_frequency', 1000)
            
            # Cargar valores desde el namespace del YAML
            self.is_machine_a = self.get_parameter('remote_teleoperation.is_machine_a').value
            self.machine_a_ip = self.get_parameter('remote_teleoperation.machine_a_ip').value
            self.machine_b_ip = self.get_parameter('remote_teleoperation.machine_b_ip').value
            self.motor_ids = self.get_parameter('remote_teleoperation.motor_ids').value
            self.socket_timeout = self.get_parameter('remote_teleoperation.socket_timeout').value
            # Aumentar timeout para evitar errores de "timed out" en UDP
            if self.socket_timeout < 0.01:
                self.socket_timeout = 0.01  # Mínimo 10ms para operaciones UDP estables
            self.control_frequency = self.get_parameter('remote_teleoperation.control_frequency').value
            
            # Configurar IPs y puertos según el tipo de máquina
            if self.is_machine_a:
                self.local_ip = self.machine_a_ip
                self.remote_ip = self.machine_b_ip
                self.send_port = 4000      # Máquina A envía al puerto 4000
                self.receive_port = 5001   # Máquina A recibe en puerto 5001
            else:
                self.local_ip = self.machine_b_ip
                self.remote_ip = self.machine_a_ip
                self.send_port = 5001      # Máquina B envía al puerto 5001
                self.receive_port = 4000   # Máquina B recibe en puerto 4000
            
            self.get_logger().info(f"Configuración cargada:")
            self.get_logger().info(f"  - Es máquina A: {self.is_machine_a}")
            self.get_logger().info(f"  - IP local: {self.local_ip}")
            self.get_logger().info(f"  - IP remota: {self.remote_ip}")
            self.get_logger().info(f"  - Puerto envío: {self.send_port}")
            self.get_logger().info(f"  - Puerto recepción: {self.receive_port}")
            self.get_logger().info(f"  - Motor IDs: {self.motor_ids}")
            
        except Exception as e:
            self.get_logger().error(f"Error cargando parámetros: {str(e)}")
            # Valores por defecto en caso de error
            self.is_machine_a = True
            self.machine_a_ip = '192.168.1.50'
            self.machine_b_ip = '192.168.1.39'
            self.motor_ids = [1]
            self.socket_timeout = 0.001
            self.control_frequency = 10000

    def wait_for_position_service(self):
        """Esperar que el servicio de posiciones esté disponible"""
        if self.is_machine_a:
            self.get_logger().info("Esperando servicio de posiciones de motores...")
            if self.get_position_client.wait_for_service(timeout_sec=10.0):
                self.get_logger().info("Servicio de posiciones conectado")
            else:
                self.get_logger().warning("Servicio de posiciones no disponible, usando valores por defecto")

    def wait_for_torque_service(self):
        """Esperar que el servicio de torque esté disponible"""
        self.get_logger().info("Esperando servicio de configuración de torque...")
        if self.set_torque_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().info("Servicio de torque conectado")
            return True
        else:
            self.get_logger().error("Servicio de torque no disponible")
            return False

    def disable_motor_torque(self):
        """Deshabilitar torque de todos los motores para permitir movimiento manual"""
        try:
            self.get_logger().info(f"Deshabilitando torque en motores {self.motor_ids} para movimiento manual...")
            
            # Crear request para deshabilitar torque
            req = SetTorqueEnable.Request()
            req.motor_ids = self.motor_ids
            req.enable_torque = [False] * len(self.motor_ids)  # False = torque deshabilitado
            
            # Llamar al servicio de forma síncrona
            future = self.set_torque_client.call_async(req)
            
            # Esperar por el resultado con timeout
            import rclpy
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                result = future.result()
                if result and result.success:
                    self.get_logger().info(f"Torque deshabilitado exitosamente - Motores listos para movimiento manual: {result.message}")
                    return True
                else:
                    error_msg = result.message if result else "Sin respuesta del servicio"
                    self.get_logger().error(f"Error deshabilitando torque: {error_msg}")
                    return False
            else:
                self.get_logger().error("Timeout deshabilitando torque")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Excepción deshabilitando torque: {str(e)}")
            return False

    def read_positions_callback(self):
        """Callback del timer para leer posiciones de los motores"""
        if not self.is_machine_a:
            return
            
        try:
            # Crear request para obtener posiciones
            req = GetMotorPositions.Request()
            req.motor_ids = self.motor_ids
            
            # Llamar al servicio de forma asíncrona
            future = self.get_position_client.call_async(req)
            
            # Usar un callback para procesar el resultado
            future.add_done_callback(self.position_response_callback)
            
        except Exception as e:
            self.get_logger().debug(f"Error llamando servicio de posiciones: {str(e)}")

    def position_response_callback(self, future):
        """Callback para procesar la respuesta del servicio de posiciones"""
        try:
            result = future.result()
            if result and result.success:
                new_positions = list(result.positions)
                
                # Actualizar posiciones actuales de forma thread-safe
                with self.positions_lock:
                    self.current_positions = new_positions[:len(self.motor_ids)]
                
                # Actualizar datos para enviar
                with self.data_lock:
                    self.send_data = self.current_positions.copy()
                    
        except Exception as e:
            self.get_logger().debug(f"Error procesando respuesta de posiciones: {str(e)}")

    def setup_udp_connection(self):
        """Configurar socket UDP según el tipo de máquina"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.settimeout(self.socket_timeout)
            
            if self.is_machine_a:
                # Máquina A: Solo necesita socket para enviar
                self.get_logger().info(f"Socket UDP creado para envío (Máquina A)")
            else:
                # Máquina B: Bind para recibir
                self.socket.bind((self.local_ip, self.receive_port))
                self.get_logger().info(f"Socket UDP configurado para recepción en {self.local_ip}:{self.receive_port} (Máquina B)")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error configurando socket UDP: {str(e)}")
            return False

    def data_to_send_callback(self, msg):
        """Callback para recibir datos a enviar (solo máquina A)"""
        if self.is_machine_a:
            with self.data_lock:
                self.send_data = list(msg.data)[:len(self.motor_ids)]

    def send_data_udp(self):
        """Enviar datos vía UDP (solo máquina A)"""
        if not self.is_machine_a or self.socket is None:
            return
        
        # Obtener posiciones actuales del servicio
        with self.data_lock:
            data_to_send = self.send_data.copy()
        
        # Log información sobre el origen de los datos
        with self.positions_lock:
            current_pos = self.current_positions.copy()
        
        # Verificar si tenemos datos reales del servicio
        has_real_data = any(val != 0.0 for val in current_pos)
        
        try:
            # Asegurar que tenemos el número correcto de elementos
            if len(data_to_send) != len(self.motor_ids):
                data_to_send = data_to_send[:len(self.motor_ids)]
                if len(data_to_send) < len(self.motor_ids):
                    data_to_send.extend([0.0] * (len(self.motor_ids) - len(data_to_send)))
            
            # Crear formato dinámico basado en número de motores
            num_motors = len(self.motor_ids)
            format_str = f'{num_motors}f'
            
            # Empaquetar y enviar datos
            struct_data = struct.pack(format_str, *data_to_send)
            self.socket.sendto(struct_data, (self.remote_ip, self.send_port))
            
            # Log continuo de envíos (máquina A)
            if not hasattr(self, '_send_count'):
                self._send_count = 0
            self._send_count += 1
            
            # Reducir frecuencia de logging para mejorar performance
            if self._send_count % 100 == 0:  # Log cada 100 envíos en lugar de cada uno
                data_str = ", ".join([f"{val:.6f}" for val in data_to_send])
                data_source = "REAL" if has_real_data else "DEFAULT"
                self.get_logger().info(f"[MÁQUINA A] Enviando #{self._send_count} ({data_source}): [{data_str}] → {self.remote_ip}:{self.send_port}")
                
        except Exception as e:
            if not hasattr(self, '_error_count'):
                self._error_count = 0
            self._error_count += 1
            
            # Solo mostrar errores cada 1000 veces para evitar spam
            if self._error_count % 1000 == 0:
                self.get_logger().warning(f"Error enviando datos UDP (#{self._error_count}): {str(e)}")
                # Información adicional de diagnóstico cada 1000 errores
                self.get_logger().info(f"Diagnóstico UDP: Enviando a {self.remote_ip}:{self.send_port}, timeout={self.socket_timeout}s")

    def receive_data_udp(self):
        """Recibir datos vía UDP (solo máquina B)"""
        if self.is_machine_a or self.socket is None:
            return
        
        try:
            data, addr = self.socket.recvfrom(1024)
            
            # Crear formato dinámico basado en número de motores
            num_motors = len(self.motor_ids)
            format_str = f'<{num_motors}f'
            
            # Desempaquetar datos
            received_values = struct.unpack(format_str, data)
            
            with self.data_lock:
                self.received_data = list(received_values)
            
            # Publicar datos recibidos
            msg = Float32MultiArray()
            msg.data = self.received_data
            self.data_publisher.publish(msg)
            
            # Log reducido de recepciones (máquina B)
            if not hasattr(self, '_receive_count'):
                self._receive_count = 0
            self._receive_count += 1
            
            # Reducir frecuencia de logging para mejorar performance
            if self._receive_count % 100 == 0:  # Log cada 100 recepciones en lugar de cada una
                data_str = ", ".join([f"{val:.6f}" for val in self.received_data])
                data_type = "REAL" if any(val != 0.0 for val in self.received_data) else "ZEROS"
                self.get_logger().info(f"[MÁQUINA B] Recibiendo #{self._receive_count} ({data_type}): [{data_str}] ← {addr[0]}:{addr[1]}")
                
        except socket.timeout:
            # Timeout normal, no hacer nada
            pass
        except Exception as e:
            self.get_logger().warning(f"Error recibiendo datos UDP: {str(e)}")

    def udp_communication_loop(self):
        """Bucle principal de comunicación UDP"""
        self.running = True
        loop_period = 1.0 / self.control_frequency  # Período en segundos
        
        self.get_logger().info(f"Iniciando bucle UDP con frecuencia {self.control_frequency} Hz")
        
        while self.running and rclpy.ok():
            start_time = time.time()
            
            if self.is_machine_a:
                # Máquina A: Solo enviar
                self.send_data_udp()
            else:
                # Máquina B: Solo recibir
                self.receive_data_udp()
            
            # Controlar frecuencia
            elapsed = time.time() - start_time
            sleep_time = max(0, loop_period - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def start_udp_communication(self):
        """Iniciar hilo de comunicación UDP"""
        self.udp_thread = threading.Thread(target=self.udp_communication_loop, daemon=True)
        self.udp_thread.start()
        
        if self.is_machine_a:
            self.get_logger().info("Hilo UDP iniciado - Máquina A lista para enviar")
        else:
            self.get_logger().info("Hilo UDP iniciado - Máquina B lista para recibir")

    def stop_udp_communication(self):
        """Detener comunicación UDP"""
        self.running = False
        if hasattr(self, 'udp_thread'):
            self.udp_thread.join(timeout=1.0)
        
        if self.socket:
            self.socket.close()
            
        self.get_logger().info("Comunicación UDP detenida")

    def destroy_node(self):
        """Limpiar recursos al destruir el nodo"""
        self.stop_udp_communication()
        super().destroy_node()


def main(args=None):
    """Función principal"""
    rclpy.init(args=args)
    
    try:
        # Crear y ejecutar el nodo
        udp_node = UDPConnectionNode()
        
        # Spin del nodo
        rclpy.spin(udp_node)
        
    except KeyboardInterrupt:
        pass
    finally:
        # Limpiar
        if 'udp_node' in locals():
            udp_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()