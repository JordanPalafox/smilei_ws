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
        
        # Inicializar conexión UDP
        self.setup_udp_connection()
        
        # Iniciar hilo de comunicación UDP
        self.start_udp_communication()

    def load_parameters(self):
        """Cargar parámetros desde el archivo robot_params.yaml"""
        try:
            # Declarar parámetros con valores por defecto
            self.declare_parameter('remote_teleoperation.is_machine_a', True)
            self.declare_parameter('remote_teleoperation.machine_a_ip', '192.168.1.50')
            self.declare_parameter('remote_teleoperation.machine_b_ip', '192.168.1.39')
            self.declare_parameter('remote_teleoperation.motor_ids', [1])
            self.declare_parameter('remote_teleoperation.socket_timeout', 0.001)
            self.declare_parameter('remote_teleoperation.control_frequency', 10000)
            
            # Cargar valores
            self.is_machine_a = self.get_parameter('remote_teleoperation.is_machine_a').value
            self.machine_a_ip = self.get_parameter('remote_teleoperation.machine_a_ip').value
            self.machine_b_ip = self.get_parameter('remote_teleoperation.machine_b_ip').value
            self.motor_ids = self.get_parameter('remote_teleoperation.motor_ids').value
            self.socket_timeout = self.get_parameter('remote_teleoperation.socket_timeout').value
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
        
        try:
            with self.data_lock:
                data_to_send = self.send_data.copy()
            
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
            
            # Log periódico (cada 1000 envíos)
            if not hasattr(self, '_send_count'):
                self._send_count = 0
            self._send_count += 1
            
            if self._send_count % 1000 == 0:
                data_str = ", ".join([f"{val:.3f}" for val in data_to_send])
                self.get_logger().info(f"Enviados #{self._send_count}: [{data_str}]")
                
        except Exception as e:
            self.get_logger().warning(f"Error enviando datos UDP: {str(e)}")

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
            
            # Log periódico (cada 1000 recepciones)
            if not hasattr(self, '_receive_count'):
                self._receive_count = 0
            self._receive_count += 1
            
            if self._receive_count % 1000 == 0:
                data_str = ", ".join([f"{val:.3f}" for val in self.received_data])
                self.get_logger().info(f"Recibidos #{self._receive_count}: [{data_str}] desde {addr[0]}")
                
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