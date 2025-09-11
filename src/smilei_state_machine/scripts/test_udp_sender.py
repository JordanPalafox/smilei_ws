#!/usr/bin/env python3
"""
Script de ejemplo para probar el envío de datos UDP (Máquina A)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import math
import time


class UDPSenderTest(Node):
    def __init__(self):
        super().__init__('udp_sender_test')
        
        # Publisher para enviar datos al nodo UDP
        self.data_publisher = self.create_publisher(
            Float32MultiArray,
            'udp_send_data',
            10
        )
        
        # Timer para enviar datos periódicamente
        self.timer = self.create_timer(0.1, self.send_test_data)  # 10 Hz
        
        self.counter = 0
        self.get_logger().info("Test UDP Sender iniciado - Enviando datos de prueba cada 100ms")

    def send_test_data(self):
        """Enviar datos de prueba"""
        # Generar datos de prueba (onda sinusoidal)
        msg = Float32MultiArray()
        
        # Crear una onda sinusoidal para testing
        t = time.time()
        test_value = math.sin(t * 2.0 * math.pi * 0.1)  # Frecuencia de 0.1 Hz
        
        # Enviar el mismo valor para todos los motores configurados
        msg.data = [test_value]  # Para un motor, ajustar según motor_ids en config
        
        self.data_publisher.publish(msg)
        
        self.counter += 1
        if self.counter % 50 == 0:  # Log cada 5 segundos
            self.get_logger().info(f"Enviando dato de prueba: {test_value:.3f}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        sender_test = UDPSenderTest()
        rclpy.spin(sender_test)
    except KeyboardInterrupt:
        pass
    finally:
        if 'sender_test' in locals():
            sender_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()