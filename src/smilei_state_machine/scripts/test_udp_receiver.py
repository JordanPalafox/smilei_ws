#!/usr/bin/env python3
"""
Script de ejemplo para probar la recepción de datos UDP (Máquina B)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class UDPReceiverTest(Node):
    def __init__(self):
        super().__init__('udp_receiver_test')
        
        # Subscriber para recibir datos del nodo UDP
        self.data_subscriber = self.create_subscription(
            Float32MultiArray,
            'udp_received_data',
            self.received_data_callback,
            10
        )
        
        self.receive_counter = 0
        self.get_logger().info("Test UDP Receiver iniciado - Esperando datos UDP")

    def received_data_callback(self, msg):
        """Callback para datos recibidos"""
        self.receive_counter += 1
        
        # Log cada 50 recepciones (aprox. cada 5 segundos a 10Hz)
        if self.receive_counter % 50 == 0:
            data_str = ", ".join([f"{val:.3f}" for val in msg.data])
            self.get_logger().info(f"Datos recibidos #{self.receive_counter}: [{data_str}]")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        receiver_test = UDPReceiverTest()
        rclpy.spin(receiver_test)
    except KeyboardInterrupt:
        pass
    finally:
        if 'receiver_test' in locals():
            receiver_test.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()