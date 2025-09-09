#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class StateCommandPublisher(Node):
    def __init__(self):
        super().__init__('state_command_publisher')
        self.publisher = self.create_publisher(String, 'state_command', 10)

    def publish_command(self, command):
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Enviando comando: {command}')

def main(args=None):
    rclpy.init(args=args)
    node = StateCommandPublisher()

    if len(sys.argv) < 2:
        print("Uso: ros2 run smilei_state_machine send_state_command <comando>")
        print("Comandos disponibles: enable, home, zero, say_hello, teleoperation, remote_teleoperation, disable, idle")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    command = sys.argv[1]
    node.publish_command(command)
    
    # Esperar un momento para asegurarse de que el mensaje se envu00eda
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()
    return 0

if __name__ == '__main__':
    sys.exit(main()) 