#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import GetMotorVelocities
from std_msgs.msg import Float32MultiArray # Changed from Float32

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.get_logger().info('🚗 Velocity Publisher Node (Refactored) - Started')

        # 1. Declarar parámetros para que sean configurables
        self.declare_parameter('motor_ids', [1])
        self.declare_parameter('publish_frequency', 5.0)
        self.declare_parameter('robot_name', '')

        # Obtener valores de los parámetros
        self.motor_ids = self.get_parameter('motor_ids').get_parameter_value().integer_array_value
        publish_frequency = self.get_parameter('publish_frequency').get_parameter_value().double_value
        self.robot_name = self.get_parameter('robot_name').value

        if not self.motor_ids:
            self.get_logger().error("❌ Parámetro 'motor_ids' no definido o vacío. Saliendo.")
            return

        # Cliente para el servicio
        self.get_velocity_client = self.create_client(
            GetMotorVelocities,
            self._get_topic_name('westwood_motor/get_motor_velocities')
        )
        
        # Esperar a que el servicio esté disponible
        while not self.get_velocity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Servicio "get_motor_velocities" no disponible, esperando...')

        # Publicador para la velocidad
        self.velocity_pub = self.create_publisher(Float32MultiArray, self._get_topic_name('current_velocities'), 10) # Changed topic and type

        # 2. Usar un Timer en lugar de un hilo y time.sleep()
        timer_period = 1.0 / publish_frequency
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info(f"✅ Nodo configurado. Publicando velocidades para motores {self.motor_ids} a {publish_frequency} Hz.")

    def _get_topic_name(self, topic_name):
        if self.robot_name:
            return f'/{self.robot_name}/{topic_name}'
        return topic_name

    def timer_callback(self):
        """
        Esta función se llama periódicamente gracias al Timer.
        Llama al servicio para obtener las velocidades de todos los motores de una vez.
        """
        request = GetMotorVelocities.Request()
        # The service expects a list of motor_ids, so we convert the int[] from params
        request.motor_ids = [int(i) for i in self.motor_ids]
        
        future = self.get_velocity_client.call_async(request)
        # Adjuntar una callback que se ejecutará cuando llegue la respuesta
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        """
        Esta función se ejecuta cuando el servicio responde.
        """
        try:
            response = future.result()
            if response is not None and response.success:
                velocity_msg = Float32MultiArray()
                velocity_msg.data = response.velocities
                
                self.velocity_pub.publish(velocity_msg)
                # self.get_logger().info(f'Publicada velocidades: {velocity_msg.data}')
            elif response is not None:
                self.get_logger().error(f'❌ La llamada al servicio no tuvo éxito: {response.message}')
            else:
                self.get_logger().error('❌ La llamada al servicio no tuvo éxito (sin respuesta).')
        except Exception as e:
            self.get_logger().error(f'❌ Excepción durante la llamada al servicio: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VelocityPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node and rclpy.ok():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()