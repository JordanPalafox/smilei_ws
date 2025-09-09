#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from westwood_motor_interfaces.srv import GetMotorPositions, GetMotorVelocities, GetMotorCurrents, GetAvailableMotors

class MotorDataPublisher(Node):
    def __init__(self):
        super().__init__('motor_data_publisher')
        self.get_logger().info('Motor Data Publisher started')

        # Parámetros
        self.declare_parameter('publish_rate', 5.0)  # Hz
        self.publish_rate = self.get_parameter('publish_rate').value

        # Publishers
        self.position_publisher = self.create_publisher(Float64MultiArray, 'motor_data/positions', 10)
        self.velocity_publisher = self.create_publisher(Float64MultiArray, 'motor_data/velocities', 10)
        self.current_publisher = self.create_publisher(Float64MultiArray, 'motor_data/currents', 10)

        # Service Clients
        self.get_positions_client = self.create_client(GetMotorPositions, 'westwood_motor/get_motor_positions')
        self.get_velocities_client = self.create_client(GetMotorVelocities, 'westwood_motor/get_motor_velocities')
        self.get_currents_client = self.create_client(GetMotorCurrents, 'westwood_motor/get_motor_currents')
        self.get_available_motors_client = self.create_client(GetAvailableMotors, 'westwood_motor/get_available_motors')

        # Esperar a que los servicios estén disponibles
        self.wait_for_services()

        self.motor_ids = []
        self.get_available_motors()

        # Atributos para almacenar resultados de servicios
        self._positions = None
        self._velocities = None
        self._currents = None

        # Timer para publicar datos
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_data)

    def wait_for_services(self):
        self.get_logger().info('Waiting for motor services...')
        self.get_positions_client.wait_for_service()
        self.get_velocities_client.wait_for_service()
        self.get_currents_client.wait_for_service()
        self.get_available_motors_client.wait_for_service()
        self.get_logger().info('Motor services available.')

    def get_available_motors(self):
        req = GetAvailableMotors.Request()
        future = self.get_available_motors_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.motor_ids = future.result().motor_ids
            self.get_logger().info(f'Available motors: {self.motor_ids}')
        else:
            self.get_logger().error('Failed to get available motors')

    def publish_data(self):
        if not self.motor_ids:
            self.get_logger().warning('No motors available to publish data')
            return

        # Get Positions
        pos_req = GetMotorPositions.Request()
        pos_req.motor_ids = self.motor_ids
        pos_future = self.get_positions_client.call_async(pos_req)
        pos_future.add_done_callback(self._service_response_callback)

        # Get Velocities
        vel_req = GetMotorVelocities.Request()
        vel_req.motor_ids = self.motor_ids
        vel_future = self.get_velocities_client.call_async(vel_req)
        vel_future.add_done_callback(self._service_response_callback)

        # Get Currents
        cur_req = GetMotorCurrents.Request()
        cur_req.motor_ids = self.motor_ids
        cur_future = self.get_currents_client.call_async(cur_req)
        cur_future.add_done_callback(self._service_response_callback)

    def _service_response_callback(self, future):
        try:
            response = future.result()
            if response is None:
                self.get_logger().error('Service call failed: No response')
                return

            # Identificar el tipo de respuesta y almacenar los datos
            if isinstance(response, GetMotorPositions.Response):
                if response.success:
                    self._positions = response.positions
                else:
                    self.get_logger().error('Failed to get motor positions')
            elif isinstance(response, GetMotorVelocities.Response):
                if response.success:
                    self._velocities = response.velocities
                else:
                    self.get_logger().error('Failed to get motor velocities')
            elif isinstance(response, GetMotorCurrents.Response):
                if response.success:
                    self._currents = response.currents
                else:
                    self.get_logger().error('Failed to get motor currents')

            # Si se han recibido todas las respuestas, publicar los datos
            if self._positions is not None and self._velocities is not None and self._currents is not None:
                pos_msg = Float64MultiArray()
                pos_msg.data = [float(p) for p in self._positions]
                self.position_publisher.publish(pos_msg)

                vel_msg = Float64MultiArray()
                vel_msg.data = [float(v) for v in self._velocities]
                self.velocity_publisher.publish(vel_msg)

                cur_msg = Float64MultiArray()
                cur_msg.data = [float(c) for c in self._currents]
                self.current_publisher.publish(cur_msg)

                # Reiniciar para el siguiente ciclo
                self._positions = None
                self._velocities = None
                self._currents = None
        
        except Exception as e:
            self.get_logger().error(f'Exception in service response callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()