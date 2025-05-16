import py_trees
import numpy as np
import rclpy
import math
import time
import sys
import select
from westwood_motor_interfaces.srv import SetMotorIdAndTarget
from westwood_motor_interfaces.srv import GetMotorPositions

class SayHello(py_trees.behaviour.Behaviour):
    """Comportamiento que mueve los motores en un patrón sinusoidal."""
    def __init__(self, name: str, motor_ids: list[int], node=None):
        super().__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.set_position_client = None
        self.get_position_client = None
        self.running = False
        self.own_node = False
        self.last_time = time.time()

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        # Usar el nodo proporcionado en lugar de crear uno nuevo
        if self.node is None:
            self.node = rclpy.create_node('say_hello_client')
            self.own_node = True
        else:
            self.own_node = False
            
        self.set_position_client = self.node.create_client(
            SetMotorIdAndTarget,
            'westwood_motor/set_motor_id_and_target'
        )
        
        self.get_position_client = self.node.create_client(
            GetMotorPositions,
            'westwood_motor/get_motor_positions'
        )
        
        if timeout_sec is None:
            timeout_sec = 1.0
        
        # Esperar a que estén disponibles los servicios
        services_available = True
        
        # Intentar esperar por el servicio de posición
        if not self.set_position_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("Servicio set_motor_id_and_target no disponible, continuando en modo simulación")
            services_available = False
        
        # Intentar esperar por el servicio de obtención de posición
        if not self.get_position_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("Servicio get_motor_positions no disponible, continuando en modo simulación")
            services_available = False
        
        return True  # Siempre retorna True para permitir que continúe

    def initialise(self) -> None:
        self.node.get_logger().info("Iniciando comportamiento Say Hello")
        self.running = True
        self.last_time = time.time()

    def update(self) -> py_trees.common.Status:
        # Si no está corriendo, devolver SUCCESS para permitir la transición al siguiente estado
        if not self.running:
            return py_trees.common.Status.SUCCESS
        
        # Verificar si hay entrada del usuario para terminar
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            if line:
                self.node.get_logger().info("Terminando comportamiento Say Hello por entrada del usuario")
                self.running = False
                return py_trees.common.Status.SUCCESS
        
        # Calcular posiciones sinusoidales
        current_time = time.time()
        qd = 0.5 * math.sin(10.0 * current_time) - 1.0
        
        # Lista para almacenar los IDs de motores y sus posiciones
        motor_ids_to_move = []
        target_positions = []
        
        # Si tenemos al menos 2 motores, mover en direcciones opuestas
        if len(self.motor_ids) >= 2:
            motor_ids_to_move = [self.motor_ids[0], self.motor_ids[-1]]
            target_positions = [-qd, qd]  # Movimiento opuesto
        # Si solo tenemos 1 motor, moverlo de todas formas
        elif len(self.motor_ids) == 1:
            motor_ids_to_move = [self.motor_ids[0]]
            target_positions = [qd]
        
        # Si tenemos motores para mover
        if motor_ids_to_move:
            # Crear solicitud para establecer posiciones
            req = SetMotorIdAndTarget.Request()
            req.motor_ids = motor_ids_to_move
            req.target_positions = target_positions
            
            # Intentar llamar al servicio
            try:
                future = self.set_position_client.call_async(req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.1)
                
                if future.done():
                    result = future.result()
                    if not result.success:
                        self.node.get_logger().warning(f"Error al establecer posiciones: {result.message}")
                else:
                    self.node.get_logger().debug("Timeout al establecer posiciones")
            except Exception as e:
                self.node.get_logger().error(f"Error al llamar al servicio: {str(e)}")
        else:
            self.node.get_logger().warning("No hay motores para el comportamiento Say Hello")
        
        # Pequeña pausa para no saturar el sistema
        if current_time - self.last_time < 0.01:
            time.sleep(0.01 - (current_time - self.last_time))
        
        self.last_time = time.time()
        
        # Siempre devuelve RUNNING mientras está activo
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.node.get_logger().info(f"Terminando comportamiento Say Hello con estado {new_status}")
        self.running = False
        
        # Solo destruir el nodo si lo creamos nosotros mismos
        if self.own_node and self.node:
            self.node.destroy_node() 