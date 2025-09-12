import py_trees
import time
import rclpy

class DisableRobot(py_trees.behaviour.Behaviour):
    """Comportamiento que deshabilita los motores y los pone en un estado seguro"""
    def __init__(self, name: str, motor_ids: list[int], node=None, robot_name: str = "", hardware_manager=None):
        super().__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.robot_name = robot_name
        self.flag = True
        self.own_node = False
        
        # Use hardware manager instead of direct Pybear
        self.hardware_manager = hardware_manager
        self.available_motors = []

    def _get_topic_name(self, topic_name):
        if self.robot_name:
            return f'/{self.robot_name}/{topic_name.lstrip("/")}'
        return topic_name
        
    def setup(self, timeout_sec=None, **kwargs) -> bool:
        # Usar el nodo proporcionado en lugar de crear uno nuevo
        if self.node is None:
            self.node = rclpy.create_node('disable_robot_client')
            self.own_node = True
        else:
            self.own_node = False
        
        # Check hardware connection using hardware manager
        if self.hardware_manager is not None:
            available_motors = self.hardware_manager.get_available_motors()
            self.available_motors = [m for m in self.motor_ids if m in available_motors]
            if self.available_motors:
                self.node.get_logger().info(f"Hardware conectado - motores disponibles: {self.available_motors}")
            else:
                self.node.get_logger().warning("No hay motores disponibles")
        else:
            self.node.get_logger().warning("Hardware manager no disponible - modo simulación")
        
        return True  # Siempre retorna True para permitir que continúe

    def home_position(self):
        """Envía todos los motores a posición home (90 grados) usando hardware_manager"""
        self.node.get_logger().info("Enviando motores a posición home")
        
        try:
            if self.hardware_manager:
                # Set all motors to home position (1.5707 rad ≈ 90 degrees)
                position_pairs = [(motor_id, 1.5707) for motor_id in self.motor_ids]
                self.hardware_manager.set_goal_position(*position_pairs)
            else:
                self.node.get_logger().debug("[SIM] Enviando motores a posición home")
            
            # Wait for motors to reach position
            time.sleep(2.0)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al establecer posición home: {str(e)}")
            return False

    def disable_torque(self):
        """Deshabilita el torque de todos los motores usando hardware_manager"""
        self.node.get_logger().info("Deshabilitando torque de los motores")
        
        try:
            if self.hardware_manager:
                # Disable torque for all motors
                disable_pairs = [(motor_id, 0) for motor_id in self.motor_ids]
                self.hardware_manager.set_torque_enable(*disable_pairs)
            else:
                self.node.get_logger().debug("[SIM] Deshabilitando torque de los motores")
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al deshabilitar torque: {str(e)}")
            return False

    def initialise(self) -> None:
        self.node.get_logger().info("Iniciando deshabilitación del robot")
        self.flag = True

    def update(self) -> py_trees.common.Status:
        if self.flag:
            # Solo deshabilitar torque sin mover los motores
            result = self.disable_torque()
            
            # Marcar como completado
            self.flag = False
            
            self.node.get_logger().info("Robot deshabilitado correctamente")
            time.sleep(1.0)
        
        # Este comportamiento siempre termina con éxito
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.node.get_logger().info(f"Terminando deshabilitación del robot con estado {new_status}")
        
        # Solo destruir el nodo si lo creamos nosotros mismos
        if self.own_node and self.node:
            self.node.destroy_node() 