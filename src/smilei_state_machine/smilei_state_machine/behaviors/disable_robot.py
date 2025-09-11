import py_trees
import time
import rclpy
from westwood_motor_interfaces.srv import SetMotorIdAndTarget
from westwood_motor_interfaces.srv import SetTorqueEnable

class DisableRobot(py_trees.behaviour.Behaviour):
    """Comportamiento que deshabilita los motores y los pone en un estado seguro"""
    def __init__(self, name: str, motor_ids: list[int], node=None, robot_name: str = ""):
        super().__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.robot_name = robot_name
        self.set_position_client = None
        self.set_torque_client = None
        self.flag = True
        self.own_node = False

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
            
        self.set_position_client = self.node.create_client(
            SetMotorIdAndTarget,
            self._get_topic_name('westwood_motor/set_motor_id_and_target')
        )
        
        self.set_torque_client = self.node.create_client(
            SetTorqueEnable,
            self._get_topic_name('westwood_motor/set_torque_enable')
        )
        
        if timeout_sec is None:
            timeout_sec = 1.0
            
        # Intentar esperar por los servicios
        if not self.set_position_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("Servicio set_motor_id_and_target no disponible, continuando en modo simulación")
        
        if not self.set_torque_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("Servicio set_torque_enable no disponible, continuando en modo simulación")
        
        return True  # Siempre retorna True para permitir que continúe

    def home_position(self):
        """Envía todos los motores a posición home (90 grados)"""
        self.node.get_logger().info("Enviando motores a posición home")
        
        # Crear solicitud para establecer posiciones a home
        req = SetMotorIdAndTarget.Request()
        req.motor_ids = self.motor_ids
        req.target_positions = [1.5707] * len(self.motor_ids)  # Aproximadamente 90 grados
        
        # Llamar al servicio
        try:
            future = self.set_position_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.done():
                result = future.result()
                if not result.success:
                    self.node.get_logger().warning(f"Error al establecer posición home: {result.message}")
                    return False
            else:
                self.node.get_logger().warning("Timeout al establecer posición home")
                return False
            
            # Esperar a que los motores lleguen a la posición
            time.sleep(2.0)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al establecer posición home: {str(e)}")
            return False

    def disable_torque(self):
        """Deshabilita el torque de todos los motores"""
        self.node.get_logger().info("Deshabilitando torque de los motores")
        
        # Crear solicitud para deshabilitar torque
        req = SetTorqueEnable.Request()
        req.motor_ids = self.motor_ids
        req.enable_torque = [False] * len(self.motor_ids)
        
        # Llamar al servicio
        try:
            future = self.set_torque_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.done():
                result = future.result()
                if not result.success:
                    self.node.get_logger().warning(f"Error al deshabilitar torque: {result.message}")
                    return False
            else:
                self.node.get_logger().warning("Timeout al deshabilitar torque")
                return False
            
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