import py_trees
import time
import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import GetAvailableMotors
from westwood_motor_interfaces.srv import SetGains
from westwood_motor_interfaces.srv import SetMode
from westwood_motor_interfaces.srv import SetTorqueEnable
import numpy as np

class EnableRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name, motor_ids, node):
        """
        Inicializar el comportamiento.
        
        Args:
            name: Nombre del comportamiento
            motor_ids: Lista de IDs de los motores a controlar
            node: Nodo ROS para comunicación
        """
        super(EnableRobot, self).__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.logger = self.node.get_logger()
        
        # Leer parámetros de configuración desde el archivo de parámetros
        self.node.declare_parameter('position_control_gains.p_gain_position', 5.0)
        self.node.declare_parameter('position_control_gains.i_gain_position', 0.0)
        self.node.declare_parameter('position_control_gains.d_gain_position', 0.2)
        self.node.declare_parameter('position_control_gains.iq_max', 1.5)
        self.node.declare_parameter('position_control_gains.p_gain_iq', 0.02)
        self.node.declare_parameter('position_control_gains.i_gain_iq', 0.02)
        self.node.declare_parameter('position_control_gains.d_gain_iq', 0.0)
        self.node.declare_parameter('position_control_gains.p_gain_id', 0.02)
        self.node.declare_parameter('position_control_gains.i_gain_id', 0.02)
        self.node.declare_parameter('position_control_gains.d_gain_id', 0.0)
        self.node.declare_parameter('position_control_gains.kt', 0.35)
        
        # Obtener parámetros
        self.p_gain_position = self.node.get_parameter('position_control_gains.p_gain_position').value
        self.i_gain_position = self.node.get_parameter('position_control_gains.i_gain_position').value
        self.d_gain_position = self.node.get_parameter('position_control_gains.d_gain_position').value
        self.iq_max = self.node.get_parameter('position_control_gains.iq_max').value
        self.p_gain_iq = self.node.get_parameter('position_control_gains.p_gain_iq').value
        self.i_gain_iq = self.node.get_parameter('position_control_gains.i_gain_iq').value
        self.d_gain_iq = self.node.get_parameter('position_control_gains.d_gain_iq').value
        self.p_gain_id = self.node.get_parameter('position_control_gains.p_gain_id').value
        self.i_gain_id = self.node.get_parameter('position_control_gains.i_gain_id').value
        self.d_gain_id = self.node.get_parameter('position_control_gains.d_gain_id').value
        self.kt = self.node.get_parameter('position_control_gains.kt').value
        
        # Crear clientes para los servicios
        self.get_available_motors_client = self.node.create_client(
            GetAvailableMotors, 
            'westwood_motor/get_available_motors'
        )
        
        self.set_position_gains_client = self.node.create_client(
            SetGains, 
            'westwood_motor/set_position_gains'
        )
        
        self.set_mode_client = self.node.create_client(
            SetMode, 
            'westwood_motor/set_mode'
        )
        
        self.set_torque_enable_client = self.node.create_client(
            SetTorqueEnable, 
            'westwood_motor/set_torque_enable'
        )
        
        # Variables para controlar el flujo de ejecución
        self.setup_complete = False
        self.error = False
        self.available_motors = []
        
    def setup(self, **kwargs):
        """
        Configuración inicial del comportamiento.
        """
        self.logger.info(f"{self.name} configurando...")
        for client in [self.get_available_motors_client, self.set_position_gains_client, 
                      self.set_mode_client, self.set_torque_enable_client]:
            if not client.wait_for_service(timeout_sec=1.0):
                self.logger.error(f"Servicio {client.srv_name} no disponible")
                return False
        
        self.logger.info(f"{self.name} configurado correctamente")
        return True
        
    def initialise(self):
        """
        Inicialización del comportamiento antes de cada ejecución.
        """
        self.logger.info(f"{self.name} inicializando...")
        self.setup_complete = False
        self.error = False
    
    def get_available_motors(self):
        """
        Obtener los motores disponibles.
        """
        request = GetAvailableMotors.Request()
        future = self.get_available_motors_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.available_motors = response.motor_ids
                self.logger.info(f"Motores disponibles: {self.available_motors}")
                return True
            else:
                self.logger.error(f"Error al obtener motores disponibles: {response.message}")
                # Usar los motor_ids proporcionados como fallback
                self.logger.warning(f"Usando motor_ids configurados: {self.motor_ids}")
                self.available_motors = self.motor_ids
                return True
        else:
            self.logger.error("No se recibió respuesta al obtener motores disponibles")
            # Usar los motor_ids proporcionados como fallback
            self.logger.warning(f"Usando motor_ids configurados: {self.motor_ids}")
            self.available_motors = self.motor_ids
            return True
    
    def set_position_gains(self):
        """
        Configurar las ganancias para el control de posición.
        """
        request = SetGains.Request()
        request.motor_ids = self.motor_ids
        request.p_gain_position = self.p_gain_position
        request.i_gain_position = self.i_gain_position
        request.d_gain_position = self.d_gain_position
        request.p_gain_iq = self.p_gain_iq
        request.i_gain_iq = self.i_gain_iq
        request.d_gain_iq = self.d_gain_iq
        request.p_gain_id = self.p_gain_id
        request.i_gain_id = self.i_gain_id
        request.d_gain_id = self.d_gain_id
        request.iq_max = self.iq_max
        request.kt = self.kt
        
        future = self.set_position_gains_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.logger.info(f"Ganancias de posición configuradas: {response.message}")
                return True
            else:
                self.logger.error(f"Error al configurar ganancias de posición: {response.message}")
                self.logger.warning("Continuando de todos modos...")
                return True
        else:
            self.logger.error("No se recibió respuesta al configurar ganancias de posición")
            self.logger.warning("Continuando de todos modos...")
            return True
    
    def set_mode(self):
        """
        Configurar el modo de operación de los motores (modo 2: posición).
        """
        request = SetMode.Request()
        request.motor_ids = self.motor_ids
        request.modes = [2] * len(self.motor_ids)  # Modo 2: posición
        
        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.logger.info(f"Modo de operación configurado: {response.message}")
                return True
            else:
                self.logger.error(f"Error al configurar modo de operación: {response.message}")
                self.logger.warning("Continuando de todos modos...")
                return True
        else:
            self.logger.error("No se recibió respuesta al configurar modo de operación")
            self.logger.warning("Continuando de todos modos...")
            return True
    
    def enable_torque(self):
        """
        Habilitar el torque de los motores.
        """
        request = SetTorqueEnable.Request()
        request.motor_ids = self.motor_ids
        request.enable_torque = [True] * len(self.motor_ids)
        
        future = self.set_torque_enable_client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.logger.info(f"Torque habilitado: {response.message}")
                return True
            else:
                self.logger.error(f"Error al habilitar torque: {response.message}")
                self.logger.warning("Continuando de todos modos...")
                return True
        else:
            self.logger.error("No se recibió respuesta al habilitar torque")
            self.logger.warning("Continuando de todos modos...")
            return True
    
    def update(self):
        """
        Actualización del comportamiento. Se ejecuta en cada tick del árbol.
        """
        # Si ya se completó la configuración, devolver SUCCESS
        if self.setup_complete:
            return py_trees.common.Status.SUCCESS
        
        # Si hubo un error, devolver FAILURE
        if self.error:
            return py_trees.common.Status.FAILURE
        
        # Verificar motores disponibles
        if not self.get_available_motors():
            self.logger.error("No se pudieron obtener los motores disponibles")
            self.error = True
            return py_trees.common.Status.FAILURE
        
        # Verificar si los motores solicitados están disponibles
        for motor_id in self.motor_ids:
            if motor_id not in self.available_motors:
                self.logger.warning(f"Motor {motor_id} no disponible")
        
        # Configurar ganancias
        if not self.set_position_gains():
            self.logger.error("Error al configurar ganancias")
            self.error = True
            return py_trees.common.Status.FAILURE
        
        # Configurar modo
        if not self.set_mode():
            self.logger.error("Error al configurar modo")
            self.error = True
            return py_trees.common.Status.FAILURE
        
        # Habilitar torque
        if not self.enable_torque():
            self.logger.error("Error al habilitar torque")
            self.error = True
            return py_trees.common.Status.FAILURE
        
        # Si todo ha ido bien, marcar como completado
        self.logger.info("Robot habilitado correctamente")
        self.setup_complete = True
        return py_trees.common.Status.SUCCESS
    
    def terminate(self, new_status):
        """
        Terminación del comportamiento.
        """
        self.logger.info(f"{self.name} terminando con estado {new_status}")
        pass 