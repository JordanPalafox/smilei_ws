import py_trees
import time
import rclpy
from rclpy.node import Node
from pybear import Manager
import numpy as np

class EnableRobot(py_trees.behaviour.Behaviour):
    def __init__(self, name, motor_ids, node, hardware_manager=None):
        """
        Inicializar el comportamiento.
        
        Args:
            name: Nombre del comportamiento
            motor_ids: Lista de IDs de los motores a controlar
            node: Nodo ROS para comunicación
            hardware_manager: Manager robusto de hardware
        """
        super(EnableRobot, self).__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.logger = self.node.get_logger()
        self.hardware_manager = hardware_manager
        
        # Variables para controlar el flujo de ejecución
        self.setup_complete = False
        self.error = False
        self.available_motors = []

    def setup(self, **kwargs):
        """
        Configuración inicial del comportamiento.
        """
        self.logger.info(f"{self.name} configurando...")
        
        # Check hardware connection using hardware manager
        if self.hardware_manager is None:
            self.logger.error("Hardware manager no disponible")
            return False
        
        # Verificar motores disponibles
        available_motors = self.hardware_manager.get_available_motors()
        self.available_motors = [m for m in self.motor_ids if m in available_motors]
        
        if not self.available_motors:
            self.logger.warning("No hay motores disponibles para habilitar")
            return True  # No es un error, continuar
        
        self.logger.info(f"Motores disponibles para habilitar: {self.available_motors}")
        
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
        Obtener los motores disponibles usando Pybear.
        """
        try:
            # Test ping for each motor to check availability
            available = []
            for motor_id in self.motor_ids:
                if self.hardware_manager.ping(motor_id)[0]:
                    available.append(motor_id)
            
            if available:
                self.available_motors = available
                self.logger.info(f"Motores disponibles: {self.available_motors}")
                return True
            else:
                self.logger.warning(f"No hay motores disponibles, usando motor_ids configurados: {self.motor_ids}")
                self.available_motors = self.motor_ids
                return True
        except Exception as e:
            self.logger.error(f"Error al obtener motores disponibles: {e}")
            # Usar los motor_ids proporcionados como fallback
            self.logger.warning(f"Usando motor_ids configurados: {self.motor_ids}")
            self.available_motors = self.motor_ids
            return True
    
    
    def set_mode(self):
        """
        Configurar el modo de operación de los motores (modo 2: posición) usando Pybear.
        """
        try:
            # Set mode 2 (position mode) for all motors
            mode_pairs = [(motor_id, 2) for motor_id in self.motor_ids]
            self.hardware_manager.set_mode(*mode_pairs)
            self.logger.info(f"Modo de operación configurado para motores: {self.motor_ids}")
            return True
        except Exception as e:
            self.logger.error(f"Error al configurar modo de operación: {e}")
            self.logger.warning("Continuando de todos modos...")
            return True
    
    def enable_torque(self):
        """
        Habilitar el torque de los motores usando Pybear.
        """
        try:
            # Enable torque for all motors
            enable_pairs = [(motor_id, 1) for motor_id in self.motor_ids]
            self.hardware_manager.set_torque_enable(*enable_pairs)
            self.logger.info(f"Torque habilitado para motores: {self.motor_ids}")
            return True
        except Exception as e:
            self.logger.error(f"Error al habilitar torque: {e}")
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
        
        
        # Configurar ganancias PID como en el código de referencia
        self.logger.info("Configurando ganancias PID (P=5.0, I=0.0, D=0.2)")
        if not self.hardware_manager.configure_pid_gains(self.motor_ids, p_gain=5.0, d_gain=0.2, i_gain=0.0):
            self.logger.error("Error al configurar ganancias PID")
            self.error = True
            return py_trees.common.Status.FAILURE
        
        # Configurar modo posición y límites como en el código de referencia
        self.logger.info("Configurando modo posición y límites (iq_max=3.0)")
        if not self.hardware_manager.set_position_mode_and_limits(self.motor_ids, iq_max=3.0):
            self.logger.error("Error al configurar modo/límites")
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