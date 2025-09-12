import py_trees
import numpy as np
import rclpy
import math
import time
from pybear import Manager

class SayHello(py_trees.behaviour.Behaviour):
    """Comportamiento que mueve los motores en un patrón sinusoidal."""
    def __init__(self, name: str, motor_ids: list[int], node=None, robot_name: str = "", hardware_manager=None):
        super().__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.robot_name = robot_name
        self.running = False
        self.own_node = False
        self.last_time = time.time()
        self.zero_position_sent = False
        
        # Use hardware manager instead of direct Pybear
        self.hardware_manager = hardware_manager

    def _get_topic_name(self, topic_name):
        if self.robot_name:
            return f'/{self.robot_name}/{topic_name.lstrip("/")}'
        return topic_name

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        # Usar el nodo proporcionado en lugar de crear uno nuevo
        if self.node is None:
            self.node = rclpy.create_node('say_hello_client')
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

    def initialise(self) -> None:
        self.node.get_logger().info("Iniciando comportamiento Say Hello")
        self.running = True
        self.last_time = time.time()
        self.zero_position_sent = False

    def update(self) -> py_trees.common.Status:
        # Si no está corriendo, devolver SUCCESS para permitir la transición al siguiente estado
        if not self.running:
            return py_trees.common.Status.SUCCESS
        
        # El comportamiento se controla a través de comandos de estado, no entrada de usuario
        
        # Primero enviar todos los motores a posición cero como en el código original
        if not self.zero_position_sent:
            self.node.get_logger().info("Say hello: primero enviando todos los motores a posición cero")
            
            # En el código original, say_hello() llama a zero_position() completo
            # Aquí hacemos lo mismo pero de forma simplificada
            try:
                # Send all motors to zero position using hardware manager
                position_pairs = [(motor_id, 0.0) for motor_id in self.motor_ids]
                if self.hardware_manager:
                    self.hardware_manager.set_goal_position(*position_pairs)
                else:
                    self.node.get_logger().debug("[SIM] Enviando motores a posición cero")
                
                # Esperar un poco para que lleguen a cero
                time.sleep(1.0)
                self.zero_position_sent = True
                self.node.get_logger().info("Motores en posición cero, iniciando movimiento sinusoidal")
            except Exception as e:
                self.node.get_logger().error(f"Error al enviar a posición cero: {str(e)}")
                return py_trees.common.Status.FAILURE
        
        # Una vez en cero, proceder con el movimiento sinusoidal
        # Calcular posición sinusoidal como en PyBEAR
        seconds = time.time()
        qd = 0.22 * math.sin(12.0 * seconds) - 1
        
        # En el código original: bear_r.set_goal_position((m_id_4, qd)) y bear_l.set_goal_position((m_id_8, qd))
        # Es decir, solo mueve el motor 4 (último del brazo derecho) y motor 8 (último del brazo izquierdo)
        # Como tenemos motor_ids [1,2], solo moveremos el motor 2 (que sería el equivalente al m_id_4)
        motor_ids_to_move = []
        target_positions = []
        
        if len(self.motor_ids) == 2:
            # Para 2 motores, mover solo el segundo (equivalente a m_id_4 del brazo derecho)
            motor_ids_to_move = [self.motor_ids[1]]  # motor ID 2
            target_positions = [qd]
        elif len(self.motor_ids) >= 4:
            # Para configuración completa (8 motores), mover motores 4 y 8 (últimos de cada brazo)
            motor_ids_to_move = [self.motor_ids[3], self.motor_ids[7]]  # motores 4 y 8
            target_positions = [qd, qd]
        elif len(self.motor_ids) >= 1:
            # Si hay menos motores, usar el último disponible
            motor_ids_to_move = [self.motor_ids[-1]]
            target_positions = [qd]
        
        # Enviar comandos solo a los motores específicos
        if motor_ids_to_move:
            try:
                # Send position commands using hardware manager
                position_pairs = [(motor_id, pos) for motor_id, pos in zip(motor_ids_to_move, target_positions)]
                if self.hardware_manager:
                    self.hardware_manager.set_goal_position(*position_pairs)
                    
                    # Show actual positions every 50 cycles (every 0.5 seconds) to avoid spam
                    current_time = time.time()
                    if not hasattr(self, 'last_position_log_time'):
                        self.last_position_log_time = current_time
                    
                    if current_time - self.last_position_log_time >= 0.5:
                        actual_positions = self.hardware_manager.get_present_position(*self.motor_ids)
                        self.node.get_logger().info(f"Say Hello - Objetivo={[f'{pos:.4f}' for pos in target_positions]}, Real motor {motor_ids_to_move[0]}={actual_positions[motor_ids_to_move[0]-1]:.4f}")
                        self.last_position_log_time = current_time
                        
                else:
                    self.node.get_logger().debug(f"[SIM] Say hello moviendo: {position_pairs}")
            except Exception as e:
                self.node.get_logger().error(f"Error al establecer posiciones: {str(e)}")
        else:
            self.node.get_logger().warning("No hay motores configurados para say_hello")
        
        # Pausa de 0.01 segundos como en PyBEAR
        time.sleep(0.01)
        
        # Siempre devuelve RUNNING mientras está activo
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.node.get_logger().info(f"Terminando comportamiento Say Hello con estado {new_status}")
        self.running = False
        
        # Solo destruir el nodo si lo creamos nosotros mismos
        if self.own_node and self.node:
            self.node.destroy_node() 