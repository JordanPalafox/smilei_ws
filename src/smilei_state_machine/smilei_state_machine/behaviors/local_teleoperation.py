import py_trees
import rclpy
import time
import sys
import select


class LocalTeleoperation(py_trees.behaviour.Behaviour):
    """Comportamiento simplificado de teleoperación local para motores configurables"""
    def __init__(self, name: str, motor_ids=None, node=None, robot_name: str = "", hardware_manager=None):
        super().__init__(name)
        self.motor_ids = motor_ids if motor_ids is not None else [1, 6]  # Motores configurables, por defecto [1, 6]
        self.node = node
        self.robot_name = robot_name
        self.running = False
        self.own_node = False
        
        # Use hardware manager instead of direct Pybear
        self.hardware_manager = hardware_manager
        self.available_motors = []
        
        # No necesitamos parámetros de control PID - usamos control por posición directo
        
        # Variables para seguimiento de posiciones (dinámicas según motor_ids)
        self.prev_positions = {motor_id: None for motor_id in self.motor_ids}
        
        # Definir líderes y seguidores fijos basados en la primera mitad de motor_ids
        mid_point = len(self.motor_ids) // 2
        self.leader_motors = self.motor_ids[:mid_point] if mid_point > 0 else [self.motor_ids[0]]
        self.follower_motors = self.motor_ids[mid_point:]
        
        # Variables para manejo de errores de comunicación
        self.communication_error_count = 0
        self.max_communication_errors = 10  # Máximo número de errores antes de salir

    def _get_topic_name(self, topic_name):
        if self.robot_name:
            return f'/{self.robot_name}/{topic_name.lstrip("/")}'
        return topic_name

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        # Usar el nodo proporcionado en lugar de crear uno nuevo
        if self.node is None:
            self.node = rclpy.create_node('local_teleoperation_client')
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

    def zero_position(self):
        """Envía todos los motores a posición cero usando hardware_manager"""
        self.node.get_logger().info("Enviando motores a posición cero")
        
        try:
            if self.hardware_manager:
                # Send all motors to zero position
                position_pairs = [(motor_id, 0.0) for motor_id in self.motor_ids]
                self.hardware_manager.set_goal_position(*position_pairs)
            else:
                self.node.get_logger().debug("[SIM] Enviando motores a posición cero")
            
            # Esperar a que los motores lleguen a la posición
            time.sleep(2.0)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al establecer posición cero: {str(e)}")
            return False

    def setup_position_control(self):
        """Configura los motores para control de posición usando hardware_manager"""
        self.node.get_logger().info(f"Configurando motores {self.motor_ids} para control de posición")
        
        try:
            if self.hardware_manager:
                # Configurar modo posición (modo 2)
                self.hardware_manager.set_mode(*[(motor_id, 2) for motor_id in self.motor_ids])
                
                # Habilitar torque
                self.hardware_manager.set_torque_enable(*[(motor_id, 1) for motor_id in self.motor_ids])
            else:
                self.node.get_logger().debug("[SIM] Configurando motores para control de posición")
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al configurar motores para control de posición: {str(e)}")
            return False


    def get_motor_position(self, motor_id):
        """Obtiene la posición actual de un motor usando hardware_manager"""
        try:
            if self.hardware_manager:
                position = self.hardware_manager.get_present_position(motor_id)[0][0][0]
                return position
            else:
                # Simulation mode - return a stable position value
                if motor_id in self.prev_positions and self.prev_positions[motor_id] is not None:
                    return self.prev_positions[motor_id]
                return 0.0
        except Exception as e:
            self.node.get_logger().debug(f"Error al obtener posición del motor {motor_id}: {str(e)}")
            # Retornar última posición conocida en caso de error
            if motor_id in self.prev_positions and self.prev_positions[motor_id] is not None:
                return self.prev_positions[motor_id]
            return 0.0


    def initialise(self) -> None:
        self.node.get_logger().info(f"Iniciando teleoperación local para motores {self.motor_ids}")
        self.node.get_logger().info(f"🏅 Líderes (libres): {self.leader_motors}")
        self.node.get_logger().info(f"🎯 Seguidores: {self.follower_motors}")
        self.node.get_logger().info("🎮 Control: [ENTER] o cualquier tecla para terminar")
        self.running = True
        
        # Inicializar variables de seguimiento
        self.prev_positions = {motor_id: None for motor_id in self.motor_ids}
        
        # Configuración inicial
        self.zero_position()
        self.setup_position_control()

    def update(self) -> py_trees.common.Status:
        if not self.running:
            return py_trees.common.Status.SUCCESS
        
        # Verificar si hay demasiados errores de comunicación
        if self.communication_error_count >= self.max_communication_errors:
            self.node.get_logger().error(f"Demasiados errores de comunicación ({self.communication_error_count}). Terminando teleoperación.")
            self.running = False
            return py_trees.common.Status.FAILURE
        
        # Verificar entrada del usuario para terminar
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            self.node.get_logger().info("📋 Terminando teleoperación local")
            self.running = False
            return py_trees.common.Status.SUCCESS
        
        try:
            success = self._update_teleoperation()
            if success:
                # Reset del contador si la operación fue exitosa
                self.communication_error_count = max(0, self.communication_error_count - 1)
                time.sleep(0.1)  # Ciclo más lento para control de posición (10Hz)
            else:
                # Incrementar contador de errores
                self.communication_error_count += 1
                time.sleep(0.2)  # Esperar más tiempo después de un error
            
        except Exception as e:
            self.node.get_logger().error(f"Error en teleoperación: {str(e)}")
            self.communication_error_count += 1
            time.sleep(0.05)
        
        return py_trees.common.Status.RUNNING

    def _update_teleoperation(self):
        """Teleoperación con líderes fijos usando control por posición directo
        
        Returns:
            bool: True si la operación fue exitosa, False si hubo errores
        """
        # Obtener posiciones actuales de todos los motores
        current_positions = {}
        for motor_id in self.motor_ids:
            current_positions[motor_id] = self.get_motor_position(motor_id)
        
        # Calcular posición objetivo para los seguidores
        success = self._send_position_commands(current_positions)
        
        # Debug simplificado
        pos_str = ", ".join([f"M{mid}={pos:.3f}" for mid, pos in current_positions.items()])
        
        if len(self.leader_motors) > 0 and len(self.follower_motors) > 0:
            leader_avg = sum(current_positions[lid] for lid in self.leader_motors) / len(self.leader_motors)
            errors = [current_positions[fid] - leader_avg for fid in self.follower_motors]
            error_str = ", ".join([f"E{fid}={err:.3f}" for fid, err in zip(self.follower_motors, errors)])
            self.node.get_logger().debug(f"Pos: {pos_str} | Errores: {error_str} | Target: {leader_avg:.3f}")
        else:
            self.node.get_logger().debug(f"Pos: {pos_str} | Líderes: {self.leader_motors}")
        
        # Actualizar posiciones anteriores solo si la operación fue exitosa
        if success:
            for motor_id in self.motor_ids:
                self.prev_positions[motor_id] = current_positions[motor_id]
        
        return success
    
    
    def _send_position_commands(self, current_positions):
        """Envía comandos de posición a los motores seguidores
        
        Líderes (primera mitad): libres para mover manualmente
        Seguidores (segunda mitad): siguen la posición promedio de los líderes
        
        Returns:
            bool: True si la operación fue exitosa, False si hubo errores
        """
        # Si solo hay un motor o no hay seguidores, no hacer nada
        if len(self.motor_ids) == 1 or len(self.follower_motors) == 0 or len(self.leader_motors) == 0:
            return True
        
        # Calcular posición promedio de los líderes
        leader_avg_position = sum(current_positions[lid] for lid in self.leader_motors) / len(self.leader_motors)
        
        # Solo enviar comando si hay un cambio significativo (evita spam de comandos)
        significant_error = False
        for follower_id in self.follower_motors:
            error = abs(current_positions[follower_id] - leader_avg_position)
            if error > 0.01:  # Umbral de 0.01 radianes (~0.6 grados)
                significant_error = True
                break
        
        if not significant_error:
            return True
        
        # Enviar comandos de posición usando hardware_manager
        try:
            if self.hardware_manager:
                position_pairs = [(motor_id, leader_avg_position) for motor_id in self.follower_motors]
                self.hardware_manager.set_goal_position(*position_pairs)
            else:
                self.node.get_logger().debug(f"[SIM] Enviando posiciones: {[(motor_id, leader_avg_position) for motor_id in self.follower_motors]}")
            return True
        except Exception as e:
            self.node.get_logger().warning(f"Excepción enviando posiciones: {str(e)}")
            return False
    


    def restore_position_control(self):
        """Restaura el control de posición de los motores usando hardware_manager"""
        self.node.get_logger().info(f"Restaurando control de posición para motores {self.motor_ids}")
        
        try:
            if self.hardware_manager:
                # Configurar modo posición (modo 2)
                self.hardware_manager.set_mode(*[(motor_id, 2) for motor_id in self.motor_ids])
                
                time.sleep(1.0)
                
                # Enviar a posición home (1.5707 rad ≈ 90 grados)
                self.hardware_manager.set_goal_position(*[(motor_id, 1.5707) for motor_id in self.motor_ids])
            else:
                self.node.get_logger().debug("[SIM] Restaurando control de posición")
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al restaurar control de posición: {str(e)}")
            return False

    def terminate(self, new_status: py_trees.common.Status) -> None:
        self.node.get_logger().info(f"Terminando teleoperación local con estado {new_status}")
        self.running = False
        
        # Restaurar control de posición antes de terminar
        self.restore_position_control()
        
        # Solo destruir el nodo si lo creamos nosotros mismos
        if self.own_node and self.node:
            self.node.destroy_node() 