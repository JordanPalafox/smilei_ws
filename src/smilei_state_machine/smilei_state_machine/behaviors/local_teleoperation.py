import py_trees
import rclpy
import time
import sys
import select
from westwood_motor_interfaces.srv import SetMotorIdAndTarget, GetMotorPositions
from westwood_motor_interfaces.srv import SetMode, SetTorqueEnable


class LocalTeleoperation(py_trees.behaviour.Behaviour):
    """Comportamiento simplificado de teleoperación local para motores configurables"""
    def __init__(self, name: str, motor_ids=None, node=None, robot_name: str = ""):
        super().__init__(name)
        self.motor_ids = motor_ids if motor_ids is not None else [1, 6]  # Motores configurables, por defecto [1, 6]
        self.node = node
        self.robot_name = robot_name
        self.running = False
        self.own_node = False
        
        # Clientes para servicios (solo necesitamos posición y configuración)
        self.set_position_client = None
        self.get_position_client = None
        self.set_mode_client = None
        self.set_torque_client = None
        
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
        
        # Crear clientes para los servicios
        self.set_position_client = self.node.create_client(
            SetMotorIdAndTarget,
            self._get_topic_name('westwood_motor/set_motor_id_and_target')
        )
        
        self.get_position_client = self.node.create_client(
            GetMotorPositions,
            self._get_topic_name('westwood_motor/get_motor_positions')
        )
        
        
        self.set_mode_client = self.node.create_client(
            SetMode,
            self._get_topic_name('westwood_motor/set_mode')
        )
        
        self.set_torque_client = self.node.create_client(
            SetTorqueEnable,
            self._get_topic_name('westwood_motor/set_torque_enable')
        )
        
        
        if timeout_sec is None:
            timeout_sec = 1.0
        
        # Intentar esperar por los servicios principales
        if not self.set_position_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("Servicio set_motor_id_and_target no disponible, continuando en modo simulación")
        
        if not self.get_position_client.wait_for_service(timeout_sec=timeout_sec):
            self.node.get_logger().warning("Servicio get_motor_positions no disponible, continuando en modo simulación")
        
        return True  # Siempre retorna True para permitir que continúe

    def zero_position(self):
        """Envía todos los motores a posición cero"""
        self.node.get_logger().info("Enviando motores a posición cero")
        
        # Crear solicitud para establecer posiciones a cero
        req = SetMotorIdAndTarget.Request()
        req.motor_ids = self.motor_ids
        req.target_positions = [0.0] * len(self.motor_ids)
        
        # Llamar al servicio
        try:
            future = self.set_position_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            if future.done():
                result = future.result()
                if not result.success:
                    self.node.get_logger().warning(f"Error al establecer posición cero: {result.message}")
                    return False
            else:
                self.node.get_logger().warning("Timeout al establecer posición cero")
                return False
            
            # Esperar a que los motores lleguen a la posición
            time.sleep(2.0)
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al establecer posición cero: {str(e)}")
            return False

    def setup_position_control(self):
        """Configura los motores para control de posición"""
        self.node.get_logger().info(f"Configurando motores {self.motor_ids} para control de posición")
        
        try:
            # Configurar modo posición (modo 2)
            req_mode = SetMode.Request()
            req_mode.motor_ids = self.motor_ids
            req_mode.modes = [2] * len(self.motor_ids)  # Todos los motores en modo posición
            
            future = self.set_mode_client.call_async(req_mode)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            # Habilitar torque
            req_torque = SetTorqueEnable.Request()
            req_torque.motor_ids = self.motor_ids
            req_torque.enable_torque = [True] * len(self.motor_ids)
            
            future = self.set_torque_client.call_async(req_torque)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al configurar motores para control de posición: {str(e)}")
            return False


    def get_motor_position(self, motor_id):
        """Obtiene la posición actual de un motor con manejo de errores mejorado"""
        req = GetMotorPositions.Request()
        req.motor_ids = [motor_id]
        
        try:
            future = self.get_position_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.1)  # Timeout más rápido
            
            if future.done():
                result = future.result()
                if result.success and len(result.positions) > 0:
                    return result.positions[0]
            
            # En caso de timeout, mantener la última posición conocida
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
        
        # Crear solicitud para mover los seguidores a la posición del líder
        try:
            req = SetMotorIdAndTarget.Request()
            req.motor_ids = list(self.follower_motors)
            req.target_positions = [leader_avg_position] * len(self.follower_motors)
            
            future = self.set_position_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.05)  # Timeout rápido
            
            if future.done():
                result = future.result()
                if not result.success:
                    self.node.get_logger().warning(f"Error al enviar posiciones: {result.message}")
                    return False
            else:
                self.node.get_logger().warning("Timeout enviando posiciones")
                return False
            
            return True
                
        except Exception as e:
            self.node.get_logger().warning(f"Excepción enviando posiciones: {str(e)}")
            return False
    


    def restore_position_control(self):
        """Restaura el control de posición de los motores"""
        self.node.get_logger().info(f"Restaurando control de posición para motores {self.motor_ids}")
        
        try:
            # Configurar modo posición (modo 2)
            req_mode = SetMode.Request()
            req_mode.motor_ids = self.motor_ids
            req_mode.modes = [2] * len(self.motor_ids)  # Todos los motores en modo posición
            
            future = self.set_mode_client.call_async(req_mode)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            time.sleep(1.0)
            
            # Enviar a posición home (1.5707 rad ≈ 90 grados)
            home_req = SetMotorIdAndTarget.Request()
            home_req.motor_ids = self.motor_ids
            home_req.target_positions = [1.5707] * len(self.motor_ids)
            
            future = self.set_position_client.call_async(home_req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
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