import py_trees
import rclpy
import time
import sys
import select
from westwood_motor_interfaces.srv import SetMotorIdAndTarget, GetMotorPositions
from westwood_motor_interfaces.srv import SetGains, SetMode, SetTorqueEnable
from westwood_motor_interfaces.srv import SetGoalIq

class CurrentControlGains:
    """
    Ganancias simplificadas para control de corriente
    """
    p_gain_position = 0.0
    d_gain_position = 0.0
    i_gain_position = 0.0
    p_gain_force = 0.0
    d_gain_force = 0.0
    i_gain_force = 0.0
    iq_max = 2.0
    p_gain_iq = 1.0  # Ganancia P baja para fine-tuning
    i_gain_iq = 0.0   # Sin componente integral
    d_gain_iq = 0.0   # Sin componente derivativa
    p_gain_id = 0.05
    i_gain_id = 0.0
    d_gain_id = 0.0
    kt = 0.35

class PositionControlGains:
    """
    Ganancias para control de posición
    """
    p_gain_position = 2.0
    d_gain_position = 0.1
    i_gain_position = 0.0
    iq_max = 1.5
    p_gain_iq = 0.02
    i_gain_iq = 0.0
    d_gain_iq = 0.0
    p_gain_id = 0.02
    i_gain_id = 0.0
    d_gain_id = 0.0
    kt = 0.35

class LocalTeleoperation(py_trees.behaviour.Behaviour):
    """Comportamiento simplificado de teleoperación local para 2 motores (IDs 1 y 6)"""
    def __init__(self, name: str, node=None):
        super().__init__(name)
        self.motor_ids = [1, 6]  # Motores fijos ID 1 y 6
        self.node = node
        self.running = False
        self.own_node = False
        
        # Clientes para servicios
        self.set_position_client = None
        self.get_position_client = None
        self.set_gains_client = None
        self.set_mode_client = None
        self.set_torque_client = None
        self.set_goal_iq_client = None
        
        # Parámetros de control simplificados
        self.kp = 8.0  # Ganancia proporcional alta para respuesta rápida
        self.Kt = CurrentControlGains.kt
        
        # Parámetros de detección de movimiento
        self.movement_threshold = 0.008  # Umbral más sensible para detección rápida
        self.max_current = 2.5  # Corriente máxima aumentada para mejor torque
        
        # Variables para seguimiento de posiciones
        self.prev_positions = {1: None, 6: None}
        self.last_moved_motor = None
        self.leader_lock_time = 0.0  # Tiempo para mantener el líder
        self.leader_lock_duration = 0.2  # Duración reducida para cambios más rápidos

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
            'westwood_motor/set_motor_id_and_target'
        )
        
        self.get_position_client = self.node.create_client(
            GetMotorPositions,
            'westwood_motor/get_motor_positions'
        )
        
        self.set_gains_client = self.node.create_client(
            SetGains,
            'westwood_motor/set_position_gains'
        )
        
        self.set_mode_client = self.node.create_client(
            SetMode,
            'westwood_motor/set_mode'
        )
        
        self.set_torque_client = self.node.create_client(
            SetTorqueEnable,
            'westwood_motor/set_torque_enable'
        )
        
        self.set_goal_iq_client = self.node.create_client(
            SetGoalIq,
            'westwood_motor/set_goal_iq'
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

    def setup_current_control(self):
        """Configura los motores para control de corriente con ganancias simplificadas"""
        self.node.get_logger().info("Configurando motores 1 y 6 para control de corriente")
        
        try:
            req_gains = SetGains.Request()
            req_gains.motor_ids = self.motor_ids
            req_gains.p_gain_position = float(CurrentControlGains.p_gain_position)
            req_gains.i_gain_position = float(CurrentControlGains.i_gain_position)
            req_gains.d_gain_position = float(CurrentControlGains.d_gain_position)
            req_gains.p_gain_iq = float(CurrentControlGains.p_gain_iq)
            req_gains.i_gain_iq = float(CurrentControlGains.i_gain_iq)
            req_gains.d_gain_iq = float(CurrentControlGains.d_gain_iq)
            req_gains.p_gain_id = float(CurrentControlGains.p_gain_id)
            req_gains.i_gain_id = float(CurrentControlGains.i_gain_id)
            req_gains.d_gain_id = float(CurrentControlGains.d_gain_id)
            req_gains.p_gain_force = float(CurrentControlGains.p_gain_force)
            req_gains.i_gain_force = float(CurrentControlGains.i_gain_force)
            req_gains.d_gain_force = float(CurrentControlGains.d_gain_force)
            req_gains.iq_max = float(CurrentControlGains.iq_max)
            req_gains.kt = float(CurrentControlGains.kt)
            
            future = self.set_gains_client.call_async(req_gains)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            # Configurar modo corriente (modo 0)
            req_mode = SetMode.Request()
            req_mode.motor_ids = self.motor_ids
            req_mode.modes = [0, 0]  # Ambos motores en modo corriente
            
            future = self.set_mode_client.call_async(req_mode)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            # Habilitar torque
            req_torque = SetTorqueEnable.Request()
            req_torque.motor_ids = self.motor_ids
            req_torque.enable_torque = [True, True]
            
            future = self.set_torque_client.call_async(req_torque)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            return True
        except Exception as e:
            self.node.get_logger().error(f"Error al configurar motores para control de corriente: {str(e)}")
            return False


    def get_motor_position(self, motor_id):
        """Obtiene la posición actual de un motor"""
        req = GetMotorPositions.Request()
        req.motor_ids = [motor_id]
        
        try:
            future = self.get_position_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
            
            if future.done():
                result = future.result()
                if result.success and len(result.positions) > 0:
                    return result.positions[0]
            
            return 0.0
        except Exception as e:
            self.node.get_logger().error(f"Error al obtener posición del motor {motor_id}: {str(e)}")
            return 0.0


    def initialise(self) -> None:
        self.node.get_logger().info("Iniciando teleoperación local para motores 1 y 6")
        self.running = True
        
        # Inicializar variables de seguimiento
        self.prev_positions = {1: None, 6: None}
        self.last_moved_motor = None
        
        # Configuración inicial
        self.zero_position()
        self.setup_current_control()

    def update(self) -> py_trees.common.Status:
        if not self.running:
            return py_trees.common.Status.SUCCESS
        
        # Verificar entrada del usuario para terminar
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip()
            if line:
                self.node.get_logger().info("Terminando teleoperación local")
                self.running = False
                return py_trees.common.Status.SUCCESS
        
        try:
            self._update_teleoperation()
            time.sleep(0.005)  # Ciclo de 200Hz para mejor respuesta
            
        except Exception as e:
            self.node.get_logger().error(f"Error en teleoperación: {str(e)}")
        
        return py_trees.common.Status.RUNNING

    def _update_teleoperation(self):
        """Teleoperación simplificada y robusta entre motores 1 y 6"""
        # Obtener posiciones actuales
        q1 = self.get_motor_position(1)
        q6 = self.get_motor_position(6)
        
        # Detectar cuál motor se está moviendo
        moved_motor = self._detect_moved_motor(q1, q6)
        
        # Calcular corrientes de control
        iq1, iq6 = self._calculate_control_currents(q1, q6, moved_motor)
        
        # Debug reducido para mejor rendimiento
        if abs(iq1) > 0.1 or abs(iq6) > 0.1:
            self.node.get_logger().debug(f"Pos: M1={q1:.2f}, M6={q6:.2f} | Corrientes: i1={iq1:.2f}A, i6={iq6:.2f}A | Líder: {moved_motor}")
        
        # Enviar comandos de corriente
        self._send_current_commands(iq1, iq6)
        
        # Actualizar posiciones anteriores
        self.prev_positions[1] = q1
        self.prev_positions[6] = q6
    
    def _detect_moved_motor(self, q1, q6):
        """Detecta cuál motor se está moviendo activamente con histéresis"""
        current_time = time.time()
        
        if self.prev_positions[1] is None or self.prev_positions[6] is None:
            return None
        
        delta1 = abs(q1 - self.prev_positions[1])
        delta6 = abs(q6 - self.prev_positions[6])
        
        # Debug reducido
        if delta1 > self.movement_threshold or delta6 > self.movement_threshold:
            self.node.get_logger().debug(f"Deltas - M1: {delta1:.4f}, M6: {delta6:.4f}")
        
        # Si estamos en período de bloqueo, mantener el líder actual
        if self.last_moved_motor is not None and current_time < self.leader_lock_time:
            self.node.get_logger().debug(f"Manteniendo líder bloqueado: Motor {self.last_moved_motor}")
            return self.last_moved_motor
        
        # Factor de histéresis: el nuevo líder debe moverse significativamente más
        hysteresis_factor = 1.5  # Reducido para permitir cambios más rápidos
        
        # Determinar motor líder con histéresis
        if self.last_moved_motor == 1:
            # Para cambiar a motor 6, debe moverse significativamente más
            if delta6 > self.movement_threshold and delta6 > delta1 * hysteresis_factor:
                self.last_moved_motor = 6
                self.leader_lock_time = current_time + self.leader_lock_duration
                self.node.get_logger().debug(f"Motor 6 nuevo líder (delta: {delta6:.4f})")
                return 6
            elif delta1 > self.movement_threshold:
                # Mantener motor 1 como líder
                self.leader_lock_time = current_time + self.leader_lock_duration
                return 1
        elif self.last_moved_motor == 6:
            # Para cambiar a motor 1, debe moverse significativamente más
            if delta1 > self.movement_threshold and delta1 > delta6 * hysteresis_factor:
                self.last_moved_motor = 1
                self.leader_lock_time = current_time + self.leader_lock_duration
                self.node.get_logger().debug(f"Motor 1 nuevo líder (delta: {delta1:.4f})")
                return 1
            elif delta6 > self.movement_threshold:
                # Mantener motor 6 como líder
                self.leader_lock_time = current_time + self.leader_lock_duration
                return 6
        else:
            # Sin líder previo, usar detección simple
            if delta1 > self.movement_threshold and delta1 > delta6:
                self.last_moved_motor = 1
                self.leader_lock_time = current_time + self.leader_lock_duration
                self.node.get_logger().debug(f"Motor 1 líder inicial (delta: {delta1:.4f})")
                return 1
            elif delta6 > self.movement_threshold and delta6 > delta1:
                self.last_moved_motor = 6
                self.leader_lock_time = current_time + self.leader_lock_duration
                self.node.get_logger().debug(f"Motor 6 líder inicial (delta: {delta6:.4f})")
                return 6
        
        # Mantener último motor líder si no hay cambio claro
        return self.last_moved_motor
    
    def _calculate_control_currents(self, q1, q6, moved_motor):
        """Calcula las corrientes de control para ambos motores"""
        # Error de posición para seguimiento directo (misma dirección)
        error_1_follows_6 = q1 - q6  # Motor 1 sigue al 6 en la misma dirección
        error_6_follows_1 = q6 - q1  # Motor 6 sigue al 1 en la misma dirección
        
        # Calcular corrientes base con control proporcional
        iq1_base = -self.kp * error_1_follows_6
        iq6_base = -self.kp * error_6_follows_1
        
        # Aplicar lógica de líder/seguidor
        if moved_motor == 1:
            # Motor 1 es líder, motor 6 sigue
            iq1 = 0.0  # Sin resistencia al movimiento manual
            iq6 = iq6_base
            pass
        elif moved_motor == 6:
            # Motor 6 es líder, motor 1 sigue
            iq1 = iq1_base
            iq6 = 0.0  # Sin resistencia al movimiento manual
            pass
        else:
            # Sin movimiento claro, aplicar control suave mutuo
            iq1 = iq1_base * 0.5
            iq6 = iq6_base * 0.5
            pass
        
        # Limitar corrientes para seguridad
        iq1_limited = max(min(iq1, self.max_current), -self.max_current)
        iq6_limited = max(min(iq6, self.max_current), -self.max_current)
        
        # Solo mostrar si hay limitación significativa
        if abs(iq1 - iq1_limited) > 0.1 or abs(iq6 - iq6_limited) > 0.1:
            self.node.get_logger().debug(f"Corrientes limitadas: {iq1:.3f}→{iq1_limited:.3f}, {iq6:.3f}→{iq6_limited:.3f}")
        
        return iq1_limited, iq6_limited
    
    def _send_current_commands(self, iq1, iq6):
        """Envía comandos de corriente a los motores"""
        req = SetGoalIq.Request()
        req.motor_ids = [1, 6]
        req.goal_iq = [iq1, iq6]
        
        try:
            future = self.set_goal_iq_client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.02)
            
            if future.done():
                result = future.result()
                if not result.success:
                    self.node.get_logger().error(f"Error al enviar corrientes: {result.message}")
            else:
                self.node.get_logger().warning("Timeout enviando corrientes")
                
        except Exception as e:
            self.node.get_logger().error(f"Excepción enviando corrientes: {str(e)}")


    def restore_position_control(self):
        """Restaura el control de posición de los motores"""
        self.node.get_logger().info("Restaurando control de posición para motores 1 y 6")
        
        try:
            req_gains = SetGains.Request()
            req_gains.motor_ids = self.motor_ids
            req_gains.p_gain_position = float(PositionControlGains.p_gain_position)
            req_gains.i_gain_position = float(PositionControlGains.i_gain_position)
            req_gains.d_gain_position = float(PositionControlGains.d_gain_position)
            req_gains.p_gain_iq = float(PositionControlGains.p_gain_iq)
            req_gains.i_gain_iq = float(PositionControlGains.i_gain_iq)
            req_gains.d_gain_iq = float(PositionControlGains.d_gain_iq)
            req_gains.p_gain_id = float(PositionControlGains.p_gain_id)
            req_gains.i_gain_id = float(PositionControlGains.i_gain_id)
            req_gains.d_gain_id = float(PositionControlGains.d_gain_id)
            req_gains.iq_max = float(PositionControlGains.iq_max)
            req_gains.kt = float(PositionControlGains.kt)
            
            future = self.set_gains_client.call_async(req_gains)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            # Configurar modo posición (modo 2)
            req_mode = SetMode.Request()
            req_mode.motor_ids = self.motor_ids
            req_mode.modes = [2, 2]  # Ambos motores en modo posición
            
            future = self.set_mode_client.call_async(req_mode)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
            
            time.sleep(1.0)
            
            # Enviar a posición home (1.5707 rad ≈ 90 grados)
            home_req = SetMotorIdAndTarget.Request()
            home_req.motor_ids = self.motor_ids
            home_req.target_positions = [1.5707, 1.5707]
            
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