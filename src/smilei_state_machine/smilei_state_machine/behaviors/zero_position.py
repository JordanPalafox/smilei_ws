import py_trees
import numpy as np
import time

class ZeroPosition(py_trees.behaviour.Behaviour):
    """Behaviour that moves all joints to zero position with gradual interpolation"""
    def __init__(self, name: str, motor_ids: list[int], node=None, hardware_manager=None):
        super().__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.hardware_manager = hardware_manager
        self.available_motors = []
        
        # Movement parameters
        self.num_steps = 100  # Same as original code
        self.current_step = 0
        self.target_positions = None
        self.initial_positions = None
        self.delta_angles = None
        self.movement_completed = False

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        # Check hardware connection using hardware manager
        if self.hardware_manager is not None:
            available_motors = self.hardware_manager.get_available_motors()
            self.available_motors = [m for m in self.motor_ids if m in available_motors]
            if self.available_motors:
                self.node.get_logger().info(f"Hardware conectado - motores disponibles: {self.available_motors}")
                return True
            else:
                self.node.get_logger().warning("No hay motores disponibles")
                return True  # Continue in simulation mode
        else:
            self.node.get_logger().warning("Hardware manager no disponible - modo simulación")
            return True  # Continue in simulation mode

    def initialise(self) -> None:
        self.node.get_logger().info(f"Iniciando movimiento a posición cero")
        
        # Reset movement state
        self.current_step = 0
        self.movement_completed = False
        
        # Target positions are all zeros (same as original code)
        self.target_positions = np.zeros(len(self.motor_ids))
        
        # Get current positions
        if self.hardware_manager and self.hardware_manager.hardware_connected:
            try:
                current_pos = self.hardware_manager.get_present_position(*self.motor_ids)
                self.initial_positions = np.array(current_pos)
                self.node.get_logger().info(f"Posiciones actuales: {self.initial_positions}")
                self.node.get_logger().info(f"Posiciones objetivo: {self.target_positions}")
            except Exception as e:
                self.node.get_logger().error(f"Error leyendo posiciones actuales: {e}")
                self.initial_positions = np.zeros(len(self.motor_ids))
        else:
            # Simulation mode - assume starting from some position
            self.initial_positions = np.array([0.5, 0.5] + [0.0] * (len(self.motor_ids) - 2))
            self.node.get_logger().info("[SIM] Iniciando movimiento gradual a cero")
        
        # Calculate delta angles for interpolation
        self.delta_angles = (self.target_positions - self.initial_positions) / self.num_steps
        self.node.get_logger().info(f"Delta por paso: {self.delta_angles}")

    def update(self) -> py_trees.common.Status:
        if self.movement_completed:
            return py_trees.common.Status.SUCCESS
        
        # Calculate current goal position
        goal_positions = self.initial_positions + self.delta_angles * (self.current_step + 1)
        
        # Send positions to hardware
        if self.hardware_manager:
            try:
                position_pairs = [(motor_id, pos) for motor_id, pos in zip(self.motor_ids, goal_positions)]
                self.hardware_manager.set_goal_position(*position_pairs)
                
                # Read and display actual positions
                actual_positions = self.hardware_manager.get_present_position(*self.motor_ids)
                self.node.get_logger().info(f"Paso {self.current_step + 1}/{self.num_steps}: Objetivo={[f'{pos:.4f}' for pos in goal_positions]}, Real={[f'{pos:.4f}' for pos in actual_positions]}")
                
            except Exception as e:
                self.node.get_logger().error(f"Error enviando posiciones: {e}")
                return py_trees.common.Status.FAILURE
        else:
            self.node.get_logger().debug(f"[SIM] Paso {self.current_step + 1}/{self.num_steps}: {goal_positions}")
        
        # Wait same as original code
        time.sleep(0.01)
        
        # Increment step
        self.current_step += 1
        
        # Check if movement is complete
        if self.current_step >= self.num_steps:
            self.movement_completed = True
            # Show final positions
            if self.hardware_manager:
                final_positions = self.hardware_manager.get_present_position(*self.motor_ids)
                self.node.get_logger().info(f"Movimiento a ZERO completado. Posiciones finales: {[f'{pos:.4f}' for pos in final_positions]}")
            return py_trees.common.Status.SUCCESS
        
        # Continue movement
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info("Zero position alcanzada exitosamente")
        else:
            self.node.get_logger().warning(f"Zero position terminado con estado: {new_status}")