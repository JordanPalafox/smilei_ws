import py_trees
import numpy as np
import rclpy
from westwood_motor_interfaces.srv import SetMotorIdAndTarget

class ZeroPosition(py_trees.behaviour.Behaviour):
    """Behaviour that sends all joints to zero position via ROS2 service"""
    def __init__(self, name: str, motor_ids: list[int], node=None):
        super().__init__(name)
        self.motor_ids = motor_ids
        self.node = node
        self.client = None
        self.future = None
        self.own_node = False

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        # Usar el nodo proporcionado en lugar de crear uno nuevo
        if self.node is None:
            self.node = rclpy.create_node('zero_position_client')
            self.own_node = True
        else:
            self.own_node = False
            
        self.client = self.node.create_client(
            SetMotorIdAndTarget,
            'westwood_motor/set_motor_id_and_target'
        )
        
        if timeout_sec is None:
            timeout_sec = 1.0
            
        return self.client.wait_for_service(timeout_sec=timeout_sec)

    def initialise(self) -> None:
        # Define zero positions for all 8 motors
        zero_pos = np.zeros(len(self.motor_ids))
        req = SetMotorIdAndTarget.Request()
        req.motor_ids = self.motor_ids
        req.target_positions = list(zero_pos)
        self.future = self.client.call_async(req)

    def update(self) -> py_trees.common.Status:
        if self.future is None:
            return py_trees.common.Status.FAILURE
        if self.future.done():
            result = self.future.result()
            return (py_trees.common.Status.SUCCESS
                    if result.success
                    else py_trees.common.Status.FAILURE)
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status) -> None:
        # Solo destruir el nodo si lo creamos nosotros mismos
        if self.own_node and self.node:
            self.node.destroy_node()
            # No rclpy.shutdown() here