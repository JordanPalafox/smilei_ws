# Westwood Motor Interfaces

Este paquete contiene las interfaces ROS2 personalizadas para el control de motores Westwood.

## Mensajes

- **MotorStatus.msg**: Representa el estado completo de un motor Westwood
- **MotorConfig.msg**: Define los parámetros de configuración de un motor Westwood

## Servicios

- **SetMotorPosition.srv**: Servicio para establecer la posición objetivo del motor
- **SetMotorVelocity.srv**: Servicio para establecer la velocidad objetivo del motor
- **SetMotorCurrent.srv**: Servicio para establecer la corriente objetivo del motor
- **GetMotorStatus.srv**: Servicio para obtener el estado completo del motor
- **ConfigureMotor.srv**: Servicio para configurar los parámetros del motor
- **SetMotorIdsAndTarget.srv**: Servicio para controlar múltiples motores con una misma posición objetivo

## Acciones

- **MoveToPosition.action**: Acción para mover el motor a una posición de forma suave con retroalimentación
- **RunVelocityProfile.action**: Acción para ejecutar un perfil de velocidad en función del tiempo

## Uso

Para usar estas interfaces en tu paquete ROS2, añade una dependencia en tu `package.xml`:

```xml
<depend>westwood_motor_interfaces</depend>
```

Y en tu CMakeLists.txt:

```cmake
find_package(westwood_motor_interfaces REQUIRED)
```

## Ejemplos

### Usando el servicio de posición

```python
from westwood_motor_interfaces.srv import SetMotorPosition
import rclpy
from rclpy.node import Node

class MotorClient(Node):
    def __init__(self):
        super().__init__('motor_client')
        self.cli = self.create_client(SetMotorPosition, 'westwood_motor/set_position')
        
    def send_position_request(self, motor_id, position, smooth=True, steps=100):
        request = SetMotorPosition.Request()
        request.motor_id = motor_id
        request.position = position
        request.smooth_motion = smooth
        request.steps = steps
        return self.cli.call_async(request)
```

### Usando la acción MoveToPosition

```python
from westwood_motor_interfaces.action import MoveToPosition
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

class MotorActionClient(Node):
    def __init__(self):
        super().__init__('motor_action_client')
        self.action_client = ActionClient(
            self, MoveToPosition, 'westwood_motor/move_to_position')
        
    def send_goal(self, motor_id, position):
        goal_msg = MoveToPosition.Goal()
        goal_msg.motor_id = motor_id
        goal_msg.target_position = position
        goal_msg.steps = 100
        goal_msg.max_velocity = 1.0
        goal_msg.max_acceleration = 0.5
        
        self.action_client.wait_for_server()
        return self.action_client.send_goal_async(goal_msg)
```

### Controlando múltiples motores con una misma posición objetivo

```python
from westwood_motor_interfaces.srv import SetMotorIdsAndTarget
import rclpy
from rclpy.node import Node

class MultiMotorClient(Node):
    def __init__(self):
        super().__init__('multi_motor_client')
        self.cli = self.create_client(
            SetMotorIdsAndTarget, 
            'westwood_motor/set_motor_ids_and_target'
        )
        
    def send_multi_motor_request(self, motor_ids, target_position):
        request = SetMotorIdsAndTarget.Request()
        request.motor_ids = motor_ids          # Lista de IDs de motores
        request.target_position = target_position
        request.smooth_motion = True
        request.steps = 100
        request.configure_pid = True
        request.p_gain = 5.0
        request.i_gain = 0.0
        request.d_gain = 0.2
        request.iq_max = 1.5
        
        self.get_logger().info(f"Enviando petición para mover motores {motor_ids} a posición {target_position}")
        return self.cli.call_async(request)
``` 