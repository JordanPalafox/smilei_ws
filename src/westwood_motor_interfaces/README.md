# Westwood Motor Interfaces

Este paquete contiene las interfaces ROS2 personalizadas para el control de motores Westwood.

## Descripción

El paquete `westwood_motor_interfaces` define un conjunto de mensajes, servicios y acciones para interactuar con los motores Westwood a través de ROS2. Estas interfaces están diseñadas para facilitar el control y monitoreo de los motores, ofreciendo una API clara y consistente.

## Interfaces disponibles

### Servicios

- **SetMotorIdAndTarget.srv**: Establece posiciones objetivo para múltiples motores
  - **Request**: `int32[] motor_ids, float32[] target_positions`
  - **Response**: `bool success, string message, float32[] previous_positions`

- **GetMotorPositions.srv**: Obtiene las posiciones actuales de los motores
  - **Request**: `int32[] motor_ids`
  - **Response**: `bool success, string message, float32[] positions`

- **GetAvailableMotors.srv**: Obtiene la lista de IDs de motores disponibles
  - **Request**: (vacío)
  - **Response**: `bool success, string message, int32[] motor_ids`

- **SetGains.srv**: Configura las ganancias de control PID para los motores
  - **Request**: Incluye parámetros para configurar ganancias de posición, fuerza y corriente
  - **Response**: `bool success, string message`

- **SetMode.srv**: Configura el modo de operación de los motores (corriente, velocidad, posición)
  - **Request**: `int32[] motor_ids, int32[] modes`
  - **Response**: `bool success, string message`

- **SetTorqueEnable.srv**: Habilita o deshabilita el torque de los motores
  - **Request**: `int32[] motor_ids, bool[] enable_torque`
  - **Response**: `bool success, string message`

- **SetGoalIq.srv**: Establece la corriente iq objetivo para múltiples motores
  - **Request**: `int32[] motor_ids, float64[] goal_iq`
  - **Response**: `bool success, string message`

## Constantes y enumeraciones

### Modos de operación

Los motores Westwood soportan diferentes modos de operación:

```
MODO_CORRIENTE = 0  # Control por corriente
MODO_VELOCIDAD = 1  # Control por velocidad
MODO_POSICIÓN = 2   # Control por posición
```

## Uso

### Integración en un paquete ROS2

Para usar estas interfaces en tu paquete ROS2, añade una dependencia en tu `package.xml`:

```xml
<depend>westwood_motor_interfaces</depend>
```

Y en tu CMakeLists.txt:

```cmake
find_package(westwood_motor_interfaces REQUIRED)
```

### Ejemplos de código

#### Establecer posición objetivo para un motor

```python
import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import SetMotorIdAndTarget

class MotorControlExample(Node):
    def __init__(self):
        super().__init__('motor_control_example')
        self.cli = self.create_client(SetMotorIdAndTarget, 'westwood_motor/set_motor_id_and_target')
        
    def send_position_request(self, motor_ids, target_positions):
        request = SetMotorIdAndTarget.Request()
        request.motor_ids = motor_ids
        request.target_positions = target_positions
        
        # Esperar a que el servicio esté disponible
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando a que el servicio esté disponible...')
            
        return self.cli.call_async(request)

def main():
    rclpy.init()
    node = MotorControlExample()
    
    # Ejemplo: Mover dos motores a posiciones específicas
    future = node.send_position_request([1, 2], [1.57, 3.14])
    
    # Esperar a que se complete la solicitud
    rclpy.spin_until_future_complete(node, future)
    
    # Procesar la respuesta
    response = future.result()
    if response.success:
        print("Motores posicionados correctamente")
        print(f"Posiciones anteriores: {response.previous_positions}")
    else:
        print(f"Error: {response.message}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Obtener las posiciones actuales de los motores

```python
import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import GetMotorPositions

class MotorPositionReader(Node):
    def __init__(self):
        super().__init__('motor_position_reader')
        self.cli = self.create_client(GetMotorPositions, 'westwood_motor/get_motor_positions')
        
    def get_positions(self, motor_ids):
        request = GetMotorPositions.Request()
        request.motor_ids = motor_ids
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Esperando a que el servicio esté disponible...')
            
        return self.cli.call_async(request)

def main():
    rclpy.init()
    node = MotorPositionReader()
    
    # Obtener posiciones para los motores 1, 2 y 3
    future = node.get_positions([1, 2, 3])
    
    rclpy.spin_until_future_complete(node, future)
    
    response = future.result()
    if response.success:
        for i, motor_id in enumerate(response.motor_ids):
            print(f"Motor {motor_id}: Posición = {response.positions[i]} radianes")
    else:
        print(f"Error: {response.message}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Configurar ganancias PID para control de posición

```python
import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import SetGains

class GainConfigExample(Node):
    def __init__(self):
        super().__init__('gain_config_example')
        self.cli = self.create_client(SetGains, 'westwood_motor/set_position_gains')

    def configure_position_pid(self, motor_ids, p_gain, i_gain, d_gain):
        request = SetGains.Request()
        request.motor_ids = motor_ids
        request.p_gain_position = p_gain
        request.i_gain_position = i_gain
        request.d_gain_position = d_gain
        
        # Valores predeterminados para otros parámetros
        request.p_gain_force = 0.0
        request.i_gain_force = 0.0
        request.d_gain_force = 0.0
        request.p_gain_iq = 0.0
        request.i_gain_iq = 0.0
        request.d_gain_iq = 0.0
        request.p_gain_id = 0.0
        request.i_gain_id = 0.0
        request.d_gain_id = 0.0
        request.iq_max = 0.0
        request.kt = 0.0

        return self.cli.call_async(request)
```