# Westwood Motor Control SDK Wrapper

Este paquete proporciona un wrapper ROS2 para el SDK de control de motores Westwood, permitiendo controlar motores Westwood a través de interfaces ROS2.

## Descripción

El paquete `westwood_motor_control_sdk_wrapper` actúa como un puente entre el SDK PyBear de Westwood Robotics y ROS2. Proporciona un conjunto de servicios ROS2 que permiten controlar y monitorear los motores Westwood de forma sencilla desde cualquier nodo ROS2.

## Características

- Control de múltiples motores Westwood a través de ROS2
- Configuración de parámetros PID para control de posición y corriente
- Configuración del modo de operación de los motores
- Habilitación/deshabilitación de torque
- Monitoreo del estado de los motores
- Lectura de posiciones actuales de los motores
- Detección de motores disponibles

## Uso

### Iniciar el servidor de motores

Para iniciar el servidor que proporciona los servicios para controlar los motores Westwood:

```bash
ros2 run westwood_motor_control_sdk_wrapper westwood_motor_server.py
```

## Servicios disponibles

### Control de motores

- **/westwood_motor/set_motor_id_and_target** (`SetMotorIdAndTarget`): Establece la posición objetivo para uno o varios motores
- **/westwood_motor/get_motor_positions** (`GetMotorPositions`): Obtiene las posiciones actuales de los motores
- **/westwood_motor/get_available_motors** (`GetAvailableMotors`): Obtiene la lista de IDs de motores disponibles

### Configuración de control

- **/westwood_motor/set_position_gains** (`SetGains`): Configura las ganancias PID para el control de posición
- **/westwood_motor/set_current_gains** (`SetGains`): Configura las ganancias PID para el control de corriente
- **/westwood_motor/set_mode** (`SetMode`): Configura el modo de operación del motor
- **/westwood_motor/set_torque_enable** (`SetTorqueEnable`): Habilita o deshabilita el torque del motor
- **/westwood_motor/set_goal_iq** (`SetGoalIq`): Establece la corriente iq objetivo

## Ejemplos de uso

### Control de posición de un motor

```python
import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import SetMotorIdAndTarget

class MotorControlExample(Node):
    def __init__(self):
        super().__init__('motor_control_example')
        self.cli = self.create_client(SetMotorIdAndTarget, 'westwood_motor/set_motor_id_and_target')

    def send_position_request(self, motor_ids, positions):
        request = SetMotorIdAndTarget.Request()
        request.motor_ids = motor_ids
        request.target_positions = positions
        return self.cli.call_async(request)

def main():
    rclpy.init()
    node = MotorControlExample()
    
    # Ejemplo: Mover el motor 1 a la posición 90 grados
    future = node.send_position_request([1], [1.57])
    rclpy.spin_until_future_complete(node, future)
    
    response = future.result()
    if response.success:
        print("Motor posicionado correctamente")
    else:
        print(f"Error: {response.message}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Configuración de ganancias PID

```python
import rclpy
from rclpy.node import Node
from westwood_motor_interfaces.srv import SetGains

class GainConfigExample(Node):
    def __init__(self):
        super().__init__('gain_config_example')
        self.cli = self.create_client(SetGains, 'westwood_motor/set_position_gains')

    def send_gain_config(self, motor_id, p_gain, i_gain, d_gain):
        request = SetGains.Request()
        request.motor_id = motor_id
        request.p_gain = p_gain
        request.i_gain = i_gain
        request.d_gain = d_gain
        return self.cli.call_async(request)

def main():
    rclpy.init()
    node = GainConfigExample()
    
    # Configurar ganancias PID para el motor 1
    future = node.send_gain_config(1, 5.0, 0.0, 0.2)
    rclpy.spin_until_future_complete(node, future)
    
    response = future.result()
    if response.success:
        print("Ganancias configuradas correctamente")
    else:
        print(f"Error: {response.message}")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Estructura del paquete

```
westwood_motor_control_sdk_wrapper/
├── CMakeLists.txt                        # Script de compilación
├── launch/
│   └── westwood_motor_server.launch.py   # Archivo de lanzamiento
├── package.xml                          # Manifest del paquete
├── scripts/
│   └── westwood_motor_server.py         # Script principal del servidor
└── westwood_motor_control_sdk_wrapper/   # Módulo Python del SDK
    ├── __init__.py
    ├── CONTROL_TABLE.py                 # Tabla de registros de control
    ├── CRC.py                           # Utilidades para cálculo CRC
    ├── Manager.py                       # Gestor principal de motores
    ├── Packet.py                        # Implementación del protocolo de comunicación
    ├── TIMING_TABLE.py                  # Constantes de tiempos
    └── pybear/                          # Submódulos del SDK PyBear
```

## Hardware compatible

Este paquete está diseñado para trabajar con actuadores Westwood BEAR a través del puerto serial. El hardware requerido incluye:

- Motores Westwood BEAR
- Conexión USB hacia el controlador de los motores
- Puerto serie asignado (por defecto: `/dev/pybear_UB00E9`)

## Resolución de problemas

### El motor no responde

- Verifica que el motor esté correctamente conectado y alimentado
- Comprueba que el ID del motor coincida con el solicitado
- Verifica el puerto serie usado por el servidor (default: `/dev/pybear_UB00E9`)
- Aumenta el nivel de debug para ver más información: `debug:=true`

### Errores de comunicación

- Verifica la velocidad en baudios (default: 8000000)
- Comprueba que los derechos de acceso al puerto serial sean correctos
- Reinicia los motores y el servidor