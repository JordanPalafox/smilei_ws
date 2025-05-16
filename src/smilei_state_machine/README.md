# SMILEi State Machine

Máquina de estados para el robot SMILEi utilizando py_trees.

## Descripción

Este paquete implementa una máquina de estados flexible para el robot SMILEi basada en la biblioteca py_trees. Permite controlar los diferentes estados y comportamientos del robot de manera organizada y extensible.

La máquina de estados recibe comandos a través de un tópico ROS2 y ejecuta el comportamiento correspondiente. Está diseñada para interactuar con los motores Westwood del robot a través de las interfaces proporcionadas por el paquete `westwood_motor_interfaces`.

## Características

- Arquitectura modular basada en comportamientos
- Transiciones de estado flexibles y configurables
- Integración con motores Westwood
- Comportamientos predefinidos para tareas comunes
- Fácil extensibilidad para añadir nuevos comportamientos

## Dependencias

- ROS2 (Humble o superior)
- `py_trees` y `py_trees_ros`
- `westwood_motor_interfaces`
- `westwood_motor_control_sdk_wrapper`
- `python3-numpy`

## Uso

### Iniciar la máquina de estados

Para iniciar la máquina de estados del robot SMILEi:

```bash
ros2 run smilei_state_machine state_machine
```

### Enviar comandos a la máquina de estados

Para enviar un comando a la máquina de estados:

```bash
ros2 run smilei_state_machine send_state_command <comando>
```

Donde `<comando>` puede ser uno de los siguientes:

- `idle`: Estado de espera sin realizar acciones
- `enable`: Habilitar los motores del robot
- `disable`: Deshabilitar los motores del robot
- `home`: Mover el robot a posición home
- `zero`: Mover el robot a posición cero
- `say_hello`: Ejecutar secuencia de saludo
- `teleop`: Activar modo de teleoperación local

También puedes publicar directamente en el tópico:

```bash
ros2 topic pub /state_command std_msgs/msg/String "data: 'home'" -1
```

## Configuración

Los parámetros del robot y de la máquina de estados se configuran en el archivo YAML:

```bash
src/smilei_state_machine/config/robot_params.yaml
```

Este archivo contiene:
- Límites de articulaciones
- Ganancias para control de posición
- Ganancias para control de corriente
- Parámetros físicos del robot

## Comportamientos disponibles

La máquina de estados incluye los siguientes comportamientos:

| Comportamiento | Descripción |
|----------------|-------------|
| EnableRobot | Habilita los motores del robot |
| DisableRobot | Deshabilita los motores del robot |
| HomePosition | Mueve el robot a la posición home |
| ZeroPosition | Mueve el robot a la posición cero |
| SayHello | Ejecuta secuencia de saludo |
| LocalTeleoperation | Modo de teleoperación local |

## Arquitectura

La máquina de estados se basa en el framework py_trees para organizar los comportamientos en una estructura jerárquica:

```
StateMachineRoot
├── IdleBehavior
├── EnableRobot
├── DisableRobot
├── HomePosition
├── ZeroPosition
├── SayHello
└── LocalTeleoperation
```

## Extensión

### Añadir un nuevo comportamiento

1. Crea una nueva clase en el directorio `behaviors/` que herede de `py_trees.behaviour.Behaviour`.
2. Implementa los métodos `setup()`, `initialise()`, `update()` y `terminate()`.
3. Registra el comportamiento en `state_machine.py`.

Ejemplo básico:

```python
class MiNuevoComportamiento(py_trees.behaviour.Behaviour):
    def __init__(self, name="MiNuevoComportamiento"):
        super().__init__(name)
        
    def setup(self):
        # Setup del comportamiento
        return True
        
    def initialise(self):
        # Inicializar comportamiento
        pass
        
    def update(self):
        # Lógica principal
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        # Limpieza al finalizar
        pass
```

## Resolución de problemas

### La máquina de estados no responde a comandos

- Verifica que el servicio de motores `westwood_motor_server` esté activo
- Comprueba que los motores estén conectados y operativos
- Inspecciona los logs en busca de errores

### El robot no se mueve

- Verifica que los motores estén habilitados (envía el comando `enable`)
- Comprueba que no haya límites de articulaciones en conflicto
- Verifica las ganancias PID en el archivo de configuración