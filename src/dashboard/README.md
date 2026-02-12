# SMILEi Robot Dashboard

Dashboard con IMGUI para visualización y control del robot SMILEi.

## Características

- **Control de Estados**: Botones para cambiar entre diferentes estados del robot
- **Visualización de Malla**: Sección dedicada para renderizar la malla 3D del robot
- **Información del Sistema**: Panel con información sobre el estado actual y estados disponibles

## Requisitos

### Dependencias de Python

```bash
pip install imgui[glfw] PyOpenGL
```

### Dependencias de ROS 2

- `rclpy`
- `std_msgs`
- `sensor_msgs`
- `geometry_msgs`

## Instalación

1. Navegar al workspace:
```bash
cd /home/jordan/smilei_ws
```

2. Compilar el paquete:
```bash
colcon build --packages-select dashboard
```

3. Source el workspace:
```bash
source install/setup.bash
```

## Uso

### Iniciar el dashboard

```bash
ros2 launch dashboard dashboard.launch.py
```

O directamente:

```bash
ros2 run dashboard dashboard_node
```

## Estados Disponibles

El dashboard permite controlar los siguientes estados del robot:

- **idle**: Estado inactivo
- **enable**: Habilitar el robot
- **home**: Ir a posición home
- **zero**: Ir a posición zero
- **say_hello**: Ejecutar comportamiento de saludo
- **teleoperation**: Modo teleoperación local
- **remote_teleoperation**: Modo teleoperación remota
- **disable**: Deshabilitar el robot

## Arquitectura

### Nodo ROS 2

- **Nombre**: `dashboard_node`
- **Publisher**: `/state_command` (String) - Publica comandos de cambio de estado
- **Subscriber**: `/current_state` (String) - Recibe actualizaciones del estado actual

### Interfaz IMGUI

La interfaz está dividida en tres paneles principales:

1. **Robot State Control**: Botones con código de colores para cambiar estados
2. **Robot Mesh Visualization**: Área de visualización 3D (placeholder por ahora)
3. **System Information**: Información del sistema y estado actual

## Desarrollo Futuro

- Integración de renderizado 3D real de la malla del robot
- Visualización de datos de sensores
- Gráficos de telemetría en tiempo real
- Control de joints individuales
- Visualización de trayectorias

## Estructura del Paquete

```
dashboard/
├── dashboard/
│   ├── __init__.py
│   └── dashboard_node.py
├── launch/
│   └── dashboard.launch.py
├── resource/
│   └── dashboard
├── package.xml
├── setup.py
└── README.md
```
