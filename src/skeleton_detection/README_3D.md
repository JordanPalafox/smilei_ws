# Skeleton Detection 3D - OAK-D Pro

Sistema de detección de esqueleto humano con posiciones 3D usando la cámara de profundidad de la OAK-D Pro.

## Características

- **Detección 2D**: MediaPipe Pose para detectar puntos del esqueleto en la imagen
- **Profundidad**: Usa la cámara estéreo de la OAK-D Pro para obtener distancias
- **Posiciones 3D**: Calcula posiciones (X, Y, Z) en metros para cada punto del esqueleto
- **Sin piernas**: Ignora puntos de las piernas (landmarks 25-32)
- **Optimizado para Jetson AGX Orin**: ~20-23 FPS

## Rendimiento

- **Cámara**: 30 FPS (RGB + Depth sincronizados)
- **Detección 3D**: 20-23 FPS
- **Puntos detectados**: 11-14 puntos 3D por frame (parte superior del cuerpo)

## Topics ROS 2

### Entrada (desde cámara)
- `/camera/image_raw` (sensor_msgs/Image): Imagen RGB 640x480
- `/camera/depth/image_raw` (sensor_msgs/Image): Mapa de profundidad 16-bit
- `/camera/camera_info` (sensor_msgs/CameraInfo): Parámetros intrínsecos de la cámara

### Salida (detección)
- `/skeleton/image` (sensor_msgs/Image): Visualización con esqueleto y distancias
- `/skeleton/poses_3d` (geometry_msgs/PoseArray): Posiciones 3D de cada punto

## Uso

### Opción 1: Script automático (recomendado)

```bash
cd /home/smilei/smilei_ws
./start_skeleton_3d.sh
```

### Opción 2: Comandos individuales

```bash
# Terminal 1 - Publicar RGB + Depth
source /opt/ros/humble/setup.bash
python3 /home/smilei/smilei_ws/oak_publisher_with_depth.py

# Terminal 2 - Detector 3D
source /opt/ros/humble/setup.bash
python3 -c "from skeleton_detection.skeleton_detector_3d_node import main; main()"
```

## Visualización

### Ver imagen con esqueleto y distancias
```bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view /skeleton/image
```

En la imagen verás:
- Puntos rojos: Articulaciones detectadas
- Líneas verdes: Conexiones del esqueleto
- Texto blanco: Distancia en metros de cada punto

### Ver posiciones 3D en terminal (formato simple)

```bash
source /opt/ros/humble/setup.bash
python3 /home/smilei/smilei_ws/view_skeleton_3d.py
```

### Ver posiciones 3D en terminal (formato raw)

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /skeleton/poses_3d
```

## Formato de Datos 3D

Cada punto del esqueleto tiene:
- **X**: Posición horizontal en metros (negativo = izquierda, positivo = derecha)
- **Y**: Posición vertical en metros (negativo = arriba, positivo = abajo)
- **Z**: Profundidad en metros (distancia desde la cámara)

El sistema de coordenadas es el de la cámara:
- Origen: Centro de la cámara
- Eje X: Horizontal (derecha +)
- Eje Y: Vertical (abajo +)
- Eje Z: Profundidad (adelante +)

## Puntos del Esqueleto (Upper Body)

Los puntos detectados son (0-24, sin piernas):

### Cara (0-10)
- 0: nose
- 1-3: left_eye (inner, center, outer)
- 4-6: right_eye (inner, center, outer)
- 7: left_ear
- 8: right_ear
- 9: mouth_left
- 10: mouth_right

### Torso y Brazos (11-22)
- 11: left_shoulder
- 12: right_shoulder
- 13: left_elbow
- 14: right_elbow
- 15: left_wrist
- 16: right_wrist
- 17-22: Manos (pinky, index, thumb para cada mano)

### Caderas (23-24)
- 23: left_hip
- 24: right_hip

## Configuración Avanzada

### Modificar parámetros de profundidad

Edita `/home/smilei/smilei_ws/oak_publisher_with_depth.py`:

```python
# Cambiar preset de profundidad
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)

# Ajustar filtros
stereo.setLeftRightCheck(True)  # Mejor calidad, más lento
stereo.setSubpixel(True)         # Mayor precisión
```

### Modificar umbral de detección

Edita `/home/smilei/smilei_ws/src/skeleton_detection/skeleton_detection/skeleton_detector_3d_node.py`:

```python
# Línea 117: Umbral de visibilidad
visibility_threshold = 0.4  # Aumentar para más confianza, reducir para más detecciones
```

## Troubleshooting

### No se detectan posiciones 3D
- Verifica que `/camera/camera_info` esté publicando: `ros2 topic echo /camera/camera_info`
- Asegúrate de tener buena iluminación
- Mantén distancia de 0.5m - 3m de la cámara

### FPS bajo
- Reduce `model_complexity` a 0 (ya está en 0)
- Verifica que no haya otros procesos usando la cámara
- Asegúrate de que el Jetson no esté en modo throttle

### Profundidad ruidosa
- Mejora la iluminación
- Evita superficies reflectantes
- Aumenta `setLeftRightCheck` para mejor calidad

## Arquitectura Técnica

```
OAK-D Pro
    ├── RGB Camera (640x480@30fps)
    └── Stereo Depth (640x480@30fps)
           ↓
   oak_publisher_with_depth.py
           ↓
    /camera/image_raw
    /camera/depth/image_raw
    /camera/camera_info
           ↓
skeleton_detector_3d_node.py
  - MediaPipe Pose (2D detection)
  - Depth lookup
  - 3D projection using camera intrinsics
           ↓
    /skeleton/image (visualization)
    /skeleton/poses_3d (3D positions)
```

## Algoritmo de Proyección 3D

```python
# Modelo pinhole camera
X = (x_pixel - cx) * Z / fx
Y = (y_pixel - cy) * Z / fy
Z = depth_mm / 1000.0  # Convertir a metros

donde:
- fx, fy: Focal lengths de la cámara
- cx, cy: Principal point (centro óptico)
- depth_mm: Profundidad del pixel en milímetros
```

## Aplicaciones

- Control de robots con gestos
- Medición de postura y ergonomía
- Tracking de movimiento 3D
- Interfaces humano-robot
- Análisis biomecánico

## Licencia

Apache-2.0
