# Skeleton Detection Package

Este paquete de ROS 2 detecta el esqueleto humano usando MediaPipe Pose, ignorando las piernas.

## Características

- Detecta el esqueleto humano en tiempo real desde la cámara OAK-D-Pro
- Ignora las piernas (rodillas, tobillos, pies)
- Publica una imagen con el esqueleto dibujado
- Usa MediaPipe Pose para detección precisa

## Dependencias

- ROS 2 (Humble/Foxy/etc.)
- Python 3
- OpenCV
- MediaPipe
- cv_bridge

## Instalación de dependencias

```bash
pip install mediapipe
```

## Compilación

```bash
cd /home/smilei/smilei_ws
colcon build --packages-select skeleton_detection
source install/setup.bash
```

## Uso

### Opción 1: Usar el archivo launch (recomendado)

```bash
ros2 launch skeleton_detection skeleton_detection.launch.py
```

### Opción 2: Ejecutar nodos individualmente

Terminal 1 - Publicar imágenes de la cámara:
```bash
python3 /home/smilei/smilei_ws/oak_publisher.py /camera/image_raw
```

Terminal 2 - Ejecutar detector de esqueleto:
```bash
ros2 run skeleton_detection skeleton_detector
```

## Topics

- **Input**: `/camera/image_raw` - Imagen de la cámara (sensor_msgs/Image)
- **Output**: `/skeleton/image` - Imagen con esqueleto dibujado (sensor_msgs/Image)

## Visualización

Para visualizar las imágenes con el esqueleto:

```bash
ros2 run rqt_image_view rqt_image_view /skeleton/image
```

## Configuración

El nodo detecta landmarks del 0-24 (cara, torso, brazos, caderas) e ignora 25-32 (piernas).

Los parámetros se pueden ajustar en `skeleton_detector_node.py`:
- `model_complexity`: 0, 1, o 2 (mayor es más preciso pero más lento)
- `min_detection_confidence`: Confianza mínima para detección (0.0-1.0)
- `min_tracking_confidence`: Confianza mínima para seguimiento (0.0-1.0)
