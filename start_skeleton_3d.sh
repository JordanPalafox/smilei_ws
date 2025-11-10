#!/bin/bash

# Script para iniciar el sistema de detección de esqueleto 3D

echo "=========================================="
echo "Sistema de Detección de Esqueleto 3D"
echo "=========================================="
echo ""

# Matar procesos anteriores
echo "Limpiando procesos anteriores..."
pkill -9 -f "oak_publisher" 2>/dev/null
pkill -9 -f "skeleton_detector" 2>/dev/null
sleep 2

# Cargar entorno de ROS 2
source /opt/ros/humble/setup.bash

# Iniciar oak_publisher con profundidad
echo "[1/2] Iniciando OAK-D Pro con RGB + Depth..."
python3 /home/smilei/smilei_ws/oak_publisher_with_depth.py &
OAK_PID=$!
echo "       OAK-D Pro iniciado (PID: $OAK_PID)"
sleep 4

# Iniciar skeleton_detector_3d
echo "[2/2] Iniciando detector de esqueleto 3D..."
python3 -c "from skeleton_detection.skeleton_detector_3d_node import main; main()" &
SKELETON_PID=$!
echo "       Detector 3D iniciado (PID: $SKELETON_PID)"
sleep 3

echo ""
echo "=========================================="
echo "Sistema 3D iniciado correctamente!"
echo "=========================================="
echo ""
echo "Rendimiento:"
echo "  - Cámara: 30 FPS (RGB + Depth)"
echo "  - Detección 3D: ~20-23 FPS"
echo "  - Puntos 3D detectados: 11-14 por frame"
echo ""
echo "Topics disponibles:"
echo "  - /camera/image_raw          (RGB)"
echo "  - /camera/depth/image_raw    (Profundidad 16-bit)"
echo "  - /camera/camera_info        (Parámetros intrínsecos)"
echo "  - /skeleton/image            (Visualización con distancias)"
echo "  - /skeleton/poses_3d         (Posiciones 3D en metros)"
echo ""
echo "Para visualizar:"
echo "  Imagen con esqueleto:"
echo "    ros2 run rqt_image_view rqt_image_view /skeleton/image"
echo ""
echo "  Ver posiciones 3D:"
echo "    ros2 topic echo /skeleton/poses_3d"
echo ""
echo "Presiona Ctrl+C para detener el sistema"
echo "=========================================="
echo ""

# Función para manejar la señal de interrupción
cleanup() {
    echo ""
    echo "Deteniendo sistema 3D..."
    kill $OAK_PID 2>/dev/null
    kill $SKELETON_PID 2>/dev/null
    pkill -9 -f "oak_publisher" 2>/dev/null
    pkill -9 -f "skeleton_detector" 2>/dev/null
    echo "Sistema detenido."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Mantener el script corriendo
wait
