#!/bin/bash

# Nombre para la sesión de tmux
SESSION_NAME="ros2_smilei"

# Configuración de ROS2
# export ROS_DOMAIN_ID=10
# export ROS_IP=192.168.0.100
# export ROS_LOCALHOST_ONLY=0
# export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Asegurarse de que no haya una sesión con el mismo nombre
tmux kill-session -t $SESSION_NAME 2>/dev/null

echo "🚀 Iniciando SMILEi Full System..."

# =============================================================================
# VENTANA 0: CONTROL (Máquina de Estados + Publisher)
# =============================================================================
tmux new-session -d -s $SESSION_NAME -n "Control"

# Panel 0.0: Máquina de Estados (Superior)
PANE_0_0="${SESSION_NAME}:0.0"
tmux send-keys -t $PANE_0_0 "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_0_0 "source install/setup.bash" C-m
tmux send-keys -t $PANE_0_0 "echo '🤖 Lanzando la máquina de estados...'" C-m
tmux send-keys -t $PANE_0_0 "ros2 launch smilei_state_machine state_machine.launch.py" C-m

# Panel 0.1: Publicador de comandos (Inferior)
tmux split-window -v -t $PANE_0_0
sleep 1
PANE_0_1="${SESSION_NAME}:0.1"
tmux send-keys -t $PANE_0_1 "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_0_1 "source install/setup.bash" C-m
tmux send-keys -t $PANE_0_1 "echo '⏳ Esperando 3s a que la máquina de estados inicie...'; sleep 3; clear" C-m
# Comando listo para ejecutar
tmux send-keys -t $PANE_0_1 'ros2 topic pub --once /operador/state_command std_msgs/msg/String "data: '\''enable'\''"'

#Panel 0.2: Operator Dashboard
tmux split-window -h -t $PANE_0_1
sleep 1
PANE_0_2="${SESSION_NAME}:0.2"
tmux send-keys -t $PANE_0_2 "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_0_2 "source install/setup.bash" C-m
tmux send-keys -t $PANE_0_2 "ros2 launch dashboard dashboard.launch.py" C-m

# =============================================================================
# VENTANA 1: AUDIO (Audio Subscriber + PavuControl)
# =============================================================================
tmux new-window -t $SESSION_NAME -n "Audio"

# Panel 1.0: Audio Subscriber (Izquierda)
PANE_1_0="${SESSION_NAME}:1.0"
tmux send-keys -t $PANE_1_0 "cd ~/my_project" C-m
tmux send-keys -t $PANE_1_0 "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t $PANE_1_0 "export ROS_IP=192.168.0.100" C-m
tmux send-keys -t $PANE_1_0 "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t $PANE_1_0 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $PANE_1_0 "source install/setup.bash" C-m
tmux send-keys -t $PANE_1_0 "echo '🔊 Lanzando audio subscriber...'; sleep 2" C-m
tmux send-keys -t $PANE_1_0 "ros2 run audio_msgs_python audio_subscriber_basic" C-m

# Panel 1.1: PavuControl (Derecha)
tmux split-window -h -t $PANE_1_0
sleep 1
PANE_1_1="${SESSION_NAME}:1.1"
tmux send-keys -t $PANE_1_1 "echo '🎚️  Lanzando PavuControl...'; sleep 2" C-m
tmux send-keys -t $PANE_1_1 "pavucontrol" C-m

# ===========================
# VENTANA 2: VISION (robusta)
# ===========================
tmux new-window -t "$SESSION_NAME" -n "Vision"

# Pane base (izquierda superior)
PANE_2_0="${SESSION_NAME}:2.0"
tmux send-keys -t "$PANE_2_0" "cd ~/my_project" C-m
tmux send-keys -t "$PANE_2_0" "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t "$PANE_2_0" "export ROS_IP=192.168.0.100" C-m
tmux send-keys -t "$PANE_2_0" "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t "$PANE_2_0" "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t "$PANE_2_0" "source install/setup.bash" C-m
tmux send-keys -t "$PANE_2_0" "cd src/webcam_publisher/src" C-m
tmux send-keys -t "$PANE_2_0" "echo '📷 Lanzando OAK camera publisher...'; sleep 1" C-m
tmux send-keys -t "$PANE_2_0" "python3 oak_publisher.py /operator/image_raw" C-m

# Split horizontal -> panel derecho (capturamos ID)
PANE_2_1=$(tmux split-window -h -t "$PANE_2_0" -P -F '#{pane_id}')
sleep 0.2
tmux send-keys -t "$PANE_2_1" "cd ~/my_project" C-m
tmux send-keys -t "$PANE_2_1" "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t "$PANE_2_1" "export ROS_IP=192.168.0.100" C-m
tmux send-keys -t "$PANE_2_1" "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t "$PANE_2_1" "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t "$PANE_2_1" "source install/setup.bash" C-m
tmux send-keys -t "$PANE_2_1" "echo '😊 Lanzando face blendshape detector...'; sleep 1" C-m
tmux send-keys -t "$PANE_2_1" "ros2 run face_blendshape_detector_py face_blendshape_detector_node" C-m

# Split vertical SOBRE el pane izquierdo superior (capturamos ID del nuevo)
PANE_2_2=$(tmux split-window -v -t "$PANE_2_0" -P -F '#{pane_id}')
sleep 0.2
tmux send-keys -t "$PANE_2_2" "cd ~/my_project" C-m
tmux send-keys -t "$PANE_2_2" "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t "$PANE_2_2" "export ROS_IP=192.168.0.100" C-m
tmux send-keys -t "$PANE_2_2" "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t "$PANE_2_2" "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t "$PANE_2_2" "source install/setup.bash" C-m
tmux send-keys -t "$PANE_2_2" "echo '📺 Lanzando receptor de video UDP...'; sleep 1" C-m
#tmux send-keys -t "$PANE_2_2" "gst-launch-1.0 -v udpsrc port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink" C-m
tmux send-keys -t "$PANE_2_2" "ros2 run rqt_image_view rqt_image_view /robot/image_raw" C-m

# =============================================================================
# VENTANA 3: VOICE (Voice Conversion System)
# =============================================================================
tmux new-window -t $SESSION_NAME -n "Voice"

# Panel 3.0: RVC-WebUI
PANE_3_0="${SESSION_NAME}:3.0"
tmux send-keys -t $PANE_3_0 "cd ~/my_project" C-m
tmux send-keys -t $PANE_3_0 "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t $PANE_3_0 "export ROS_IP=192.168.0.100" C-m
tmux send-keys -t $PANE_3_0 "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t $PANE_3_0 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $PANE_3_0 "source install/setup.bash" C-m
tmux send-keys -t $PANE_3_0 "cd ~/my_project/src/Retrieval-based-Voice-Conversion-WebUI" C-m
tmux send-keys -t $PANE_3_0 "source audioenv/bin/activate" C-m
tmux send-keys -t $PANE_3_0 "cd .." C-m
tmux send-keys -t $PANE_3_0 "cd .." C-m
tmux send-keys -t $PANE_3_0 "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t $PANE_3_0 "export ROS_IP=192.168.0.100" C-m
tmux send-keys -t $PANE_3_0 "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t $PANE_3_0 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $PANE_3_0 "source install/setup.bash" C-m
tmux send-keys -t $PANE_3_0 "cd src/Retrieval-based-Voice-Conversion-WebUI" C-m
tmux send-keys -t $PANE_3_0 "echo '🎤 Lanzando sistema de conversión de voz...'; sleep 2" C-m
tmux send-keys -t $PANE_3_0 "python3 EssentialInference.py" C-m

# =============================================================================
# CONFIGURACIÓN FINAL
# =============================================================================

# Seleccionar ventana de Control y panel inferior (para comando publisher)
tmux select-window -t "${SESSION_NAME}:0"
tmux select-pane -t $PANE_0_1

echo ""
echo "✅ Sesión tmux '$SESSION_NAME' crenada con éxito!"
echo ""
echo "📋 Layout de ventanas:"
echo "   0: Control     - Máquina de estados + Publisher"
echo "   1: Audio       - Audio subscriber + PavuControl"
echo "   2: Vision      - OAK Camera + Video receptor + Face detector"
echo "   3: Voice       - Sistema de conversión de voz"
echo ""
echo "🎯 Atajos útiles de tmux:"
echo "   Ctrl+b, 0-3    - Cambiar a ventana 0-3"
echo "   Ctrl+b, n      - Siguiente ventana"
echo "   Ctrl+b, p      - Ventana anterior"
echo "   Ctrl+b, flechas - Navegar entre paneles"
echo "   Ctrl+b, d      - Desconectar de la sesión"
echo ""
echo "🔗 Para reconectar: tmux attach-session -t $SESSION_NAME"
echo ""

# Adjuntar a la sesión de tmux
tmux attach-session -t $SESSION_NAME
