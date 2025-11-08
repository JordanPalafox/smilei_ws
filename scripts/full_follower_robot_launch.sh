#!/bin/bash

# Nombre para la sesión de tmux
SESSION_NAME="ros2_smilei"

# Asegurarse de que no haya una sesión con el mismo nombre
tmux kill-session -t $SESSION_NAME 2>/dev/null

echo "🤖 Iniciando SMILEi Full Robot System..."

# =============================================================================
# VENTANA 0: CONTROL (Máquina de Estados + Publisher)
# =============================================================================
tmux new-session -d -s $SESSION_NAME -n "Control"

# Panel 0.0: Máquina de Estados (Superior)
PANE_0_0="${SESSION_NAME}:0.0"
tmux send-keys -t $PANE_0_0 "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_0_0 "source install/setup.bash" C-m
tmux send-keys -t $PANE_0_0 "echo '🤖 Lanzando la máquina de estados...'" C-m
tmux send-keys -t $PANE_0_0 "ros2 launch smilei_state_machine state_machine.launch.py namespace:=/seguidor" C-m

# Panel 0.1: Publicador de comandos (Inferior)
tmux split-window -v -t $PANE_0_0
sleep 2
PANE_0_1="${SESSION_NAME}:0.1"
tmux send-keys -t $PANE_0_1 "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_0_1 "source install/setup.bash" C-m
tmux send-keys -t $PANE_0_1 "echo '⏳ Esperando 3s a que la máquina de estados inicie...'; sleep 3; clear" C-m
# Comando listo para ejecutar
tmux send-keys -t $PANE_0_1 'ros2 topic pub --once /operador/state_command std_msgs/msg/String "data: '\''enable'\''"'

# =============================================================================
# VENTANA 1: AUDIO (Audio Subscriber + Publisher + PavuControl)
# =============================================================================
tmux new-window -t $SESSION_NAME -n "Audio"

# Panel 1.0: Audio Subscriber (Superior)
PANE_1_0="${SESSION_NAME}:1.0"
tmux send-keys -t $PANE_1_0 "cd ~/my_project" C-m
tmux send-keys -t $PANE_1_0 "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t $PANE_1_0 "export ROS_IP=192.168.0.2" C-m
tmux send-keys -t $PANE_1_0 "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t $PANE_1_0 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $PANE_1_0 "source install/setup.bash" C-m
tmux send-keys -t $PANE_1_0 "echo '🔊 Lanzando audio subscriber...'; sleep 2" C-m
tmux send-keys -t $PANE_1_0 "ros2 run audio_msgs_python audio_subscriber" C-m

# Panel 1.1: Audio Publisher (Inferior izquierda)
tmux split-window -v -t $PANE_1_0
sleep 1
PANE_1_1="${SESSION_NAME}:1.1"
tmux send-keys -t $PANE_1_1 "cd ~/my_project" C-m
tmux send-keys -t $PANE_1_1 "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t $PANE_1_1 "export ROS_IP=192.168.0.2" C-m
tmux send-keys -t $PANE_1_1 "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t $PANE_1_1 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $PANE_1_1 "source install/setup.bash" C-m
tmux send-keys -t $PANE_1_1 "echo '📢 Lanzando audio publisher...'; sleep 2" C-m
tmux send-keys -t $PANE_1_1 "ros2 run audio_msgs_python audio_publisher" C-m

# Panel 1.2: PavuControl (Inferior derecha)
tmux split-window -h -t $PANE_1_1
sleep 1
PANE_1_2="${SESSION_NAME}:1.2"
tmux send-keys -t $PANE_1_2 "echo '🎚️  Lanzando PavuControl...'; sleep 2" C-m
tmux send-keys -t $PANE_1_2 "pavucontrol" C-m

# =============================================================================
# VENTANA 2: VISION (OAK Camera + Terminal libre)
# =============================================================================
tmux new-window -t $SESSION_NAME -n "Vision"

# Panel 2.0: OAK Camera Publisher para /robot/image_raw (Izquierda)
PANE_2_0="${SESSION_NAME}:2.0"
tmux send-keys -t $PANE_2_0 "cd ~/my_project" C-m
tmux send-keys -t $PANE_2_0 "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t $PANE_2_0 "export ROS_IP=192.168.0.2" C-m
tmux send-keys -t $PANE_2_0 "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t $PANE_2_0 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $PANE_2_0 "source install/setup.bash" C-m
tmux send-keys -t $PANE_2_0 "cd src/webcam_publisher/src" C-m
tmux send-keys -t $PANE_2_0 "echo '📷 Lanzando OAK camera publisher para /robot/image_raw...'; sleep 2" C-m
tmux send-keys -t $PANE_2_0 "python3 oak_publisher.py /robot/image_raw" C-m

# Panel 2.1: Terminal libre (Superior derecha)
tmux split-window -h -t $PANE_2_0
sleep 1
PANE_2_1="${SESSION_NAME}:2.1"
tmux send-keys -t $PANE_2_1 "cd ~/my_project" C-m
tmux send-keys -t $PANE_2_1 "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t $PANE_2_1 "export ROS_IP=192.168.0.2" C-m
tmux send-keys -t $PANE_2_1 "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t $PANE_2_1 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $PANE_2_1 "source install/setup.bash" C-m
tmux send-keys -t $PANE_2_1 "echo '💻 Terminal libre lista'" C-m

# Panel 2.2: Terminal libre (Inferior derecha)
tmux split-window -v -t $PANE_2_1
sleep 1
PANE_2_2="${SESSION_NAME}:2.2"
tmux send-keys -t $PANE_2_2 "cd ~/my_project" C-m
tmux send-keys -t $PANE_2_2 "export ROS_DOMAIN_ID=10" C-m
tmux send-keys -t $PANE_2_2 "export ROS_IP=192.168.0.2" C-m
tmux send-keys -t $PANE_2_2 "export ROS_LOCALHOST_ONLY=0" C-m
tmux send-keys -t $PANE_2_2 "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" C-m
tmux send-keys -t $PANE_2_2 "source install/setup.bash" C-m
tmux send-keys -t $PANE_2_2 "echo '💻 Terminal libre lista'" C-m


# =============================================================================
# CONFIGURACIÓN FINAL
# =============================================================================

# Seleccionar ventana de Control y panel inferior (para comando publisher)
tmux select-window -t "${SESSION_NAME}:0"
tmux select-pane -t $PANE_0_1

echo ""
echo "✅ Sesión tmux '$SESSION_NAME' creada con éxito!"
echo ""
echo "📋 Layout de ventanas:"
echo "   0: Control     - Máquina de estados + Publisher"
echo "   1: Audio       - Audio subscriber + Publisher + PavuControl"
echo "   2: Vision      - OAK Camera (/robot/image_raw) + Terminal libre"
echo ""
echo "🎯 Atajos útiles de tmux:"
echo "   Ctrl+b, 0-2    - Cambiar a ventana 0-2"
echo "   Ctrl+b, n      - Siguiente ventana"
echo "   Ctrl+b, p      - Ventana anterior"
echo "   Ctrl+b, flechas - Navegar entre paneles"
echo "   Ctrl+b, d      - Desconectar de la sesión"
echo ""
echo "🔗 Para reconectar: tmux attach-session -t $SESSION_NAME"
echo ""

# Adjuntar a la sesión de tmux
tmux attach-session -t $SESSION_NAME
