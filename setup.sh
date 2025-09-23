#!/bin/bash

# Nombre para la sesión de tmux
SESSION_NAME="ros2_smilei"

# Asegurarse de que no haya una sesión con el mismo nombre
tmux kill-session -t $SESSION_NAME 2>/dev/null

# Iniciar una nueva sesión de tmux en segundo plano (-d)
# La primera ventana creada es la ventana 0
tmux new-session -d -s $SESSION_NAME -n "ROS"

# --- Panel 0 (Superior): Máquina de Estados ---
# El objetivo es sesion:ventana.panel -> ros2_smilei:0.0
PANE_0_TARGET="${SESSION_NAME}:0.0"
tmux send-keys -t $PANE_0_TARGET "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_0_TARGET "source install/setup.bash" C-m
tmux send-keys -t $PANE_0_TARGET "echo '🚀 Lanzando la máquina de estados...'" C-m
tmux send-keys -t $PANE_0_TARGET "ros2 launch smilei_state_machine state_machine.launch.py" C-m

# --- Panel 1 (Inferior): Publicador ---

# Dividir el panel 0 verticalmente
tmux split-window -v -t $PANE_0_TARGET

# 🔥 ARREGLO #1: Aumentar un poco la espera para asegurar que el shell del nuevo panel esté 100% listo.
sleep 2

# 🔥 ARREGLO #2: Usar el formato de objetivo correcto. El nuevo panel es ahora el 1.
PANE_1_TARGET="${SESSION_NAME}:0.1"

# Enviar comandos al nuevo panel (ahora con el objetivo correcto)
tmux send-keys -t $PANE_1_TARGET "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_1_TARGET "source install/setup.bash" C-m
tmux send-keys -t $PANE_1_TARGET "echo '⏳ Esperando 3s a que la máquina de estados inicie...'; sleep 3; clear" C-m

# Escribir el comando del publicador en la terminal, sin ejecutarlo.
tmux send-keys -t $PANE_1_TARGET 'ros2 topic pub --once /operador/state_command std_msgs/msg/String "data: '\''enable'\''"'

# Seleccionar el panel inferior para que el cursor esté listo ahí
tmux select-pane -t $PANE_1_TARGET

# Adjuntar a la sesión de tmux para ver los paneles
tmux attach-session -t $SESSION_NAME