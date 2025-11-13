#!/bin/bash

# Tmux session name
SESSION_NAME="trajectory_smilei"

# Make sure no session with the same name exists
tmux kill-session -t $SESSION_NAME 2>/dev/null

echo "🤖 Starting SMILEi Trajectory Design in RVIZ..."

# =============================================================================
# WINDOW 0: RVIZ Visualization (Trajectory Execution + Trajectory Design Marker Node)
# =============================================================================
tmux new-session -d -s $SESSION_NAME -n "Trajectory"

# Pane 0.0: Trajectory execution launchfile (Upper pane)
PANE_0_0="${SESSION_NAME}:0.0"
tmux send-keys -t $PANE_0_0 "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_0_0 "source install/setup.bash" C-m
tmux send-keys -t $PANE_0_0 "echo '🤖 Launching trajectory execution node...'" C-m
tmux send-keys -t $PANE_0_0 "ros2 launch smilei_dual_arm_ik view_single_arm_parametric.launch.py" C-m

# Pane 0.1: Interactive Marker Node (Left lower pane)
tmux split-window -v -t $PANE_0_0
sleep 2
PANE_0_1="${SESSION_NAME}:0.1"
tmux send-keys -t $PANE_0_1 "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_0_1 "source install/setup.bash" C-m
tmux send-keys -t $PANE_0_1 "echo '⏳ Waiting 2s for RVIZ to load...'; sleep 2; clear" C-m
# Launch interactive marker node with parameters (IK validation enabled)
tmux send-keys -t $PANE_0_1 "echo '🎮 Launching Interactive Marker Node with IK Validation...'" C-m
tmux send-keys -t $PANE_0_1 'ros2 run smilei_dual_arm_ik interactive_marker_node --ros-args -p step_size:=0.01 -p initial_x:=0.10 -p initial_y:=0.00 -p initial_z:=0.16 -p enable_ik_validation:=true' C-m

# Pane 0.2: Free Terminal (Right lower pane)
tmux split-window -h -t $PANE_0_1
PANE_0_2="${SESSION_NAME}:0.2"
tmux send-keys -t $PANE_0_2 "cd ~/smilei_ws" C-m
tmux send-keys -t $PANE_0_2 "source install/setup.bash" C-m
tmux send-keys -t $PANE_0_2 "echo '💻 Free terminal ready for commands'" C-m
tmux send-keys -t $PANE_0_2 "echo ''" C-m
tmux send-keys -t $PANE_0_2 "echo '📝 Useful commands:'" C-m
tmux send-keys -t $PANE_0_2 "echo '   ros2 topic list'" C-m
tmux send-keys -t $PANE_0_2 "echo '   ros2 topic echo /marker_position'" C-m
tmux send-keys -t $PANE_0_2 "echo '   ros2 node list'" C-m

# =============================================================================
# FINAL SETUP
# =============================================================================

# Seleccionar ventana de Control y panel inferior (para comando publisher)
# Select 
tmux select-window -t "${SESSION_NAME}:0"
tmux select-pane -t $PANE_0_2

echo ""
echo "✅ Sesión tmux '$SESSION_NAME' creada con éxito!"
echo ""
echo "📋 Window layout:"
echo "   Window 0: Trajectory Design"
echo "     └─ Pane 0: Trajectory execution launch (RVIZ + trajectory_executor)"
echo "     └─ Pane 1: Interactive Marker Node (keyboard controls)"
echo "     └─ Pane 2: Free terminal"
echo ""
echo "🎮 Interactive Marker Controls (Pane 1):"
echo "   W/S: Move X    A/D: Move Y    Q/E: Move Z"
echo "   Space: Save waypoint    P: Print position    L: List waypoints"
echo "   R: Reset    ESC: Exit"
echo ""
echo "🎯 Useful tmux keyboard shortcuts:"
echo "   Ctrl+b, arrows - Navigate between panes"
echo "   Ctrl+b, d      - Detach from session"
echo "   Ctrl+b, [      - Enter scroll mode (q to exit)"
echo "   Ctrl+b, :      - Enter tmux command mode"
echo ""
echo "🔗 To reconnect: tmux attach-session -t $SESSION_NAME"
echo ""

# Attach to the tmux session
tmux attach-session -t $SESSION_NAME
