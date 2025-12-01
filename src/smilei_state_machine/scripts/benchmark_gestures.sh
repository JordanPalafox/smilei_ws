#!/bin/bash
#
# Gesture Performance Benchmarking Script
#
# This script executes gestures multiple times to collect performance metrics
# for comparison between Cartesian and Joint control modes.
#

set -e

GESTURE_NAME=${1:-"wave"}
NUM_ITERATIONS=${2:-5}
WAIT_TIME=${3:-12}

echo "════════════════════════════════════════════════════════════════"
echo "  GESTURE PERFORMANCE BENCHMARK"
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "  Gesture: $GESTURE_NAME"
echo "  Iterations: $NUM_ITERATIONS"
echo "  Wait time between executions: ${WAIT_TIME}s"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Please source your ROS2 workspace first:"
    echo "   source ~/smilei_ws/install/setup.bash"
    exit 1
fi

# Create metrics directory if it doesn't exist
METRICS_DIR="$HOME/smilei_ws/src/smilei_state_machine/performance_metrics"
mkdir -p "$METRICS_DIR"
echo "📊 Metrics will be saved to: $METRICS_DIR"
echo ""

# Function to execute gesture
execute_gesture() {
    local iteration=$1
    echo "─────────────────────────────────────────────────────────────"
    echo "  Iteration $iteration/$NUM_ITERATIONS"
    echo "─────────────────────────────────────────────────────────────"

    # Send gesture command
    ros2 topic pub /gesture_command std_msgs/String "data: '$GESTURE_NAME'" --once

    echo "✅ Gesture command sent. Waiting ${WAIT_TIME}s for execution..."
    sleep "$WAIT_TIME"
}

# Main benchmark loop
echo "🚀 Starting benchmark..."
echo ""

for i in $(seq 1 "$NUM_ITERATIONS"); do
    execute_gesture "$i"
done

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "  BENCHMARK COMPLETE"
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "📊 Collected $NUM_ITERATIONS executions of gesture '$GESTURE_NAME'"
echo ""
echo "Next steps:"
echo "  1. Review metrics in: $METRICS_DIR"
echo "  2. Run analysis: python3 scripts/analyze_performance_metrics.py"
echo "  3. Check latest metrics: cat $METRICS_DIR/${GESTURE_NAME}_*_latest.yaml"
echo ""

# Optionally run analysis automatically
read -p "Run performance analysis now? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    cd "$HOME/smilei_ws/src/smilei_state_machine"
    python3 scripts/analyze_performance_metrics.py
fi
