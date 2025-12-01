#!/bin/bash
#
# Gesture Performance Benchmarking Script
#
# This script executes gestures multiple times to collect performance metrics
# for comparison between Cartesian and Joint control modes.
#

set -e

echo "════════════════════════════════════════════════════════════════"
echo "  GESTURE PERFORMANCE BENCHMARK"
echo "════════════════════════════════════════════════════════════════"
echo ""

# Interactive input mode if no arguments provided
if [ $# -eq 0 ]; then
    # Ask for gesture name
    echo "📝 Enter the gesture name to benchmark:"
    read -p "   Gesture name: " GESTURE_NAME

    # Validate gesture name is not empty
    while [ -z "$GESTURE_NAME" ]; do
        echo "⚠️  Gesture name cannot be empty!"
        read -p "   Gesture name: " GESTURE_NAME
    done

    # Ask for number of iterations
    echo ""
    echo "🔢 Enter the number of iterations (default: 10):"
    read -p "   Number of iterations: " NUM_ITERATIONS

    # Use default if empty
    if [ -z "$NUM_ITERATIONS" ]; then
        NUM_ITERATIONS=10
    fi

    # Validate it's a number
    while ! [[ "$NUM_ITERATIONS" =~ ^[0-9]+$ ]] || [ "$NUM_ITERATIONS" -lt 1 ]; do
        echo "⚠️  Please enter a valid positive number!"
        read -p "   Number of iterations: " NUM_ITERATIONS
        if [ -z "$NUM_ITERATIONS" ]; then
            NUM_ITERATIONS=10
            break
        fi
    done

    # Ask for wait time
    echo ""
    echo "⏱️  Enter wait time between executions in seconds (default: 12):"
    read -p "   Wait time (seconds): " WAIT_TIME

    # Use default if empty
    if [ -z "$WAIT_TIME" ]; then
        WAIT_TIME=12
    fi

    # Validate it's a number
    while ! [[ "$WAIT_TIME" =~ ^[0-9]+$ ]] || [ "$WAIT_TIME" -lt 1 ]; do
        echo "⚠️  Please enter a valid positive number!"
        read -p "   Wait time (seconds): " WAIT_TIME
        if [ -z "$WAIT_TIME" ]; then
            WAIT_TIME=12
            break
        fi
    done

    echo ""
else
    # Use command-line arguments
    GESTURE_NAME=${1:-"wave"}
    NUM_ITERATIONS=${2:-10}
    WAIT_TIME=${3:-12}
fi

# Show configuration
echo "════════════════════════════════════════════════════════════════"
echo "  BENCHMARK CONFIGURATION"
echo "════════════════════════════════════════════════════════════════"
echo ""
echo "  Gesture: $GESTURE_NAME"
echo "  Iterations: $NUM_ITERATIONS"
echo "  Wait time between executions: ${WAIT_TIME}s"
echo ""
echo "════════════════════════════════════════════════════════════════"
echo ""

# Confirm to start
read -p "Start benchmark? (y/n) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "❌ Benchmark cancelled."
    exit 0
fi
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
