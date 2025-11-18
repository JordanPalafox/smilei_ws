#!/usr/bin/env python3
"""
Autonomous Gesture Execution Behavior

Behavior that executes predefined gestures by calling the gesture_executor_hardware Action Server.
The gesture name can be set via parameter or topic subscription.
This behavior integrates with the smilei_state_machine for autonomous gesture execution.
"""

import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String, Bool
from smilei_dual_arm_ik_interfaces.action import ExecuteGesture


class AutonomousGestureExecution(py_trees.behaviour.Behaviour):
    """
    Behavior that executes predefined gestures for the dual arm robot.

    This behavior acts as an Action Client to the gesture_executor_dual_arm Action Server.
    It can receive gesture commands via:
    - Direct parameter (gesture_name)
    - Topic subscription (/gesture_command)
    """

    def __init__(self, name: str, node=None, gesture_name=None, action_server_name='execute_gesture'):
        """
        Initialize the behavior

        Args:
            name: Behavior name
            node: ROS2 node (optional, will create one if not provided)
            gesture_name: Initial gesture to execute (optional)
            action_server_name: Name of the action server (default: 'execute_gesture')
        """
        super().__init__(name)
        self.node = node
        self.own_node = False
        self.gesture_name = gesture_name
        self.pending_gesture = None
        self.action_server_name = action_server_name

        # Action client
        self.action_client = None
        self.goal_handle = None
        self.result = None
        self.feedback = None

        # Publishers for state machine integration
        self.status_pub = None
        self.executing_pub = None

        # Execution state
        self.execution_started = False
        self.execution_complete = False
        self.execution_success = False

    def setup(self, timeout_sec=None, **kwargs) -> bool:
        """Setup the behavior - create node and action client"""
        # Create or use provided node
        if self.node is None:
            self.node = rclpy.create_node('autonomous_gesture_execution_client')
            self.own_node = True
        else:
            self.own_node = False

        # Create action client
        self.action_client = ActionClient(
            self.node,
            ExecuteGesture,
            self.action_server_name
        )

        # Subscribe to gesture command topic (for receiving gesture commands)
        self.gesture_command_sub = self.node.create_subscription(
            String,
            '/gesture_command',
            self.gesture_command_callback,
            10
        )

        # Create publishers for state machine integration
        # Publishes current gesture execution status (gesture_name or "idle")
        self.status_pub = self.node.create_publisher(
            String,
            '/gesture_execution_status',
            10
        )

        # Publishes whether a gesture is currently executing
        self.executing_pub = self.node.create_publisher(
            Bool,
            '/gesture_executing',
            10
        )

        self.node.get_logger().info('Autonomous Gesture Execution behavior setup complete')

        # Check for action server availability (non-blocking)
        # Use very short timeout to be compatible with state_machine.py setup
        self.node.get_logger().info(f'Checking for "{self.action_server_name}" action server...')
        server_available = self.action_client.wait_for_server(timeout_sec=timeout_sec if timeout_sec else 0.1)

        if not server_available:
            self.node.get_logger().warning(
                f'Action server "{self.action_server_name}" not available yet. '
                'Will retry when executing a gesture. '
                'Make sure to run: ros2 launch smilei_dual_arm_ik gesture_executor_hardware.launch.py'
            )
        else:
            self.node.get_logger().info('✅ Action server connected!')

        # Always return True to allow state machine to continue
        # Similar to other behaviors (e.g., SayHello)
        return True

    def gesture_command_callback(self, msg):
        """Callback for receiving gesture commands via topic"""
        self.node.get_logger().info(f'Received gesture command: {msg.data}')
        self.pending_gesture = msg.data

    def initialise(self) -> None:
        """Called when behavior is activated"""
        # Use pending gesture if available, otherwise use initial gesture_name
        if self.pending_gesture:
            self.gesture_name = self.pending_gesture
            self.pending_gesture = None

        if not self.gesture_name:
            self.node.get_logger().warning(
                'No gesture name provided. Set gesture_name parameter or publish to /gesture_command'
            )
            # Publish idle status
            self.publish_status("idle", False)
            return

        self.node.get_logger().info(f'🎭 Initializing gesture execution: {self.gesture_name}')

        # Reset state
        self.execution_started = False
        self.execution_complete = False
        self.execution_success = False
        self.result = None
        self.feedback = None
        self.goal_handle = None

        # Publish initial status
        self.publish_status(f"initializing_{self.gesture_name}", False)

    def update(self) -> py_trees.common.Status:
        """Main update loop"""
        # Check if we have a gesture to execute
        if not self.gesture_name:
            self.node.get_logger().warning('No gesture to execute')
            self.publish_status("no_gesture", False)
            return py_trees.common.Status.FAILURE

        # Start execution if not started
        if not self.execution_started:
            return self.start_gesture_execution()

        # Check execution status
        if not self.execution_complete:
            # Spin the node to process callbacks
            rclpy.spin_once(self.node, timeout_sec=0.01)

            # Still executing - publish status
            if self.feedback:
                status_msg = f"executing_{self.gesture_name}_{self.feedback.current_phase}_{int(self.feedback.progress*100)}pct"
                self.publish_status(status_msg, True)

                # Log feedback periodically
                if hasattr(self, '_last_logged_progress'):
                    progress_delta = abs(self.feedback.progress - self._last_logged_progress)
                    if progress_delta >= 0.1:  # Log every 10% progress
                        self.node.get_logger().info(
                            f'📊 Gesture progress: {self.feedback.progress*100:.0f}% - {self.feedback.current_phase}'
                        )
                        self._last_logged_progress = self.feedback.progress
                else:
                    self._last_logged_progress = self.feedback.progress
            else:
                # No feedback yet, still starting
                self.publish_status(f"executing_{self.gesture_name}_starting", True)

            return py_trees.common.Status.RUNNING

        # Execution complete - return result
        if self.execution_success:
            self.node.get_logger().info(
                f'✅ Gesture "{self.gesture_name}" executed successfully! '
                f'({self.result.execution_time:.2f}s)'
            )
            self.publish_status(f"completed_{self.gesture_name}_success", False)
            return py_trees.common.Status.SUCCESS
        else:
            error_msg = self.result.message if self.result else "Unknown error"
            self.node.get_logger().error(
                f'❌ Gesture "{self.gesture_name}" failed: {error_msg}'
            )
            self.publish_status(f"completed_{self.gesture_name}_failed", False)
            return py_trees.common.Status.FAILURE

    def start_gesture_execution(self):
        """Send goal to action server"""
        self.node.get_logger().info(f'🚀 Sending gesture goal: {self.gesture_name}')

        # Check if action server is available (with retry)
        if not self.action_client.server_is_ready():
            self.node.get_logger().warning(f'Action server not ready, waiting...')
            server_available = self.action_client.wait_for_server(timeout_sec=2.0)

            if not server_available:
                self.node.get_logger().error(
                    f'❌ Action server "{self.action_server_name}" still not available. '
                    'Cannot execute gesture. Make sure gesture_executor_hardware is running.'
                )
                self.publish_status(f"error_{self.gesture_name}_server_unavailable", False)
                # Mark as complete with failure
                self.execution_started = True
                self.execution_complete = True
                self.execution_success = False
                return py_trees.common.Status.RUNNING

        # Publish status - starting execution
        self.publish_status(f"starting_{self.gesture_name}", True)

        # Create goal
        goal_msg = ExecuteGesture.Goal()
        goal_msg.gesture_name = self.gesture_name

        # Send goal asynchronously
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

        self.execution_started = True
        return py_trees.common.Status.RUNNING

    def publish_status(self, status_str: str, is_executing: bool):
        """Publish current execution status to topics"""
        if self.status_pub:
            status_msg = String()
            status_msg.data = status_str
            self.status_pub.publish(status_msg)

        if self.executing_pub:
            executing_msg = Bool()
            executing_msg.data = is_executing
            self.executing_pub.publish(executing_msg)

    def goal_response_callback(self, future):
        """Called when action server accepts/rejects goal"""
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.node.get_logger().error('Goal rejected by action server')
            self.execution_complete = True
            self.execution_success = False
            return

        self.node.get_logger().info('Goal accepted by action server')

        # Get result asynchronously
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Called when action server sends feedback"""
        self.feedback = feedback_msg.feedback

    def result_callback(self, future):
        """Called when action completes"""
        self.result = future.result().result
        self.execution_complete = True
        self.execution_success = self.result.success

        if self.execution_success:
            self.node.get_logger().info(f'Result: {self.result.message}')
        else:
            self.node.get_logger().error(f'Result: {self.result.message}')

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Cleanup when behavior terminates"""
        if new_status == py_trees.common.Status.SUCCESS:
            self.node.get_logger().info(f'✅ Gesture execution completed: {self.gesture_name}')
            self.publish_status(f"terminated_{self.gesture_name}_success", False)
        elif new_status == py_trees.common.Status.FAILURE:
            self.node.get_logger().warning(f'⚠️ Gesture execution failed: {self.gesture_name}')
            self.publish_status(f"terminated_{self.gesture_name}_failure", False)
        else:
            self.node.get_logger().info(f'🛑 Gesture execution interrupted: {self.gesture_name}')
            self.publish_status(f"terminated_{self.gesture_name}_interrupted", False)

            # Cancel goal if still running
            if self.goal_handle and not self.execution_complete:
                self.node.get_logger().info('Cancelling gesture execution...')
                cancel_future = self.goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self.node, cancel_future, timeout_sec=1.0)

        # Publish idle status
        self.publish_status("idle", False)

        # Destroy node if we created it
        if self.own_node and self.node:
            self.node.destroy_node()


# Convenience functions for easy integration

def create_gesture_behavior(gesture_name: str, node=None, action_server_name='execute_gesture'):
    """
    Create a gesture execution behavior for a specific gesture

    Args:
        gesture_name: Name of the gesture to execute (e.g., 'heart', 'wave', 'pointing')
        node: ROS2 node (optional, will create one if not provided)
        action_server_name: Name of the action server (default: 'execute_gesture')

    Returns:
        AutonomousGestureExecution behavior instance

    Example:
        >>> # Create a behavior to execute the 'heart' gesture
        >>> heart_behavior = create_gesture_behavior('heart')
        >>> heart_behavior.setup()
    """
    return AutonomousGestureExecution(
        name=f'execute_{gesture_name}',
        node=node,
        gesture_name=gesture_name,
        action_server_name=action_server_name
    )


def create_dynamic_gesture_behavior(name: str = 'gesture_executor', node=None, action_server_name='execute_gesture'):
    """
    Create a gesture execution behavior that listens for commands via topic

    This behavior doesn't execute a specific gesture initially, but waits for
    commands via the /gesture_command topic.

    Args:
        name: Behavior name (default: 'gesture_executor')
        node: ROS2 node (optional, will create one if not provided)
        action_server_name: Name of the action server (default: 'execute_gesture')

    Returns:
        AutonomousGestureExecution behavior instance

    Example:
        >>> # Create a dynamic behavior
        >>> dynamic_behavior = create_dynamic_gesture_behavior()
        >>> dynamic_behavior.setup()
        >>>
        >>> # Then publish a gesture command:
        >>> # ros2 topic pub /gesture_command std_msgs/msg/String "{data: 'heart'}"
    """
    return AutonomousGestureExecution(
        name=name,
        node=node,
        gesture_name=None,  # Will be set via topic
        action_server_name=action_server_name
    )


# Example integration with py_trees behavior tree
"""
EXAMPLE USAGE IN STATE MACHINE:

from smilei_state_machine.behaviors.autonomous_gesture_execution import (
    create_gesture_behavior,
    create_dynamic_gesture_behavior
)
import py_trees

# Option 1: Create a behavior for a specific gesture
heart_gesture = create_gesture_behavior('heart', node=your_node)

# Option 2: Create a dynamic behavior that responds to topic commands
dynamic_gesture = create_dynamic_gesture_behavior(node=your_node)

# Add to your behavior tree
root = py_trees.composites.Sequence(
    name="GestureSequence",
    memory=False,
    children=[
        create_gesture_behavior('wave', node=your_node),
        create_gesture_behavior('heart', node=your_node),
        create_gesture_behavior('pointing', node=your_node),
    ]
)

# Setup and tick
root.setup_with_descendants()
while True:
    root.tick_once()
    time.sleep(0.1)

TOPICS PUBLISHED:
- /gesture_execution_status (std_msgs/String): Current execution status
  Examples: "executing_heart_planning_50pct", "completed_wave_success", "idle"

- /gesture_executing (std_msgs/Bool): Whether a gesture is currently executing
  True when executing, False when idle

TOPICS SUBSCRIBED:
- /gesture_command (std_msgs/String): Send gesture name to execute
  Example: ros2 topic pub /gesture_command std_msgs/msg/String "{data: 'heart'}"

ACTION SERVER REQUIRED:
- /execute_gesture (smilei_dual_arm_ik_interfaces/action/ExecuteGesture)
  Provided by: ros2 launch smilei_dual_arm_ik gesture_executor_hardware.launch.py
"""
