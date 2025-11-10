#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import os

# MediaPipe landmark names for upper body (0-24)
LANDMARK_NAMES = [
    "nose", "left_eye_inner", "left_eye", "left_eye_outer",
    "right_eye_inner", "right_eye", "right_eye_outer",
    "left_ear", "right_ear", "mouth_left", "mouth_right",
    "left_shoulder", "right_shoulder",
    "left_elbow", "right_elbow",
    "left_wrist", "right_wrist",
    "left_pinky", "right_pinky",
    "left_index", "right_index",
    "left_thumb", "right_thumb",
    "left_hip", "right_hip"
]


class TensorViewer(Node):
    def __init__(self):
        super().__init__('tensor_viewer')

        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/skeleton/joints_tensor',
            self.tensor_callback,
            10
        )

        self.frame_count = 0
        self.get_logger().info('Skeleton Tensor Viewer initialized')
        self.get_logger().info('Listening to /skeleton/joints_tensor...')
        print("\n" + "="*70)
        print("SKELETON JOINTS TENSOR VIEWER")
        print("="*70)
        print("Format: [25 joints x 3 coordinates (X, Y, Z)]")
        print("NaN = Joint not detected | Mouth joints (9, 10) excluded")
        print("="*70 + "\n")

    def tensor_callback(self, msg):
        self.frame_count += 1

        # Only display every 30 frames (~1 second at 30 FPS)
        if self.frame_count % 30 != 0:
            return

        # Clear screen
        os.system('clear')

        # Reshape flat array to [25, 3]
        tensor = np.array(msg.data).reshape(25, 3)

        print("\n" + "="*70)
        print(f"FRAME {self.frame_count} - Skeleton Joints Tensor")
        print("="*70)
        print(f"{'IDX':<4} {'JOINT NAME':<20} {'X (m)':<12} {'Y (m)':<12} {'Z (m)':<12}")
        print("-"*70)

        detected_count = 0
        for idx in range(25):
            # Skip mouth joints (9, 10) - they are ignored
            if idx in [9, 10]:
                continue

            x, y, z = tensor[idx]

            if not np.isnan(x):
                detected_count += 1
                print(f"{idx:<4} {LANDMARK_NAMES[idx]:<20} "
                      f"{x:>11.4f}  {y:>11.4f}  {z:>11.4f}")
            else:
                print(f"{idx:<4} {LANDMARK_NAMES[idx]:<20} "
                      f"{'---':>11}  {'---':>11}  {'---':>11}")

        print("-"*70)
        print(f"Detected joints: {detected_count}/23 (mouth excluded)")
        print("="*70)

        # Show tensor shape and statistics
        valid_joints = tensor[~np.isnan(tensor).any(axis=1)]
        if len(valid_joints) > 0:
            print(f"\nTensor Statistics (valid joints only):")
            print(f"  X range: [{valid_joints[:, 0].min():.4f}, {valid_joints[:, 0].max():.4f}] m")
            print(f"  Y range: [{valid_joints[:, 1].min():.4f}, {valid_joints[:, 1].max():.4f}] m")
            print(f"  Z range: [{valid_joints[:, 2].min():.4f}, {valid_joints[:, 2].max():.4f}] m")

        print("\n" + "="*70)
        print("Press Ctrl+C to exit")
        print("="*70 + "\n")


def main(args=None):
    rclpy.init(args=args)
    viewer = TensorViewer()

    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        viewer.get_logger().info('Viewer stopped by user')
    finally:
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
