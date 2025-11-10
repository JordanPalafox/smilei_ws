#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time


class SkeletonDetectorNode(Node):
    def __init__(self):
        super().__init__('skeleton_detector_node')

        # Create publisher for the skeleton visualization
        self.publisher = self.create_publisher(Image, '/skeleton/image', 10)

        # Performance monitoring
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0

        # Create subscriber to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.bridge = CvBridge()

        # Initialize MediaPipe Pose
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        # Configure pose detection - Optimized for Jetson AGX Orin
        # static_image_mode=False for video processing
        # model_complexity=0 for maximum speed
        # min_detection_confidence=0.3 lower threshold for faster detection
        # min_tracking_confidence=0.3 lower threshold for smoother tracking
        # smooth_landmarks=True for stable results
        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=0,  # 0 = fastest, best for real-time on Jetson
            smooth_landmarks=True,
            enable_segmentation=False,  # Disable segmentation for speed
            min_detection_confidence=0.3,
            min_tracking_confidence=0.3
        )

        # Define which landmarks to draw (excluding legs)
        # MediaPipe Pose has 33 landmarks (0-32)
        # We want to exclude: 25-32 (knees, ankles, feet)
        # Keep: 0-24 (face, torso, arms, hips)
        self.upper_body_landmarks = list(range(0, 25))

        # Define custom connections for upper body only
        # Original connections from MediaPipe, but filtered
        self.custom_connections = [
            # Face
            (0, 1), (1, 2), (2, 3), (3, 7),  # Left eye
            (0, 4), (4, 5), (5, 6), (6, 8),  # Right eye
            (9, 10),  # Mouth

            # Torso
            (11, 12),  # Shoulders
            (11, 23), (12, 24),  # Shoulders to hips
            (23, 24),  # Hips

            # Left arm
            (11, 13), (13, 15),  # Shoulder to elbow to wrist
            (15, 17), (15, 19), (15, 21),  # Wrist to hand
            (17, 19),  # Hand connections

            # Right arm
            (12, 14), (14, 16),  # Shoulder to elbow to wrist
            (16, 18), (16, 20), (16, 22),  # Wrist to hand
            (18, 20),  # Hand connections
        ]

        self.get_logger().info('Skeleton Detector Node initialized')
        self.get_logger().info('Subscribing to: /camera/image_raw')
        self.get_logger().info('Publishing to: /skeleton/image')
        self.get_logger().info('Legs will be ignored in skeleton detection')

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert BGR to RGB for MediaPipe
            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Process the image and detect pose
            results = self.pose.process(image_rgb)

            # Create a copy of the image to draw on
            annotated_image = cv_image.copy()

            if results.pose_landmarks:
                # Draw only upper body landmarks (no legs)
                self.draw_upper_body_skeleton(annotated_image, results.pose_landmarks)

            # Convert back to ROS Image message
            skeleton_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            skeleton_msg.header = msg.header  # Keep the same header

            # Publish the annotated image
            self.publisher.publish(skeleton_msg)

            # FPS calculation
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                current_time = time.time()
                elapsed = current_time - self.last_fps_time
                self.fps = 30.0 / elapsed
                self.last_fps_time = current_time
                self.get_logger().info(f'FPS: {self.fps:.1f}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def draw_upper_body_skeleton(self, image, landmarks):
        """
        Draw skeleton on the image, excluding legs - Optimized for speed
        """
        height, width = image.shape[:2]
        visibility_threshold = 0.4  # Lowered for better detection

        # Pre-compute all points for faster access
        points = []
        for idx in self.upper_body_landmarks:
            landmark = landmarks.landmark[idx]
            if landmark.visibility > visibility_threshold:
                points.append((
                    idx,
                    int(landmark.x * width),
                    int(landmark.y * height),
                    landmark.visibility
                ))
            else:
                points.append((idx, -1, -1, landmark.visibility))

        # Create a lookup dictionary for faster access
        point_dict = {p[0]: (p[1], p[2], p[3]) for p in points}

        # Draw connections (lines between landmarks)
        for start_idx, end_idx in self.custom_connections:
            if start_idx in point_dict and end_idx in point_dict:
                start_x, start_y, start_vis = point_dict[start_idx]
                end_x, end_y, end_vis = point_dict[end_idx]

                if (start_vis > visibility_threshold and
                    end_vis > visibility_threshold and
                    start_x >= 0 and end_x >= 0):
                    cv2.line(image, (start_x, start_y), (end_x, end_y),
                            (0, 255, 0), 2, cv2.LINE_AA)

        # Draw landmarks (circles at joint positions)
        for idx, x, y, vis in points:
            if vis > visibility_threshold and x >= 0:
                cv2.circle(image, (x, y), 4, (0, 0, 255), -1, cv2.LINE_AA)

    def destroy_node(self):
        # Clean up MediaPipe resources
        self.pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SkeletonDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
