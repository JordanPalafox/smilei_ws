#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, Point
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time
import message_filters


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


class SkeletonDetector3DNode(Node):
    def __init__(self):
        super().__init__('skeleton_detector_3d_node')

        # Publishers (minimal queue for low latency)
        self.image_publisher = self.create_publisher(Image, '/skeleton/image', 1)
        self.poses_3d_publisher = self.create_publisher(PoseArray, '/skeleton/poses_3d', 1)
        self.joints_tensor_publisher = self.create_publisher(Float32MultiArray, '/skeleton/joints_tensor', 1)

        # Synchronized subscribers for RGB and Depth
        self.rgb_sub = message_filters.Subscriber(self, Image, '/camera/image_raw')
        self.depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')

        # Synchronize RGB and Depth (reduced queue for lower latency)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub],
            queue_size=2,  # Minimal queue for lower latency
            slop=0.05      # 50ms tolerance
        )
        self.ts.registerCallback(self.sync_callback)

        # Subscribe to camera info for intrinsics
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            10
        )

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Depth filtering parameters (balanced for stability + responsiveness)
        self.depth_filter_alpha = 0.4   # EMA filter strength (0-1, lower = smoother)
        self.spatial_filter_size = 5    # Kernel size for spatial averaging
        self.max_jump_distance = 0.20   # Max movement between frames in meters
        self.previous_positions = {}    # Store previous 3D positions for temporal filtering

        # Initialize MediaPipe Pose - Optimized for Jetson AGX Orin
        self.mp_pose = mp.solutions.pose
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

        self.pose = self.mp_pose.Pose(
            static_image_mode=False,
            model_complexity=0,  # Fastest for real-time on Jetson
            smooth_landmarks=True,
            enable_segmentation=False,
            min_detection_confidence=0.3,  # Optimized for speed
            min_tracking_confidence=0.3
        )

        # Define which landmarks to use (excluding legs and mouth)
        self.upper_body_landmarks = list(range(0, 9)) + list(range(11, 25))  # Skip mouth (9, 10)

        # Define custom connections for upper body only (excluding mouth)
        self.custom_connections = [
            # Face
            (0, 1), (1, 2), (2, 3), (3, 7),  # Left eye
            (0, 4), (4, 5), (5, 6), (6, 8),  # Right eye
            # Mouth removed

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

        # Performance monitoring
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0

        self.get_logger().info('3D Skeleton Detector Node initialized')
        self.get_logger().info('Subscribing to: /camera/image_raw and /camera/depth/image_raw')
        self.get_logger().info('Publishing to: /skeleton/image, /skeleton/poses_3d, and /skeleton/joints_tensor')
        self.get_logger().info('Joints tensor format: [25, 3] = [joints, xyz] (NaN for undetected)')
        self.get_logger().info('Legs and mouth will be ignored in skeleton detection')
        self.get_logger().info(f'Depth filtering enabled: spatial={self.spatial_filter_size}x{self.spatial_filter_size}, temporal_alpha={self.depth_filter_alpha}, max_jump={self.max_jump_distance}m')

    def camera_info_callback(self, msg):
        """Extract camera intrinsics from CameraInfo message"""
        if self.camera_matrix is None:
            self.fx = msg.k[0]  # Focal length X
            self.fy = msg.k[4]  # Focal length Y
            self.cx = msg.k[2]  # Principal point X
            self.cy = msg.k[5]  # Principal point Y

            self.camera_matrix = np.array([
                [self.fx, 0, self.cx],
                [0, self.fy, self.cy],
                [0, 0, 1]
            ])

            self.get_logger().info(f'Camera intrinsics received: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}')

    def get_filtered_depth(self, depth_image, x, y):
        """
        Get spatially filtered depth value around a pixel

        Args:
            depth_image: Depth image
            x, y: Pixel coordinates

        Returns:
            Filtered depth in mm (median of surrounding area)
        """
        height, width = depth_image.shape
        half_size = self.spatial_filter_size // 2

        # Get bounds
        y_min = max(0, y - half_size)
        y_max = min(height, y + half_size + 1)
        x_min = max(0, x - half_size)
        x_max = min(width, x + half_size + 1)

        # Extract region
        region = depth_image[y_min:y_max, x_min:x_max]

        # Filter out zeros (invalid depth)
        valid_depths = region[region > 0]

        if len(valid_depths) == 0:
            return 0

        # Use median to reject outliers
        return np.median(valid_depths)

    def apply_temporal_filter(self, idx, new_position):
        """
        Apply exponential moving average filter to 3D position with jump protection

        Args:
            idx: Joint index
            new_position: New 3D position (x, y, z)

        Returns:
            Filtered 3D position
        """
        new_pos_array = np.array(new_position, dtype=np.float32)

        if idx not in self.previous_positions:
            # First time seeing this joint
            self.previous_positions[idx] = new_pos_array
            return new_position

        prev_pos = self.previous_positions[idx]

        # Check for unrealistic jumps (likely noise/errors)
        distance = np.linalg.norm(new_pos_array - prev_pos)

        if distance > self.max_jump_distance:
            # Jump too large - likely noise, use more of previous position
            filtered_pos = 0.1 * new_pos_array + 0.9 * prev_pos
        else:
            # Normal movement - apply EMA
            filtered_pos = (self.depth_filter_alpha * new_pos_array +
                           (1 - self.depth_filter_alpha) * prev_pos)

        # Update previous position
        self.previous_positions[idx] = filtered_pos

        return tuple(filtered_pos)

    def pixel_to_3d(self, x_pixel, y_pixel, depth_mm):
        """
        Convert pixel coordinates and depth to 3D position

        Args:
            x_pixel: X coordinate in pixels
            y_pixel: Y coordinate in pixels
            depth_mm: Depth in millimeters

        Returns:
            (x, y, z) in meters in camera frame
        """
        if self.fx is None:
            return None

        # Convert depth to meters
        z = depth_mm / 1000.0

        # Convert pixel to 3D using pinhole camera model
        x = (x_pixel - self.cx) * z / self.fx
        y = (y_pixel - self.cy) * z / self.fy

        return (x, y, z)

    def sync_callback(self, rgb_msg, depth_msg):
        """Process synchronized RGB and Depth images"""
        try:
            # Convert ROS Image messages to OpenCV images
            cv_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

            # Convert BGR to RGB for MediaPipe
            image_rgb = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Process the image and detect pose
            results = self.pose.process(image_rgb)

            # Create a copy of the image to draw on
            annotated_image = cv_image.copy()

            # Prepare 3D poses message
            poses_3d_msg = PoseArray()
            poses_3d_msg.header = rgb_msg.header

            # Initialize joints tensor (25 joints x 3 coordinates)
            # Use NaN for undetected joints
            joints_tensor = np.full((25, 3), np.nan, dtype=np.float32)

            if results.pose_landmarks and self.camera_matrix is not None:
                # Draw skeleton and extract 3D positions
                height, width = cv_image.shape[:2]

                for idx in self.upper_body_landmarks:
                    landmark = results.pose_landmarks.landmark[idx]

                    if landmark.visibility > 0.4:
                        # Get pixel coordinates
                        x_pixel = int(landmark.x * width)
                        y_pixel = int(landmark.y * height)

                        # Ensure within bounds
                        x_pixel = max(0, min(x_pixel, width - 1))
                        y_pixel = max(0, min(y_pixel, height - 1))

                        # Get filtered depth value (spatial + median filter)
                        depth_mm = self.get_filtered_depth(depth_image, x_pixel, y_pixel)

                        # Calculate 3D position
                        if depth_mm > 0:  # Valid depth
                            pos_3d_raw = self.pixel_to_3d(x_pixel, y_pixel, depth_mm)

                            if pos_3d_raw is not None:
                                # Apply temporal filter for smoother tracking
                                pos_3d = self.apply_temporal_filter(idx, pos_3d_raw)

                                # Store in tensor
                                joints_tensor[idx] = [pos_3d[0], pos_3d[1], pos_3d[2]]

                                # Create Pose message for this landmark
                                pose = Pose()
                                pose.position.x = float(pos_3d[0])
                                pose.position.y = float(pos_3d[1])
                                pose.position.z = float(pos_3d[2])
                                poses_3d_msg.poses.append(pose)

                                # Draw on image with depth info
                                cv2.circle(annotated_image, (x_pixel, y_pixel), 4,
                                          (0, 0, 255), -1, cv2.LINE_AA)

                                # Add depth text near the point
                                depth_text = f"{pos_3d[2]:.2f}m"
                                cv2.putText(annotated_image, depth_text,
                                           (x_pixel + 5, y_pixel - 5),
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.3,
                                           (255, 255, 255), 1, cv2.LINE_AA)
                        else:
                            # No valid depth, draw point without 3D info
                            cv2.circle(annotated_image, (x_pixel, y_pixel), 4,
                                      (128, 128, 128), -1, cv2.LINE_AA)

                # Draw connections
                self.draw_connections(annotated_image, results.pose_landmarks)

            # Convert back to ROS Image message
            skeleton_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            skeleton_msg.header = rgb_msg.header

            # Create and publish joints tensor message
            tensor_msg = Float32MultiArray()
            tensor_msg.layout.dim.append(MultiArrayDimension())
            tensor_msg.layout.dim.append(MultiArrayDimension())
            tensor_msg.layout.dim[0].label = "joints"
            tensor_msg.layout.dim[0].size = 25
            tensor_msg.layout.dim[0].stride = 75  # 25 * 3
            tensor_msg.layout.dim[1].label = "coordinates"
            tensor_msg.layout.dim[1].size = 3
            tensor_msg.layout.dim[1].stride = 3
            tensor_msg.layout.data_offset = 0
            # Flatten tensor to 1D array
            tensor_msg.data = joints_tensor.flatten().tolist()

            # Publish messages
            self.image_publisher.publish(skeleton_msg)
            self.joints_tensor_publisher.publish(tensor_msg)

            if len(poses_3d_msg.poses) > 0:
                self.poses_3d_publisher.publish(poses_3d_msg)

            # FPS calculation
            self.frame_count += 1
            if self.frame_count % 30 == 0:
                current_time = time.time()
                elapsed = current_time - self.last_fps_time
                self.fps = 30.0 / elapsed
                self.last_fps_time = current_time
                self.get_logger().info(f'FPS: {self.fps:.1f}, 3D Points: {len(poses_3d_msg.poses)}')

        except Exception as e:
            self.get_logger().error(f'Error processing images: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    def draw_connections(self, image, landmarks):
        """Draw skeleton connections on the image"""
        height, width = image.shape[:2]
        visibility_threshold = 0.4

        for start_idx, end_idx in self.custom_connections:
            if start_idx < len(landmarks.landmark) and end_idx < len(landmarks.landmark):
                start_landmark = landmarks.landmark[start_idx]
                end_landmark = landmarks.landmark[end_idx]

                if (start_landmark.visibility > visibility_threshold and
                    end_landmark.visibility > visibility_threshold):

                    start_x = int(start_landmark.x * width)
                    start_y = int(start_landmark.y * height)
                    end_x = int(end_landmark.x * width)
                    end_y = int(end_landmark.y * height)

                    cv2.line(image, (start_x, start_y), (end_x, end_y),
                            (0, 255, 0), 2, cv2.LINE_AA)

    def destroy_node(self):
        # Clean up MediaPipe resources
        self.pose.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SkeletonDetector3DNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
