#!/usr/bin/env python3
"""
OAK-D publisher with RGB and Depth for 3D skeleton detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import sys
import numpy as np

# Import depthai
try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False
    print("ERROR: depthai not available")
    sys.exit(1)


class OakDepthPublisherNode(Node):
    def __init__(self):
        super().__init__('oak_depth_publisher_node')

        # Publishers
        self.rgb_publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.bridge = CvBridge()

        # Camera configuration
        self.frame_width = 640
        self.frame_height = 480
        self.fps = 30

        # Initialize OAK-D camera with stereo depth
        self.get_logger().info("Initializing OAK-D Pro with depth...")
        pipeline = dai.Pipeline()

        # RGB Camera
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setPreviewSize(self.frame_width, self.frame_height)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setFps(self.fps)

        # Stereo Depth
        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_right = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        # Stereo configuration for better accuracy
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)  # Align depth to RGB
        stereo.setOutputSize(self.frame_width, self.frame_height)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(False)
        stereo.setSubpixel(True)

        # Linking
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        # Outputs
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName("rgb")
        cam_rgb.preview.link(xout_rgb.input)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        # Start pipeline
        self.oak_device = dai.Device(pipeline)
        self.oak_q_rgb = self.oak_device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.oak_q_depth = self.oak_device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        # Get calibration data for camera info
        calib = self.oak_device.readCalibration()
        self.camera_info_msg = self.create_camera_info_msg(calib)

        self.get_logger().info(f"OAK-D Pro ready: {self.frame_width}x{self.frame_height}@{self.fps}fps with depth")

    def create_camera_info_msg(self, calib):
        """Create camera info message from calibration data"""
        msg = CameraInfo()
        msg.header.frame_id = "camera_color_optical_frame"
        msg.width = self.frame_width
        msg.height = self.frame_height

        # Get intrinsics for RGB camera
        intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_A, self.frame_width, self.frame_height)

        # Camera matrix (K)
        msg.k = [
            intrinsics[0][0], 0.0, intrinsics[0][2],
            0.0, intrinsics[1][1], intrinsics[1][2],
            0.0, 0.0, 1.0
        ]

        # Projection matrix (P)
        msg.p = [
            intrinsics[0][0], 0.0, intrinsics[0][2], 0.0,
            0.0, intrinsics[1][1], intrinsics[1][2], 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        # Distortion coefficients
        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]

        # Rectification matrix (identity)
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

        return msg

    def publish_loop(self):
        loop_count = 0
        while rclpy.ok():
            # Get current timestamp
            timestamp = self.get_clock().now().to_msg()

            # Get RGB frame
            in_rgb = self.oak_q_rgb.tryGet()
            if in_rgb is not None:
                rgb_frame = in_rgb.getCvFrame()

                if rgb_frame.size > 0:
                    # Publish RGB
                    rgb_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding="bgr8")
                    rgb_msg.header.stamp = timestamp
                    rgb_msg.header.frame_id = "camera_color_optical_frame"
                    self.rgb_publisher.publish(rgb_msg)

            # Get Depth frame
            in_depth = self.oak_q_depth.tryGet()
            if in_depth is not None:
                depth_frame = in_depth.getFrame()

                if depth_frame.size > 0:
                    # Publish Depth (16-bit, millimeters)
                    depth_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding="16UC1")
                    depth_msg.header.stamp = timestamp
                    depth_msg.header.frame_id = "camera_color_optical_frame"
                    self.depth_publisher.publish(depth_msg)

            # Publish camera info
            if in_rgb is not None or in_depth is not None:
                self.camera_info_msg.header.stamp = timestamp
                self.camera_info_publisher.publish(self.camera_info_msg)

                loop_count += 1
                if loop_count % 30 == 0:
                    self.get_logger().info(f"Published {loop_count} RGB+Depth frames")

        self.cleanup()

    def cleanup(self):
        self.get_logger().info("Cleaning up...")
        if self.oak_device:
            self.oak_device.close()
            self.get_logger().info("OAK-D device closed")


def main(args=None):
    rclpy.init(args=args)
    node = OakDepthPublisherNode()

    try:
        node.publish_loop()
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
