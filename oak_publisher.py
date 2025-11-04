import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import subprocess
import time # For potential sleep
import argparse # <<< NEW: For command-line argument parsing
import sys      # <<< NEW: For sys.argv access
import threading
import queue

# Attempt to import depthai, but don't fail if it's not essential for webcam-only mode
try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False

class SmartWebcamPublisher(Node):
    # <<< MODIFIED: __init__ now accepts output_topic_name
    def __init__(self, output_topic_name):
        super().__init__('smart_webcam_publisher') # Node name is hardcoded
        
        # <<< MODIFIED: Use the provided topic name
        self.get_logger().info(f"Attempting to publish to ROS topic: {output_topic_name}")
        self.publisher = self.create_publisher(Image, output_topic_name, 10)
        self.bridge = CvBridge()

        self.cap = None
        self.oak_device = None
        self.oak_q_rgb = None
        self.using_oak_d = False
        self.gst_process = None

        # GStreamer Threading
        self.gst_queue = queue.Queue(maxsize=10) # Buffer up to 10 frames
        self.gst_thread = None
        self.gst_thread_stop_event = threading.Event()

        # Desired camera properties (can be made ROS parameters)
        self.frame_width = 640
        self.frame_height = 480
        self.fps = 30
        self.gstreamer_host = '192.168.0.100' # Make this a ROS parameter
        self.gstreamer_port = 5000         # Make this a ROS parameter

        gstreamer_input_format = 'bgr' # Both OAK-D (configured) and OpenCV provide BGR

        # 1. Try OAK-D Pro first
        if DEPTHAI_AVAILABLE:
            try:
                self.get_logger().info("Attempting to initialize OAK-D camera...")
                pipeline = dai.Pipeline()

                cam_rgb = pipeline.create(dai.node.ColorCamera)
                cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
                cam_rgb.setPreviewSize(self.frame_width, self.frame_height)
                cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
                cam_rgb.setInterleaved(False)
                cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
                cam_rgb.setFps(self.fps)

                xout_rgb = pipeline.create(dai.node.XLinkOut)
                xout_rgb.setStreamName("rgb")
                cam_rgb.preview.link(xout_rgb.input)

                self.oak_device = dai.Device(pipeline)
                self.oak_q_rgb = self.oak_device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
                self.using_oak_d = True
                self.get_logger().info(f"✅ OAK-D camera initialized successfully (Output: {self.frame_width}x{self.frame_height}@{self.fps}fps BGR).")

            except Exception as e:
                self.get_logger().warn(f"⚠️ Could not initialize OAK-D camera: {e}. Falling back to webcam.")
                if self.oak_device:
                    self.oak_device.close()
                self.oak_device = None
                self.using_oak_d = False
        else:
            self.get_logger().info("DepthAI library not found. Skipping OAK-D attempt, trying webcam.")

        # 2. Fallback to USB webcam if OAK-D is not used
        if not self.using_oak_d:
            self.get_logger().info("Attempting to initialize USB webcam...")
            camera_indices_to_try = [0, 1, 2, -1]
            for index in camera_indices_to_try:
                self.cap = cv2.VideoCapture(index)
                if self.cap.isOpened():
                    self.get_logger().info(f"Found webcam at index {index}.")
                    break
            
            if self.cap and self.cap.isOpened():
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                self.cap.set(cv2.CAP_PROP_FPS, self.fps)
                
                actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

                self.get_logger().info(f"✅ USB webcam initialized (Target: {self.frame_width}x{self.frame_height}@{self.fps}fps, Actual: {actual_width}x{actual_height}@{actual_fps}fps BGR).")
            else:
                self.get_logger().error("❌ Could not open any USB webcam!")
                return

        # 3. Start GStreamer pipeline if a camera is available
        if self.using_oak_d or (self.cap and self.cap.isOpened()):
            gst_command = [
                'gst-launch-1.0', '-v',
                'fdsrc', '!',
                'rawvideoparse', f'format={gstreamer_input_format}', f'width={self.frame_width}', f'height={self.frame_height}', f'framerate={self.fps}/1', '!',
                'queue', 'max-size-buffers=2', '!',
                'videoconvert', '!',
                'autovideosink', 'sync=false'
            ]
            try:
                self.get_logger().info(f"Starting GStreamer pipeline: {' '.join(gst_command)}")
                self.gst_process = subprocess.Popen(gst_command, stdin=subprocess.PIPE)
                self.get_logger().info("✅ GStreamer subprocess started.")
            except FileNotFoundError:
                self.get_logger().error("❌ gst-launch-1.0 not found. GStreamer will not be used.")
                self.gst_process = None
            except Exception as e:
                self.get_logger().error(f"❌ Failed to start GStreamer subprocess: {e}")
                self.gst_process = None

            if self.gst_process:
                self.gst_thread_stop_event.clear()
                self.gst_thread = threading.Thread(target=self._gstreamer_writer)
                self.gst_thread.daemon = True # Allow main thread to exit even if this one is running
                self.gst_thread.start()
                self.get_logger().info("✅ GStreamer writer thread started.")

        else:
            self.get_logger().warn("No camera source available, GStreamer not started.")


    def _gstreamer_writer(self):
        self.get_logger().info("GStreamer writer thread started.")
        while not self.gst_thread_stop_event.is_set():
            try:
                # Wait for a frame, but with a timeout to allow checking the stop event
                frame = self.gst_queue.get(timeout=0.5)
                if frame is None: # Sentinel value to stop the thread
                    break

                if self.gst_process and self.gst_process.stdin and not self.gst_process.stdin.closed:
                    try:
                        self.gst_process.stdin.write(frame.tobytes())
                    except BrokenPipeError:
                        self.get_logger().error("GStreamer pipe broke. Terminating GStreamer process from writer thread.")
                        if self.gst_process:
                            self.gst_process.terminate()
                        self.gst_process = None
                        break # Exit the writer thread
                    except Exception as e:
                        self.get_logger().error(f"Error writing to GStreamer in writer thread: {e}")
                
                self.gst_queue.task_done()

            except queue.Empty:
                # This is expected when the queue is empty, just continue
                continue
            except Exception as e:
                self.get_logger().error(f"An unexpected error occurred in the GStreamer writer thread: {e}")
                break
        self.get_logger().info("GStreamer writer thread finished.")


    def publish_loop(self):
        if not (self.using_oak_d or (self.cap and self.cap.isOpened())):
            self.get_logger().error("No camera source configured. Exiting publish loop.")
            return

        loop_count = 0
        while rclpy.ok():
            frame = None
            ret = False

            try:
                if self.using_oak_d and self.oak_q_rgb:
                    in_rgb = self.oak_q_rgb.tryGet()
                    if in_rgb is not None:
                        frame = in_rgb.getCvFrame()
                        ret = True
                elif self.cap and self.cap.isOpened():
                    ret, frame = self.cap.read()
                else:
                    self.get_logger().error("Camera source became unavailable unexpectedly.")
                    break

                if ret and frame is not None:
                    if frame.size == 0:
                        self.get_logger().warn("Received an empty frame.")
                        continue

                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_color_optical_frame"
                    self.publisher.publish(msg)

                    # Put frame into the queue for the GStreamer thread
                    if self.gst_thread and self.gst_thread.is_alive():
                        try:
                            # Don't block if the queue is full, just drop the frame
                            self.gst_queue.put_nowait(frame)
                        except queue.Full:
                            self.get_logger().warn("GStreamer queue is full. Dropping a frame.")

                    loop_count += 1
                    if loop_count % self.fps == 0:
                        self.get_logger().debug(f"Published frame {loop_count}")

                elif not ret and self.cap and not self.cap.isOpened():
                    self.get_logger().error("Webcam disconnected during loop.")
                    break
                elif not ret and self.using_oak_d:
                    pass

            except Exception as e:
                self.get_logger().error(f"Error in publish loop: {e}")
                import traceback
                self.get_logger().error(traceback.format_exc())
                break

        self.get_logger().info("Publish loop finished.")
        self.cleanup()

    def cleanup(self):
        self.get_logger().info("Cleaning up resources...")
        if self.cap:
            self.cap.release()
            self.get_logger().info("Webcam released.")
        if self.oak_device:
            self.oak_device.close()
            self.get_logger().info("OAK-D device closed.")

        # Stop the GStreamer writer thread first
        if self.gst_thread and self.gst_thread.is_alive():
            self.get_logger().info("Stopping GStreamer writer thread...")
            self.gst_thread_stop_event.set()
            self.gst_queue.put(None)  # Sentinel to unblock the queue.get()
            self.gst_thread.join(timeout=2) # Wait for the thread to finish
            if self.gst_thread.is_alive():
                self.get_logger().warn("GStreamer writer thread did not stop in time.")
            else:
                self.get_logger().info("GStreamer writer thread stopped.")
        
        if self.gst_process:
            self.get_logger().info("Terminating GStreamer process...")
            if self.gst_process.stdin and not self.gst_process.stdin.closed:
                try:
                    self.gst_process.stdin.close()
                except Exception as e:
                    self.get_logger().warn(f"Could not close GStreamer stdin: {e}")
            self.gst_process.terminate()
            try:
                self.gst_process.wait(timeout=5)
                self.get_logger().info("GStreamer process terminated.")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("GStreamer process did not terminate in time, killing.")
                self.gst_process.kill()
                self.gst_process.wait()
                self.get_logger().info("GStreamer process killed.")
            self.gst_process = None

        self.get_logger().info("Cleanup complete.")

def main(args=None):
    # <<< MODIFIED: Argument parsing section
    # Determine the arguments to parse. If args is None, use sys.argv[1:].
    # Otherwise, use the provided args list (e.g., from a launch file).
    cli_args_to_parse = sys.argv[1:] if args is None else args

    parser = argparse.ArgumentParser(
        description="ROS 2 node: Publishes images from OAK-D/USB webcam to a specified ROS topic and streams via GStreamer.",
        formatter_class=argparse.RawTextHelpFormatter # Allows for better formatting in help description
    )
    parser.add_argument(
        'output_topic',  # This makes it a positional, required argument
        type=str,
        help="REQUIRED: The ROS 2 topic to publish images to.\n(e.g., /camera/image_raw or /my_robot/front_camera/image_raw)"
    )

    # Parse known arguments. This allows ROS-specific arguments (e.g., --ros-args) to be present
    # without causing an error here. They will be handled by rclpy.init().
    # If 'output_topic' is missing or '-h' is used, argparse will print help and exit.
    parsed_custom_args, remaining_ros_args = parser.parse_known_args(cli_args_to_parse)
    
    # Initialize rclpy with the original args (or sys.argv if args was None)
    # rclpy will process its own arguments from this list.
    rclpy.init(args=args) 
    
    # <<< MODIFIED: Pass the parsed topic name to the constructor
    node = SmartWebcamPublisher(output_topic_name=parsed_custom_args.output_topic)
    
    if node.using_oak_d or (node.cap and node.cap.isOpened()):
        try:
            node.publish_loop()
        except KeyboardInterrupt:
            node.get_logger().info("KeyboardInterrupt received.")
        except Exception as e:
            node.get_logger().error(f"Unhandled exception in main: {e}")
            import traceback
            node.get_logger().error(traceback.format_exc())
        finally:
            node.get_logger().info("Shutting down node...")
            # Cleanup is called within publish_loop's end or its own except blocks
            node.destroy_node()
            rclpy.shutdown()
    else:
        node.get_logger().fatal("Failed to initialize any camera source. Node will not start publishing.")
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
