#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import subprocess
import threading
import os

class DirectCameraNode(Node):
    def __init__(self):
        super().__init__("direct_camera_node")
        
        # Publisher for camera images and camera info
        self.image_publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Camera setup
        self.setup_camera()
        
        # Setup camera info
        self.setup_camera_info()
        
        # Timer for publishing frames
        self.timer = self.create_timer(1.0/30.0, self.publish_frame)
        
        # Frame counter
        self.frames_published = 0
        
        # Launch RTAB-Map viewer in a separate thread
        self.get_logger().info("Starting RTAB-Map viewer...")
        threading.Thread(target=self.launch_rtabmap_viewer, daemon=True).start()
        
    def setup_camera(self):
        """Try to open the camera with multiple backends"""
        self.get_logger().info("Checking available cameras...")
        os.system("ls -l /dev/video*")
        
        # Try multiple backends and device indices
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY, None]
        devices = [0, 1]
        
        for device in devices:
            for backend in backends:
                try:
                    self.get_logger().info(f"Trying camera {device} with backend {backend}")
                    
                    if backend is None:
                        cap = cv2.VideoCapture(device)
                    else:
                        cap = cv2.VideoCapture(device, backend)
                    
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret and frame is not None and frame.size > 0:
                            self.get_logger().info(f"Successfully opened camera {device}")
                            self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            self.camera = cap
                            
                            # Display camera info
                            self.get_logger().info(f"Camera resolution: {self.width}x{self.height}")
                            return
                except Exception as e:
                    self.get_logger().error(f"Error opening camera {device}: {str(e)}")
        
        # If no camera could be accessed, use test pattern
        self.get_logger().error("Failed to open any camera, will use test pattern")
        self.camera = None
        self.width = 640
        self.height = 480
    
    def setup_camera_info(self):
        """Set up the camera info message with reasonable defaults"""
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_link"
        self.camera_info_msg.height = self.height
        self.camera_info_msg.width = self.width
        
        # Use approximate camera parameters (focal length and principal point)
        fx = self.width * 0.8
        fy = self.width * 0.8
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        # Set up the camera intrinsic parameters
        self.camera_info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        
        # Set projection matrix P (3x4 projection matrix)
        self.camera_info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        # Initialize empty distortion parameters (assume no distortion)
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Identity rotation matrix
        self.camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
    def create_test_pattern(self):
        """Create a test pattern when camera is not available"""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Create a dynamic test pattern
        t = time.time()
        
        # Draw a moving gradient
        for y in range(self.height):
            for x in range(self.width):
                b = int(128 + 127 * np.sin(x / 30.0 + t))
                g = int(128 + 127 * np.sin(y / 30.0 + t * 1.5))
                r = int(128 + 127 * np.sin((x+y) / 30.0 + t * 2.0))
                frame[y, x] = [b, g, r]
        
        # Draw a grid
        for y in range(0, self.height, 40):
            cv2.line(frame, (0, y), (self.width, y), (0, 0, 0), 1)
        for x in range(0, self.width, 40):
            cv2.line(frame, (x, 0), (x, self.height), (0, 0, 0), 1)
            
        # Add frame counter
        cv2.putText(frame, f"Frame {self.frames_published}", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Add test pattern text
        cv2.putText(frame, "TEST PATTERN", (self.width//2 - 80, self.height//2), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    
        return frame
        
    def publish_frame(self):
        """Capture and publish camera frame"""
        frame = None
        
        # Try to capture from camera first
        if self.camera is not None:
            ret, frame = self.camera.read()
            if not ret or frame is None or frame.size == 0:
                self.get_logger().warn("Failed to capture camera frame, using test pattern")
                frame = self.create_test_pattern()
        else:
            # Use test pattern if no camera
            frame = self.create_test_pattern()
        
        # Get current timestamp for both messages
        current_time = self.get_clock().now().to_msg()
        
        # Publish image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = current_time
        img_msg.header.frame_id = "camera_link"
        self.image_publisher.publish(img_msg)
        
        # Publish camera info with same timestamp
        self.camera_info_msg.header.stamp = current_time
        self.camera_info_publisher.publish(self.camera_info_msg)
        
        self.frames_published += 1
        
        # Log periodically
        if self.frames_published % 100 == 0:
            self.get_logger().info(f"Published {self.frames_published} frames")
            
    def launch_rtabmap_viewer(self):
        """Launch RTAB-Map viewer in a separate process"""
        time.sleep(2)  # Wait a bit for camera node to start publishing
        
        self.get_logger().info("Launching RTAB-Map in display mode...")
        
        # Try with direct display settings
        env = os.environ.copy()
        env["QT_X11_NO_MITSHM"] = "1"
        env["XDG_RUNTIME_DIR"] = "/tmp/runtime-root"
        
        # Try methods in order of reliability
        methods = [
            # Method 1: rtabmap_viz directly
            {
                "name": "rtabmap_viz direct",
                "cmd": [
                    "ros2", "run", "rtabmap_viz", "rtabmap_viz",
                    "--ros-args",
                    "-p", "subscribe_depth:=false",
                    "-p", "subscribe_rgb:=true", 
                    "-p", "subscribe_stereo:=false",
                    "-p", "subscribe_scan:=false",
                    "-p", "approx_sync:=true",
                    "-r", "rgb/image:=/camera/image_raw",
                    "-r", "rgb/camera_info:=/camera/camera_info"
                ]
            },
            # Method 2: rtabmap directly
            {
                "name": "rtabmap direct",
                "cmd": [
                    "ros2", "run", "rtabmap_slam", "rtabmap",
                    "--ros-args",
                    "-p", "subscribe_depth:=false",
                    "-p", "subscribe_rgb:=true",
                    "-p", "subscribe_stereo:=false",
                    "-p", "subscribe_scan:=false",
                    "-p", "approx_sync:=true",
                    "-p", "Vis/MaxFeatures:=600",
                    "-p", "Vis/EstimationType:=0", 
                    "-p", "RGBD/OptimizeMaxError:=1.0", 
                    "-r", "rgb/image:=/camera/image_raw",
                    "-r", "rgb/camera_info:=/camera/camera_info"
                ]
            },
            # Method 3: shell command as last resort
            {
                "name": "shell command",
                "cmd": [
                    "bash", "-c", 
                    "source /opt/ros/humble/setup.bash && " +
                    "export QT_X11_NO_MITSHM=1 && " +
                    "export XDG_RUNTIME_DIR=/tmp/runtime-root && " +
                    "export QT_DEBUG_PLUGINS=1 && " +
                    "ros2 run rtabmap_viz rtabmap_viz --ros-args " +
                    "-p subscribe_depth:=false -p subscribe_rgb:=true " +
                    "-r rgb/image:=/camera/image_raw -r rgb/camera_info:=/camera/camera_info"
                ]
            }
        ]
        
        # Try each method until one works
        for method in methods:
            try:
                self.get_logger().info(f"Trying method: {method['name']}")
                self.get_logger().info(f"Running: {' '.join(method['cmd'])}")
                
                # Run detached
                process = subprocess.Popen(method['cmd'], env=env, 
                                         stdout=subprocess.PIPE, 
                                         stderr=subprocess.PIPE)
                
                # Wait briefly to see if it crashes immediately
                time.sleep(1)
                
                # Check if still running
                if process.poll() is None:
                    self.get_logger().info(f"Method {method['name']} appears to be working")
                    break
                else:
                    self.get_logger().warn(f"Method {method['name']} failed immediately")
            except Exception as e:
                self.get_logger().error(f"Error with method {method['name']}: {str(e)}")
        
        self.get_logger().info("RTAB-Map launch attempted, continuing with camera streaming")

def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin the node
    node = DirectCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if hasattr(node, 'camera') and node.camera is not None:
            node.camera.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
