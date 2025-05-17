#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import time

class FlexibleCameraNode(Node):
    def __init__(self):
        super().__init__("flexible_camera_node")
        
        # Publisher for camera images and camera info
        self.image_publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Print system info
        self.get_logger().info("System information:")
        os.system("uname -a")
        os.system("ls -l /dev/video*")
        
        # Try to find a working camera
        self.camera = None
        self.width = 640
        self.height = 480
        
        # Try multiple camera indices and backends
        self.try_all_cameras()
        
        if self.camera is None or not self.camera.isOpened():
            self.get_logger().error("Could not open any camera, will generate test pattern")
            self.camera = None
        else:
            # Get camera properties if camera opened
            self.width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(f"Camera opened: {self.width}x{self.height}")
            
        # Create timer for publishing frames
        self.timer = self.create_timer(1.0/30.0, self.publish_frame)
        
        # Setup camera info message
        self.setup_camera_info()
        
        # Frame counter
        self.frame_count = 0
        
    def try_all_cameras(self):
        """Try multiple camera indices and backends"""
        # Try different device indices
        devices = list(range(4))  # Try devices 0-3
        
        # Try different backends
        backends = [cv2.CAP_ANY, cv2.CAP_V4L2, cv2.CAP_GSTREAMER, None]
        
        # Try different camera access methods
        for device in devices:
            for backend in backends:
                try:
                    self.get_logger().info(f"Trying camera device {device} with backend {backend}")
                    
                    if backend is None:
                        cap = cv2.VideoCapture(device)
                    else:
                        cap = cv2.VideoCapture(device, backend)
                        
                    if cap.isOpened():
                        # Try to read a frame to confirm it works
                        ret, frame = cap.read()
                        if ret and frame is not None and frame.size > 0:
                            self.get_logger().info(f"Successfully opened camera {device} with backend {backend}")
                            self.camera = cap
                            return
                        else:
                            self.get_logger().warn(f"Could open camera {device} but reading frames failed")
                            cap.release()
                    else:
                        self.get_logger().warn(f"Failed to open camera {device} with backend {backend}")
                        
                except Exception as e:
                    self.get_logger().error(f"Error trying camera {device}: {str(e)}")
        
        self.get_logger().error("Failed to open any camera")
                    
    def setup_camera_info(self):
        """Set up the camera info message"""
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_link"
        self.camera_info_msg.height = self.height
        self.camera_info_msg.width = self.width
        
        # Set default camera matrix (approximate values for most webcams)
        fx = self.width * 0.8  # focal length
        fy = self.width * 0.8
        cx = self.width / 2.0   # principal point
        cy = self.height / 2.0
        
        # Set camera matrix K (3x3 projection matrix) 
        self.camera_info_msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        
        # Set projection matrix P (3x4 projection matrix)
        self.camera_info_msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        
        # Initialize empty distortion parameters
        self.camera_info_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
    def create_test_pattern(self):
        """Create a test pattern when no camera is available"""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Current time for animation
        t = time.time()
        
        # Draw a colorful test pattern
        for y in range(self.height):
            for x in range(self.width):
                b = int(128 + 127 * np.sin(x / 50.0 + t))
                g = int(128 + 127 * np.sin(y / 50.0 + t))
                r = int(128 + 127 * np.sin((x+y) / 50.0 + t))
                frame[y, x] = [b, g, r]
        
        # Add grid lines
        for y in range(0, self.height, 40):
            cv2.line(frame, (0, y), (self.width, y), (0, 0, 0), 1)
        for x in range(0, self.width, 40):
            cv2.line(frame, (x, 0), (x, self.height), (0, 0, 0), 1)
            
        # Add text
        cv2.putText(frame, "NO CAMERA AVAILABLE", (50, self.height//2 - 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, "USING TEST PATTERN", (70, self.height//2 + 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(frame, f"Frame: {self.frame_count}", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    
        return frame
    
    def publish_frame(self):
        """Read camera frame (or generate test pattern) and publish it"""
        frame = None
        
        # Try to read from camera if available
        if self.camera is not None:
            ret, frame = self.camera.read()
            if not ret or frame is None or frame.size == 0:
                self.get_logger().warn("Failed to capture camera frame, using test pattern")
                frame = self.create_test_pattern()
                
                # Try to reinitialize camera occasionally
                if self.frame_count % 100 == 0:
                    self.get_logger().info("Trying to reinitialize camera...")
                    self.try_all_cameras()
        else:
            # Generate test pattern if no camera
            frame = self.create_test_pattern()
            
        self.frame_count += 1
            
        # Get current timestamp
        now = self.get_clock().now().to_msg()
        
        # Create and publish image message
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        img_msg.header.stamp = now
        img_msg.header.frame_id = "camera_link"
        self.image_publisher.publish(img_msg)
        
        # Publish camera info with same timestamp
        self.camera_info_msg.header.stamp = now
        self.camera_info_publisher.publish(self.camera_info_msg)
        
        # Log periodic status
        if self.frame_count % 100 == 0:
            self.get_logger().info(f"Published {self.frame_count} frames")
    
    def destroy_node(self):
        if self.camera is not None:
            self.camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Create and spin node
    node = FlexibleCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in camera node: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
