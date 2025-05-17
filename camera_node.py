#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self, camera_device=0, fps=30.0):
        super().__init__("camera_node")
        
        # Publisher for camera images and camera info
        self.image_publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Print available cameras for diagnostics
        self.get_logger().info("Checking available cameras...")
        import os
        os.system("ls -l /dev/video*")
        
        # Try different camera backends
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]
        for backend in backends:
            self.get_logger().info(f"Trying camera backend: {backend}")
            # Open camera device
            self.camera = cv2.VideoCapture(camera_device, backend)
            if self.camera.isOpened():
                self.get_logger().info(f"Successfully opened camera with backend {backend}")
                break
            else:
                self.get_logger().warn(f"Failed to open camera with backend {backend}")
        
        if not self.camera.isOpened():
            self.get_logger().error(f"Failed to open any camera device {camera_device}")
            # Don't raise exception, just report the error
            self.get_logger().error("No camera available, will continue without camera feed")
            
        # Get camera properties
        self.width = int(self.camera.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        # Set up timer for publishing frames
        publish_period = 1.0 / fps
        self.timer = self.create_timer(publish_period, self.publish_frame)
        
        # Create default camera info
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
        
        self.get_logger().info(f"Camera node initialized")
        self.get_logger().info(f"Resolution: {self.width}x{self.height}, FPS: {fps}")

    def publish_frame(self):
        # Read frame from camera
        ret, frame = self.camera.read()
        
        # If failed to read frame
        if not ret:
            self.get_logger().error("Failed to capture frame from camera")
            # Try to create a test frame with diagnostic information
            height, width = 480, 640
            frame = np.zeros((height, width, 3), dtype=np.uint8)
            cv2.putText(frame, "NO CAMERA FEED AVAILABLE", (50, height//2), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.putText(frame, "Check camera permissions", (80, height//2 + 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            # Log if frame was captured successfully
            if self.get_logger().get_effective_level() <= 20:  # Only log at INFO level or below
                height, width = frame.shape[:2]
                self.get_logger().info(f"Captured frame: {width}x{height}")
        
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

    def destroy_node(self):
        if self.camera.isOpened():
            self.camera.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    # Try to list available cameras
    import os
    os.system("echo 'Available video devices:'")
    os.system("ls -l /dev/video*")
    os.system("echo 'v4l2 device capabilities:'")
    os.system("v4l2-ctl --list-devices || echo 'v4l2-ctl not available'")
    
    # Default camera device (usually 0 for built-in webcam)
    camera_device = 0
    
    # Create and spin the node
    print("Creating camera node...")
    node = CameraNode(camera_device)
    
    try:
        print("Starting camera node...")
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
