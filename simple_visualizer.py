#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import os

class SimpleVisualizer(Node):
    def __init__(self):
        super().__init__("simple_visualizer_node")
        
        # OpenCV bridge for converting ROS images
        self.bridge = CvBridge()
        
        # Subscribe to the camera image topic
        self.image_sub = self.create_subscription(
            Image, 
            "/camera/image_raw", 
            self.image_callback, 
            10
        )
        
        # Create diagnostic topics
        self.diagnostic_pub = self.create_publisher(
            Image, 
            "/visualization/debug", 
            10
        )
        
        # Data storage
        self.current_image = None
        self.image_lock = threading.Lock()
        self.frame_count = 0
        self.feature_points = []
        self.poses = []
        
        # Feature detector
        self.feature_detector = cv2.GFTTDetector_create(
            maxCorners=1000,
            qualityLevel=0.01,
            minDistance=10,
            blockSize=3
        )
        
        # Start display thread
        threading.Thread(target=self.display_thread, daemon=True).start()
        
        self.get_logger().info("Simple visualizer started")
        
    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Process image - detect features
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            features = self.feature_detector.detect(gray)
            
            # Draw features on image
            vis_image = cv_image.copy()
            for feature in features:
                x, y = feature.pt
                cv2.circle(vis_image, (int(x), int(y)), 3, (0, 255, 0), 1)
            
            # Store processed image
            with self.image_lock:
                self.current_image = vis_image
                self.frame_count += 1
                self.feature_points = features
            
            # Publish diagnostic image
            if self.frame_count % 10 == 0:
                debug_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding="bgr8")
                debug_msg.header.stamp = msg.header.stamp
                debug_msg.header.frame_id = msg.header.frame_id
                self.diagnostic_pub.publish(debug_msg)
                
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
            
    def display_thread(self):
        """Thread for displaying visualization"""
        cv2.namedWindow("SLAM Visualization", cv2.WINDOW_NORMAL)
        cv2.moveWindow("SLAM Visualization", 50, 50)
        
        # Wait for first image
        while rclpy.ok():
            with self.image_lock:
                image = self.current_image
            
            if image is not None:
                break
                
            time.sleep(0.1)
            
        # Main display loop
        while rclpy.ok():
            with self.image_lock:
                image = self.current_image
                if image is None:
                    continue
                frame_count = self.frame_count
                
            # Add text overlay
            cv2.putText(
                image, 
                f"Frame: {frame_count}", 
                (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                1, 
                (0, 255, 0), 
                2
            )
            
            # Add feature count
            cv2.putText(
                image, 
                f"Features: {len(self.feature_points)}", 
                (10, 60), 
                cv2.FONT_HERSHEY_SIMPLEX, 
                1, 
                (0, 255, 0), 
                2
            )
            
            # Show the image
            cv2.imshow("SLAM Visualization", image)
            cv2.waitKey(1)
            
            time.sleep(0.03)  # ~30fps
            
    def add_pose(self, pose):
        """Add a new pose to trajectory"""
        self.poses.append((pose.position.x, pose.position.y))
        
def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()
