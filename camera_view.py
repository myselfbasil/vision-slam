#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraViewNode(Node):
    def __init__(self):
        super().__init__("camera_view_node")
        
        # Create subscription to camera topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )
        
        # OpenCV bridge
        self.bridge = CvBridge()
        self.get_logger().info("Camera viewer started - waiting for images...")
        
        # Keep track of received frames
        self.frames_received = 0
        
        # Create a window for displaying the camera feed
        cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    
    def camera_callback(self, msg):
        self.frames_received += 1
        
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Log information about the frame
            if self.frames_received % 30 == 0 or self.frames_received < 5:
                height, width = cv_image.shape[:2]
                self.get_logger().info(f"Received frame {self.frames_received}: {width}x{height}")
            
            # Display the frame
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Process GUI events
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraViewNode()
    
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
