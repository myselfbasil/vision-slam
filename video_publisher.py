#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import argparse
import os

class VideoPublisherNode(Node):
    def __init__(self, video_path, loop=True, fps=30.0):
        super().__init__("video_publisher")
        
        # Publisher for camera images and camera info
        self.image_publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.camera_info_publisher = self.create_publisher(CameraInfo, "/camera/camera_info", 10)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Check if video file exists
        if not os.path.exists(video_path):
            self.get_logger().error(f"Video file not found: {video_path}")
            raise FileNotFoundError(f"Video file not found: {video_path}")
            
        # Open video file
        self.video = cv2.VideoCapture(video_path)
        if not self.video.isOpened():
            self.get_logger().error(f"Failed to open video file: {video_path}")
            raise RuntimeError(f"Failed to open video file: {video_path}")
            
        # Get video properties
        self.width = int(self.video.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.total_frames = int(self.video.get(cv2.CAP_PROP_FRAME_COUNT))
        self.original_fps = self.video.get(cv2.CAP_PROP_FPS)
        
        # Loop video if requested
        self.loop = loop
        
        # Set up timer for publishing frames
        publish_period = 1.0 / fps
        self.timer = self.create_timer(publish_period, self.publish_frame)
        
        # Create default camera info
        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.header.frame_id = "camera_link"
        self.camera_info_msg.height = self.height
        self.camera_info_msg.width = self.width
        
        # Set default camera matrix (approximate values for most cameras)
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
        
        self.get_logger().info(f"Video publisher initialized: {video_path}")
        self.get_logger().info(f"Resolution: {self.width}x{self.height}, FPS: {fps} (original: {self.original_fps})")
        self.get_logger().info(f"Total frames: {self.total_frames}, Loop: {loop}")

    def publish_frame(self):
        # Read frame from video
        ret, frame = self.video.read()
        
        # If we've reached the end of the video
        if not ret:
            if self.loop:
                # Reset video to beginning
                self.video.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ret, frame = self.video.read()
                if not ret:
                    self.get_logger().error("Failed to loop video")
                    return
                self.get_logger().info("Looping video back to start")
            else:
                self.get_logger().info("End of video reached")
                return
        
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
        
        # Log progress occasionally
        current_frame = int(self.video.get(cv2.CAP_PROP_POS_FRAMES))
        if current_frame % 100 == 0:
            self.get_logger().info(f"Publishing frame {current_frame}/{self.total_frames}", throttle_duration_sec=1.0)

    def destroy_node(self):
        if self.video.isOpened():
            self.video.release()
        super().destroy_node()

def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='ROS2 Video Publisher Node')
    parser.add_argument('--video', type=str, required=True, help='Path to video file')
    parser.add_argument('--loop', type=bool, default=True, help='Loop video when finished')
    parser.add_argument('--fps', type=float, default=30.0, help='Publishing rate (frames per second)')
    
    rclpy.init()
    
    # Get command line args or use defaults if run without args
    if len(os.sys.argv) > 1:
        args = parser.parse_args()
        node = VideoPublisherNode(args.video, args.loop, args.fps)
    else:
        # Default values if run without arguments
        node = VideoPublisherNode('/videos/input.mp4', True, 30.0)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
