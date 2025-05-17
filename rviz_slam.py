#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster
import cv2
import numpy as np
import time
from std_msgs.msg import Header
import math
import os
import subprocess
import threading

class RvizSlamNode(Node):
    def __init__(self):
        super().__init__("rviz_slam_node")
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Create image subscription
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Transform broadcaster for the camera frame
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Path publisher for trajectory visualization
        self.path_pub = self.create_publisher(Path, '/slam/trajectory', 10)
        
        # Point cloud publisher for map
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/slam/map_points', 10)
        
        # Publishers for processed images
        self.feature_img_pub = self.create_publisher(Image, '/slam/feature_image', 10)
        
        # Initialize state variables
        self.frame_count = 0
        self.last_feature_time = time.time()
        self.camera_path = Path()
        self.camera_path.header.frame_id = "map"
        self.camera_pose = PoseStamped()
        self.camera_pose.header.frame_id = "map"
        
        # Feature detector and tracker
        self.feature_detector = cv2.GFTTDetector_create(
            maxCorners=500,
            qualityLevel=0.01,
            minDistance=10
        )
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        
        # For tracking the features
        self.prev_gray = None
        self.prev_pts = None
        self.tracked_pts = None
        
        # For 3D transformation estimation (simple VO)
        self.K = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ])  # Approximate camera matrix
        
        # Current position (initially at origin)
        self.position = np.zeros(3)
        # Using simple floats for orientation to avoid numpy type issues
        self.quat_x = 0.0
        self.quat_y = 0.0
        self.quat_z = 0.0
        self.quat_w = 1.0
        
        # Map points (placeholder for now)
        self.map_points = []
        
        # Launch RViz in a separate thread
        threading.Thread(target=self.launch_rviz, daemon=True).start()
        
        # Debug info
        self.get_logger().info("RVIZ SLAM node initialized")
        self.get_logger().info("Publish and view in RViz:")
        self.get_logger().info("  - Camera feed: /slam/feature_image")
        self.get_logger().info("  - Camera trajectory: /slam/trajectory")
        self.get_logger().info("  - Map points: /slam/map_points")
        
        # Start a timer to publish the trajectory
        self.timer = self.create_timer(0.1, self.publish_tf)
        
    def launch_rviz(self):
        """Launch RViz with a simple configuration"""
        self.get_logger().info("Launching RViz...")
        
        # Small delay to ensure node is up
        time.sleep(2.0)
        
        # Create RViz config
        config_content = """
Panels:
  - Class: rviz_common/Displays
    Name: Displays
    Tree:
      - Class: rviz_default_plugins/Grid
        Name: Grid
        Value: true
      - Class: rviz_default_plugins/TF
        Name: TF
        Value: true
      - Class: rviz_default_plugins/Image
        Name: Camera
        Topic: /slam/feature_image
        Value: true
      - Class: rviz_default_plugins/Path
        Name: Trajectory
        Topic: /slam/trajectory
        Color: 255; 0; 0
        Value: true
    Visualization Manager:
      Frame: map
"""
        # Write config file
        config_path = "/tmp/slam_config.rviz"
        try:
            with open(config_path, "w") as f:
                f.write(config_content)
            
            # Launch RViz
            cmd = ["ros2", "run", "rviz2", "rviz2", "-d", config_path]
            subprocess.Popen(cmd)
        except Exception as e:
            self.get_logger().error(f"Error launching RViz: {e}")
            
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            vis_image = cv_image.copy()
            
            # Extract or track features
            self.frame_count += 1
            if self.prev_gray is None or self.frame_count % 10 == 0:
                # Find new features
                pts = self.feature_detector.detect(gray)
                self.prev_pts = np.array([p.pt for p in pts], dtype=np.float32).reshape(-1, 1, 2)
            else:
                # Track existing features
                if self.prev_pts is not None and len(self.prev_pts) > 0:
                    next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                        self.prev_gray, gray, self.prev_pts, None, **self.lk_params
                    )
                    
                    # Filter out points we couldn't track
                    good_idx = status.ravel() == 1
                    if np.any(good_idx):
                        self.tracked_pts = next_pts[good_idx]
                        self.prev_pts = next_pts
                        
                        # Visualize tracked points
                        for i, (new_pt, status_pt) in enumerate(zip(next_pts, status)):
                            if status_pt:
                                pt = new_pt.ravel()
                                cv2.circle(vis_image, (int(pt[0]), int(pt[1])), 3, (0, 255, 0), -1)
                        
                        # Estimate motion (simple VO, not using RTAB-Map here)
                        if self.frame_count % 15 == 0 and len(self.tracked_pts) > 8:
                            # Simulate a bit of motion for testing
                            # In a real system, this would be from PnP or essential matrix
                            dx = 0.05 * math.sin(self.frame_count * 0.01)
                            dy = 0.02 * math.cos(self.frame_count * 0.01)
                            dz = 0.01
                            
                            self.position[0] += dx
                            self.position[1] += dy
                            self.position[2] += dz
                            
                            # Update camera pose
                            now = self.get_clock().now().to_msg()
                            self.camera_pose.header.stamp = now
                            self.camera_pose.pose.position.x = self.position[0]
                            self.camera_pose.pose.position.y = self.position[1]
                            self.camera_pose.pose.position.z = self.position[2]
                            # Set proper quaternion values
                            self.camera_pose.pose.orientation.w = float(1.0)  # Identity rotation for now
                            self.camera_pose.pose.orientation.x = float(0.0)
                            self.camera_pose.pose.orientation.y = float(0.0)
                            self.camera_pose.pose.orientation.z = float(0.0)
                            
                            # Add to path for trajectory visualization
                            self.camera_path.header.stamp = now
                            self.camera_path.poses.append(self.camera_pose)
                            
                            # Simulate map points (in a real system, these would be triangulated)
                            for pt in self.tracked_pts[:10]:  # Use some tracked points
                                x, y = pt.ravel()
                                # Project to 3D (simplified)
                                depth = 2.0 + 0.5 * np.random.rand()  # Random depth between 2-2.5m
                                pt_3d = np.array([
                                    (x - self.K[0, 2]) * depth / self.K[0, 0],
                                    (y - self.K[1, 2]) * depth / self.K[1, 1],
                                    depth
                                ])
                                # Transform to world coordinates
                                pt_3d = pt_3d + self.position
                                self.map_points.append(pt_3d)
            
            # Keep only last 100 poses to avoid UI lag
            if len(self.camera_path.poses) > 100:
                self.camera_path.poses = self.camera_path.poses[-100:]
                
            # Keep max 500 map points
            if len(self.map_points) > 500:
                self.map_points = self.map_points[-500:]
            
            # Store frame for next iteration
            self.prev_gray = gray.copy()
            
            # Add text overlay
            cv2.putText(
                vis_image,
                f"Frame: {self.frame_count}  Features: {0 if self.prev_pts is None else len(self.prev_pts)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )
            
            # Create ROS message from image and publish
            feature_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding="bgr8")
            feature_msg.header = msg.header
            self.feature_img_pub.publish(feature_msg)
            
            # Publish trajectory and map (done in the timer callback)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
            
    def publish_tf(self):
        """Publish TF transforms and visualization data"""
        # Current timestamp
        now = self.get_clock().now().to_msg()
        
        # Publish the camera trajectory path
        self.camera_path.header.stamp = now
        self.path_pub.publish(self.camera_path)
        
        # Publish TF transform for camera position
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "map"
        t.child_frame_id = "camera_link"
        
        # Set the transform from map to camera
        t.transform.translation.x = float(self.position[0])
        t.transform.translation.y = float(self.position[1])
        t.transform.translation.z = float(self.position[2])
        t.transform.rotation.w = float(1.0)  # Identity quaternion
        t.transform.rotation.x = float(0.0)
        t.transform.rotation.y = float(0.0)
        t.transform.rotation.z = float(0.0)
        
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RvizSlamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
