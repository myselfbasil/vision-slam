#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import os
import math

class OpenCVSlamVisualizer(Node):
    def __init__(self):
        super().__init__("opencv_slam_node")
        
        # OpenCV bridge for converting ROS images
        self.bridge = CvBridge()
        
        # Subscribe to the camera image
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # Initialize visualization state
        self.frame_count = 0
        self.last_keyframe = None
        self.last_gray = None
        self.trajectory = []
        self.map_points = []
        self.current_pos = np.zeros(3)  # x, y, z
        
        # Feature detector
        self.detector = cv2.GFTTDetector_create(
            maxCorners=1000,
            qualityLevel=0.01,
            minDistance=10
        )
        
        # LK optical flow parameters
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )
        
        # For feature tracking
        self.prev_pts = None
        self.tracking_history = {}  # track feature points across frames
        self.next_feature_id = 0
        
        # Approximate camera matrix
        self.K = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ])
        
        # Create display windows
        cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Map View", cv2.WINDOW_NORMAL)
        cv2.moveWindow("Camera View", 50, 50)
        cv2.moveWindow("Map View", 700, 50)
        
        self.get_logger().info("OpenCV SLAM Visualizer initialized")
        
    def image_callback(self, msg):
        """Process incoming camera images"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            h, w = gray.shape
            
            # Create visualization images
            camera_view = cv_image.copy()
            map_view = np.ones((600, 600, 3), dtype=np.uint8) * 255
            
            # Draw grid on map view
            grid_size = 50
            for i in range(0, 600, grid_size):
                cv2.line(map_view, (i, 0), (i, 600), (200, 200, 200), 1)
                cv2.line(map_view, (0, i), (600, i), (200, 200, 200), 1)
            
            # Draw coordinate axes at center
            center = (300, 300)
            cv2.line(map_view, center, (center[0] + 50, center[1]), (0, 0, 255), 2)  # X-axis (red)
            cv2.line(map_view, center, (center[0], center[1] - 50), (0, 255, 0), 2)  # Y-axis (green)
            
            self.frame_count += 1
            
            # Feature detection and tracking
            if self.last_gray is None:
                # First frame, detect features
                features = self.detector.detect(gray)
                self.prev_pts = np.array([f.pt for f in features], dtype=np.float32).reshape(-1, 1, 2)
                
                # Initialize tracking with unique IDs
                for i, pt in enumerate(self.prev_pts):
                    x, y = pt[0]
                    self.tracking_history[self.next_feature_id] = [(int(x), int(y))]
                    self.next_feature_id += 1
            else:
                # Track features from previous frame
                if self.prev_pts is not None and len(self.prev_pts) > 0:
                    curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                        self.last_gray, gray, self.prev_pts, None, **self.lk_params
                    )
                    
                    # Filter good points
                    good_new = curr_pts[status==1]
                    good_old = self.prev_pts[status==1]
                    
                    # Update tracking history
                    active_ids = list(self.tracking_history.keys())
                    idx = 0
                    
                    for i, (new, old) in enumerate(zip(good_new, good_old)):
                        if idx >= len(active_ids):
                            break
                            
                        track_id = active_ids[idx]
                        x, y = new.ravel()
                        self.tracking_history[track_id].append((int(x), int(y)))
                        
                        # Limit history length
                        if len(self.tracking_history[track_id]) > 50:
                            self.tracking_history[track_id] = self.tracking_history[track_id][-50:]
                            
                        idx += 1
                    
                    # Remove lost tracks
                    for i in range(idx, len(active_ids)):
                        if active_ids[i] in self.tracking_history:
                            del self.tracking_history[active_ids[i]]
                    
                    # Update points for next frame
                    self.prev_pts = good_new.reshape(-1, 1, 2)
                    
                    # Simulate camera motion (in a real system this would be computed from PnP)
                    if self.frame_count % 5 == 0:
                        dx = 0.05 * math.sin(self.frame_count * 0.01)
                        dy = 0.05 * math.cos(self.frame_count * 0.02)
                        self.current_pos[0] += dx
                        self.current_pos[1] += dy
                        
                        # Add to trajectory
                        self.trajectory.append((self.current_pos[0], self.current_pos[1]))
                        
                        # Keep trajectory limited
                        if len(self.trajectory) > 500:
                            self.trajectory = self.trajectory[-500:]
                            
                        # Add random map points around current position
                        if self.frame_count % 10 == 0 and len(good_new) > 0:
                            for _ in range(5):
                                point = good_new[np.random.randint(0, len(good_new))]
                                x, y = point.ravel()
                                
                                # Create 3D point (simulated)
                                depth = 2.0 + np.random.rand()
                                pt3d = np.array([
                                    (x - self.K[0, 2]) * depth / self.K[0, 0],
                                    (y - self.K[1, 2]) * depth / self.K[1, 1],
                                    depth
                                ])
                                
                                # Transform to world
                                world_pt = pt3d + self.current_pos
                                self.map_points.append((world_pt[0], world_pt[1]))
                                
                                # Limit map points
                                if len(self.map_points) > 1000:
                                    self.map_points = self.map_points[-1000:]
            
            # Draw features and tracks on camera view
            for track_id, history in self.tracking_history.items():
                if len(history) > 1:
                    # Draw the trail with gradient color (older points are darker)
                    for i in range(1, len(history)):
                        intensity = int(255 * i / len(history))
                        color = (0, intensity, 0)  # Green with varying intensity
                        cv2.line(camera_view, history[i-1], history[i], color, 1)
                    
                    # Draw current point
                    cv2.circle(camera_view, history[-1], 3, (0, 255, 0), -1)
            
            # Draw trajectory on map view (transform to map coordinates)
            map_scale = 20
            for i in range(1, len(self.trajectory)):
                pt1 = (
                    int(center[0] + self.trajectory[i-1][0] * map_scale),
                    int(center[1] - self.trajectory[i-1][1] * map_scale)
                )
                pt2 = (
                    int(center[0] + self.trajectory[i][0] * map_scale),
                    int(center[1] - self.trajectory[i][1] * map_scale)
                )
                cv2.line(map_view, pt1, pt2, (255, 0, 0), 2)
            
            # Draw current position
            curr_pos_map = (
                int(center[0] + self.current_pos[0] * map_scale),
                int(center[1] - self.current_pos[1] * map_scale)
            )
            cv2.circle(map_view, curr_pos_map, 5, (0, 0, 255), -1)
            
            # Draw map points
            for point in self.map_points:
                pt_map = (
                    int(center[0] + point[0] * map_scale),
                    int(center[1] - point[1] * map_scale)
                )
                cv2.circle(map_view, pt_map, 1, (100, 100, 255), -1)
            
            # Add text overlays
            cv2.putText(
                camera_view,
                f"Frame: {self.frame_count}  Features: {len(self.tracking_history)}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )
            
            cv2.putText(
                map_view,
                f"Position: ({self.current_pos[0]:.2f}, {self.current_pos[1]:.2f}, {self.current_pos[2]:.2f})",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 0),
                2
            )
            
            # Store current frame for next iteration
            self.last_gray = gray.copy()
            
            # Show the visualizations
            cv2.imshow("Camera View", camera_view)
            cv2.imshow("Map View", map_view)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
            
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OpenCVSlamVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
