#!/usr/bin/env python3
import cv2
import numpy as np
import time
import math
import os
import threading
import sys

class IntegratedSLAM:
    def __init__(self):
        print("Initializing Integrated SLAM system...")
        
        # Initialize camera capture directly
        self.setup_camera()
        
        # Initialize visualization windows
        cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
        cv2.namedWindow("Map View", cv2.WINDOW_NORMAL)
        cv2.moveWindow("Camera View", 50, 50)
        cv2.moveWindow("Map View", 700, 50)
        
        # Variables for feature tracking
        self.frame_count = 0
        self.prev_gray = None
        self.prev_pts = None
        
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
        
        # SLAM state
        self.position = np.zeros(3)
        self.trajectory = []
        self.map_points = []
        self.tracking_history = {}
        self.next_feature_id = 0
        
        # Approximate camera parameters (adjust as needed)
        self.K = np.array([
            [500, 0, 320],
            [0, 500, 240],
            [0, 0, 1]
        ])
        
        print("Initialization complete. Starting main loop...")
        
    def setup_camera(self):
        """Try to initialize camera with multiple approaches"""
        print("Trying to open camera...")
        print("Available camera devices:")
        os.system("ls -l /dev/video*")
        
        # Try multiple camera backends and indices
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY, None]
        devices = [0, 1, 2, 3]
        
        for device in devices:
            for backend in backends:
                try:
                    print(f"Trying camera {device} with backend {backend if backend is not None else 'default'}")
                    
                    if backend is None:
                        cap = cv2.VideoCapture(device)
                    else:
                        cap = cv2.VideoCapture(device, backend)
                    
                    if cap.isOpened():
                        # Try to read a frame to verify it works
                        ret, frame = cap.read()
                        if ret and frame is not None and frame.size > 0:
                            print(f"Successfully opened camera {device}!")
                            self.camera = cap
                            self.width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            self.height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            print(f"Camera resolution: {self.width}x{self.height}")
                            return
                        else:
                            print(f"Camera {device} opened but couldn't read frames")
                            cap.release()
                except Exception as e:
                    print(f"Error trying camera {device}: {str(e)}")
        
        print("WARNING: Failed to open any camera. Will use test pattern instead.")
        self.camera = None
        self.width = 640
        self.height = 480
    
    def generate_test_image(self):
        """Create a test pattern when no camera is available"""
        frame = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        
        # Create a moving pattern
        t = time.time()
        
        # Draw a shifting pattern
        for y in range(0, self.height, 10):
            for x in range(0, self.width, 10):
                b = int(128 + 127 * np.sin(x / 50.0 + t))
                g = int(128 + 127 * np.sin(y / 50.0 + t * 1.5))
                r = int(128 + 127 * np.sin((x+y) / 50.0 + t * 2.0))
                
                cv2.rectangle(frame, (x, y), (x+9, y+9), (b, g, r), -1)
        
        # Add frame number
        cv2.putText(
            frame,
            f"TEST PATTERN - Frame {self.frame_count}",
            (50, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2
        )
        
        # Add some moving circles to track
        for i in range(5):
            x = int(self.width/2 + 100 * np.sin(t * (i+1) * 0.1))
            y = int(self.height/2 + 80 * np.cos(t * (i+1) * 0.1))
            cv2.circle(frame, (x, y), 10, (0, 255, 0), -1)
        
        return frame
    
    def process_frame(self, frame):
        """Process a single camera frame"""
        self.frame_count += 1
        
        # Create visualization images
        camera_view = frame.copy()
        map_view = np.ones((600, 600, 3), dtype=np.uint8) * 255
        
        # Convert to grayscale for feature detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Draw grid on map view
        grid_size = 50
        for i in range(0, 600, grid_size):
            cv2.line(map_view, (i, 0), (i, 600), (200, 200, 200), 1)
            cv2.line(map_view, (0, i), (600, i), (200, 200, 200), 1)
        
        # Draw coordinate axes at center
        center = (300, 300)
        cv2.line(map_view, center, (center[0] + 50, center[1]), (0, 0, 255), 2)  # X-axis (red)
        cv2.line(map_view, center, (center[0], center[1] - 50), (0, 255, 0), 2)  # Y-axis (green)
        
        # Feature detection and tracking
        if self.prev_gray is None:
            # First frame - detect features
            features = self.detector.detect(gray)
            self.prev_pts = np.array([f.pt for f in features], dtype=np.float32).reshape(-1, 1, 2)
            
            # Initialize tracking
            for i, pt in enumerate(self.prev_pts):
                x, y = pt[0]
                self.tracking_history[self.next_feature_id] = [(int(x), int(y))]
                self.next_feature_id += 1
        else:
            # Track features from previous frame
            if self.prev_pts is not None and len(self.prev_pts) > 0:
                next_pts, status, _ = cv2.calcOpticalFlowPyrLK(
                    self.prev_gray, gray, self.prev_pts, None, **self.lk_params
                )
                
                # Filter good points
                good_idx = status.ravel() == 1
                if np.any(good_idx):
                    good_new = next_pts[good_idx]
                    good_old = self.prev_pts[good_idx]
                    
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
                    
                    # Simulate camera motion for visualization
                    if self.frame_count % 5 == 0:
                        dx = 0.05 * math.sin(self.frame_count * 0.01)
                        dy = 0.05 * math.cos(self.frame_count * 0.02)
                        self.position[0] += dx
                        self.position[1] += dy
                        
                        # Add to trajectory
                        self.trajectory.append((self.position[0], self.position[1]))
                        
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
                                world_pt = pt3d + self.position
                                self.map_points.append((world_pt[0], world_pt[1]))
                                
                                # Limit map points
                                if len(self.map_points) > 1000:
                                    self.map_points = self.map_points[-1000:]
                else:
                    # If tracking fails, find new features
                    features = self.detector.detect(gray)
                    self.prev_pts = np.array([f.pt for f in features], dtype=np.float32).reshape(-1, 1, 2)
                    
                    # Initialize tracking with unique IDs
                    for i, pt in enumerate(self.prev_pts):
                        x, y = pt[0]
                        self.tracking_history[self.next_feature_id] = [(int(x), int(y))]
                        self.next_feature_id += 1
        
        # Draw features and tracks on camera view
        for track_id, history in self.tracking_history.items():
            if len(history) > 1:
                # Draw the trail with gradient color
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
            int(center[0] + self.position[0] * map_scale),
            int(center[1] - self.position[1] * map_scale)
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
            f"Position: ({self.position[0]:.2f}, {self.position[1]:.2f}, {self.position[2]:.2f})",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 0),
            2
        )
        
        # Store current frame for next iteration
        self.prev_gray = gray.copy()
        
        # Show the visualizations
        cv2.imshow("Camera View", camera_view)
        cv2.imshow("Map View", map_view)
        
    def run(self):
        """Main loop"""
        print("Starting main processing loop...")
        
        try:
            while True:
                # Capture or generate frame
                if self.camera is not None:
                    ret, frame = self.camera.read()
                    if not ret or frame is None or frame.size == 0:
                        print("Failed to capture frame, using test pattern")
                        frame = self.generate_test_image()
                else:
                    frame = self.generate_test_image()
                
                # Process the frame
                self.process_frame(frame)
                
                # Check for exit
                key = cv2.waitKey(1) & 0xFF
                if key == 27 or key == ord('q'):  # ESC or 'q' to quit
                    break
                
                # Maintain reasonable frame rate
                time.sleep(0.03)  # ~30fps
                
        except KeyboardInterrupt:
            print("Interrupted by user")
        except Exception as e:
            print(f"Error in main loop: {str(e)}")
        finally:
            # Clean up
            if self.camera is not None:
                self.camera.release()
            cv2.destroyAllWindows()
            print("SLAM visualization stopped")
            
if __name__ == "__main__":
    print("Starting integrated SLAM system...")
    
    try:
        slam = IntegratedSLAM()
        slam.run()
    except Exception as e:
        print(f"Error initializing SLAM: {str(e)}")
        sys.exit(1)
