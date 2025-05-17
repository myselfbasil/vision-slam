#!/usr/bin/env python3
import cv2
import numpy as np
import time
import os

def test_camera_with_display():
    """Test camera access directly with OpenCV and display frames"""
    print("Starting camera test with display...")
    
    # Print device permissions
    os.system("ls -l /dev/video*")
    
    # Try both video devices with multiple backends
    devices = [0, 1]
    backends = [cv2.CAP_ANY, cv2.CAP_V4L2, cv2.CAP_GSTREAMER]
    
    for device in devices:
        for backend in backends:
            print(f"\nTrying device {device} with backend {backend}...")
            cap = cv2.VideoCapture(device, backend)
            
            if not cap.isOpened():
                print(f"Failed to open device {device} with backend {backend}")
                continue
                
            print(f"Successfully opened device {device} with backend {backend}")
            print(f"  Width: {cap.get(cv2.CAP_PROP_FRAME_WIDTH)}")
            print(f"  Height: {cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}")
            print(f"  FPS: {cap.get(cv2.CAP_PROP_FPS)}")
            
            window_name = f"Camera {device} (Backend {backend})"
            cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
            cv2.moveWindow(window_name, 20, 20)
            
            # Try to capture and display frames
            frames_captured = 0
            start_time = time.time()
            
            print("Capturing frames for 5 seconds...")
            while time.time() - start_time < 5:
                ret, frame = cap.read()
                if ret:
                    frames_captured += 1
                    cv2.putText(frame, f"Frame {frames_captured}", (10, 30), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    cv2.imshow(window_name, frame)
                    cv2.waitKey(1)
                else:
                    print(f"Failed to capture frame at {time.time() - start_time:.2f}s")
                    
                time.sleep(0.033)  # ~30fps
            
            cap.release()
            cv2.destroyWindow(window_name)
            
            print(f"Test complete: {frames_captured} frames captured in ~5 seconds")
            
            # If we successfully captured frames, no need to try other options
            if frames_captured > 0:
                return True
    
    # If we got here, all attempts failed
    print("All camera access attempts failed!")
    return False

if __name__ == "__main__":
    # Make sure camera files are accessible
    os.system("sudo chmod 666 /dev/video* 2>/dev/null || echo 'Could not change permissions (normal in container)'")
    
    # Test camera with display
    success = test_camera_with_display()
    
    # Print final result
    if success:
        print("\nSUCCESS: Camera test passed!")
    else:
        print("\nFAILURE: Could not access camera!")
        
    print("\nThis window will close in 5 seconds...")
    time.sleep(5)
