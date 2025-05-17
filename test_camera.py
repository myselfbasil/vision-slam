#!/usr/bin/env python3
import cv2
import time
import sys

def test_camera(device_id=0):
    """Simple test to verify camera access and display frames"""
    print(f"Opening camera device {device_id}...")
    cap = cv2.VideoCapture(device_id)
    
    if not cap.isOpened():
        print(f"Failed to open camera {device_id}")
        # Try alternative API
        print(f"Trying with V4L2 backend...")
        cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        
    if not cap.isOpened():
        print(f"Failed to open camera with any backend")
        return False
        
    # Get camera properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f"Camera opened successfully: {width}x{height} @ {fps}fps")
    
    # Try to read frames
    success_count = 0
    for i in range(10):
        ret, frame = cap.read()
        if ret:
            success_count += 1
            h, w = frame.shape[:2]
            print(f"Frame {i+1}: Captured {w}x{h} image")
        else:
            print(f"Frame {i+1}: Failed to capture")
        time.sleep(0.1)
    
    cap.release()
    print(f"Test complete: {success_count}/10 frames captured successfully")
    return success_count > 0

if __name__ == "__main__":
    device_id = 0
    if len(sys.argv) > 1:
        device_id = int(sys.argv[1])
    
    result = test_camera(device_id)
    print(f"Camera test {'PASSED' if result else 'FAILED'}")
