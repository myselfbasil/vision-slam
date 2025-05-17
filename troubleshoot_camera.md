# Camera Troubleshooting Guide for ROS2 Live SLAM

If your RTAB-Map GUI is showing up but not displaying camera data, try these steps:

## 1. Check Camera Access in Docker

First, make sure Docker can access your camera device:

```bash
# List available cameras
ls -l /dev/video*

# Check camera device permissions
sudo chmod 666 /dev/video0  # (or your camera device)
```

## 2. Verify Camera in Container

Inside the Docker container, check if the camera is recognized:

```bash
# Install v4l-utils if not already installed
apt-get update && apt-get install -y v4l-utils

# List video devices and capabilities
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --all
```

## 3. Common Issues and Solutions

### Wrong Camera Device

If your camera is not at `/dev/video0`, pass the correct device:

```bash
docker run -it --rm --network=host --privileged --device=/dev/video1:/dev/video0 -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $(pwd)/videos:/videos ros2-live-slam
```

### Permission Issues

On some systems, you might need additional permissions:

```bash
# Add user to video group
sudo usermod -a -G video $USER
```

### Try Different Camera Index

If the default camera index (0) doesn't work, modify camera_node.py to try a different index (1, 2, etc.):

```python
camera_device = 1  # Try different values
```

## 4. Test Camera Directly

Test the camera outside of ROS2:

```bash
# Inside Docker
apt-get install -y python3-opencv
python3 -c 'import cv2; cap = cv2.VideoCapture(0); ret, frame = cap.read(); print(f"Camera working: {ret}, shape: {frame.shape if ret else None}"); cap.release()'
```

## 5. Check ROS2 Topics

Ensure the camera topics are being published:

```bash
# List all topics
ros2 topic list

# Check if camera topics exist
ros2 topic echo /camera/image_raw --once
ros2 topic echo /camera/camera_info --once
```

## 6. Alternative: Use Video File First

If live camera troubleshooting is difficult, verify the system works with a video file:

```bash
# Place a test video in the videos directory, then:
ros2 launch /ros2_ws/live_slam.py use_video:=true video_path:=/videos/your_video.mp4
```
