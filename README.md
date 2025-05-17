# ROS2 Live Camera SLAM System for Ubuntu

This project provides a Docker setup for running vision-based SLAM (Simultaneous Localization and Mapping) using either a live camera or pre-recorded videos with ROS2 and RTAB-Map on Ubuntu systems. You can perform real-time mapping of your environment using just a webcam.

## Prerequisites

- Ubuntu system with Docker installed
- USB webcam or built-in camera
- X11 display server (standard on Ubuntu)
- (Optional) Pre-recorded video files for testing

## Project Files

- **Dockerfile**: Sets up the ROS2 environment with RTAB-Map for live SLAM
- **camera_node.py**: ROS2 node that captures frames from a live camera
- **video_publisher.py**: ROS2 node that reads a video file (for testing)
- **live_slam.py**: Launch file that coordinates camera input and RTAB-Map

## Setting Up and Running the System

### 1. Clone the Repository

```bash
git clone https://github.com/myselfbasil/vision-slam.git
cd vision-slam
```

### 2. Build the Docker Image

```bash
docker build -t ros2-live-slam .
```

### 3. Run the Docker Container

On Ubuntu, we need to give the Docker container access to the camera device. Run the container with:

```bash
# Allow X server connections from localhost
xhost +local:docker

# Run the Docker container with camera access and X11 forwarding
docker run -it --rm \
  --network=host \
  --privileged \
  --device=/dev/video0:/dev/video0 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/videos:/videos \
  ros2-live-slam
```

Note: If your camera is not at `/dev/video0`, you can find the correct device with `ls -l /dev/video*`

### 4. Launch the Live SLAM System

Inside the container, start the live SLAM system:

```bash
# Start the SLAM system with the live camera
ros2 launch /ros2_ws/live_slam.py
```

The SLAM visualization will appear automatically in separate windows showing:
1. The live camera feed
2. Feature tracking visualization
3. The map being built in real-time

If you want to test with a pre-recorded video instead, use:

```bash
ros2 launch /ros2_ws/live_slam.py use_video:=true video_path:=/videos/your_video.mp4
```

## How Video-Based SLAM Works

The system uses RTAB-Map's monocular SLAM capabilities to perform:

1. **Video frame extraction** - Reading frames from the pre-recorded video
2. **Feature extraction** - Detecting visual features in each frame
3. **Feature matching** - Matching features between consecutive frames
4. **Visual odometry** - Estimating camera motion between frames
5. **Loop closure detection** - Recognizing previously visited places
6. **Map optimization** - Refining the map and trajectory

## Running Components Separately (for Debugging)

You can also run each component separately for debugging:

```bash
# In one terminal (inside the container) - run the camera node
python3 /ros2_ws/camera_node.py

# In another terminal (inside the container) - run the SLAM processing
ros2 run rtabmap_slam rtabmap \
  --ros-args -p frame_id:=camera_link -p subscribe_depth:=false -p subscribe_rgb:=true \
  -p approx_sync:=true -r rgb/image:=/camera/image_raw -r rgb/camera_info:=/camera/camera_info

# In a third terminal (inside the container) - run just the visualization
ros2 run rtabmap_viz rtabmap_viz \
  --ros-args -p subscribe_depth:=false -p subscribe_rgb:=true \
  -r rgb/image:=/camera/image_raw -r rgb/camera_info:=/camera/camera_info
```

Or with a video file instead of the camera:

```bash
python3 /ros2_ws/video_publisher.py --video /videos/your_video.mp4
```

## Tips for Good SLAM Results

- Ensure good lighting in your environment
- Move the camera slowly and smoothly
- Include distinctive visual features in the scene (posters, objects, furniture)
- Create loop closures by revisiting the same place from different angles
- For best results, calibrate your camera (using the ROS camera_calibration package)

## Camera Calibration (Optional but Recommended)

For optimal SLAM performance, calibrate your camera:

```bash
# Inside the Docker container, install calibration tools
apt update && apt install -y ros-humble-camera-calibration

# Run the calibration with a checkerboard pattern
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera/camera_info
```

Follow the on-screen instructions to calibrate. After calibration, update the camera parameters in camera_node.py.

## Troubleshooting

- If the camera isn't detected, check with `ls -l /dev/video*` and update the --device parameter
- If visualization doesn't appear, make sure X11 forwarding is working (`xhost +local:docker`)
- If you see errors about cv_bridge or numpy, try rebuilding the Docker image
- For performance issues on slower computers, adjust parameters in live_slam.py:
  - Increase `Mem/ImagePreDecimation` to 4
  - Increase `Mem/ImagePostDecimation` to 4
  - Set `Vis/FeatureType` to 3 (FAST detector) for faster processing

