# ROS2 Video-Based SLAM System for Linux

This project provides a Docker setup for running vision-based SLAM (Simultaneous Localization and Mapping) using pre-recorded videos with ROS2 and RTAB-Map on Linux systems. This approach allows for consistent testing and development without requiring special hardware.

## Prerequisites

- Linux system with Docker installed
- A pre-recorded video file for SLAM processing (MP4 format recommended)
- X11 display server (standard on most Linux distributions)

## Project Files

- **Dockerfile**: Sets up the ROS2 environment with RTAB-Map for video-based SLAM
- **video_publisher.py**: ROS2 node that reads a video file and publishes frames to ROS2 topics
- **video_slam.py**: Launch file that coordinates the video publisher and RTAB-Map for SLAM

## Setting Up and Running the System

### 1. Prepare Your Video File

Place your pre-recorded video file in a videos directory:

```bash
mkdir -p videos
# Copy your video file to this directory
cp your-video-file.mp4 videos/trial_video.mp4
```

### 2. Build the Docker Image

```bash
docker build -t ros2-video-slam .
```

### 3. Run the Docker Container

On Linux, X11 forwarding works seamlessly with Docker. Run the container with:

```bash
# Allow X server connections from localhost
xhost +local:docker

# Run the Docker container with X11 forwarding
docker run -it --rm \
  --network=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/videos:/videos \
  ros2-video-slam
```

### 4. Launch the SLAM System

Inside the container, start the SLAM system:

```bash
# Start the complete SLAM system with visualization
ros2 launch /ros2_ws/video_slam.py
```

The SLAM visualization will appear automatically in separate windows showing:
1. The camera feed from your video
2. Feature tracking visualization
3. The map being built in real-time

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
# In one terminal (inside the container)
python3 /ros2_ws/video_publisher.py --video /videos/trial_video.mp4

# In another terminal (inside the container) - run the SLAM processing
ros2 run rtabmap_slam rtabmap \
  --ros-args -p frame_id:=camera_link -p subscribe_depth:=false -p subscribe_rgb:=true \
  -p approx_sync:=true -r rgb/image:=/camera/image_raw -r rgb/camera_info:=/camera/camera_info

# In a third terminal (inside the container) - run just the visualization
ros2 run rtabmap_viz rtabmap_viz \
  --ros-args -p subscribe_depth:=false -p subscribe_rgb:=true \
  -r rgb/image:=/camera/image_raw -r rgb/camera_info:=/camera/camera_info
```

## Tips for Good SLAM Results

- Use videos with good lighting and texture-rich environments
- Camera movement should be smooth and not too fast
- Videos with loop closures (revisiting the same place) work best
- Using calibrated camera parameters improves accuracy

## Troubleshooting

- If visualization doesn't appear, make sure X11 forwarding is working properly (`xhost +local:docker`)
- If RTAB-Map reports no data received, check the topic names and ensure the video publisher is running
- If SLAM results are poor, try a different video with clearer features and slower movement
- For performance issues, adjust the `Mem/ImagePreDecimation` parameter in the launch file to a higher value

