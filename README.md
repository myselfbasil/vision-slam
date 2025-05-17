# Vision-Based SLAM System

A lightweight camera-based SLAM (Simultaneous Localization and Mapping) system that visualizes camera movement in real-time.

## Features

- Real-time feature tracking and visualization
- Live camera trajectory mapping
- No ROS2 or complex dependencies required
- Dockerized for easy deployment
- Works on systems with restricted permissions

## Prerequisites

- Docker
- A camera (built-in or USB webcam)
- Basic X11 display support (on Linux)

## Getting Started

### Building the Docker Image

```bash
docker build -t vision-slam .
```

### Running with Camera

```bash
docker run -it --rm \
  --privileged \
  --device=/dev/video0:/dev/video0 \
  --device=/dev/video1:/dev/video1 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  vision-slam
```

Then inside the container:

```bash
python3 /ros2_ws/integrated_slam.py
```

The system will show two windows:

1. **Camera View**: Shows the live camera feed with tracked feature points
2. **Map View**: Displays a top-down view of the estimated camera trajectory and map points

## How It Works

The system uses OpenCV for both camera capture and visualization:

1. **Feature detection** - Using Shi-Tomasi corner detector to find trackable points
2. **Feature tracking** - Implementing Lucas-Kanade optical flow to track points between frames
3. **Visual odometry** - Estimating camera motion using tracked feature points
4. **Mapping** - Building a simple map showing camera trajectory and landmark points

## Troubleshooting

If you have camera access issues, try:

```bash
python3 /ros2_ws/test_camera.py
```

To test a different camera device (e.g., /dev/video1), use:

```bash
python3 /ros2_ws/test_camera.py 1
```

## Tips for Good Results

- Ensure good lighting in your environment
- Move the camera slowly and smoothly
- Include distinctive visual features in the scene (textured surfaces, objects with clear patterns)
- Try to maintain some features in view as you move the camera

## Troubleshooting

- If the camera isn't detected, check with `ls -l /dev/video*` and update the --device parameter
- If visualization doesn't appear, make sure X11 forwarding is working (`xhost +local:docker`)
- If the system is running slowly, try running on a computer with better hardware

