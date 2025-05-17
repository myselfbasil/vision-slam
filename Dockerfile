# Base image for ROS2 with RTABMap for Ubuntu live SLAM
FROM ros:humble-perception

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set up environment
WORKDIR /ros2_ws

# Install ROS dependencies and tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-tf2-ros \
    ros-humble-rtabmap \
    ros-humble-rtabmap-ros \
    v4l-utils \
    libv4l-dev \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# Install additional Python packages
# Use fixed NumPy version for compatibility with cv_bridge
RUN pip3 install --no-cache-dir \
    numpy==1.24.0 \
    opencv-python \
    matplotlib \
    argparse

# Copy the Python scripts
COPY video_publisher.py /ros2_ws/video_publisher.py
COPY camera_node.py /ros2_ws/camera_node.py
COPY live_slam.py /ros2_ws/live_slam.py
COPY test_camera.py /ros2_ws/test_camera.py
COPY camera_view.py /ros2_ws/camera_view.py
COPY troubleshoot_camera.md /ros2_ws/troubleshoot_camera.md
COPY flexible_camera.py /ros2_ws/flexible_camera.py
COPY direct_camera_access.py /ros2_ws/direct_camera_access.py
COPY fix_permissions.sh /ros2_ws/fix_permissions.sh

# Make the Python scripts executable
RUN chmod +x /ros2_ws/video_publisher.py && \
    chmod +x /ros2_ws/camera_node.py && \
    chmod +x /ros2_ws/live_slam.py && \
    chmod +x /ros2_ws/test_camera.py && \
    chmod +x /ros2_ws/camera_view.py && \
    chmod +x /ros2_ws/flexible_camera.py && \
    chmod +x /ros2_ws/direct_camera_access.py && \
    chmod +x /ros2_ws/fix_permissions.sh

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]

# Create directory for mounting videos
RUN mkdir -p /videos

# Default command that runs our vision-based SLAM system
CMD ["bash", "-c", "echo 'To start live SLAM: ros2 launch /ros2_ws/live_slam.py\nTo start with video: ros2 launch /ros2_ws/live_slam.py use_video:=true video_path:=/videos/your_video.mp4\n\nDiagnostic commands:\n- Test camera directly: python3 /ros2_ws/test_camera.py\n- Flexible camera node: python3 /ros2_ws/flexible_camera.py\n- Direct camera test: python3 /ros2_ws/direct_camera_access.py\n- View camera feed: python3 /ros2_ws/camera_view.py\n- More troubleshooting: cat /ros2_ws/troubleshoot_camera.md' && bash"]
