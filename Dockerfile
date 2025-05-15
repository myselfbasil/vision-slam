# Base image for ROS2 with RTABMap (compatible with both ARM64 and x86_64)
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
COPY video_slam.py /ros2_ws/video_slam.py

# Make the Python scripts executable
RUN chmod +x /ros2_ws/video_publisher.py && \
    chmod +x /ros2_ws/video_slam.py

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]

# Create directory for mounting videos
RUN mkdir -p /videos

# Default command that runs our vision-based SLAM system
CMD ["bash", "-c", "echo 'To start vision-based SLAM with a video file, run: ros2 launch /ros2_ws/video_slam.py' && bash"]
