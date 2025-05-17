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
    # Qt and X11 dependencies for visualization
    libqt5gui5 \
    libqt5widgets5 \
    libqt5core5a \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xinerama0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
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
COPY direct_rtabmap.py /ros2_ws/direct_rtabmap.py
COPY simple_visualizer.py /ros2_ws/simple_visualizer.py

# Make the Python scripts executable
RUN chmod +x /ros2_ws/video_publisher.py && \
    chmod +x /ros2_ws/camera_node.py && \
    chmod +x /ros2_ws/live_slam.py && \
    chmod +x /ros2_ws/test_camera.py && \
    chmod +x /ros2_ws/camera_view.py && \
    chmod +x /ros2_ws/flexible_camera.py && \
    chmod +x /ros2_ws/direct_camera_access.py && \
    chmod +x /ros2_ws/fix_permissions.sh && \
    chmod +x /ros2_ws/direct_rtabmap.py && \
    chmod +x /ros2_ws/simple_visualizer.py

# Set up environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Configure environment variables for Qt
ENV QT_X11_NO_MITSHM=1
ENV XDG_RUNTIME_DIR=/tmp/runtime-root

# Create required directories
RUN mkdir -p /videos /tmp/runtime-root
RUN chmod 700 /tmp/runtime-root

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec \"$@\"", "--"]

# Default command that runs our vision-based SLAM system
CMD ["bash", "-c", "echo '\n\033[1;32mVision-Based SLAM System\033[0m\n\n\033[1;33mMain Commands:\033[0m\n- Simple visualization: \033[1;36mpython3 /ros2_ws/simple_visualizer.py\033[0m (Best for university systems)\n- All-in-one SLAM: \033[1;36mpython3 /ros2_ws/direct_rtabmap.py\033[0m (Full RTAB-Map)\n- Launch with ROS2: \033[1;36mros2 launch /ros2_ws/live_slam.py\033[0m\n- Start with video: \033[1;36mros2 launch /ros2_ws/live_slam.py use_video:=true video_path:=/videos/your_video.mp4\033[0m\n\n\033[1;33mDiagnostic Commands:\033[0m\n- Test camera: \033[1;36mpython3 /ros2_ws/test_camera.py\033[0m\n- Flexible camera: \033[1;36mpython3 /ros2_ws/flexible_camera.py\033[0m\n- View camera feed: \033[1;36mpython3 /ros2_ws/camera_view.py\033[0m\n- Direct camera test: \033[1;36mpython3 /ros2_ws/direct_camera_access.py\033[0m\n\n\033[1;33mTroubleshooting:\033[0m\n- Camera issues: \033[1;36mcat /ros2_ws/troubleshoot_camera.md\033[0m\n' && bash"]
