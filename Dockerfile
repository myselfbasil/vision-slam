FROM ros:humble-perception

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies and setup new ROS 2 GPG key
RUN apt-get update && apt-get install -y wget gnupg2 && \
    # Remove any old ROS keys and sources
    rm -f /etc/apt/sources.list.d/ros2.list && \
    # Add the new ROS 2 key from keyserver
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys F42ED6FBAB17C654 && \
    # Add ROS 2 repository
    echo "deb http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" > /etc/apt/sources.list.d/ros2.list && \
    apt-get update || true

# Set up environment
WORKDIR /ros2_ws

# Install essential dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    # Camera access packages
    v4l-utils \
    libv4l-dev \
    usbutils \
    # Minimal X11/OpenCV display dependencies
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install --no-cache-dir \
    opencv-python \
    numpy \
    matplotlib

# Copy the essential scripts
COPY integrated_slam.py /ros2_ws/integrated_slam.py
COPY test_camera.py /ros2_ws/test_camera.py
COPY README.md /ros2_ws/README.md

# Make the Python scripts executable
RUN chmod +x /ros2_ws/integrated_slam.py && \
    chmod +x /ros2_ws/test_camera.py

# Set up environment

# Create videos directory for optional video files
RUN mkdir -p /videos

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "exec \"$@\"", "--"]

# Default command that runs our vision-based SLAM system
CMD ["bash", "-c", "echo '\n\033[1;32mVision-Based SLAM System\033[0m\n\n\033[1;33mCommands:\033[0m\n- Start SLAM visualization: \033[1;36mpython3 /ros2_ws/integrated_slam.py\033[0m\n- Test camera only: \033[1;36mpython3 /ros2_ws/test_camera.py\033[0m\n\n' && bash"]
