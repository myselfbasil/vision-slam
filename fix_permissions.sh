#!/bin/bash
# Fix permissions for camera devices in the container

echo "Fixing camera permissions..."
sudo chmod 666 /dev/video0
sudo chmod 666 /dev/video1

echo "Checking camera devices:"
ls -l /dev/video*

echo "Testing direct camera access..."
python3 /ros2_ws/test_camera.py

echo "Done!"
