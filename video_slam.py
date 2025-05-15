#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    # Full launch file configured for optimal Linux compatibility
    return LaunchDescription([
        # Video Publisher Node
        ExecuteProcess(
            cmd=['python3', '/ros2_ws/video_publisher.py', '--video', '/videos/trial_video.mp4'],
            output='screen',
        ),
        
        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            emulate_tty=True,
            parameters=[
                # Basic parameters
                {'frame_id': 'camera_link'},
                {'subscribe_depth': False},
                {'subscribe_rgb': True},
                {'approx_sync': True},
                {'Vis/MinInliers': 15},
                {'Vis/RoiRatios': '0.03 0.03 0.04 0.04'},
                {'Mem/ImagePreDecimation': 2},
                {'Mem/ImagePostDecimation': 2},
            ],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
            ],
        ),
        
        # RTAB-Map Visualization Node (separate from SLAM processing)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'subscribe_depth': False},
                {'subscribe_rgb': True},
                {'subscribe_odom_info': False},
                {'frame_id': 'camera_link'},
                {'approx_sync': True},
            ],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
            ],
        ),
    ])
