#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # Launch arguments to switch between live camera and pre-recorded video
    use_video_arg = DeclareLaunchArgument(
        'use_video',
        default_value='false',
        description='Use pre-recorded video instead of live camera'
    )
    
    video_path_arg = DeclareLaunchArgument(
        'video_path',
        default_value='/videos/video.mp4',
        description='Path to video file if using pre-recorded video'
    )
    
    use_video = LaunchConfiguration('use_video')
    video_path = LaunchConfiguration('video_path')
    
    # Full launch file configured for optimal Ubuntu Linux compatibility
    return LaunchDescription([
        # Launch arguments
        use_video_arg,
        video_path_arg,
        
        # Live Camera Node - only run when not using video
        Node(
            package='vision_slam',  # Update with your package name if different
            executable='camera_node.py',
            name='camera_node',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(use_video)
        ),
        
        # Video Publisher Node - only run when using video
        ExecuteProcess(
            cmd=['python3', '/ros2_ws/video_publisher.py', '--video', video_path],
            output='screen',
            condition=IfCondition(use_video)
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
                # Parameters to improve live performance
                {'Vis/EstimationType': '1'},  # 0=3D->3D, 1=3D->2D (PnP), 2=2D->2D
                {'Vis/MaxDepth': '10.0'},     # Maximum depth of visual features
                {'GFTT/MinDistance': '10.0'}, # Minimum distance between features
                {'GFTT/QualityLevel': '0.002'}, # Quality level for feature extraction
                {'Mem/STMSize': '30'},        # Short-term memory size
                {'Kp/DetectorStrategy': '6'},  # 0=SURF, 1=SIFT, 2=ORB, 3=FAST/FREAK, 4=FAST/BRIEF, 5=GFTT/FREAK, 6=GFTT/BRIEF, 7=BRISK
                {'Kp/RoiRatios': '0.03 0.03 0.04 0.04'} # ROI used to extract features (top, right, bottom, left)                
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
