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
    
    # Add debug level parameter
    debug_arg = DeclareLaunchArgument(
        'debug', 
        default_value='true',
        description='Enable debug output'
    )
    
    use_video = LaunchConfiguration('use_video')
    video_path = LaunchConfiguration('video_path')
    debug = LaunchConfiguration('debug')
    
    # Full launch file configured for optimal Ubuntu Linux compatibility
    return LaunchDescription([
        # Launch arguments
        use_video_arg,
        video_path_arg,
        debug_arg,
        
        # Topic Echo for Debugging - to see if camera data is actually being published
        ExecuteProcess(
            cmd=['ros2', 'topic', 'list'],
            name='topic_list',
            output='screen',
            condition=IfCondition(debug)
        ),
        
        # Live Camera Node - only run when not using video
        ExecuteProcess(
            cmd=['python3', '/ros2_ws/camera_node.py'],
            output='screen',
            name='camera_node',
            condition=UnlessCondition(use_video),
            # Give it time to initialize
            on_exit=[ExecuteProcess(
                cmd=['sleep', '2'],
                output='screen'
            )]
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
                {'subscribe_rgbd': False},
                {'subscribe_stereo': False},
                {'approx_sync': True},
                {'queue_size': 20},  # Increase queue size for better sync
                {'Vis/MinInliers': 10},  # Reduced for easier matching
                {'Vis/RoiRatios': '0.03 0.03 0.04 0.04'},
                {'Mem/ImagePreDecimation': 1},  # Don't decimate to ensure frames are processed
                {'Mem/ImagePostDecimation': 1},
                # Parameters to improve live performance with webcams
                {'Vis/EstimationType': '1'},  # 3D->2D (PnP) better for monocular
                {'Vis/MaxDepth': '10.0'},
                {'GFTT/MinDistance': '5.0'},  # Smaller distance to get more features
                {'GFTT/QualityLevel': '0.001'},  # More sensitive feature detection
                {'Mem/STMSize': '30'},
                {'Kp/DetectorStrategy': '6'},  # GFTT/BRIEF works well for many scenarios
                {'Kp/MaxFeatures': '1000'},  # More features 
                {'Kp/RoiRatios': '0.03 0.03 0.04 0.04'},
                # Debug parameters
                {'Rtabmap/DetectionRate': '2.0'},  # Slower detection rate (better for webcams)
                {'RGBD/CreateOccupancyGrid': 'false'},  # Disable occupancy grid creation for basic test
                {'Rtabmap/TimeThr': '700'},  # More time for loop closure
                {'Rtabmap/LoopThr': '0.11'},  # More permissive loop closure
                {'Vis/CorType': '1'},  # 0=SURF 1=ORB
                {'Vis/MaxFeatures': '1000'},  # Max visual features
                {'Grid/RangeMax': '4.0'},  # Max range for grid mapping
                {'Grid/CellSize': '0.05'},  # Grid cell size (5cm)
                {'Reg/Force3DoF': 'true'}  # Force 2D mapping (easier for webcam)
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
                {'subscribe_rgbd': False},
                {'subscribe_stereo': False},
                {'subscribe_scan': False},
                {'subscribe_scan_cloud': False},
                {'subscribe_user_data': False},
                {'subscribe_odom_info': False},
                {'frame_id': 'camera_link'},
                {'approx_sync': True},
                {'queue_size': 20},
            ],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
            ],
        ),
        
        # Debug: Topic echo to verify camera image is published (only if debug=true)
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/camera/image_raw', '--csv', '--once'],
            output='screen',
            condition=IfCondition(debug),
            name='topic_echo',
            on_exit=[ExecuteProcess(
                cmd=['echo', 'If you see no output above, camera data is not being published']
            )]
        ),
    ])
