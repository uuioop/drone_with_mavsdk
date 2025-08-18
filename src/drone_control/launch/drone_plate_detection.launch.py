#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的共享目录
    drone_control_pkg_dir = get_package_share_directory('drone_control')
    plate_detection_pkg_dir = get_package_share_directory('plate_detection')
    
    # 声明启动参数
    use_usb_cam_arg = DeclareLaunchArgument(
        'use_usb_cam',
        default_value='true',
        description='是否启动USB摄像头'
    )
    
    use_realsense_arg = DeclareLaunchArgument(
        'use_realsense',
        default_value='false',
        description='是否启动Intel RealSense D435（已注释）'
    )
    
    use_plate_detection_arg = DeclareLaunchArgument(
        'use_plate_detection',
        default_value='true',
        description='是否启动车牌检测节点'
    )
    
    use_drone_control_arg = DeclareLaunchArgument(
        'use_drone_control',
        default_value='true',
        description='是否启动无人机控制节点'
    )
    
    # 获取参数
    use_usb_cam = LaunchConfiguration('use_usb_cam')
    use_realsense = LaunchConfiguration('use_realsense')
    use_plate_detection = LaunchConfiguration('use_plate_detection')
    use_drone_control = LaunchConfiguration('use_drone_control')
    
    # 1. 无人机控制节点
    drone_control_node = Node(
        package='drone_control',
        executable='drone_control_node',
        name='drone_control_node',
        output='screen',
        condition=IfCondition(use_drone_control),
        parameters=[{
            # 可以在这里添加无人机控制参数
        }]
    )
    
    # 2. 车牌检测节点
    plate_detection_node = Node(
        package='plate_detection',
        executable='plate_detection_node',
        name='plate_detection_node',
        output='screen',
        condition=IfCondition(use_plate_detection),
        parameters=[{
            'input_topic': '/image_raw',
            'output_topic': '/plate_detection_result',
            'result_image_topic': '/plate_detection_result_image',
            'detect_model_path': 'weights/plate_detect.pt',
            'rec_model_path': 'weights/plate_rec_color.pth',
            'is_color': True,
            'img_size': 640
        }]
    )
    
    # 3. USB摄像头节点
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        condition=IfCondition(use_usb_cam),
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera',
            'camera_name': 'camera'
        }],
        remappings=[
            ('image_raw', '/image_raw'),
            ('camera_info', '/camera_info')
        ]
    )
    
    # 4. Intel RealSense D435节点 (暂时注释，因为缺少 realsense2_camera 包)
    # realsense_launch_file = PathJoinSubstitution([
    #     get_package_share_directory('realsense2_camera'),
    #     'launch',
    #     'rs_launch.py'
    # ])
    
    # realsense_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([realsense_launch_file]),
    #     condition=IfCondition(use_realsense),
    #     launch_arguments={
    #         'camera_type': 'D435',
    #         'enable_color': 'true',
    #         'enable_depth': 'false',  # 只启用彩色图像
    #         'enable_sync': 'false',
    #         'color_width': '640',
    #         'color_height': '480',
    #         'color_fps': '30'
    #     }.items()
    # )
    
    # 5. 图像话题重映射节点（用于RealSense）
    # 当使用RealSense时，将彩色图像话题重映射到车牌检测节点期望的话题
    # image_remap_node = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     name='image_relay',
    #     output='screen',
    #     condition=IfCondition(use_realsense),
    #     parameters=[{
    #         'input_topic': '/camera/color/image_raw',
    #         'output_topic': '/image_raw'
    #     }]
    # )
    
    return LaunchDescription([
        # 声明参数
        use_usb_cam_arg,
        use_realsense_arg,
        use_plate_detection_arg,
        use_drone_control_arg,
        
        # ROS-Gazebo 桥接节点
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/camera@sensor_msgs/msg/Image@gz.msgs.Image'],
            name='image_bridge_process',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'],
            name='camera_info_bridge_process',
            output='screen',
        ),
        
        # 启动节点
        drone_control_node
        # plate_detection_node,
        # usb_cam_node,
        # realsense_node,  # 已注释，需要时取消注释
        # image_remap_node,  # 已注释，需要时取消注释
    ]) 