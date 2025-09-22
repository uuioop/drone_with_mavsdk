#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包的共享目录
    pkg_share = get_package_share_directory('plate_detection')
    
    # 声明启动参数
    detect_model_arg = DeclareLaunchArgument(
        'detect_model_path',
        default_value='weights/plate_detect.pt',
        description='Path to detection model'
    )
    
    rec_model_arg = DeclareLaunchArgument(
        'rec_model_path',
        default_value='weights/plate_rec_color.pth',
        description='Path to recognition model'
    )
    
    is_color_arg = DeclareLaunchArgument(
        'is_color',
        default_value='true',
        description='Enable color recognition'
    )
    
    img_size_arg = DeclareLaunchArgument(
        'img_size',
        default_value='640',
        description='Input image size'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/image_raw',
        description='Input image topic'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/plate_detection_result',
        description='Output result topic'
    )
    
    result_image_topic_arg = DeclareLaunchArgument(
        'result_image_topic',
        default_value='/plate_detection_result_image',
        description='Output result image topic'
    )
    
    # 创建节点
    plate_detection_node = Node(
        package='plate_detection',
        executable='plate_detection_node',
        name='plate_detection_node',
        output='screen',
        parameters=[{
            'detect_model_path': LaunchConfiguration('detect_model_path'),
            'rec_model_path': LaunchConfiguration('rec_model_path'),
            'is_color': LaunchConfiguration('is_color'),
            'img_size': LaunchConfiguration('img_size'),
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'result_image_topic': LaunchConfiguration('result_image_topic'),
        }]
    )
    
    return LaunchDescription([
        detect_model_arg,
        rec_model_arg,
        is_color_arg,
        img_size_arg,
        input_topic_arg,
        output_topic_arg,
        result_image_topic_arg,
        plate_detection_node,
    ]) 