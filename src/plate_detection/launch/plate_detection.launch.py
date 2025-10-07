#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    启动门牌检测节点
    功能：检测图像中的门牌并识别门牌号码
    """
    # 获取包的共享目录
    pkg_share = get_package_share_directory('plate_detection')
    
    # 声明启动参数
    detect_model_arg = DeclareLaunchArgument(
        'detect_model_path',
        default_value='weights/plate_detect.pt',
        description='门牌检测模型路径'
    )
    
    rec_model_arg = DeclareLaunchArgument(
        'rec_model_path',
        default_value='weights/plate_rec_color.pth',
        description='门牌识别模型路径'
    )
    
    is_color_arg = DeclareLaunchArgument(
        'is_color',
        default_value='true',
        description='是否启用颜色识别'
    )
    
    img_size_arg = DeclareLaunchArgument(
        'img_size',
        default_value='640',
        description='输入图像尺寸'
    )
    
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/camera/camera/color/image_raw',
        description='输入图像话题'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/license_detection_result',
        description='检测结果输出话题'
    )
    
    result_image_topic_arg = DeclareLaunchArgument(
        'result_image_topic',
        default_value='/license_detection_result_image',
        description='检测结果图像输出话题'
    )
    
    # 创建门牌检测节点
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
        # 启动rqt图像查看器显示检测结果
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            arguments=['--standalone', 'rqt_image_view/ImageView',
                       '--args', '/license_detection_result_image'],
            output='screen')
    ])