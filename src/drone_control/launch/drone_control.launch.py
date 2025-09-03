#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # 1. 无人机控制节点
    drone_control_node = Node(
        package='drone_control',
        executable='drone_control_node',
        name='drone_control_node',
        output='screen'
    )
    
    # 2. 启动 Intel RealSense D435 的 ExecuteProcess
    # 假设校准文件位于 ~/.ros/camera_info/realsense_infra1_848x480.yaml
    realsense_camera_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'realsense2_camera', 'realsense2_camera_node',
            '--ros-args',
            # '--remap', '__ns:=/realsense', # 设置命名空间
            '-p', 'enable_infra1:=true',
            '-p', 'enable_color:=true',
            '-p', 'infra_width:=848',
            '-p', 'infra_height:=480',
            '-p', 'infra1_camera_info_url:=file://${HOME}/.ros/camera_info/realsense_infra1_848x480.yaml',
            '-p', 'enable_depth:=false',
            '-p', 'enable_imu:=false',
            '-p', 'pointcloud.enable:=false'
        ],
        name='realsense_camera_process',
        output='screen'
    )

    # 3. 启动 USB 相机的 ExecuteProcess
    usb_camera_process = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'usb_cam', 'usb_cam_node_exe',
            '--ros-args',
            # '--remap', '__ns:=/usb', # 设置命名空间
            '-p', 'video_device:=/dev/video0', # 请确认这是正确的设备号
            '-p', 'image_width:=640',
            '-p', 'image_height:=480',
            '-p', 'pixel_format:=yuyv',
            '-p', 'camera_name:=usb_camera_node'
        ],
        name='usb_camera_process',
        output='screen'
    )
    
    return LaunchDescription([
        drone_control_node,
        realsense_camera_process,
        usb_camera_process
    ])