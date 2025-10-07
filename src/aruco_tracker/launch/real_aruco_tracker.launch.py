from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    启动真实环境中的ArUco标记追踪系统
    功能：同时处理RealSense红外相机和USB相机，检测ArUco标记并发布位姿信息
    """
    return LaunchDescription([
        # 启动RealSense相机（对应指令: ros2 launch realsense2_camera rs_launch.py enable_infra1:=true）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'enable_infra1': 'true',  # 启用红外相机
                'serial_no': "'141222072356'"  # 相机序列号
            }.items()
        ),

        # 启动USB相机（对应指令: ros2 run usb_cam usb_cam_node_exe --ros-args -p ...）
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            parameters=[{
                'video_device': '/dev/video0',  # USB设备路径（会变，建议USB相机先插入香橙派）
                'brightness': 0,  # 亮度设置
                'exposure_auto': 1,  # 自动曝光
                'exposure_absolute': 150  # 曝光值
            }]
        ),
        
        # === 实例一：处理RealSense红外相机 ===
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='infra_aruco_tracker',
            namespace='infra_tracker',
            output='screen',
            parameters=[{
                'aruco_id': 1,  # 标记ID
                'dictionary': 2,  # 字典类型
                'marker_size': 0.289  # 标记实际尺寸（米）
            }],
            remappings=[
                # 输入话题重映射
                ('/image_raw', '/camera/camera/infra1/image_rect_raw'),
                ('/camera_info', '/camera/camera/infra1/camera_info'),
                # 输出话题重映射
                ('/target_pose', '/infra_tracker/target_pose'),
                ('/image_proc', '/infra_tracker/image_proc'),
            ]
        ),

        # === 实例二：处理USB相机 ===
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='mono_aruco_tracker',
            namespace='mono_tracker',
            output='screen',
            parameters=[{
                'aruco_id': 0,  # 标记ID
                'dictionary': 2,  # 字典类型
                'marker_size': 0.198  # 标记实际尺寸（米）
            }],
            remappings=[
                # 输入话题重映射
                ('/image_raw', '/image_raw'),
                ('/camera_info', '/camera_info'),
                # 输出话题重映射
                ('/target_pose', '/mono_tracker/target_pose'),
                ('/image_proc', '/mono_tracker/image_proc'),
            ]
        ),
        # 可视化USB相机检测结果
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            arguments=['--standalone', 'rqt_image_view/ImageView',
                       '--args', '/mono_tracker/image_proc'],
            output='screen'),
        # 可视化红外相机检测结果
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            arguments=['--standalone', 'rqt_image_view/ImageView',
                       '--args', '/infra_tracker/image_proc'],
            output='screen')
    ])