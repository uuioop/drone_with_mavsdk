from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """
    启动Gazebo仿真环境中的ArUco标记追踪系统
    功能：建立Gazebo到ROS2的话题桥接，处理单目相机和红外相机，检测ArUco标记
    """
    return LaunchDescription([
        # =================================================================
        # ==================== Gazebo到ROS2的话题桥接 =====================
        # =================================================================

        # --- 单目相机（mono_cam_down）---
        # 桥接单目相机图像话题
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/mono_cam_down/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/mono_cam_down/link/camera_link/sensor/imager/image:=/image_raw'],
            name='mono_image_bridge',
            output='screen',
        ),
        # 桥接单目相机参数信息
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/mono_cam_down/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/mono_cam_down/link/camera_link/sensor/imager/camera_info:=/camera_info'],
            name='mono_camera_info_bridge',
            output='screen',
        ),

        # --- 深度相机（depth_cam_front）---
        # 用于门牌识别的RGB图像
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/IMX214/image:=/camera/camera/color/image_raw'],
            name='rgb_image_bridge',
            output='screen',
        ),
        # 红外相机图像
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/left_ir_camera/image@sensor_msgs/msg/Image@gz.msgs.Image', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/left_ir_camera/image:=/depth_camera/infra1/image_rect_raw'],
            name='ir_image_bridge',
            output='screen',
        ),
        # 红外相机参数信息
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/left_ir_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/left_ir_camera/camera_info:=/depth_camera/camera_info'],
            name='rgb_camera_info_bridge',
            output='screen',
        ),


        # =================================================================
        # ======================= ArUco Tracker节点 ========================
        # =================================================================

        # === 实例一：处理单目相机 ===
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='mono_aruco_tracker',
            namespace='mono_tracker',
            output='screen',
            parameters=[{
                'aruco_id': 0,  # 标记ID
                'dictionary': 2,  # 字典类型
                'marker_size': 0.5  # 标记实际尺寸（米）
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

        # === 实例二：处理红外相机 ===
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='infra_aruco_tracker',
            namespace='infra_tracker',
            output='screen',
            parameters=[{
                'aruco_id': 1,  # 标记ID
                'dictionary': 2,  # 字典类型
                'marker_size': 0.5  # 标记实际尺寸（米）
            }],
            remappings=[
                # 输入话题重映射
                ('/image_raw', '/depth_camera/infra1/image_rect_raw'),
                ('/camera_info', '/depth_camera/camera_info'),
                # 输出话题重映射
                ('/target_pose', '/infra_tracker/target_pose'),
                ('/image_proc', '/infra_tracker/image_proc'),
            ]
        ),
        # === 可视化单目相机结果 ===
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            arguments=['--standalone', 'rqt_image_view/ImageView',
                       '--args', '/mono_tracker/image_proc'],
            output='screen'),
        # === 可视化红外相机结果 ===
        Node(
            package='rqt_gui',
            executable='rqt_gui',
            arguments=['--standalone', 'rqt_image_view/ImageView',
                       '--args', '/infra_tracker/image_proc'],
            output='screen')
    ])