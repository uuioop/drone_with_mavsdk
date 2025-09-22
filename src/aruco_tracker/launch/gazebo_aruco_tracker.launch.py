from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            name='ir_image_view',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view'],
            name='mono_image_view',
            output='screen',
        ),

        # =================================================================
        # ==================== Gazebo 到 ROS2 的話題橋接 ====================
        # =================================================================

        # --- 單目相機 (mono_cam_down) ---
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/mono_cam_down/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image@gz.msgs.Image', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/mono_cam_down/link/camera_link/sensor/imager/image:=/image_raw'],
            name='mono_image_bridge',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/mono_cam_down/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/mono_cam_down/link/camera_link/sensor/imager/camera_info:=/camera_info'],
            name='mono_camera_info_bridge',
            output='screen',
        ),

        # --- 深度相機 (depth_cam_front) ---
        # 用于车牌识别
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/IMX214/image@sensor_msgs/msg/Image@gz.msgs.Image', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/IMX214/image:=/camera/camera/color/image_raw'],
            name='rgb_image_bridge',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/left_ir_camera/image@sensor_msgs/msg/Image@gz.msgs.Image', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/left_ir_camera/image:=/depth_camera/infra1/image_rect_raw'],
            name='ir_image_bridge',
            output='screen',
        ),
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/left_ir_camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo', '--ros-args', '-r', '/world/default/model/x500_depth_0/model/depth_cam_front/link/camera_link/sensor/left_ir_camera/camera_info:=/depth_camera/camera_info'],
            name='rgb_camera_info_bridge',
            output='screen',
        ),


        # =================================================================
        # ======================= Aruco Tracker 節點 ======================
        # =================================================================

        # === 實例一：處理單目相機 ===
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='mono_aruco_tracker',
            namespace='mono_tracker',
            output='screen',
            parameters=[{
                'aruco_id': 0,
                'dictionary': 2,
                'marker_size': 0.5
            }],
            remappings=[
                # 輸入話題重映射
                ('/image_raw', '/image_raw'),
                ('/camera_info', '/camera_info'),
                # **新增：輸出話題重映射**
                ('/target_pose', '/mono_tracker/target_pose'),
                ('/image_proc', '/mono_tracker/image_proc'),
            ]
        ),

        # === 實例二：處理紅外相機 ===
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='infra_aruco_tracker',
            namespace='infra_tracker',
            output='screen',
            parameters=[{
                'aruco_id': 1,
                'dictionary': 2,
                'marker_size': 0.5
            }],
            remappings=[
                # 輸入話題重映射
                ('/image_raw', '/depth_camera/infra1/image_rect_raw'),
                ('/camera_info', '/depth_camera/camera_info'),
                # 新增：輸出話題重映射
                ('/target_pose', '/infra_tracker/target_pose'),
                ('/image_proc', '/infra_tracker/image_proc'),
            ]
        ),
    ])