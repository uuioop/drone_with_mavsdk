from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    
    # 讀取共享的參數檔案
    params_file = PathJoinSubstitution([
        FindPackageShare('aruco_tracker'), 
        'cfg', 
        'params.yaml'
    ])

    return LaunchDescription([
        # === 新增：啟動 RealSense 相機驅動 ===
        # 對應指令: ros2 launch realsense2_camera rs_launch.py enable_infra1:=true
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('realsense2_camera'),
                    'launch',
                    'rs_launch.py'
                ])
            ]),
            launch_arguments={
                'enable_infra1': 'true',
            }.items()
        ),

        # === 新增：啟動 USB 相機驅動 ===
        # 對應指令: ros2 run usb_cam usb_cam_node_exe --ros-args -p ...
        # Node(
        #     package='usb_cam',
        #     executable='usb_cam_node_exe',
        #     name='usb_cam_node',
        #     parameters=[{
        #         'video_device': '/dev/video0',
        #         'brightness': 0,
        #         'exposure_auto': 1,
        #         'exposure_absolute': 150
        #     }]
        # ),
        
        # === 實例一：處理 RealSense 紅外相機 ===
        Node(
            package='aruco_tracker',
            executable='aruco_tracker',
            name='infra_aruco_tracker',
            namespace='infra_tracker',
            output='screen',
            parameters=[{
                'aruco_id': 0,
                'dictionary': 2,
                'marker_size': 0.289
            }],
            remappings=[
                # 輸入話題重映射
                ('/image_raw', '/camera/camera/infra1/image_rect_raw'),
                ('/camera_info', '/camera/camera/infra1/camera_info'),
                # 輸出話題重映射
                ('/target_pose', '/infra_tracker/target_pose'),
                ('/image_proc', '/infra_tracker/image_proc'),
            ]
        ),

        # === 實例二：處理 USB 相機 ===
        # Node(
        #     package='aruco_tracker',
        #     executable='aruco_tracker',
        #     name='mono_aruco_tracker',
        #     namespace='mono_tracker',
        #     output='screen',
        #     parameters=[params_file],
        #     remappings=[
        #         # 輸入話題重映射
        #         ('/image_raw', '/image_raw'),
        #         ('/camera_info', '/camera_info'),
        #         # 輸出話題重映射
        #         ('/target_pose', '/mono_tracker/target_pose'),
        #         ('/image_proc', '/mono_tracker/image_proc'),
        #     ]
        # )
    ])