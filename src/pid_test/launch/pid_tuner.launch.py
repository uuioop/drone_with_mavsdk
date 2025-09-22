from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([
        #         PathJoinSubstitution([
        #             FindPackageShare('realsense2_camera'),
        #             'launch',
        #             'rs_launch.py'
        #         ])
        #     ]),
        #     launch_arguments={
        #         'enable_infra1': 'true'
        #     }.items()
        # ),
        # === 實例一：處理 RealSense 紅外相機 ===
        # Node(
        #     package='aruco_tracker',
        #     executable='aruco_tracker',
        #     name='infra_aruco_tracker',
        #     namespace='infra_tracker',
        #     output='screen',
        #     parameters=[{
        #         'aruco_id': 0,
        #         'dictionary': 2,
        #         'marker_size': 0.289
        #     }],
        #     remappings=[
        #         # 輸入話題重映射
        #         ('/image_raw', '/camera/camera/infra1/image_rect_raw'),
        #         ('/camera_info', '/camera/camera/infra1/camera_info'),
        #         # 輸出話題重映射
        #         ('/target_pose', '/infra_tracker/target_pose'),
        #         ('/image_proc', '/infra_tracker/image_proc'),
        #     ]
        # ),

        # === 實例二：處理 USB 相機 ===
        Node(
            package='pid_test',
            executable='pid_tuner',
            name='pid_tuner',
            output='screen',
            parameters=[{
                'kp': 1.0,
                'ki': 0.0,
                'kd': 0.0,
                'max_speed': 1.5
            }]
        )
    ])