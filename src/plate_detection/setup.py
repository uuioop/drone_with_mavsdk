import os
from glob import glob
from setuptools import setup

def get_files_and_destinations(source_dir, package_name):
    """递归获取指定目录下所有文件及其目标安装路径"""
    data_files = []
    for root, _, files in os.walk(source_dir):
        for file in files:
            source_file = os.path.join(root, file)
            # 构建相对于 package_name 的目标路径
            relative_path = os.path.relpath(root, source_dir)
            destination_dir = os.path.join('share', package_name, source_dir, relative_path)
            data_files.append((destination_dir, [source_file]))
    return data_files

package_name = 'plate_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # 安装配置文件
        (os.path.join('share', package_name, 'config'), 
         glob(os.path.join('config', '*.yaml'))),
        # 安装模型权重文件
        (os.path.join('share', package_name, 'weights'), 
         glob(os.path.join('weights', '*.pt*'))),
        (os.path.join('share', package_name, 'weights'), 
         glob(os.path.join('weights', '*.pth*'))),
        # 安装typing_patch.py
        (os.path.join('share', package_name), ['typing_patch.py']),
    ] + get_files_and_destinations('models', package_name) + \
        get_files_and_destinations('fonts', package_name) + \
        get_files_and_destinations('utils', package_name) + \
        get_files_and_destinations('plate_recognition', package_name),
    install_requires=['setuptools','drone_control_interfaces'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'plate_detection_node = plate_detection.plate_detection_node:main'
        ],
    },
)