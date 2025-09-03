import os
from glob import glob
from setuptools import setup

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plate_detection_node = plate_detection.plate_detection_node:main',
            'plate_detection_rgbd_node = plate_detection.plate_detection_rgbd_node:main',
        ],
    },
)