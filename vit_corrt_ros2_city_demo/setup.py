from setuptools import find_packages, setup
from glob import glob

package_name = 'vit_corrt_ros2_city_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/data', glob('data/*.json')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzou',
    maintainer_email='zouwenxing719@gmail.com',
    description='ROS2/RViz2 visualization demo for structured 3D city multi-UAV path planning results.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'city_result_visualizer = vit_corrt_ros2_city_demo.city_result_visualizer_node:main',
        ],
    },
)
