from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'track_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일을 설치
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_drive = track_drive.track_drive:main',
            'traffic_light = track_drive.traffic_light:main',
            'cone_drive = track_drive.cone_drive_node:main',
            'with_yolo = track_drive.with_yolo:main',
            'yolo_light = track_drive.with_yolo_traffic_light:main',
            'child_zone = track_drive.child_zone:main',
            'overtake = track_drive.overtaking:main',
            'center_line = track_drive.center_line:main',
            'lane = track_drive.lane:main',
            'line = track_drive.line:main',
            'rabar = track_drive.rabacon:main'
        ],
    },
)
