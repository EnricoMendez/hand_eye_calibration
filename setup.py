import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'hand_eye_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'Config'), glob('Config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='airlab2',
    maintainer_email='airlab2@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'aruco_detection = hand_eye_calibration.aruco_detection:main',
            'aruco_pose_estimation = hand_eye_calibration.aruco_pose_estimation:main',
            'hand_eye_calibrator = hand_eye_calibration.hand_eye_calibrator:main',
            'random_marker_motion = hand_eye_calibration.random_marker_motion:main'
        ],
    },
)
