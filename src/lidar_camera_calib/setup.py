from setuptools import setup

package_name = 'lidar_camera_calib'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name + '/launch', ['launch/calibrate.launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/lidar_camera_calib']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='LiDAR-Camera extrinsic calibration using chessboard',
    license='MIT',
    entry_points={
        'console_scripts': [
            'calibrate = lidar_camera_calib.calibrate:main',
        ],
    },
)
