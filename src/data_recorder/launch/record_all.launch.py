"""Launch LiDAR + IMU + Camera and record data bag.

Usage:
  ros2 launch data_recorder record_all.launch.py
  # Ctrl+C to stop recording
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_dir = os.path.join(os.path.expanduser('~'), 'bag_files')
    os.makedirs(bag_dir, exist_ok=True)

    astra_share = get_package_share_directory('astra_camera')
    velodyne_share = get_package_share_directory('velodyne')
    dm_imu_share = get_package_share_directory('dm_imu')

    return LaunchDescription([
        # Camera
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(astra_share, 'launch', 'astra.launch.xml')
            ),
        ),
        # LiDAR (driver + pointcloud + laserscan)
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(velodyne_share, 'launch', 'velodyne-all-nodes-VLP16-launch.py')
            ),
        ),
        # IMU
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(dm_imu_share, 'launch', 'dm_imu.launch.py')
            ),
        ),
        # Record all topics
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '/velodyne_points', '/imu/data',
                 '/camera/color/image_raw', '/camera/color/camera_info',
                 '-o', os.path.join(bag_dir, 'all_sensors')],
            output='screen',
        ),
    ])
