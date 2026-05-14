"""Launch LiDAR + IMU and record data bag.

Usage:
  ros2 launch data_recorder record_lidar_imu.launch.py
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

    velodyne_share = get_package_share_directory('velodyne')
    dm_imu_share = get_package_share_directory('dm_imu')

    return LaunchDescription([
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
        # Record LiDAR (raw packets + point cloud) + IMU
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record',
                 '/velodyne_packets', '/velodyne_points', '/imu/data',
                 '-o', os.path.join(bag_dir, 'lidar_imu')],
            output='screen',
        ),
    ])
