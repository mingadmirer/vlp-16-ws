import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    astra_share = get_package_share_directory('astra_camera')
    velodyne_driver_share = get_package_share_directory('velodyne_driver')
    velodyne_pointcloud_share = get_package_share_directory('velodyne_pointcloud')

    return LaunchDescription([

        # --- Camera (use upstream launch XML) ---
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                os.path.join(astra_share, 'launch', 'astra.launch.xml')
            ),
        ),

        # --- VLP-16 driver ---
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=[os.path.join(
                velodyne_driver_share, 'config',
                'VLP16-velodyne_driver_node-params.yaml'
            )],
        ),

        # --- VLP-16 point cloud transform ---
        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            output='screen',
            parameters=[{
                'calibration': os.path.join(
                    velodyne_pointcloud_share, 'params', 'VLP16db.yaml'
                ),
                'model': 'VLP16',
                'min_range': 0.9,
                'max_range': 130.0,
                'organize_cloud': True,
            }],
        ),

        # --- Calibration ---
        Node(
            package='lidar_camera_calib',
            executable='calibrate',
            name='lidar_camera_calib',
            output='screen',
            parameters=[{
                'chessboard_cols': 9,
                'chessboard_rows': 6,
                'square_size': 0.025,
                'min_samples': 15,
            }],
        ),
    ])
