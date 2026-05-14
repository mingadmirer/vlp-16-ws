import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    vlp16_driver_config = os.path.join(
        get_package_share_directory('velodyne_driver'), 'config',
        'VLP16-velodyne_driver_node-params.yaml')

    vlp16_transform_config = os.path.join(
        get_package_share_directory('velodyne_pointcloud'), 'config',
        'VLP16-velodyne_transform_node-params.yaml')

    vlp16_calib = os.path.join(
        get_package_share_directory('velodyne_pointcloud'), 'params',
        'VLP16db.yaml')

    camera_config = os.path.join(
        get_package_share_directory('fast_livo'), 'config',
        'camera_astra.yaml')

    fast_livo_config = os.path.join(
        get_package_share_directory('fast_livo'), 'config',
        'vlp16_astra.yaml')

    dm_imu_config = os.path.join(
        get_package_share_directory('dm_imu'), 'config',
        'params.yaml')

    rviz_config = os.path.join(
        get_package_share_directory('fast_livo'), 'rviz_cfg',
        'fast_livo2.rviz')

    astra_launch = os.path.join(
        get_package_share_directory('astra_camera'), 'launch',
        'astra.launch.xml')

    return LaunchDescription([
        Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=[vlp16_driver_config, {'timestamp_first_packet': True}],
        ),

        Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            output='screen',
            parameters=[vlp16_transform_config, {'calibration': vlp16_calib}],
        ),

        Node(
            package='dm_imu',
            executable='dm_imu_node',
            name='dm_imu',
            output='screen',
            parameters=[dm_imu_config],
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(astra_launch),
        ),

        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            parameters=[camera_config],
            output='screen',
        ),

        Node(
            package='fast_livo',
            executable='fastlivo_mapping',
            name='laserMapping',
            parameters=[fast_livo_config],
            output='screen',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        ),
    ])
