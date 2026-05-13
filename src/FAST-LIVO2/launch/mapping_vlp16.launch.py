#!/usr/bin/python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    config_file_dir = os.path.join(get_package_share_directory("fast_livo"), "config")
    rviz_config_file = os.path.join(get_package_share_directory("fast_livo"), "rviz_cfg", "ntu_viral.rviz")

    vlp16_config_cmd = os.path.join(config_file_dir, "vlp16_astra.yaml")
    camera_config_cmd = os.path.join(config_file_dir, "camera_astra.yaml")

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="False",
        description="Whether to launch Rviz2",
    )

    vlp16_config_arg = DeclareLaunchArgument(
        'vlp16_params_file', default_value=vlp16_config_cmd,
        description='Full path to the ROS2 parameters file for fast_livo2 nodes',
    )

    camera_config_arg = DeclareLaunchArgument(
        'camera_params_file', default_value=camera_config_cmd,
        description='Full path to the ROS2 parameters file for vikit_ros nodes',
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn', default_value='True',
        description='Whether to respawn if a node crashes.',
    )

    return LaunchDescription([
        use_rviz_arg, vlp16_config_arg, camera_config_arg, use_respawn_arg,

        Node(
            package='demo_nodes_cpp',
            executable='parameter_blackboard',
            name='parameter_blackboard',
            parameters=[camera_config_cmd],
            output='screen'
        ),

        Node(
            package="fast_livo",
            executable="fastlivo_mapping",
            name="laserMapping",
            parameters=[vlp16_config_cmd],
            output="screen"
        ),

        Node(
            condition=IfCondition(LaunchConfiguration("use_rviz")),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen"
        ),
    ])
