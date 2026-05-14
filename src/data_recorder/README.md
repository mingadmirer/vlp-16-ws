# data_recorder

传感器数据录制包。一键启动所有传感器并录制 ROS2 bag。

## Launch 文件

### `record_all.launch.py`

启动所有传感器（LiDAR + IMU + Camera）并录制关键话题。

录制话题：
- `/velodyne_points`
- `/imu/data`
- `/camera/color/image_raw`
- `/camera/color/camera_info`

输出路径：`~/bag_files/all_sensors_*.bag`

### `record_lidar_imu.launch.py`

仅启动 LiDAR + IMU 并录制（不含相机），适合纯 LiDAR-IMU 建图或标定数据采集。

录制话题：
- `/velodyne_packets`
- `/velodyne_points`
- `/imu/data`

输出路径：`~/bag_files/lidar_imu_*.bag`

## 使用方式

```bash
ros2 launch data_recorder record_all.launch.py
# Ctrl+C 停止录制
```

## 依赖

- [velodyne](../velodyne/README.md)（LiDAR 驱动）
- [dm_imu](../dm_imu/README.md)（IMU 驱动）
- [ros2_astra_camera](../ros2_astra_camera/README.md)（相机驱动）

## 联动

- 录制的 bag 可用于 [FAST-LIVO2](../FAST-LIVO2/README.md) 的离线测试
- 录制的标定数据可用于 [LiDAR_IMU_Init_ROS2](../LiDAR_IMU_Init_ROS2/README.md) 和 [lidar_camera_calib](../lidar_camera_calib/README.md)
