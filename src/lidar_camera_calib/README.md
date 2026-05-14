# lidar_camera_calib

LiDAR-相机外参标定工具。使用棋盘格标定板，通过 Python 实现 LiDAR 点云与相机图像的联合标定。

## 节点

- 入口：`lidar_camera_calib/calibrate.py`
- 可执行：`calibrate`

## 使用方式

```bash
ros2 launch lidar_camera_calib calibrate.launch.py
```

## 功能

- 使用棋盘格标定板进行 LiDAR-Camera 外参标定
- 输出 LiDAR→Camera 的旋转矩阵 `Rcl` 和平移向量 `Pcl`

## 输入

| 话题                | 来源                                     |
|---------------------|------------------------------------------|
| `/velodyne_points`  | [velodyne_pointcloud](../velodyne/README.md) |
| `/camera/color/image_raw` | [astra_camera](../ros2_astra_camera/README.md) |

## 依赖

- rclpy、sensor_msgs、cv_bridge
- OpenCV、NumPy
- message_filters（时间同步）

## 联动

标定结果填入 [FAST-LIVO2](../FAST-LIVO2/README.md) 的配置文件 `vlp16_astra.yaml` 中的 `Rcl` 和 `Pcl` 字段。
