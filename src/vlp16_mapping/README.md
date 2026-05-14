# vlp16_mapping

一键启动建图的 launch 包。整合 LiDAR、IMU、相机驱动和 FAST-LIVO2，无需逐个启动节点。

## 架构

纯 launch 包（CMake 仅用于安装 launch 文件），无源代码。

启动顺序：

```
vlp16_mapping.launch.py
 ├── velodyne_driver_node      # VLP-16 LiDAR 驱动
 ├── velodyne_transform_node   # 点云转换
 ├── dm_imu_node               # IMU 驱动
 ├── astra.launch.xml          # 相机驱动（IncludeLaunchDescription）
 ├── parameter_blackboard      # 相机参数服务器
 ├── fastlivo_mapping          # FAST-LIVO2 主节点
 └── rviz2                     # 可视化
```

## 话题流

```
velodyne_driver → /velodyne_packets → velodyne_transform → /velodyne_points ─┐
dm_imu          → /imu/data ─────────────────────────────────────────────────┼──→ FAST-LIVO2
astra_camera    → /camera/color/image_raw ──────────────────────────────────┘
```

## 配置来源

| 包                     | 配置路径                                 |
|------------------------|------------------------------------------|
| VLP-16 驱动            | `velodyne_driver/config/VLP16-*.yaml`    |
| VLP-16 点云转换        | `velodyne_pointcloud/config/VLP16-*.yaml`|
| DM IMU                 | `dm_imu/config/params.yaml`              |
| Astra 相机参数         | `fast_livo/config/camera_astra.yaml`     |
| FAST-LIVO2             | `fast_livo/config/vlp16_astra.yaml`      |
| RVIZ                   | `fast_livo/rviz_cfg/fast_livo2.rviz`     |
| LiDAR→Camera 外参标定  | `fast_livo/config/vlp16_astra.yaml:Rcl/Pcl` |

> ⚠️ launch 文件加载的是 `install/` 目录下的配置，修改源代码 `src/` 中的 yaml 后需 `colcon build` 或手动 `cp` 到 install 目录。

## 使用方式

```bash
# 完整建图
ros2 launch vlp16_mapping mapping.launch.py

# 停止：Ctrl+C
```

## 依赖

- [velodyne](../velodyne/README.md)（驱动 + 点云转换）
- [dm_imu](../dm_imu/README.md)（IMU 驱动）
- [FAST-LIVO2](../FAST-LIVO2/README.md)（建图核心）
- [ros2_astra_camera](../ros2_astra_camera/README.md)（相机驱动）
- rviz2

## 联动

- 依赖所有传感器驱动的 package 和 FAST-LIVO2
- 修改参数后需要 `colcon build --packages-select vlp16_mapping fast_livo` 或手动同步 yaml 到 install 目录
