# FAST-LIVO2

FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry。

本仓库是 FAST-LIVO2 的修改版，适配 VLP-16 + DM IMU + Astra 相机。

原仓库：[hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2)

## 架构

C++ ROS2 包，核心源文件：

| 文件               | 职责                            |
|--------------------|---------------------------------|
| `LIVMapper.cpp`    | 主节点，融合 LiDAR/视觉/IMU 数据 |
| `IMU_Processing.cpp` | IMU 初始化（重力估计）、IMU 传播 |
| `voxel_map.cpp`    | 体素地图 ICP 匹配               |
| `vio.cpp`          | 视觉惯性里程计                  |
| `preprocess.cpp`   | LiDAR 点云预处理                |
| `frame.cpp`        | 帧管理                         |
| `main.cpp`         | 入口                           |

## 模式

- **LIVO**（默认）：LiDAR + 视觉 + IMU 融合
- **LIO**（纯 LiDAR-IMU）：可通过关闭视觉（`img_en: 0`）降级

## 发布话题

| 话题                                 | 类型                              |
|--------------------------------------|-----------------------------------|
| `/Odometry`                          | `nav_msgs/Odometry`               |
| `/path`                              | `nav_msgs/Path`                   |
| `/cloud_registered`                  | `sensor_msgs/PointCloud2`         |
| `/cloud_effect`                      | `sensor_msgs/PointCloud2`         |
| TF `camera_init` → `aft_mapped`      | `tf2_msgs/TFMessage`              |

## 参数 (`config/vlp16_astra.yaml`)

### common

| 参数             | 值                      | 说明               |
|------------------|-------------------------|-------------------|
| `img_topic`      | `/camera/color/image_raw` | 图像话题          |
| `lid_topic`      | `/velodyne_points`        | 雷达点云话题      |
| `imu_topic`      | `/imu/data`               | IMU 话题          |
| `img_en` / `lidar_en` | `1` / `1`            | 启用视觉/激光     |

### imu

| 参数             | 默认值 | 说明                        |
|------------------|--------|-----------------------------|
| `imu_int_frame`  | `500`  | IMU 初始化帧数（越大重力估计越稳） |
| `acc_cov`        | `0.5`  | 加速度协方差（越小越信任 IMU）    |
| `gyr_cov`        | `0.3`  | 陀螺协方差                      |
| `b_acc_cov`      | `0.0001` | 加速度偏置随机游走             |
| `b_gyr_cov`      | `0.0001` | 陀螺偏置随机游走               |

### vio

| 参数                | 默认值 | 说明            |
|---------------------|--------|----------------|
| `max_iterations`    | `5`    | 视觉优化迭代数  |
| `img_point_cov`     | `100`  | 图像点协方差    |
| `patch_size`        | `8`    | 图像 patch 大小 |
| `normal_en`         | `true` | 启用法线约束    |

### lio

| 参数                | 默认值 | 说明            |
|---------------------|--------|----------------|
| `max_iterations`    | `5`    | LIO 优化迭代数  |
| `dept_err`          | `0.02` | 深度误差阈值    |
| `beam_err`          | `0.05` | 光束误差阈值    |
| `voxel_size`        | `0.5`  | 体素大小        |

### extrin_calib

| 参数          | 说明                         |
|---------------|------------------------------|
| `extrinsic_T` | LiDAR→IMU 平移 (m)            |
| `extrinsic_R` | LiDAR→IMU 旋转（在线优化，初始 identity） |
| `Rcl`         | LiDAR→Camera 旋转（标定值）    |
| `Pcl`         | LiDAR→Camera 平移（标定值, m）  |

### 相机内参 (`config/camera_astra.yaml`)

| 参数      | 值              | 说明       |
|-----------|----------------|------------|
| `cam_fx`  | 596.278        | 焦距 x     |
| `cam_fy`  | 598.538        | 焦距 y     |
| `cam_cx`  | 303.465        | 主点 x     |
| `cam_cy`  | 245.185        | 主点 y     |
| D0~D4     | 畸变系数       | 径向/切向  |

## 使用方式

```bash
# 单独启动（不含传感器驱动，需先启动各驱动节点）
ros2 launch fast_livo mapping_vlp16.launch.py

# 完整建图（推荐，通过 vlp16_mapping 一键启动）
ros2 launch vlp16_mapping mapping.launch.py
```

## 输入依赖

| 话题                          | 来源                                      |
|-------------------------------|-------------------------------------------|
| `/velodyne_points`            | [velodyne_pointcloud](../velodyne/README.md) |
| `/imu/data`                   | [dm_imu](../dm_imu/README.md)             |
| `/camera/color/image_raw`     | [astra_camera](../ros2_astra_camera/README.md) |

## 输出文件

- **PCD 点云地图**：`pcd_save_en: true` 时保存，默认在工作目录
- **位姿轨迹**（TUM 格式）：`pose_output_en: true`

## 关键配置注意事项

1. **修改 yaml 后需同步到 install 目录**：launch 文件从 `install/fast_livo/share/fast_livo/config/` 加载配置，修改 `src/FAST-LIVO2/config/` 后需 `colcon build --packages-select fast_livo` 或手动 `cp`
2. **IMU 重力估计**：`imu_int_frame` 控制初始化时累积的 IMU 帧数，对 DM IMU（其读数会漂移）需要设得较大（300~500）

## 联动

- [vlp16_mapping](../vlp16_mapping/README.md) 整合所有传感器驱动一键启动
- [rpg_vikit](../rpg_vikit/README.md) 提供相机参数获取基础库
- [dm_imu](../dm_imu/README.md) IMU 驱动需先启动
