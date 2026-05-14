# velodyne

Velodyne VLP-16 LiDAR 驱动（ROS2 版）。从网络接收原始 UDP 数据包，转换为点云。

## 子包

| 包                    | 功能                             |
|-----------------------|----------------------------------|
| `velodyne_driver`     | 网络驱动，接收 UDP 原始数据包      |
| `velodyne_pointcloud` | 原始包 → 点云转换                |
| `velodyne_laserscan`  | 发布 LaserScan 消息              |
| `velodyne_msgs`       | 自定义消息定义                   |

## 工作流程

```
VLP-16 LiDAR (UDP 数据包 @ 600 RPM / 10 Hz)
    ↓  端口 2368
velodyne_driver_node
    ↓  /velodyne_packets
velodyne_transform_node
    ↓  /velodyne_points (sensor_msgs/PointCloud2)
FAST-LIVO2 / 其他订阅者
```

## VLP-16 规格

- 16 线，水平 360°，垂直 ±15°
- 默认转速 600 RPM（10 Hz 扫描频率）
- 每帧约 76 个 UDP 包，每包 1206 字节
- 最大测距 ~100 m
- 精度 ±3 cm

## 参数

### 驱动节点 (`config/VLP16-velodyne_driver_node-params.yaml`)

| 参数                    | 默认值            | 说明                 |
|------------------------|-------------------|---------------------|
| `device_ip`            | `192.168.1.201`   | LiDAR IP 地址        |
| `port`                 | `2368`            | 数据接收 UDP 端口     |
| `model`                | `VLP16`           | LiDAR 型号           |
| `rpm`                  | `600.0`           | 转速                 |
| `frame_id`             | `velodyne`        | 输出点云坐标系         |
| `gps_time`             | `false`           | 使用 GPS 时间戳       |
| `timestamp_first_packet` | `false`         | 用帧首包时间戳         |

### 转换节点 (`config/VLP16-velodyne_transform_node-params.yaml`)

| 参数             | 默认值  | 说明              |
|------------------|---------|------------------|
| `model`          | `VLP16` | LiDAR 型号        |
| `calibration`    | `VLP16db.yaml` | 校准文件  |
| `min_range`      | `0.9`   | 最小有效距离 (m)   |
| `max_range`      | `130.0` | 最大有效距离 (m)   |
| `organize_cloud` | `true`  | 组织化点云输出     |

## 使用方式

```bash
# 单独启动
ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py

# 检查点云
ros2 topic echo /velodyne_points
```

## 网络配置

VLP-16 通过有线网口直连工控机：

- 工控机网口 IP：`192.168.1.100`
- VLP-16 默认 IP：`192.168.1.201`
- 子网掩码：`255.255.255.0`

## 联动

- 发布 `/velodyne_points` 供 [FAST-LIVO2](../FAST-LIVO2/README.md) 使用
- [vlp16_mapping](../vlp16_mapping/README.md) 启动时包含此节点
- [data_recorder](../data_recorder/README.md) 可录制 LiDAR 数据
