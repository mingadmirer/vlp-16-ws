# dm_imu

达妙（DaMiao）DMxxx 系列 IMU 的 ROS2 驱动。基于 BMI088，通过串口读取，发布标准 ROS2 IMU 消息。

## 架构

Python ROS2 包，内部包含两个核心模块：

- `node.py` — ROS2 节点，负责参数声明、话题发布、定时器调度
- `modules/dm_serial.py` — 串口读取与解析层，后台独立线程循环读取

## 通信协议

IMU 固件以 200Hz 发送固定 19 字节帧，三种帧类型由 RID 区分：

| RID | 数据     | 说明                   |
|-----|----------|-----------------------|
| 0x01 | ACC      | 三轴加速度 (m/s²)      |
| 0x02 | GYRO     | 三轴角速度 (度/秒)      |
| 0x03 | RPY      | 欧拉角 Roll/Pitch/Yaw (度) |

串口参数：921600 baud, 8N1, 硬件流控关。

> ⚠️ 该 IMU 固件内部对 ACC 做了姿态变换（非原始传感器值），且其内部姿态估计会发生漂移。
> 长时间运行后，静止时 ACC 读数会变化。如果将此数据直接喂给 FAST-LIVO2 等需要稳定重力参考的算法，
> 会导致重力方向估计误差，引发里程计漂移。

## 发布话题

| 话题          | 类型             | 频率  | 说明                               |
|---------------|------------------|-------|------------------------------------|
| `/imu/data`   | `sensor_msgs/Imu` | 200Hz | IMU 完整数据（姿态四元数、角速度、加速度） |
| `/imu/rpy`    | `Vector3Stamped`  | 200Hz | 欧拉角（可选度或弧度）                |
| `/imu/pose`   | `PoseStamped`     | 200Hz | 仅姿态（位置恒为 0），默认关闭         |

## 参数 (`config/params.yaml`)

| 参数                  | 默认值           | 说明                          |
|-----------------------|------------------|-------------------------------|
| `port`                | `/dev/ttyACM0`   | IMU 串口设备路径               |
| `baudrate`            | `921600`         | 波特率                        |
| `frame_id`            | `imu_link`       | 消息 frame_id                 |
| `publish_imu_data`    | `true`           | 发布 `/imu/data`              |
| `publish_rpy`         | `true`           | 发布 `/imu/rpy`               |
| `publish_pose`        | `false`          | 发布 `/imu/pose`              |
| `publish_rpy_in_degree` | `true`         | RPY 以度发布（false=弧度）      |
| `qos_reliable`        | `true`           | 使用 RELIABLE QoS             |
| `verbose`             | `false`          | 详细日志输出                   |

## 使用方式

```bash
# 单独启动
ros2 launch dm_imu dm_imu.launch.py

# 检查数据
ros2 topic echo /imu/data
```

## 联动

- [vlp16_mapping](../vlp16_mapping/README.md) 的 launch 文件会加载此节点
- 输出被 [FAST-LIVO2](../FAST-LIVO2/README.md) 订阅用于 IMU 传播
- [data_recorder](../data_recorder/README.md) 可录制 `/imu/data` 做离线分析
