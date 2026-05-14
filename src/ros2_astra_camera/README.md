# ros2_astra_camera

Orbbec Astra 系列 3D 相机的 ROS2 驱动。提供彩色、深度、IR 图像流。

## 子包

| 包                   | 功能           |
|----------------------|----------------|
| `astra_camera`       | 相机驱动 C++ 节点 |
| `astra_camera_msgs`  | 自定义消息和服务  |

## 发布话题

| 话题                               | 类型                           | 说明        |
|------------------------------------|--------------------------------|-------------|
| `/camera/color/image_raw`          | `sensor_msgs/Image`            | 彩色图      |
| `/camera/color/camera_info`        | `sensor_msgs/CameraInfo`       | 彩色相机内参 |
| `/camera/depth/image_raw`          | `sensor_msgs/Image`            | 深度图      |
| `/camera/depth/camera_info`        | `sensor_msgs/CameraInfo`       | 深度相机内参 |
| `/camera/ir/image_raw`             | `sensor_msgs/Image`            | IR 图       |
| `/camera/color/points`             | `sensor_msgs/PointCloud2`      | RGB 点云    |

## 重要参数

| 参数                     | 默认值   | 说明                   |
|--------------------------|----------|-----------------------|
| `color_width`/`height`   | 640×480  | RGB 分辨率              |
| `color_fps`              | 30       | RGB 帧率                |
| `depth_width`/`height`   | 640×480  | 深度分辨率               |
| `depth_fps`              | 30       | 深度帧率                 |
| `enable_color`           | `true`   | 启用 RGB 相机            |
| `enable_depth`           | `true`   | 启用深度相机              |
| `enable_ir`              | `false`  | 启用 IR 相机             |
| `depth_registration`     | `false`  | 深度对齐到彩色            |
| `enable_point_cloud`     | `false`  | 启用点云输出              |

## 使用方式

```bash
# 单独启动
ros2 launch astra_camera astra.launch.xml

# 检查图像
ros2 run rqt_image_view rqt_image_view
```

## 依赖安装

```bash
# 安装 libuvc
git clone https://github.com/libuvc/libuvc.git
cd libuvc && mkdir build && cd build
cmake .. && make -j4 && sudo make install && sudo ldconfig

# 安装 udev 规则
sudo bash src/ros2_astra_camera/astra_camera/scripts/install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```

> ⚠️ 启动前需清理共享内存：`ros2 run astra_camera cleanup_shm_node`

## 联动

- `/camera/color/image_raw` 被 [FAST-LIVO2](../FAST-LIVO2/README.md) 订阅用于视觉里程计
- [vlp16_mapping](../vlp16_mapping/README.md) 启动时通过 astra.launch.xml 包含此驱动
- [data_recorder](../data_recorder/README.md) 可录制图像数据
- 相机标定参数存储在 [lidar_camera_calib](../lidar_camera_calib/README.md) 过程中获取

## 标定

相机内参通过 `camera_info_url` 加载，本工程标定值见 `config/camera_astra.yaml`（被 FAST-LIVO2 使用）。
