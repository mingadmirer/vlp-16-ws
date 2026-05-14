# Orbbec Astra 相机启动方式

## 基本启动

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch astra_camera astra.launch.xml
```

默认只开启彩色流。其他流按需传参：

```bash1
# 开启深度
ros2 launch astra_camera astra.launch.xml enable_depth:=true

# 开启深度 + IR
ros2 launch astra_camera astra.launch.xml enable_depth:=true enable_ir:=true

# 开启深度 + 点云
ros2 launch astra_camera astra.launch.xml enable_depth:=true enable_point_cloud:=true

# 全开
ros2 launch astra_camera astra.launch.xml enable_depth:=true enable_ir:=true enable_point_cloud:=true
```

## 话题列表

| 话题 | 类型 | 说明 |
|---|---|---|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 彩色图（640x480 MJPEG） |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 彩色相机内参 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图（需 enable_depth） |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | 深度相机内参 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 点云（需 enable_point_cloud） |
| `/camera/ir/image_raw` | `sensor_msgs/Image` | IR 图（需 enable_ir） |
| `/camera/ir/camera_info` | `sensor_msgs/CameraInfo` | IR 相机内参 |
| `/tf` | `tf2_msgs/TFMessage` | 坐标变换 |

## 可视化

```bash
ros2 run rviz2 rviz2
# 添加 Color、Depth、PointCloud2 等显示
```

## UVC 参数

彩色流通过 UVC 后端（libuvc）获取，可调参数：

```bash
ros2 launch astra_camera astra.launch.xml \
  color_width:=1280 color_height:=720 color_fps:=15 \
  uvc_camera_format:=mjpeg
```

## 相机内参

### 默认值（驱动输出，未标定）

| 参数 | 值 |
|---|---|
| 模型 | Pinhole |
| 分辨率 | 640×480 |
| fx | 454.606628 |
| fy | 454.606628 |
| cx | 323.811493 |
| cy | 241.962662 |
| k1 | 0.0 |
| k2 | 0.0 |
| p1 | 0.0 |
| p2 | 0.0 |
| k3 | 0.0 |

> fx == fy 且畸变全零，明显是未标定的通用默认值，不建议用于视觉 SLAM。

### 标定值（OpenCV 棋盘格，2026-05-14）

| 参数 | 值 |
|---|---|
| 模型 | Pinhole |
| 分辨率 | 640×480 |
| fx | 596.278151 |
| fy | 598.538272 |
| cx | 303.464712 |
| cy | 245.184849 |
| k1 (d0) | 0.222562 |
| k2 (d1) | -0.729829 |
| p1 (d2) | 0.008878 |
| p2 (d3) | -0.025815 |
| k3 (d4) | 0.761221 |

- 标定方法：OpenCV `calibrateCamera`，8×6 内角点棋盘格，方格 25mm
- 样本数：45 张（全部检测成功）
- RMS 重投影误差：2.38 px

```bash
# 标定命令
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 --no-service-check \
  --ros-args -r image:=/camera/color/image_raw
```

## 数据流对应硬件

| 硬件 | PID | 协议 |
|---|---|---|
| 深度传感器 | `2bc5:0614` | OpenNI2 |
| 彩色相机 | `2bc5:0511` | UVC |
