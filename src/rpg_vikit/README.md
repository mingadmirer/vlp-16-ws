# rpg_vikit

Vikit: Vision-Kit for Robotics。计算机视觉与机器人通用工具库，适配 ROS2。

原仓库：[uzh-rpg/rpg_vikit](https://github.com/uzh-rpg/rpg_vikit)

## 子模块

| 模块            | 功能                                         |
|-----------------|----------------------------------------------|
| `vikit_common`  | 纯 C++ 通用工具（数学、文件 IO、计时器等）      |
| `vikit_ros`     | ROS2 集成（参数获取、话题辅助等）              |
| `vikit_py`      | Python 工具包装                               |

## 关键特性（ROS2）

- `params_helper.hpp`：多层级参数获取策略
  1. 直接节点参数访问（高速）
  2. `SyncParametersClient`：跨节点参数获取（通过 service 调用 `parameter_blackboard`）
  3. CLI 回退：`ros2 param get`
- 支持 Sophus（李群库），版本要求 1.22.10

## 依赖安装

```bash
# Sophus
git clone https://github.com/strasdat/Sophus.git -b 1.22.10
cd Sophus && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install

# vikit_common（全局安装）
cd vikit_common && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

## 在工程中的角色

本包被 [FAST-LIVO2](../FAST-LIVO2/README.md) 依赖。

FAST-LIVO2 通过 `vikit_ros` 的 `params_helper.hpp` 从 `parameter_blackboard` 节点获取相机内参（camera_astra.yaml），避免硬编码相机参数。
