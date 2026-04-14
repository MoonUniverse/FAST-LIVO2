# FAST-LIVO2 (ROS2 Humble + Odin1)

## FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry

> **本分支**是 FAST-LIVO2 从 ROS1 (catkin) 到 **ROS2 Humble (ament_cmake)** 的完整移植版本，并集成了 **Manifold Odin1** 一体化传感器（LiDAR + IMU + Camera）的支持。

### 📢 News

- 🔧 **2026-04**: Migrated to **ROS2 Humble**, added **Odin1** sensor support
- 🔓 **2025-01-23**: Code released!
- 🎉 **2024-10-01**: Accepted by **T-RO '24**!

### 📬 Contact

Original author: [zhengcr@connect.hku.hk](mailto:zhengcr@connect.hku.hk)

---

## 1. Introduction

FAST-LIVO2 is an efficient and accurate LiDAR-inertial-visual fusion localization and mapping system, demonstrating significant potential for real-time 3D reconstruction and onboard robotic localization in severely degraded environments.

**Original Developer**: [Chunran Zheng 郑纯然](https://github.com/xuankuzcr)

<div align="center">
    <img src="pics/Framework.png" width = 100% >
</div>

### 1.1 Related papers

- [FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2408.14035)
- [FAST-LIVO2 on Resource-Constrained Platforms](https://arxiv.org/pdf/2501.13876)
- [FAST-LIVO: Fast and Tightly-coupled Sparse-Direct LiDAR-Inertial-Visual Odometry](https://arxiv.org/pdf/2203.00893)

### 1.2 ROS2 Migration Summary

本次移植相比原版的主要变更：

| 模块 | ROS1 (原版) | ROS2 (本版) |
|------|-------------|-------------|
| 构建系统 | catkin / CMake | **ament_cmake** / colcon |
| 节点模型 | `ros::NodeHandle` | `rclcpp::Node` 继承 |
| 参数系统 | `nh.param<T>()` | `declare_parameter` / `get_parameter` |
| 消息类型 | `sensor_msgs::PointCloud2` | `sensor_msgs::msg::PointCloud2` |
| TF | `tf::TransformBroadcaster` | `tf2_ros::TransformBroadcaster` |
| Launch | XML `.launch` | Python `.launch.py` |
| Vikit 依赖 | 外部 catkin 包 | 内置于 `thirdparty/vikit/` |
| LiDAR 驱动 | Livox (CustomMsg) | **Odin1** (标准 PointCloud2) |
| 支持 LiDAR | Livox Avia, Velodyne, Ouster, Hesai, RoboSense | 上述 + **Odin1 (lidar_type=8)** |

---

## 2. Prerequisites

### 2.1 Ubuntu and ROS2

- **Ubuntu 22.04** + **ROS2 Humble**

```bash
# 验证 ROS2 安装
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2.2 系统依赖

以下依赖通过 ROS2 包管理安装：

```bash
sudo apt install -y \
  ros-humble-pcl-conversions \
  ros-humble-tf2-ros \
  ros-humble-tf2-eigen \
  ros-humble-tf2-geometry-msgs \
  ros-humble-cv-bridge \
  ros-humble-image-transport \
  ros-humble-rviz2 \
  libsophus-dev
```

> **注意**：如果系统没有 Sophus，手动编译：
> ```bash
> git clone https://github.com/strasdat/Sophus.git
> cd Sophus && git checkout a621ff
> mkdir build && cd build && cmake .. && make -j && sudo make install
> ```

### 2.3 Odin1 驱动 (如使用 Odin1 传感器)

```bash
# 参考 https://github.com/manifoldsdk/odin_ros_driver
cd ~/your_ws/src
git clone https://github.com/manifoldsdk/odin_ros_driver.git
cd .. && colcon build --packages-select odin_ros_driver
```

### 2.4 Vikit (已内置)

Vikit 已作为本地库集成在 `thirdparty/vikit/`，**无需单独安装**。

---

## 3. Build

```bash
# 创建工作空间（如果还没有）
mkdir -p ~/fast_livo2/FAST-LIVO2
# 将源码放入上述目录

# 编译
cd ~/fast_livo2
source /opt/ros/humble/setup.bash
colcon build --packages-select fast_livo

# 加载环境
source install/setup.bash
```

---

## 4. Configuration

### 4.1 配置文件结构

```
config/
├── odin1.yaml                  # Odin1 传感器配置（主配置）
├── avia.yaml                   # Livox Avia 配置
├── MARS_LVIG.yaml              # MARS-LVIG 数据集配置
├── HILTI22.yaml                # HILTI 2022 数据集配置
├── NTU_VIRAL.yaml              # NTU VIRAL 数据集配置
├── camera_pinhole.yaml         # Odin1 去畸变针孔相机内参
├── camera_MARS_LVIG.yaml       # MARS-LVIG 相机内参
├── camera_fisheye_HILTI22.yaml # HILTI22 鱼眼相机内参
└── camera_NTU_VIRAL.yaml       # NTU VIRAL 相机内参
```

### 4.2 Odin1 外参配置

`odin1.yaml` 中的关键参数：

```yaml
laserMapping:
  ros__parameters:
    # 话题 - 使用去畸变图像
    common.img_topic: "/odin1/image/undistorted"
    common.lid_topic: "/odin1/cloud_raw"
    common.imu_topic: "/odin1/imu"
    common.sensor_sub_qos_depth: 10
    common.sensor_sub_qos_reliable: true
    common.ros_driver_bug_fix: false

    # IMU→LiDAR 外参 (T^imu_lidar，所有 Odin1 固定值)
    extrin_calib.extrinsic_T: [-0.02663, 0.03447, 0.02174]
    extrin_calib.extrinsic_R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

    # Camera→LiDAR 外参 (T^camera_lidar，设备相关，来自驱动 calib.yaml 的 Tcl_0)
    extrin_calib.Rcl: [-0.00745, -0.99997, -0.00018, ...]  # 替换为你的设备值
    extrin_calib.Pcl: [0.03127, 0.01817, -0.00955]          # 替换为你的设备值

    # LiDAR 类型
    preprocess.lidar_type: 8  # ODIN1
```

这里有两个和当前 Odin1 驱动直接相关的点：

1. `common.sensor_sub_qos_depth: 10` 与 `common.sensor_sub_qos_reliable: true` 用于匹配 Odin1 当前 ROS2 发布侧的 `RELIABLE + KEEP_LAST(10)`。
2. `common.ros_driver_bug_fix` 需要保持为 `false`。当前 Odin1 的 IMU / undistorted image / cloud header stamp 已经在同一设备时间轴上对齐；如果这里继续开启兼容修正，FAST-LIVO2 会对 IMU 时间做整秒级补偿，进而触发 `imu loop back`。

### 4.3 Odin1 相机内参配置

`camera_pinhole.yaml` 使用去畸变后的针孔模型参数：

```yaml
laserMapping:
  ros__parameters:
    cam_model: "Pinhole"
    cam_width: 1600                 # Odin1 原始分辨率
    cam_height: 1296
    cam_fx: 737.3568377326869       # = A11 from calib.yaml
    cam_fy: 737.2915871767854       # = A22
    cam_cx: 794.371920804624        # = u0
    cam_cy: 666.2588672902901       # = v0
    cam_d0: 0.0                     # 去畸变后无畸变
    cam_d1: 0.0
    cam_d2: 0.0
    cam_d3: 0.0
```

### 4.4 获取你设备的标定参数

每台 Odin1 设备的 Camera-LiDAR 外参和相机内参略有不同：

```bash
# 1. 启动 Odin 驱动
ros2 launch odin_ros_driver odin1.launch.py

# 2. 驱动会在 config/ 目录生成 calib.yaml，包含：
#    - Tcl_0: Camera→LiDAR 4x4 变换矩阵 → 填入 Rcl 和 Pcl
#    - cam_0: 相机内参 → A11→cam_fx, A22→cam_fy, u0→cam_cx, v0→cam_cy
```

---

## 5. Run

### 5.1 Odin1 传感器

```bash
# 终端1：启动 Odin 驱动
ros2 launch odin_ros_driver odin1.launch.py

# 终端2：启动 FAST-LIVO2
source ~/fast_livo2/install/setup.bash
ros2 launch fast_livo mapping_odin1.launch.py
```

### 5.2 其他传感器 / 数据集回放

```bash
# Livox Avia
ros2 launch fast_livo mapping_avia.launch.py
ros2 bag play YOUR_BAG_PATH

# MARS-LVIG 数据集
ros2 launch fast_livo mapping_avia_marslvig.launch.py

# HILTI 2022 数据集
ros2 launch fast_livo mapping_hesaixt32_hilti22.launch.py

# NTU VIRAL 数据集
ros2 launch fast_livo mapping_ouster_ntu.launch.py
```

### 5.3 Launch 文件列表

| Launch 文件 | 传感器 | 配置文件 |
|-------------|--------|----------|
| `mapping_odin1.launch.py` | Odin1 | `odin1.yaml` + `camera_pinhole.yaml` |
| `mapping_avia.launch.py` | Livox Avia | `avia.yaml` + `camera_pinhole.yaml` |
| `mapping_avia_marslvig.launch.py` | Livox Avia | `MARS_LVIG.yaml` + `camera_MARS_LVIG.yaml` |
| `mapping_hesaixt32_hilti22.launch.py` | Hesai XT32 | `HILTI22.yaml` + `camera_fisheye_HILTI22.yaml` |
| `mapping_ouster_ntu.launch.py` | Ouster OS1 | `NTU_VIRAL.yaml` + `camera_NTU_VIRAL.yaml` |

---

## 6. Supported LiDAR Types

| lidar_type | 传感器 | 点云格式 |
|------------|--------|----------|
| 1 | Livox Avia | PointCloud2 |
| 2 | Velodyne VLP-16 | PointCloud2 |
| 3 | Ouster OS1-64 | PointCloud2 |
| 4 | Intel RealSense L515 | PointCloud2 |
| 5 | Hesai XT32 | PointCloud2 |
| 6 | Hesai Pandar128 | PointCloud2 |
| 7 | RoboSense | PointCloud2 |
| **8** | **Manifold Odin1** | **PointCloud2** |

---

## 7. ROS2 Topics

### 发布

| Topic | 类型 | 说明 |
|-------|------|------|
| `/cloud_registered` | `sensor_msgs/msg/PointCloud2` | 配准后的点云 |
| `/cloud_effected` | `sensor_msgs/msg/PointCloud2` | 有效特征点 |
| `/Laser_map` | `sensor_msgs/msg/PointCloud2` | 全局地图 |
| `/aft_mapped_to_init` | `nav_msgs/msg/Odometry` | 里程计 |
| `/path` | `nav_msgs/msg/Path` | 轨迹 |
| `/voxels` | `visualization_msgs/msg/MarkerArray` | 体素地图可视化 |
| `/img_out` | `sensor_msgs/msg/Image` | 输出图像 |

### 订阅 (Odin1)

| Topic | 类型 | 说明 |
|-------|------|------|
| `/odin1/cloud_raw` | `sensor_msgs/msg/PointCloud2` | LiDAR 点云 |
| `/odin1/imu` | `sensor_msgs/msg/Imu` | IMU 数据 |
| `/odin1/image/undistorted` | `sensor_msgs/msg/Image` | 去畸变相机图像 |

---

## 8. Project Structure

```
FAST-LIVO2/
├── CMakeLists.txt              # ament_cmake 构建脚本
├── package.xml                 # ROS2 包描述
├── config/                     # YAML 参数文件
├── launch/                     # Python launch 文件
├── rviz_cfg/                   # RViz2 配置
├── include/
│   ├── LIVMapper.h             # 主节点 (继承 rclcpp::Node)
│   ├── common_lib.h            # 公共数据结构
│   ├── preprocess.h            # 点云预处理 + Odin1 handler
│   ├── IMU_Processing.h        # IMU 处理
│   └── voxel_map.h             # 体素地图
├── src/
│   ├── main.cpp                # 入口
│   ├── LIVMapper.cpp           # 主逻辑
│   ├── preprocess.cpp          # 预处理实现
│   ├── IMU_Processing.cpp      # IMU 实现
│   └── voxel_map.cpp           # 体素地图实现
└── thirdparty/
    └── vikit/                  # vikit_common (内置，ROS2 适配)
```

---

## 9. Troubleshooting

### ROS2 参数类型错误

ROS2 对参数类型非常严格。如果遇到 `InvalidParameterTypeException`：

```
parameter 'xxx' has invalid type: Wrong parameter type,
parameter {xxx} is of type {double_array}, setting it to {integer_array} is not allowed.
```

**原因**：YAML 中的整数值（如 `[1, 0, 0]`）会被识别为 `integer_array`，而代码期望 `double_array`。

**解决**：确保所有浮点参数在 YAML 中使用小数点，例如 `[1.0, 0.0, 0.0]`。

### RViz2 插件错误

本版本已将 RViz 配置从 ROS1 格式转换为 ROS2 格式：
- `rviz/` 前缀 → `rviz_default_plugins/`
- `rviz/Group` → 已展平为独立 Display
- Panel 类名 → `rviz_common/Displays` 等

---

## 10. License

The source code of this package is released under the [**GPLv2**](http://www.gnu.org/licenses/) license. For commercial use, please contact the original author at <zhengcr@connect.hku.hk> and Prof. Fu Zhang at <fuzhang@hku.hk>.

---

## 11. Acknowledgements

- Original FAST-LIVO2 by [HKU-MARS Lab](https://github.com/hku-mars/FAST-LIVO2)
- Odin1 sensor by [Manifold Technology](https://github.com/manifoldsdk/odin_ros_driver)
- ROS2 migration adapted for Odin1 integration
