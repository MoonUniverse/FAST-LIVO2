# Odin1 ROS2 / FAST-LIVO2 调试与频率验证文档

## 1. 目标与结论

这次工作的目标有四个：

1. 修复 `odin_ros_driver` 在 ROS2 下 `/odin1/image` 与 `/odin1/cloud_raw` 频率不稳定的问题。
2. 打通 `/odin1/image/undistorted` 无畸变图像输出。
3. 验证 `/odin1/imu`、`/odin1/image/undistorted`、`/odin1/cloud_raw` 在最终 ROS2 层是否同步。
4. 对接 `FAST-LIVO2`，解决接入后的 `imu loop back` / `IMU and LiDAR not synced!` 问题。

最终结果：

- Odin ROS2 基础 topic 已稳定。
- `/odin1/image/undistorted` 已稳定输出。
- 三路 ROS2 时间戳已验证在同一设备时间轴上对齐。
- FAST-LIVO2 已能正常跑通。
- 额外定位到 Odin SDK 上游 IMU 存在偶发回退，已在 `odin_ros_driver` 侧做过滤兜底，避免影响 FAST-LIVO2。

---

## 2. 调试环境

### 2.1 Odin 驱动环境

- 工作区：`/home/nuc11/odin_ws`
- 包：`odin_ros_driver`
- RMW：`rmw_cyclonedds_cpp`
- 发布范围：仅本机

推荐环境变量：

```bash
source /opt/ros/humble/setup.bash
source /home/nuc11/odin_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
```

### 2.2 FAST-LIVO2 环境

- 工作区：`/home/nuc11/fast_livo2`
- 源码目录：`/home/nuc11/fast_livo2/src/FAST-LIVO2`
- 包：`fast_livo`

推荐环境变量：

```bash
source /opt/ros/humble/setup.bash
source /home/nuc11/fast_livo2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
```

---

## 3. 最终使用的关键 topic

FAST-LIVO2 最终接入的 3 个 topic：

| Topic | 类型 | 说明 |
|---|---|---|
| `/odin1/imu` | `sensor_msgs/msg/Imu` | IMU |
| `/odin1/image/undistorted` | `sensor_msgs/msg/Image` | 无畸变 RGB 图像 |
| `/odin1/cloud_raw` | `sensor_msgs/msg/PointCloud2` | 原始点云 |

当前不建议 FAST-LIVO2 直接吃：

- `/odin1/image`
- `/odin1/image/compressed`

因为当前最终验证与对接都是基于无畸变图像路径完成的。

---

## 4. 最终关键配置

### 4.1 Odin `control_command.yaml`

文件：

`/home/nuc11/odin_ws/src/odin_ros_driver/config/control_command.yaml`

关键项：

```yaml
use_host_ros_time: 0
sendrgb: 1
sendrgbcompressed: 0
sendrgbundistort: 1
sendimu: 1
senddtof: 1
dtof_fps: 100

ros2_sensor_qos_depth: 10
ros2_sensor_qos_reliable: 1
ros2_state_qos_depth: 10

enable_imu_smooth: 0
imu_smooth_frequency: 400
```

说明：

- `use_host_ros_time: 0`：使用设备时间戳，不覆盖成 host 接收时间。
- `sendrgbcompressed: 0`：关闭压缩图，避免无用额外流。
- `sendrgbundistort: 1`：打开无畸变图。
- `dtof_fps: 100`：10 Hz 点云。
- `ros2_sensor_qos_depth: 10` + `RELIABLE`：保证大消息 topic 稳定。

### 4.2 FAST-LIVO2 `odin1.yaml`

源码配置：

`/home/nuc11/fast_livo2/src/FAST-LIVO2/config/odin1.yaml`

安装后实际运行配置：

`/home/nuc11/fast_livo2/install/fast_livo/share/fast_livo/config/odin1.yaml`

关键项：

```yaml
common.img_topic: "/odin1/image/undistorted"
common.lid_topic: "/odin1/cloud_raw"
common.imu_topic: "/odin1/imu"

common.sensor_sub_qos_depth: 10
common.sensor_sub_qos_reliable: true

common.ros_driver_bug_fix: false
```

说明：

- `common.ros_driver_bug_fix` 对当前 Odin ROS2 路径必须是 `false`。
- 改完源码配置后一定要重新 `colcon build --packages-select fast_livo`，否则 launch 仍会读取旧的安装配置。

### 4.3 FAST-LIVO2 图像 / PCD 保存行为

当前相关参数：

```yaml
pcd_save.pcd_save_en: false
pcd_save.type: 1
pcd_save.colmap_output_en: false
pcd_save.filter_size_pcd: 0.15
pcd_save.interval: -1

image_save.img_save_en: true
image_save.interval: 1
```

当前实际行为：

- `pcd_save.pcd_save_en: false`：**不会保存 PCD**
- `image_save.img_save_en: true`：**启动后自动保存图像**
- `image_save.interval: 1`：每个满足条件的 VIO 图像都保存

现在已改成按**每次启动时间戳**建独立目录，目录结构为：

```text
/home/nuc11/fast_livo2/src/FAST-LIVO2/Log/<seq_name>_<YYYYMMDD_HHMMSS>/
├── all_image/
│   ├── <timestamp>.png
│   └── image_poses.txt
├── all_pcd_body/
│   ├── <timestamp>.pcd
│   └── lidar_poses.txt
├── colmap/
│   ├── images/
│   └── sparse/0/
│       ├── cameras.txt
│       ├── images.txt
│       └── points3D.txt
├── depth/
└── reproj/
```

说明：

- 顶层目录名来自 `seq_name + 启动时间戳`
- 即使多次启动也不会覆盖上一次结果
- 当前 `pcd_save.pcd_save_en: false` 时，`all_pcd_body/` 会存在，但不会写入 PCD 文件

---

## 5. 最终频率结果

### 5.1 用户初始现象

最开始用户侧观察：

| Topic | 初始现象 |
|---|---|
| `/odin1/image` | 约 `5 Hz` |
| `/odin1/cloud_raw` | 约 `8 Hz` |

这说明问题出在 ROS2 发布链路，而不是最终需求本身。

### 5.2 设备/源侧排查结果

排查过程中确认：

- 设备侧 RGB ODR 稳定在约 `10 Hz`
- 设备侧 DTOF ODR 稳定在约 `10 Hz`

因此最初的不稳定不是传感器采集问题，而是 host 发布/ROS2 传输路径问题。

### 5.3 修复后最终 ROS2 频率

使用匹配 QoS 的显式订阅脚本做测量，得到：

| Topic | 最终结果 |
|---|---|
| `/odin1/image` | 约 `10.3 Hz` |
| `/odin1/cloud_raw` | 约 `10.2 Hz` |
| `/odin1/image/undistorted` | 约 `10.33 Hz` |
| `/odin1/image/compressed` | `0` 条消息（关闭后不再发送） |

### 5.4 IMU 频率

在修复 IMU 回退过滤后，对 `/odin1/imu` 抓取 10 秒：

- 共收到 `3996` 条
- 折算约 `399.6 Hz`
- `0` 次时间回退

所以当前最终状态下 IMU 频率可视为约 `400 Hz`。

---

## 6. 三路同步验证结果

最终 ROS2 层直接验证了以下 3 路：

- `/odin1/imu`
- `/odin1/image/undistorted`
- `/odin1/cloud_raw`

结论：

| 配对 | 结果 |
|---|---|
| IMU ↔ image | `100%` 在 `2 ms` 内 |
| IMU ↔ cloud | `99.5%` 在 `2 ms` 内 |
| image ↔ cloud | `96.6%` 在 `2 ms` 内 |

解释：

- 三路 header stamp 在同一设备时间轴上。
- IMU 与 image / cloud 对齐都很好。
- image-cloud 少量离群点更像 cloud 偶发缺样，不像系统性时间轴错误。

---

## 7. 主要问题、根因与修复

## 7.1 基础 image / cloud topic 频率不稳定

### 现象

- `/odin1/image` 约 `5 Hz`
- `/odin1/cloud_raw` 约 `8 Hz`

### 根因

`host_sdk_sample` 发布高带宽 topic 时，QoS 太激进：

- `KeepLast(1)`
- `BEST_EFFORT`

在 localhost 场景下，一旦 RViz 或下游消费者略微跟不上，就会出现明显丢样和抖动。

### 修复

把 ROS2 QoS 做成可配置，并将当前配置调成：

- 传感器 topic：`KEEP_LAST(10)` + `RELIABLE`
- 状态 topic：`KEEP_LAST(10)`

同时让 RViz 不再默认启动，减轻本机负载。

---

## 7.2 `config_file` 参数无效

### 现象

运行时明明传了配置文件，但驱动实际仍然读包内默认配置。

### 根因

`host_sdk_sample` 的 ROS2 `config_file` 参数没有真正接到实际配置加载路径。

### 修复

修正参数处理逻辑，确保运行时 override 生效。

---

## 7.3 `sendrgbcompressed: 0` 仍在发压缩图

### 现象

关闭压缩图后，`/odin1/image/compressed` 仍然在跑。

### 根因

压缩图发布路径没有严格受配置开关控制。

### 修复

修正发布条件，让它严格 obey config。

---

## 7.4 打开 undistort 后启动崩溃

### 现象

打开 `sendrgbundistort: 1` 后，驱动启动阶段会崩。

### 根因

启动顺序有竞态：

1. stream 先启动
2. undistort map 后构建
3. RGB 处理线程可能先一步用到未准备好的 undistort 状态

### 修复

把顺序改成：

1. 先读相机参数
2. 先构建 undistort map
3. 再启动 stream

并且给 undistort-ready 状态加线程安全检查。

---

## 7.5 FAST-LIVO2 的 `IMU and LiDAR not synced!`

### 现象

FAST-LIVO2 一开始大量报：

- `IMU and LiDAR not synced! delta time: -0.50x ...`

### 根因

这条 warning 的旧逻辑会把：

- “当前 IMU 比最近处理到的 lidar 更新”

也当成同步错误。

但在 FAST-LIVO2 的单线程 `spin_some()` 模式下，这种现象可能只是调度滞后，不是真正不同步。

### 修复

调整 warning 判定，只在：

- IMU 明显比 lidar 更旧

时才报。

---

## 7.6 FAST-LIVO2 的 `imu loop back`

### 现象

用户后续测试仍会出现：

- `imu loop back, offset: ...`

### 根因定位过程

后面实测抓到：

- `/odin1/imu` 12 秒内有 `58` 次 header.stamp 回退
- 最大回退约 `0.3079s`
- `/odin1/cloud_raw` 同时 `0` 次回退

说明这不是 FAST-LIVO2 自己造的，而是 Odin 上游 IMU 已经乱序。

继续向上追：

1. `odin_ros_driver` 只是把 `imu_convert_data_t.stamp` 转成 ROS2 header.stamp。
2. 自己这一层是本地 FIFO 队列，不会主动把单调时间改成回退。
3. 回查预编译 SDK `liblydHostApi_amd.a` 后发现：
   - `lidar_system_init()` 会同时起 `publisher_thread_routine()` 和 `imu_publisher_thread_routine()`
   - IMU 还会经过 SDK 内部 `enqueue_callback_data(...)` 队列路径
4. 因此根因落在 SDK 内部 IMU 线程/队列路径，而不是 ROS2 时间戳对齐函数。

### 最终结论

**Odin SDK 上游 IMU 会偶发把一批旧 IMU 晚送出来，导致 `/odin1/imu` 时间回退。**

由于 SDK 是预编译静态库，当前无法继续精确到源码行级，但定位已经足够明确：

- 问题不在 FAST-LIVO2
- 问题不在 ROS2 QoS
- 问题不在 `make_aligned_stamp(...)`
- 问题在 `lydHostApi` 内部 IMU 发布/排队路径

### 当前修复方式

在 `odin_ros_driver` 发布 `/odin1/imu` 前加了兜底过滤：

- 如果新 IMU 的时间戳小于上一条已发布 IMU
- 直接丢弃，不再发布到 ROS2

修复后验证：

- `/odin1/imu` 10 秒 `0` 次回退
- FAST-LIVO2 联跑不再出现 `imu loop back`

---

## 8. 实际修改过的关键文件

### 8.1 Odin

- `/home/nuc11/odin_ws/src/odin_ros_driver/include/host_sdk_sample.h`
- `/home/nuc11/odin_ws/src/odin_ros_driver/src/host_sdk_sample.cpp`
- `/home/nuc11/odin_ws/src/odin_ros_driver/config/control_command.yaml`
- `/home/nuc11/odin_ws/src/odin_ros_driver/launch_ROS2/odin1_ros2.launch.py`
- `/home/nuc11/odin_ws/src/odin_ros_driver/README.md`

### 8.2 FAST-LIVO2

- `/home/nuc11/fast_livo2/src/FAST-LIVO2/include/LIVMapper.h`
- `/home/nuc11/fast_livo2/src/FAST-LIVO2/src/LIVMapper.cpp`
- `/home/nuc11/fast_livo2/src/FAST-LIVO2/config/odin1.yaml`
- `/home/nuc11/fast_livo2/src/FAST-LIVO2/README.md`

---

## 9. 推荐调试流程

## 9.1 先确认只有一个 Odin 发布者

先看 topic 和 publisher：

```bash
ros2 topic info /odin1/imu -v
ros2 topic info /odin1/cloud_raw -v
```

如果 publisher 数量不是 1，先不要继续测频率。

也可以直接查进程：

```bash
ps -eo pid,lstart,cmd | grep '[h]ost_sdk_sample'
```

---

## 9.2 单独启动 Odin 驱动

```bash
cd /home/nuc11/odin_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
ros2 run odin_ros_driver host_sdk_sample
```

或：

```bash
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

如果只是做驱动验证，不建议默认带 RViz。

---

## 9.3 不要只看 `ros2 topic hz`

`ros2 topic hz` 无法自定义 QoS，面对 `RELIABLE` 的大消息 topic 时很容易误判。

建议：

- image / cloud / imu 都用显式 QoS 脚本订阅
- QoS 要和发布端一致

关键原因：

- 发布端现在是 `RELIABLE`
- topic 较大
- 默认 CLI 订阅行为可能与发布端不完全匹配

---

## 9.4 频率验证建议

优先验证：

1. `/odin1/image`
2. `/odin1/cloud_raw`
3. `/odin1/image/undistorted`
4. `/odin1/imu`

观察点：

- 平均频率
- 是否有明显丢样
- IMU 是否有 header.stamp 回退

---

## 9.5 FAST-LIVO2 联跑验证

```bash
cd /home/nuc11/fast_livo2
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
ros2 launch fast_livo mapping_odin1.launch.py rviz:=false
```

重点观察日志里是否出现：

- `imu loop back`
- `IMU and LiDAR not synced!`
- `Get image`
- `Raw feature num`
- `Update Voxel Map`

如果后 3 类持续正常，而前 2 类不再出现，说明系统已经进入可用状态。

---

## 10. 如果后面还要继续调

### 10.1 如果想继续查 Odin SDK IMU 问题

当前最可能的深层原因是：

- `lydHostApi` 内部 IMU 专线线程
- 或 SDK 内部 IMU 回调队列
- 在某些 attach / resume / backlog 条件下释放了滞后的旧 IMU 批次

但因为它是预编译静态库，后续如果要彻底根治，只能：

1. 向 SDK 提供方拿源码或说明
2. 或让 SDK 提供方修正 IMU 输出顺序

### 10.2 如果只追求工程可用

当前更推荐：

1. 保持 `odin_ros_driver` 的回退 IMU 过滤
2. 保持当前 QoS 配置
3. 保持 FAST-LIVO2 的 `common.ros_driver_bug_fix: false`
4. 基于当前 topic 继续下游算法联调

---

## 11. 当前最终状态摘要

| 项目 | 当前状态 |
|---|---|
| `/odin1/image` | 稳定，约 `10.3 Hz` |
| `/odin1/cloud_raw` | 稳定，约 `10.2 Hz` |
| `/odin1/image/undistorted` | 稳定，约 `10.33 Hz` |
| `/odin1/imu` | 约 `400 Hz`，已过滤回退 |
| 三路时间对齐 | 已验证通过 |
| FAST-LIVO2 topic 对接 | 已完成 |
| `IMU and LiDAR not synced!` | 已处理 |
| `imu loop back` | 已处理 |

最终可用于 FAST-LIVO2 的 Odin topic 组合：

- `/odin1/imu`
- `/odin1/image/undistorted`
- `/odin1/cloud_raw`