# Jetson Orin 室内精准降落

本项目用于 Jetson Orin、PX4 v1.12、MAVROS、D455 向下相机和 Livox
MID360。提供两套互斥的精准降落方案：

- 伴随控制方案：Jetson 根据 AprilTag 偏差发布 BODY_NED 速度控制量。
- PX4 原生方案：Jetson 只发布绝对 landing target，由 PX4 执行精准降落。

两个完整启动脚本都包含飞控连接、D455、MID360/FAST-LIO、定位传输和
AprilTag 检测，但不会启动 Falcon/Ego 探索规划、`px4_replan_sender`、
`auto_control` 或 `pubcmd`。脚本不会自动解锁、切换模式或调用 LAND。

> 两套方案不能同时运行。解锁和飞行模式切换始终由飞行员手动完成。

## 定位数据链路

```text
MID360 + FAST-LIO
  -> /drone_Odometry
  -> px4_pos_estimator
  -> /mavros/vision_pose/pose
  -> PX4 EKF2
  -> /mavros/local_position/pose
  -> 精准降落节点
```

默认硬件与路径：

| 项目 | 默认值 |
|---|---|
| Jetson 工作空间 | `/home/wxh/nb_code/autofly_ws` |
| 飞控串口 | `/dev/ttyTHS0:3000000` |
| QGC 转发 | `udp://@192.168.20.57` |
| D455 序列号 | `046322251249` |
| D455 彩色图像 | 640×480，30 FPS |
| ROS | Noetic |
| OpenCV | 4.5.5，`/usr/local/opencv-455-cuda` |

## 编译

```bash
cd /home/wxh/nb_code/autofly_ws
./build_precision_landing_opencv455.sh
```

该脚本强制使用 OpenCV 4.5.5，并在检测到 OpenCV 4.2 动态链接时终止构建。

## 方案一：伴随控制精准降落

### 完整运行

```bash
cd /home/wxh/nb_code/autofly_ws
./run_companion_precision_landing.sh
```

无参数运行会设置 `enable_flight=true`。在飞行员手动切换到 OFFBOARD 前，
控制器的安全门不会允许实际降落控制。

### 地面测试

```bash
./run_companion_precision_landing.sh test
```

`test` 会完整启动 MAVROS、D455、FAST-LIO、外部视觉位置和 AprilTag 检测，
但不会启动伴随控制节点，因此 `/mavros/setpoint_raw/local` 没有本方案的发布者。

### 查看与停止

```bash
./run_companion_precision_landing.sh status
tmux attach -t landing_companion
./run_companion_precision_landing.sh stop
```

## 方案二：PX4 v1.12 原生精准降落

### 完整运行

```bash
cd /home/wxh/nb_code/autofly_ws
./run_px4_native_precision_landing.sh
```

无参数运行会设置 `enable_target_publish=true`，只向
`/mavros/landing_target/pose` 发布目标，不发布 BODY_NED 控制量。PX4 模式仍由
飞行员手动切换。

飞行前必须确认：

- `/mavros/landing_target/mav_frame` 为 `LOCAL_NED`；
- QGroundControl 或 PX4 MAVLink Console 中存在并正确设置 `PLD_*` 参数；
- 当前自定义 PX4 v1.12 固件确实包含 precision-land/landing-target 支持。

### 地面测试

```bash
./run_px4_native_precision_landing.sh test
```

`test` 会完整启动传感器、定位和检测链路，但不会创建本节点的
`/mavros/landing_target/pose` 发布器。

### 查看与停止

```bash
./run_px4_native_precision_landing.sh status
tmux attach -t landing_px4_native
./run_px4_native_precision_landing.sh stop
```

## 图像检测与状态

查看叠加后的检测图像：

```bash
rqt_image_view /precision_landing/debug/image_raw
```

伴随方案状态：

```bash
rostopic echo /precision_landing/status
rostopic info /mavros/setpoint_raw/local
```

PX4 原生方案状态：

```bash
rostopic echo /precision_landing/px4_native/status
rostopic echo /precision_landing/px4_native/target_local_enu
rostopic info /mavros/landing_target/pose
```

## 日志位置

### AprilTag 联合 PnP 检测日志

```text
/home/wxh/nb_code/autofly_ws/log/precision_landing_joint_pnp_error.csv
```

主要字段包括：时间、有效 Tag 数量、重投影 RMS、前/右/下方向偏差、水平误差、
引导速度、实际 BODY_NED 控制量和检测状态。伴随与 PX4 原生方案共用这份视觉
检测日志。

### PX4 原生降落逐帧日志

```text
/home/wxh/nb_code/autofly_ws/log/px4_native/flight_YYYYMMDD_HHMMSS.csv
```

每次检测到从地面进入空中状态后创建新文件。记录目标状态、目标年龄、相机坐标、
FRD 偏差、飞机 ENU 位置、缓存目标 ENU 位置、PX4 模式、落地状态及是否发布目标。

### PX4 原生降落精度汇总

```text
/home/wxh/nb_code/autofly_ws/log/px4_native/summary.csv
```

飞机状态转为 `ON_GROUND` 时追加一行。记录最终前/右/下偏差、水平误差、样本时间、
有效性和逐帧日志文件。最终精度使用落地前 1 秒内最新的有效 AprilTag 观测；若该
时间窗内没有有效观测，则写入 `valid=0`，不会复用过期数据。

## AprilTag 打印图案

Jetson 上的 A4 PNG 和 PDF：

```text
/home/wxh/nb_code/autofly_ws/src/precision_landing/tags/apriltag_landing_board_a4_large_outer.png
/home/wxh/nb_code/autofly_ws/src/precision_landing/tags/apriltag_landing_board_a4_large_outer.pdf
```

模型使用 Tag 黑色正方形边长。ID 0 的设计黑边为 14.4 mm。如果打印时发生统一
缩放，测量实际黑边后设置：

```text
board_scale = 实测_ID0_黑边_mm / 14.4
```

例如黑边实测 14.4 mm 时保持默认 `BOARD_SCALE=1.0`。

## 常见问题

| 现象 | 检查方法 |
|---|---|
| 提示已有 session | 运行对应脚本的 `status`，确认后运行 `stop` |
| 提示冲突 ROS 节点 | 使用 `rosnode list` 查明旧 MAVROS、D455、FAST-LIO 或降落节点的来源；脚本不会自动杀死未知进程 |
| 找不到 D455 | 检查 `rs-enumerate-devices -s` 和序列号 `046322251249` |
| 没有 `/drone_Odometry` | `tmux attach` 后检查 `fastlio` 窗口、MID360 网络和 `/livox/lidar`、`/livox/imu` |
| 没有 `/mavros/vision_pose/pose` | 检查 `vision` 窗口及 `/drone_Odometry` 是否持续发布 |
| 没有 `/mavros/local_position/pose` | 检查 PX4 EKF2 外部视觉参数和 MAVROS 连接 |
| 启动超时 | 根据报错进入对应 tmux 窗口；停止后可用 `STARTUP_TIMEOUT_S=60 ./脚本名` 增加单项等待时间 |
| PX4 原生状态一直 `WAITING` | 检查 AprilTag 是否进入 D455 彩色相机画面以及 `/precision_landing/debug/image_raw` |

更多板面尺寸、坐标系、launch 参数与日志字段说明见
[precision_landing 详细文档](src/precision_landing/README.md)。
