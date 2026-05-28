# N10P ROS2 SLAM 项目 — 使用教程

> 版本: v1.0 | 创建: 2026-05-27
>
> 保姆级教程，每一步直接复制命令运行即可。
> **有新节点/新包时，实时更新此文件。**

---

## 目录

- [0. 硬件准备](#0-硬件准备)
- [1. 环境激活](#1-环境激活)
- [2. 编译](#2-编译)
- [3. 仅运行激光雷达（查看点云）](#3-仅运行激光雷达查看点云)
- [4. 运行雷达 + 飞控桥接（传感器全开）](#4-运行雷达--飞控桥接传感器全开)
- [5. 运行 SLAM 建图](#5-运行-slam-建图)
- [6. 保存地图](#6-保存地图)
- [7. 验证命令速查](#7-验证命令速查)
- [8. 常见问题](#8-常见问题)

---

## 0. 硬件准备

| 设备 | 串口 by-id 路径 | 波特率 | 备注 |
|------|----------------|--------|------|
| N10P 激光雷达 | `usb-1a86_USB_Single_Serial_58EB011256-if00` | 460800 | CH9102 芯片 |
| 匿名数传（凌霄飞控） | `usb-ANO_TC_ANO_RadioLink-if00` | 921600 | ANO RadioLink |

> 两个设备**同时**插入时，by-id 路径固定不变，与插入顺序无关。

插入后确认识别：

```bash
ls /dev/serial/by-id/
```

应看到两个蓝绿色的软链接。

---

## 1. 环境激活

```bash
# 激活 ROS2 Humble 环境（先清除 conda 变量）
ros2env

# 加载工作空间
cd ~/ROS2/n10p_leishen/n10p_ws
source install/setup.bash
```

> `ros2env` 是用户自定义命令。如果提示找不到，检查 `~/.bashrc` 中是否已配置。

---

## 2. 编译

```bash
cd ~/ROS2/n10p_leishen/n10p_ws
colcon build --packages-select lslidar_msgs lslidar_driver n10p_bringup n10p_slam
```

编译成功后激活：

```bash
source install/setup.bash
```

---

## 3. 仅运行激光雷达（查看点云）

启动驱动 + RViz2：

```bash
ros2 launch lslidar_driver lslidar_launch.py
```

验证雷达数据：

```bash
# 查看 /scan 话题频率
ros2 topic hz /scan

# 查看一帧数据
ros2 topic echo /scan --once
```

---

## 4. 运行雷达 + 飞控桥接（传感器全开）

一键启动：N10P 驱动 + 匿名飞控解析 + TF 发布：

```bash
ros2 launch n10p_bringup n10p_bringup_launch.py
```

这会同时启动：
- `ano_bridge_node` — 解析飞控串口，发布 `/odom`（20Hz）和 `/imu`
- `lslidar_driver_node` — N10P 驱动，发布 `/scan`（10Hz）
- `static_transform_publisher` — 发布 `base_link → laser_frame` 静态 TF（Z=-0.1m）

验证各话题：

```bash
ros2 topic hz /scan     # 应约 10Hz
ros2 topic hz /odom     # 应约 20Hz
ros2 topic echo /imu --once
ros2 run tf2_ros tf2_echo odom base_link   # 检查 TF（飞控解锁后才有非零值）
```

---

## 5. 运行 SLAM 建图

### 5.1 手持建图模式（不需要飞控）

使用占位里程计，只需雷达插上即可：

```bash
ros2 launch n10p_slam slam_launch.py
```

启动后 RViz2 窗口自动弹出。**手持雷达缓慢走动**，地图逐渐建立。

### 5.2 飞控在线模式（完整传感器）

雷达 + 飞控都插上时：

```bash
ros2 launch n10p_bringup n10p_bringup_launch.py
```

然后另开终端启动 SLAM（等传感器就绪后）：

```bash
ros2 launch n10p_slam slam_launch.py   # 或用 slam-toolbox 单独启动
```

> **注意**：飞控需要上电/解锁后才会输出里程计数据。静止桌面时 /odom 可能无数据。

---

## 6. 保存地图

建图完成后，用 slam-toolbox 自带服务保存（`map_saver_cli` 有生命周期兼容问题）：

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/ubuntu22/ROS2/n10p_leishen/maps/n10p_map'}}"
```

`result=0` 表示保存成功。会生成两个文件：
- `n10p_map.yaml` — 地图元信息（分辨率、原点、占用阈值）
- `n10p_map.pgm` — 栅格图像

---

## 7. 查看地图

用项目自带脚本查看 `.pgm` 地图：

```bash
# 交互显示
python3 ~/ROS2/n10p_leishen/scripts/map_viewer.py ~/ROS2/n10p_leishen/maps/n10p_map.yaml

# 保存为 PNG (不弹窗)
python3 ~/ROS2/n10p_leishen/scripts/map_viewer.py ~/ROS2/n10p_leishen/maps/n10p_map.yaml --save map.png --no-show
```

---

## 8. 验证命令速查

```bash
# —— 话题检查 ——
ros2 topic list                          # 列出所有话题
ros2 topic hz /scan                      # 雷达频率（应为 10Hz）
ros2 topic hz /odom                      # 里程计频率（应为 20Hz）
ros2 topic echo /scan --once             # 查看一帧激光数据
ros2 topic echo /odom --once             # 查看一帧里程计

# —— TF 检查 ——
ros2 run tf2_ros tf2_echo odom base_link # odom→base_link（动态）
ros2 run tf2_ros tf2_echo base_link laser_frame  # base_link→laser（静态）
ros2 run tf2_tools view_frames           # 生成 TF 树 PDF

# —— 节点检查 ——
ros2 node list                           # 列出所有节点
ros2 node info /ano_bridge_node          # 查看飞控节点详情
ros2 node info /slam_toolbox             # 查看 SLAM 节点详情

# —— 串口检查 ——
ls /dev/serial/by-id/                    # 确认设备已识别
```

---

## 8. 常见问题

### 8.1 串口打不开

```bash
# 检查串口是否存在
ls /dev/serial/by-id/

# 确认用户有权限
groups $USER | grep dialout
```

如果没有 dialout 权限：
```bash
sudo usermod -a -G dialout $USER
# 然后重新登录
```

### 8.2 `ros2env` 找不到

确认 `~/.bashrc` 中有定义。如果没有，联系管理员恢复配置。

### 8.3 `Message Filter dropping message: queue is full`

QoS 不匹配。确认 RViz2 中 `/scan` 话题的 Reliability 设为 **Best Effort**，Frame Rate 设为 **10 Hz**。

### 8.4 编译报 "package not found"

确认已 source 工作空间：
```bash
source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash
```

### 8.5 雷达无数据

- 确认雷达供电正常（电机转动）
- 确认串口 by-id 路径与配置文件一致
- `ros2 topic echo /scan --once` 查看是否有输出

### 8.6 ACC Z 轴显示 ~6.4 而不是 ~9.8

飞控静止时重力加速度应在 Z 轴。当前 ACC scale 因子需校准，修改 `ano_bridge.yaml` 中的 `acc_scale` 参数。

---

## 9. 运行 Nav2 导航 (Phase 4)

需要有已保存的地图。一键启动导航系统：

```bash
ros2 launch n10p_nav nav_launch.py map:=/home/ubuntu22/ROS2/n10p_leishen/maps/n10p_map.yaml
```

启动后 RViz2 窗口显示静态地图。操作步骤：

1. 点击工具栏 **"2D Pose Estimate"**，在地图上标记雷达当前大概位置（AMCL 初始化）
2. 等待粒子收束（绿色箭头稳定）
3. 点击 **"2D Goal Pose"**，在地图上设置导航目标（箭头方向 = 期望朝向）
4. 全局路径（绿色线）自动规划，`/cmd_vel` 话题输出速度指令

> 目前 `/cmd_vel` 只是输出指令，无人机实际执行需要飞控对接（Phase 7: MAVROS 集成）

---

## 配置文件速查

| 文件 | 用途 |
|------|------|
| `n10p_ws/src/Lslidar_ROS2_driver/lslidar_driver/params/lsx10.yaml` | N10P 驱动参数（串口、量程、型号） |
| `n10p_ws/src/n10p_bringup/params/ano_bridge.yaml` | 飞控桥接参数（串口、波特率、ACC/GYR scale） |
| `n10p_ws/src/n10p_slam/config/mapper_params_online_async.yaml` | SLAM 参数（分辨率、求解器、回环检测） |
| `n10p_ws/src/n10p_slam/config/n10p_slam.rviz` | SLAM RViz2 配置 |
| `n10p_ws/src/n10p_nav/config/nav2_params_n10p.yaml` | Nav2 导航参数 |
| `n10p_ws/src/n10p_nav/config/n10p_nav.rviz` | Nav2 导航 RViz2 配置 |
| `n10p_ws/src/Lslidar_ROS2_driver/lslidar_driver/rviz/lslidar.rviz` | 仅雷达 RViz2 配置 |
