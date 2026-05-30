# N10P ROS2 SLAM 项目 — 使用教程

> 版本: v2.0 | 更新: 2026-05-28
>
> 保姆级教程，每一步直接复制命令运行即可。
> **有新节点/新包时，实时更新此文件。**

---

## 目录

- [0. 硬件准备](#0-硬件准备)
- [1. 环境激活](#1-环境激活)
- [2. 编译](#2-编译)
- [3. 仅运行激光雷达](#3-仅运行激光雷达查看点云)
- [4. 运行雷达 + 飞控桥接](#4-运行雷达--飞控桥接传感器全开)
- [5. 运行 SLAM 建图](#5-运行-slam-建图)
- [6. 保存地图](#6-保存地图)
- [7. 查看地图](#7-查看地图)
- [8. 验证命令速查](#8-验证命令速查)
- [9. Nav2 导航（真实雷达）](#9-nav2-导航真实雷达)
- [10. Gazebo 仿真导航（无硬件）](#10-gazebo-仿真导航无硬件)
- [11. 桌面测试模式（真实N10P + 键盘导航）](#11-桌面测试模式真实n10p--键盘导航)
- [常见问题](#常见问题)
- [配置文件速查](#配置文件速查)

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

# 全量编译（首次或大改动后）
colcon build

# 增量编译（仅指定包）
colcon build --packages-select lslidar_msgs lslidar_driver n10p_bringup n10p_slam n10p_nav n10p_gazebo
```

编译成功后激活：

```bash
source install/setup.bash
```

> 仿真包 `n10p_gazebo` 每次编译后需额外操作，见 [第 10 章](#10-gazebo-仿真导航无硬件)。

---

## 3. 仅运行激光雷达（查看点云）

启动驱动 + RViz2：

```bash
ros2 launch lslidar_driver lslidar_launch.py
```

验证雷达数据：

```bash
ros2 topic hz /scan              # 应约 10Hz
ros2 topic echo /scan --once     # 查看一帧数据
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
- `static_transform_publisher` — 发布 `base_link → laser_frame` 静态 TF

验证各话题：

```bash
ros2 topic hz /scan     # 应约 10Hz
ros2 topic hz /odom     # 应约 20Hz
ros2 topic echo /imu --once
ros2 run tf2_ros tf2_echo odom base_link   # 飞控解锁后才有非零值
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

**终端1** — 传感器层（飞控 + 雷达）：

```bash
ros2 launch n10p_bringup n10p_bringup_launch.py
```

**终端2** — SLAM + RViz2（只启动建图，不重复启动传感器）：

```bash
ros2 launch n10p_slam slam_only_launch.py
```

> 飞控需要上电/解锁后才会输出里程计数据。静止桌面时 `/odom` 可能无数据。
>
> 注意：不能用 `slam_launch.py`（那个是独立手持模式，自带驱动和 dummy_odom，会跟 bringup 冲突）。

---

## 6. 保存地图

建图完成后，用 slam-toolbox 自带服务保存（`map_saver_cli` 有生命周期兼容问题）：

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/ubuntu22/ROS2/n10p_leishen/maps/n10p_map'}}"
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
ros2 run tf2_ros tf2_echo odom base_link         # odom→base_link（动态）
ros2 run tf2_ros tf2_echo base_link laser_frame  # base_link→laser（静态）
ros2 run tf2_tools view_frames                   # 生成 TF 树 PDF

# —— 节点检查 ——
ros2 node list                           # 列出所有节点
ros2 node info /ano_bridge_node          # 查看飞控节点详情
ros2 node info /slam_toolbox             # 查看 SLAM 节点详情

# —— Nav2 检查 ——
ros2 lifecycle get /planner_server       # 应为 active [3]
ros2 action list                         # 列出 action server
ros2 topic echo /plan --once             # 查看当前路径

# —— 串口检查 ——
ls /dev/serial/by-id/                    # 确认设备已识别
```

---

## 9. Nav2 导航（真实雷达）

需要有已保存的地图。一键启动导航系统：

```bash
ros2 launch n10p_nav nav_launch.py map:=/home/ubuntu22/ROS2/n10p_leishen/maps/n10p_map.yaml
```

启动后 RViz2 窗口显示静态地图。操作步骤：

1. 点击工具栏 **"2D Pose Estimate"**，在地图上标记雷达当前大概位置（AMCL 粒子初始化）
2. 等待粒子收束（绿色箭头群聚拢变稳定）
3. 点击 **"2D Goal Pose"**，在地图上设置导航目标（箭头方向 = 期望朝向）
4. 全局路径（绿色线）自动规划，机器人开始沿路径移动

验证导航工作：

```bash
ros2 lifecycle get /planner_server       # 应为 active [3]
ros2 lifecycle get /controller_server    # 应为 active [3]
ros2 action list                         # 应有 /navigate_to_pose, /follow_path
ros2 topic echo /plan --once             # 查看规划路径
ros2 topic echo /cmd_vel --once          # 查看速度指令
```

> 目前 `/cmd_vel` 只是输出指令，无人机实际执行需要飞控对接（Phase 7: MAVROS 集成）。

---

## 10. Gazebo 仿真导航（无硬件）

无需真实硬件，在虚拟环境中完整测试 Nav2 导航。

### 10.1 编译与准备

```bash
cd ~/ROS2/n10p_leishen/n10p_ws
colcon build --packages-select n10p_gazebo

# 手动修复 entry_points 安装路径（已知问题）
mkdir -p install/n10p_gazebo/lib/n10p_gazebo
cp install/n10p_gazebo/bin/scan_relay install/n10p_gazebo/lib/n10p_gazebo/scan_relay

source install/setup.bash
```

### 10.2 启动仿真

```bash
# 方式一（推荐）：使用启动脚本自动清理 SHM
bash ~/ROS2/n10p_leishen/scripts/start_simulation.sh

# 方式二（手动）：
rm -f /dev/shm/fastrtps_* 2>/dev/null  # 清理 DDS 共享内存
pkill gzserver 2>/dev/null; pkill gzclient 2>/dev/null
ros2 daemon stop; ros2 daemon start
source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash
ros2 launch n10p_gazebo sim_launch.py
```

> 每次启动前必须清理 `/dev/shm/fastrtps_*` 文件，否则 Fast-DDS 共享内存端口被锁，TF/scan 消息无法传递，RViz2 一片空白。

启动后依次出现：
- **Gazebo 窗口**：3D 世界，圆柱体蓝色无人机 + 4 个棕色箱子障碍物
- **RViz2 窗口**：显示 RobotModel + LaserScan（红点）+ 全局/局部 Costmap + Path

等待约 20 秒，终端输出 `Managed nodes are active`。

### 10.3 发送导航目标

RViz2 中点选 "2D Goal Pose" 工具在地图上设目标，或命令行：

```bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}" -1
```

### 10.4 验证

```bash
ros2 lifecycle get /planner_server    # 应为 active [3]
ros2 topic echo /plan --once          # 完整路径点列表
ros2 topic echo /odom --once          # 机器人位置已移动
```

### 10.5 仿真系统架构

| 节点 | 功能 |
|------|------|
| `gzserver` + `gzclient` | 3D 物理引擎 + GUI |
| `planar_move` | 全向运动插件, `/cmd_vel`→`/odom`, TF `odom→base_footprint` |
| `n10p_lidar_plugin` | 360° 2D LiDAR, 0.02-12m, 10Hz, 1058 点 |
| `scan_relay` | 话题转发: `/n10p_lidar_plugin/out` → `/scan` |
| `robot_state_publisher` | URDF TF: `base_footprint→base_link→laser_frame` |
| `map_server` | 空白静态地图 `/map`（10m×10m, 全空闲） |
| `planner_server` | NavfnPlanner 全局路径规划 |
| `controller_server` | RegulatedPurePursuit 局部路径跟踪 |
| `bt_navigator` | 行为树 `navigate_w_replanning_time.xml` |
| `lifecycle_manager` | 自动激活所有生命周期节点 |

**TF 树**：`map` (static identity) → `odom` (planar_move 发布) → `base_footprint` → `base_link` → `laser_frame`

---

## 11. 桌面测试模式（真实N10P + 键盘导航）

无需机器人底盘，用真实 N10P 雷达 + 键盘控制虚拟里程计 + AMCL 定位 + Nav2 路径规划，在 RViz 中完整演示导航全流程。

### 11.1 前置准备

1. N10P 雷达插入电脑，`ls /dev/serial/by-id/` 确认设备存在
2. 一张预建地图（先跑 [第 5 章](#5-运行-slam-建图) 走遍房间建图，[第 6 章](#6-保存地图) 保存）

### 11.2 运行方法

**终端1 — 键盘里程计**（先启动，负责模拟无人机移动）：

```bash
ros2env
cd ~/ROS2/n10p_leishen/n10p_ws
source install/setup.bash
ros2 run n10p_bringup keyboard_odom_node
```

终端1 会打印键盘映射表。

**终端2 — 导航系统**（雷达 + 地图 + AMCL + Nav2 + RViz2）：

```bash
ros2env
cd ~/ROS2/n10p_leishen/n10p_ws
source install/setup.bash
ros2 launch n10p_nav desktop_test_launch.py
```

如地图在其他路径，用 `map:=` 指定：
```bash
ros2 launch n10p_nav desktop_test_launch.py map:=/path/to/your_map.yaml
```

### 11.3 键盘控制

在**终端1**（键盘里程计窗口）按键：

| 键 | 功能 |
|----|------|
| W / X | 前进 / 后退 |
| A / D | 左移 / 右移 |
| Q / E | 左转 / 右转 |
| S | 停止 |
| R | 重置位置到原点 |
| Ctrl+C | 退出 |

### 11.4 操作步骤

1. RViz2 弹出后，点击顶部 "**2D Pose Estimate**"，在地图上标记初始位置（AMCL 粒子初始化）
2. 回到终端1，按 W 键 2-3 秒 → 虚拟机器人"前进"，AMCL 粒子开始收束（绿色箭头聚拢）
3. 多按几次不同方向键，让机器人"走"一小段 → 粒子越来越集中在真实位置
4. 点 "**2D Goal Pose**" 设导航目标 → 绿色路径线出现
5. `/cmd_vel` 速度指令生成（因为没有真实底盘，指令仅输出到日志，不会实际移动无人机）

### 11.5 验证

```bash
ros2 topic hz /scan              # 真实雷达 10Hz
ros2 topic echo /odom --once      # 键盘控制的里程计（x/y/yaw 每次按键会变化）
ros2 lifecycle get /planner_server  # 应为 active [3]
ros2 topic echo /plan --once      # 规划出的路径点
```

### 11.6 功能讲解

| 组件 | 数据来源 | 说明 |
|------|---------|------|
| `/scan` | **真实 N10P 雷达** | 360° 激光扫描，10Hz |
| `/odom` | 键盘虚拟 | WASD 积分生成，模拟全向运动 |
| `/map` | 预建地图文件 | slam-toolbox 保存的地图 |
| AMCL 定位 | 真实扫描 + 地图匹配 | 自动修正虚拟里程计的漂移 |
| 路径规划 | Nav2 计算 | 全局 NavfnPlanner + 局部 RegulatedPurePursuit |
| `/cmd_vel` | Nav2 输出 | 速度指令（无硬件执行，仅可视化） |

**与真实 Nav2 导航（第 9 章）的区别**：仅里程计来源不同——真机用飞控，桌面测试用键盘模拟。所有传感器数据处理、AMCL 定位、路径规划逻辑完全一致。

### 串口打不开

```bash
ls /dev/serial/by-id/           # 检查设备是否识别
groups $USER | grep dialout     # 确认用户有串口权限
```

如无 `dialout` 权限：`sudo usermod -a -G dialout $USER`，然后重新登录。

### `ros2env` 找不到

确认 `~/.bashrc` 中有定义。如果没有，联系管理员恢复配置。

### `Message Filter dropping message: queue is full`

QoS 不匹配。RViz2 中 `/scan` 话题的 Reliability 设为 **Best Effort**，Frame Rate 设为 **10 Hz**。

### 编译报 "package not found"

```bash
source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash
```

### 雷达无数据

- 确认雷达供电正常（电机转动）
- 确认串口 by-id 路径与驱动配置文件一致
- `ros2 topic echo /scan --once` 查看是否有输出

### ACC Z 轴显示 ~6.4 而不是 ~9.8

飞控静止时重力加速度应在 Z 轴。当前 ACC scale 因子需校准，修改 `ano_bridge.yaml` 中的 `acc_scale` 参数。

### Nav2：初始位姿在障碍物内 → "Starting point in lethal space"

AMCL 粒子初始位置落在 costmap 黑色区域内。在 RViz2 中用 "2D Pose Estimate" 重新标定位姿，避开墙壁。

### Nav2：bt_navigator "Action server spin not available"

行为树配置文件使用了 Spin 等不存在于当前 BT 插件列表的节点。确认 `nav2_params_n10p.yaml` 中 `default_nav_to_pose_bt_xml` 指向 `navigate_w_replanning_time.xml`。

### 仿真：planner_server 崩溃 (exit code -11)

全局 costmap 使用 `rolling_window` + `obstacle_layer` 会导致空指针崩溃。已修复为 `static_layer` + 空白静态地图。如仍出现，检查 `n10p_sim_nav.yaml` 中全局 costmap 配置。

### 仿真：scan_relay 找不到

ament_python 的 `console_scripts` 安装到 `bin/` 而非 `lib/`。每次 `colcon build` 后执行：

```bash
cp install/n10p_gazebo/bin/scan_relay install/n10p_gazebo/lib/n10p_gazebo/scan_relay
```

### 仿真：lifecycle_manager "Failed to change state for node: controller_server"

controller_server 初始化时间较长，bond_timeout 不足。已设为 10s。如仍出现，增大 `sim_launch.py` 中的 `bond_timeout` 值。

### 仿真：gzserver 首次启动 exit code 255

Gazebo 模型数据库首次下载超时导致。再运行一次即可（`pkill -f gzserver` 清理残留后重试）。

### 仿真：RViz2 一片空白、无雷达点云、无 Costmap

**必做**：每次启动仿真前执行：
```bash
rm -f /dev/shm/fastrtps_*
pkill gzserver; pkill gzclient
ros2 daemon stop; ros2 daemon start
```
原因为之前进程残留的 Fast-DDS 共享内存文件锁死端口，TF 变换和 scan 消息全部无法传递。

### 仿真：Gazebo 窗口启动慢

首次启动需从 OSRF 服务器下载 `ground_plane`、`sun` 等模型，耗时 30-60s。后续启动使用缓存，正常 5-10s。

---

## 配置文件速查

| 文件 | 用途 |
|------|------|
| `n10p_ws/src/Lslidar_ROS2_driver/lslidar_driver/params/lsx10.yaml` | N10P 驱动参数（串口、量程、型号） |
| `n10p_ws/src/n10p_bringup/params/ano_bridge.yaml` | 飞控桥接参数（串口、波特率、ACC/GYR scale） |
| `n10p_ws/src/n10p_slam/config/mapper_params_online_async.yaml` | SLAM 参数（分辨率、求解器、回环检测） |
| `n10p_ws/src/n10p_slam/config/n10p_slam.rviz` | SLAM RViz2 配置 |
| `n10p_ws/src/n10p_nav/config/nav2_params_n10p.yaml` | Nav2 导航参数（真实硬件） |
| `n10p_ws/src/n10p_nav/config/n10p_nav.rviz` | Nav2 导航 RViz2 配置 |
| `n10p_ws/src/n10p_gazebo/urdf/n10p_drone.urdf` | 仿真无人机 URDF 模型 |
| `n10p_ws/src/n10p_gazebo/worlds/simple_obstacles.world` | 仿真世界（4 个箱子障碍物） |
| `n10p_ws/src/n10p_gazebo/launch/sim_launch.py` | 仿真启动文件 |
| `n10p_ws/src/n10p_gazebo/config/n10p_sim_nav.yaml` | 仿真 Nav2 参数 |
| `n10p_ws/src/n10p_gazebo/config/n10p_sim.rviz` | 仿真 RViz2 配置 |
| `n10p_ws/src/Lslidar_ROS2_driver/lslidar_driver/rviz/lslidar.rviz` | 仅雷达 RViz2 配置 |
