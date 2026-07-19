# 进入环境
```bash
cd n10p_leishen/n10p_ws
ros2env
```


# 单独运行雷达节点看点云
### 终端1  发布laser的坐标
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame
```
### 终端2
```bash
ros2 launch lslidar_driver lslidar_launch.py
```
#### 然后rviz2打开即可，注意fix frame选择laser_frame而不是map或odom


# SLAM有卡尔曼滤波
```bash
ros2 launch n10p_slam slam_ekf_launch.py
```

# 导航有卡尔曼滤波
```bash
# 默认加载配置n10p_ws/src/n10p_nav/config/nav2_params_n10p.yaml的初始无人机姿态
ros2 launch n10p_nav nav_ekf_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml

# 直接传入初始无人机姿态
ros2 launch n10p_nav nav_ekf_launch.py initial_x:=-1.23 initial_y:=0.87 initial_yaw:=-0.24   #注意是弧度，不是角度！！！90度就填1.57!!!
```

# 保存地图
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/ylz/n10p_leishen/maps/n10p_map'}}"
```

# 将地图pgm变成png图片
```bash
python3 /home/ylz/n10p_leishen/scripts/pgm2png.py 输入.pgm 输出.png
```


# 测试节点健康状态
```bash
cd ~/n10p_leishen

# SLAM 模式
python3 n10p_ws/scripts/n10p_health_check.py --mode slam

# NAV 模式  
python3 n10p_ws/scripts/n10p_health_check.py --mode nav

# 自动检测当前模式
python3 n10p_ws/scripts/n10p_health_check.py

# 持续监控 (每5秒刷新)
python3 n10p_ws/scripts/n10p_health_check.py --mode nav --watch

```

# 获取当前yaw朝向（单位：弧度rad)
```bash
python3 /home/ylz/n10p_leishen/n10p_ws/scripts/get_fc_yaw.py
```


# colcon build 通用指南
### 基本规则
必须在工作空间根目录执行：/home/ylz/n10p_leishen/n10p_ws

### 每次 build 前必须 source ROS2 环境：
```bash
source /opt/ros/humble/setup.bash
```

```bash
#全量编译（所有包）
cd ~/n10p_leishen/n10p_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --parallel-workers 2
#场景1：只编译某一个包
colcon build --packages-select n10p_fusion --symlink-install --parallel-workers 2
#场景2：编译多个指定的包
colcon build --packages-select n10p_bringup n10p_fusion n10p_slam --symlink-install --parallel-workers 2
#场景3：跳过某个包编译
colcon build --packages-ignore n10p_gazebo lslidar_driver --symlink-install --parallel-workers 2
# n10p_gazebo 仿真包 和 lslidar_driver C++包 编译慢，日常改 Python 时可以跳过
#场景4：只编译 Python 包（跳过慢的 C++ 包）
colcon build --packages-select n10p_bringup n10p_slam n10p_nav n10p_fusion --symlink-install --parallel-workers 2
#场景5：改了 C++ 代码需要重新编译
colcon build --packages-select lslidar_driver lslidar_msgs --parallel-workers 2
# C++ 包必须编译，不能用 --symlink-install 跳过
#场景6：全量但跳过仿真
colcon build --packages-ignore n10p_gazebo --symlink-install --parallel-workers 2
```

| 参数 | 作用 | 什么时候用 |
|------|------|------------|
| `--symlink-install` | Python 文件软链接到 install，改代码不用重编 | 每次都加（改 Python 时） |
| `--parallel-workers 2` | 限制编译并行数 | 树莓派必加（防止 OOM） |
| `--packages-select A B C` | 只编译指定的包 | 只改了一个包时，快很多 |
| `--packages-ignore A B` | 跳过指定包 | 跳过大包/仿真包 |
| `--cmake-clean-cache` | 清除 CMake 缓存强制重编 | C++ 包改 CMakeLists 后 |
| `--continue-on-error` | 某个包报错也不停 | 调试时，看哪些包能编译通过 |
# SLAM无卡尔曼滤波

## 一、有飞控时，运行两个命令：
### 终端1
```bash
ros2 launch n10p_bringup n10p_bringup_launch.py
```
### 终端2
```bash
ros2 launch n10p_slam slam_only_launch.py
```


## 二、没飞控时，运行一个命令：
```bash
ros2 launch n10p_slam slam_launch.py
```


# 导航无卡尔曼滤波

## 一、有飞控时，运行两个命令：
```bash
# 1. 启动传感器（保持运行）
ros2 launch n10p_bringup n10p_bringup_launch.py

# 2. 启动导航
ros2 launch n10p_nav nav_only_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml
```

## 二、没飞控时，运行一个命令：
```bash
ros2 launch n10p_nav nav_launch.py
```

# 僵尸节点强制清理
```bash
bash ~/n10p_leishen/scripts/clean_ros2.sh 
```




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
- [12. ESP32 WiFi 无线雷达 — 整合经验](#12-esp32-wifi-无线雷达--整合经验总结)
- [配置文件速查](#配置文件速查)

---

## 0. 硬件准备

| 设备 | 串口 by-id 路径 | 波特率 | 备注 |
|------|----------------|--------|------|
| N10P 激光雷达 | `usb-1a86_USB_Single_Serial_58EB011256-if00` | 460800 | CH9102 芯片 |
| 匿名数传（凌霄飞控） | `usb-ANO_TC_ANO_RadioLink-if00` | 921600 | ANO RadioLink |
| ESP32-S3 WiFi 桥接 | 192.168.0.184:8888 (TCP) | — | N10P 接 ESP32 IO18(RX)，无线收发 |

> 两个 USB 设备**同时**插入时，by-id 路径固定不变，与插入顺序无关。

插入后确认识别：

```bash
ls /dev/serial/by-id/
```

应看到两个蓝绿色的软链接。

> ESP32 上电自动连 WiFi 并启动 TCP Server，电脑连同一 WiFi 即可无线接收雷达数据。

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

> 每次新开终端先激活环境：`ros2env && source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash`

### 有线模式（USB 数据线直连）

启动驱动 + RViz2：

```bash
ros2 launch lslidar_driver lslidar_launch.py
```

### 无线模式（ESP32 WiFi）

ESP32 上电连接 N10P，电脑连同一 WiFi，启动桥接节点即可收到 `/scan`：

```bash
ros2 run n10p_bringup n10p_wifi_bridge_node
# 或指定 ESP32 IP
ros2 run n10p_bringup n10p_wifi_bridge_node --ros-args -p host:=192.168.0.184
```

> ESP32 上电自动连接 WiFi 并启动 TCP Server (默认 192.168.0.184:8888)。
> 不需要数据线！雷达 + ESP32 可以自由移动。

验证雷达数据：

```bash
ros2 topic hz /scan              # 应约 10Hz
ros2 topic echo /scan --once     # 查看一帧数据
```

---

## 4. 运行雷达 + 飞控桥接（传感器全开）

> 新终端先激活：`ros2env && source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash`

一键启动：N10P 驱动 + 匿名飞控解析 + TF 发布：

```bash
# 有线模式 (USB 数据线)
ros2 launch n10p_bringup n10p_bringup_launch.py
# 无线模式 (ESP32 WiFi)
ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wireless
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

> 新终端先激活：`ros2env && source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash`

### 5.1 手持建图模式（不需要飞控）

使用占位里程计，只需雷达插上即可：

```bash
# 有线模式 (USB 数据线)
ros2 launch n10p_slam slam_launch.py

# 无线模式 (ESP32 WiFi, 雷达可自由移动)
ros2 launch n10p_slam slam_launch.py scan_source:=wireless
```

启动后 RViz2 窗口自动弹出。**手持雷达缓慢走动**，地图逐渐建立。
> 无线模式下雷达不再被数据线束缚，可以走遍整个房间建图。

### 5.2 飞控在线模式（完整传感器）

雷达 + 飞控都插上时：

**终端1** — 传感器层（飞控 + 雷达）：

```bash
# 有线模式
ros2 launch n10p_bringup n10p_bringup_launch.py
# 无线模式
ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wireless
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

> 新终端先激活：`ros2env && source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash`

需要有已保存的地图。一键启动导航系统：

```bash
# 有线模式
ros2 launch n10p_nav nav_launch.py map:=/home/ubuntu22/ROS2/n10p_leishen/maps/n10p_map.yaml
# 无线模式
ros2 launch n10p_nav nav_launch.py map:=/home/ubuntu22/ROS2/n10p_leishen/maps/n10p_map.yaml scan_source:=wireless
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

> 新终端先激活：`ros2env && source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash`

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

> 新终端先激活：`ros2env && source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash`

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
# 有线模式
ros2 launch n10p_nav desktop_test_launch.py
# 无线模式 (ESP32 WiFi)
ros2 launch n10p_nav desktop_test_launch.py scan_source:=wireless
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

## 12. ESP32 WiFi 无线雷达 — 整合经验总结

> 供后续开发者参考：如何将一个 ESP32 WiFi 串口转 TCP 设备接入现有 ROS2 项目，
> 以及在此过程中踩过的坑和解决方案。

### 12.1 架构设计原则

**零侵入、双路径并存**。原有有线路径完全不动，新增无线路径独立运行：

```
N10P 原始数据 ───┬── 有线: lslidar_driver ──────→ /scan ──→ SLAM/Nav2/RViz2
                 │
                 └── 无线: ESP32 WiFi TCP → n10p_wifi_bridge_node → /scan (同上)
```

下游（SLAM/Nav2/RViz2）只订阅 `/scan`，对数据来源完全无感知。

### 12.2 实现要点

1. **wifi_bridge 必须是独立节点** — 发布与 lslidar_driver 完全相同的 /scan
   （frame_id=laser_frame, 10Hz, 360° 激光扫描），不修改任何现有驱动代码
2. **launch 文件用条件节点切换** — `scan_source:=wired` (默认) / `scan_source:=wireless`
   通过 `IfCondition`/`UnlessCondition` 二选一启动数据源，不会同时运行
3. **wifi_bridge 用 ROS2 参数声明 host/port** — 同时兼容 CLI 参数和 YAML 参数文件

### 12.3 完整踩坑记录

#### 坑 1：socat PTY 方案不可行

**尝试**：用 socat 将 ESP32 TCP 映射为 PTY 虚拟串口（`socat PTY,link=/tmp/n10p_esp32,raw TCP:192.168.0.184:8888`），
  让 lslidar_driver 无改动接入。
**失败原因**：lslidar_driver 启动时调用 `tcsetattr()` 设置终端属性（波特率、VMIN、VTIME 等），
  改变了 PTY 的行规约（line discipline），导致 poll() 不报告数据就绪 → 驱动永久卡住。
**解决**：放弃 PTY 方案，改为独立 Python ROS2 节点，TCP socket 直接读流式数据，
  在应用层做帧同步，完全不经过终端层。
**教训**：PTY 不是真正的串口，`tcsetattr()` 对 PTY 的影响与对真实串口完全不同。

#### 坑 2：N10P 帧的字节序不一致

**现象**：wifi_bridge 解析出的距离值为天文数字（几万米）。
**根因**：N10P 原始帧（108 字节）中距离和角度的字节序不同——
  **角度用大端（Big Endian）**，起始角度在字节 5-6 (`struct.unpack('>H')`)，单位 0.01°；
  **距离用小端（Little Endian）**，每个点 6 字节，距离在偏移 0-1 (`struct.unpack('<H')`)，单位 mm。
  **关键：不可全部用同一种字节序解析。**
**解决**：距离用 `<H`（小端），角度用 `>H`（大端）。
**教训**：直接对接原始帧时，必须逐字节对照 lslidar_driver 源码确认每段的字节序。

#### 坑 3：N10P 帧角度映射与 count_num 参数

**现象**：初版 wifi_bridge 的 ScanAccumulator 积累点数极少（20-70 点/圈），/scan 无有效数据。
**根因**：N10P 每帧（108 字节）16 个点，仅覆盖约 6° 扇形区域。
  一圈扫描由约 200 帧拼接而成（332+ fps → 10Hz 发布约 33 帧/次）。
  lslidar_driver 的点索引公式：`point_idx = round((360 - degree) * count_num / 360)`，
  使用 `count_num`（半圈点数）而非 `scan_num`（全圈点数）。
  wifi_bridge 需要与驱动保持完全一致的 count_num 和角度映射。
**解决**：
- count_num 固定为 529（N10P 典型值），scan_num = 2 × 529 = 1058
- 角度映射：`point_idx = int(round(a * scan_num / 360.0)) % scan_num`
- 10Hz 定时器强制发布，不等完整一圈（匹配驱动行为）
- 发布前检查有效点数 > 10 才发布

#### 坑 4：wifi_bridge 发布太早 → scan 先于 TF → costmap 消息队列爆满

**现象**：启动后 RViz 和 costmap 连续报 `Message Filter dropping message: frame 'laser_frame' ... discarding message because the queue is full`。
**根因**：wifi_bridge 连接 ESP32 后立即每秒收 330+ 帧、10Hz 发布 /scan，
  但此时 AMCL 未初始化（无 map→odom TF）、用户未设初始位姿。
  local_costmap 的 obstacle_layer 收到 scan 后尝试做 TF 变换
  （laser_frame → odom），但 TF 不完整 → 变换失败 → scan 堆积在消息队列中 →
  队列满（默认 10 条）→ 新 scan 被丢弃。
**解决**：wifi_bridge 加入 5 秒**启动延迟**——节点启动后先连接 TCP 收帧但不发布，
  等 5 秒后（SLAM/Nav2 已就绪）才开始发 /scan。
  同时延迟期间积累的旧数据全部丢弃，避免一次性涌入。

#### 坑 5：手持建图旋转时地图严重变形

**现象**：无线手持建图，直走时地图正常（长方形），一旋转地图就跟着转，
  同一个房间的长方形被画了好几层重叠在一起。
**根因**：手持模式使用 dummy_odom（全零里程计），slam-toolbox 必须**全靠扫描匹配**
  来估计运动。当前配置 `correlation_search_space_dimension: 0.5` 意味着
  帧间匹配的搜索窗口只有 ±0.5m 和 **±0.5rad（≈±28°）**。
  手持建图时如果旋转超过 28°/帧，扫描匹配器找不到对应帧 →
  误判为"房间旋转了" → 地图变形。
**解决**：调整 mapper_params_online_async.yaml 三个参数：
```yaml
correlation_search_space_dimension: 1.5  # 从 0.5 → 1.5 (±86°旋转搜索)
link_scan_maximum_distance: 3.0          # 从 1.5 → 3.0 (帧间平移匹配)
loop_search_maximum_distance: 8.0        # 从 5.0 → 8.0 (回环检测)
```
**补充**：**慢速转动**、**贴墙走**（墙提供更多特征点）、走完一圈**回到起点**
  （回环检测会修正累积误差）能显著提升建图质量。

#### 坑 6：launch 文件条件节点的 IfCondition 陷阱

**现象**：加 `scan_source` 参数后无论设什么值都只走有线模式。
**原因**：`IfCondition(scan_source)` 直接取 LaunchConfiguration 的布尔值，
  `'wired'` 和 `'wireless'` 作为字符串都不会自动转为 True/False。
**解决**：必须用 `PythonExpression` 做字符串比较：
```python
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression

scan_source = LaunchConfiguration('scan_source', default='wired')
is_wireless = PythonExpression(["'", scan_source, "' == 'wireless'"])

driver_node = Node(..., condition=UnlessCondition(is_wireless))   # 有线时启动
wifi_node   = Node(..., condition=IfCondition(is_wireless))       # 无线时启动
```

#### 坑 7：map 帧不存在导致 RViz 死锁

**现象**：桌面测试启动后 RViz 一片空白，终端刷 `Timed out waiting for transform from base_link to map: frame 'map' does not exist`。
**原因链**：
  1. AMCL 需要用户先点 "2D Pose Estimate" 才激活并发布 map→odom TF
  2. 激活前 `map` 帧不存在
  3. RViz 的 Fixed Frame 设为 `map` → 所有显示渲染失败
  4. 地图不显示 → 用户不知道在哪设初始位姿 → **死锁**
**解决**：在 launch 文件中加一个**静态 `map→odom` 全零 TF** 做 bootstrap：
```python
Node(package='tf2_ros', executable='static_transform_publisher',
     name='static_tf_map_odom',
     arguments=['0','0','0','0','0','0','map','odom']),
```
AMCL 初始化后会用自己的 TF 自动覆盖这个静态值。
**这个坑也适用于 nav_launch.py**，已在两处都加了。

#### 坑 8：两个里程计源同时运行 → TF 冲突

**现象**：costmap 警告 `Sensor origin at (22.11, 0.00) is out of map bounds (64.70, -1.95) to (68.67, 2.02)`，
  机器人位置飞到了 66 米外（地图只有 10 米宽）。
**原因**：一次修改中把 `dummy_odom` 加进了 desktop_test_launch.py，
  但用户同时在另一终端跑 `keyboard_odom_node`——**两个节点都发布 odom→base_link TF**，
  数值互相矛盾。
**解决**：**二选一**。桌面测试模式固定为 launch 文件不启动里程计，
  用户必须在单独终端先启动 `ros2 run n10p_bringup keyboard_odom_node`。

#### 坑 9：AMCL 粒子云在 RViz 里看不见

**现象**：AMCL 已激活、/particle_cloud 有数据，但 RViz 里看不到绿色粒子箭头。
**原因链**：
  1. AMCL 的 `/particle_cloud` 话题有双重类型（`nav2_msgs/ParticleCloud` 和 `geometry_msgs/PoseArray`），
    `ros2 topic echo` 因此拒绝回显
  2. RViz 中 PoseArray 显示的箭头默认大小是 0.01m — **肉眼完全看不见**
**解决**：
  - 验证 AMCL 是否在发：`ros2 node info /amcl | grep particle` 看 Publishers 列表
  - RViz 左侧面板 PoseArray → "Arrow Length" 调为 **0.3**，"Arrow Width" 调为 **0.1**

#### 坑 10：/particle_cloud 双重类型导致 ros2 topic 命令失败

**现象**：`ros2 topic echo /particle_cloud` 报 `contains more than one type`。
**原因**：AMCL 同时以 `nav2_msgs/ParticleCloud` 和 `geometry_msgs/PoseArray` 两种类型发布。
**解决**：用 `ros2 node info /amcl` 确认发布者，或用 `ros2 topic hz /particle_cloud`（同样不行）。
  最终判断法：只要 `ros2 node info /amcl` 的 Publishers 里有 `/particle_cloud`、且
  `/amcl_pose` 有数据 → AMCL 正常工作。RViz 看不到纯粹是箭头尺寸问题。

#### 坑 11：Fast-DDS 共享内存僵尸文件

**影响范围**：主要是 Gazebo 仿真。桌面测试（无线+真实雷达）一般不受影响。
**现象**：所有 ROS2 节点日志报 `RTPS_TRANSPORT_SHM Error: Failed init_port`。
  虽然 DDS 会回退到 UDP 模式因此消息不完全丢失，但大量 SHM 错误会拖慢
  DDS 发现和服务调用速度，导致 lifecycle_manager 调用 configure service 超时失败。
**解决**：`rm -f /dev/shm/fastrtps_*` 清理所有共享内存僵尸文件。

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
