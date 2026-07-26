# CLAUDE.md — N10P ROS2 SLAM 项目最高指令

> 版本: v4.0-nofcyaw | 更新: 2026-07-26 | 此文件每次对话自动加载
>
> 项目根目录: `/home/ylz/n10p_leishen/`
>
> **当前阶段**: Phase 8 — 飞控 0xF5 下行联调中 | **⭐ 基线 v4.0: FC偏航解耦+速度纯FC平滑+AMCL增强**

---

## 1. 语言规则 (最高优先级)

**全程中文交流，不要英文。**
- 所有对用户的回复、解释、报错分析必须使用中文
- 代码注释默认中文（除非是公开 API 或开源库约定用英文）
- commit message 使用中文

---

## 2. 环境约束 (最高优先级)

### 2.1 当前机器

| 项目 | 实际值 |
|------|--------|
| 型号 | Raspberry Pi 4 Model B Rev 1.5 |
| 架构 | aarch64 (ARM64) |
| 系统 | Ubuntu 22.04.5 LTS Server |
| 内存 | ~7.6GB（8GB 版本，可用约 6.5GB） |
| CPU | 4 核 Cortex-A72 @1.8GHz |
| 存储 | microSD 59GB（无 SSD），可用 ~49GB |
| 用户 | ylz |
| 项目路径 | `/home/ylz/n10p_leishen/` |
| 工作空间 | `/home/ylz/n10p_leishen/n10p_ws/` |

### 2.2 ROS2 环境激活

本机无 Anaconda/conda，直接 source 即可：

```bash
source /opt/ros/humble/setup.bash
source /home/ylz/n10p_leishen/n10p_ws/install/setup.bash
```

两条命令合成一条：
```bash
source /opt/ros/humble/setup.bash && source /home/ylz/n10p_leishen/n10p_ws/install/setup.bash
```

- **任何 ROS2 操作前**，必须先 source 上述两个 setup.bash
- 需要完整 ROS2 + 工作空间环境时执行完整版

### 2.3 禁止擅自安装任何软件包

**绝对禁止**在未经用户明确确认的情况下执行：
- `apt install` / `apt-get install` / `apt remove` / `apt purge`
- `pip install` / `pip3 install` / `pip uninstall`
- `snap install` / `snap remove`
- `npm install -g`
- `sudo` 开头的任何命令

**流程**：如果需要安装新包，必须：
1. 先向用户说明：需要安装什么包、为什么需要、是否会影响现有环境
2. 等待用户明确回复"可以安装"或"同意"
3. 方可执行

### 2.4 已安装的 ROS2 包（树莓派）

191 个 ros-humble 包（ros-base，非 desktop），包括：
- `ros-humble-slam-toolbox`（SLAM 建图）
- `ros-humble-navigation2` + `nav2-*` 全家桶
- `ros-humble-tf2-*`（坐标变换）
- `ros-humble-pcl-conversions`、`ros-humble-diagnostic-updater`

**本机不安装/不需要的**：
- `ros-humble-desktop`（无 GUI，用 ros-base）
- `ros-humble-gazebo-ros`（无 GPU，不仿真）
- `ros-humble-rviz2`（无桌面，如需可视化用 ssh -X 或开发机远端）
- ESP-IDF（固件已在开发机编译烧录，树莓派不需要）

---

## 3. 硬件约束 ⚠️

### 3.1 内存预算（8GB 版本 — 较充裕）

| 资源 | 容量 | 备注 |
|------|------|------|
| 总内存 | ~7.6GB | 系统占用约 1GB |
| **可用内存** | **约 6.5GB** | 较为充裕 |
| SLAM 预算 | < 1GB | slam-toolbox + Karto 求解器 |
| 单节点 | < 150MB | 驱动/桥接/里程计等 |
| Nav2 总占用 | < 500MB | AMCL + planner + controller + costmaps |

### 3.2 性能红线

- **编译：`colcon build --parallel-workers 2`**（不允许不带此参数编译，会 OOM）
- **SLAM 分辨率：0.1m**（不用 0.05m，地图数据量差 4 倍）
- **SLAM 线程：`ceres_num_threads: 2`**（最多 2，不用 4）
- **AMCL 粒子：`max_particles: 500`**（不用 2000，当前基线 500 足够且已减半）
- **controller_frequency：10Hz**（不用 20Hz）
- **QoS：Best Effort + Keep Last，队列深度 5**（减少内存堆积）
- **日志级别：INFO**（DEBUG 仅在排错时临时开启）
- **禁止在回调中写文件、sleep、阻塞 I/O**
- **禁止 Gazebo 相关代码编译或运行**
- **避免频繁 TF 卡写入**（rosbag 按需录制，不长期运行）

### 3.3 微SD 卡注意事项

- IO 速度有限，编译产物（build/install/log）已在 SD 卡上
- 地图文件较小（PGM 几十 KB），直接放 SD 卡路径 `/home/ylz/n10p_leishen/maps/`
- 不建议在 SD 卡上做大量 rosbag 录制

---

## 4. 项目架构

### 4.1 项目目标

基于镭神智能 N10P 单线激光雷达的 ROS2 SLAM 建图与定位项目，部署于树莓派 4B（无人机机载计算机）。

### 4.2 数据流

```
N10P 原始数据 ───┬── 有线: USB串口 → lslidar_driver ──────→ /scan ──→ SLAM/Nav2
                 │
                 └── 无线: ESP32 WiFi TCP → n10p_wifi_bridge_node → /scan (同上)
```

两条路径输出完全相同的 `/scan`（LaserScan, frame_id=laser_frame, 10Hz），下游无感知。

### 4.3 TF 树（含 EKF 滤波）

```
map → odom → base_link → laser_frame
  (AMCL)  (EKF滤波/里程计)  (静态TF)
```

**有飞控 + EKF（推荐）**：
- `odom→base_link` 由 `imu_filter_node` 发布（IMU 陀螺仪+四元数互补滤波）
- slam-toolbox 通过扫描匹配估计运动
- `map→odom` 由 slam-toolbox 发布

**无飞控（传统方案，保留）**：
- `odom→base_link` 由 dummy_odom 发布（全零，SLAM 自估）

### 4.3.1 EKF 互补滤波 (导航跟踪基线 ⭐)

- **节点**: `imu_filter_node` (n10p_fusion 包, Python)
- **算法**: 互补滤波 — 高频 IMU 陀螺仪积分 + 低频飞控四元数修正 (自适应 alpha=0.005~0.05)
- **输入**: `/imu` + `/odom` (来自 ano_bridge)
- **输出**: `/odometry/filtered` + `odom→base_link` TF (100Hz)
- **验证**: 导航跟踪基线验证通过 (2026-07-24)，三层速度防御体系确认有效

### 4.3.1.1 速度处理链 (v4.0 基线)

```
FC 0x07 → ano_bridge[vx/vy_sign YAML + 死区0.02 + 交叉轴3:1] → /odom
    → imu_filter[FC指数平滑b=0.5 + 交叉轴双层 + 位置积分] → /odometry/filtered + TF
```

| 层 | 位置 | 功能 |
|----|------|------|
| 1 | ano_bridge YAML | vx_sign/vy_sign 参数化 (+1.0/+1.0) |
| 2 | ano_bridge `_publish_odometry` | FC死区 0.02m/s |
| 3 | ano_bridge `_publish_odometry` | 交叉轴抑制 (|主|>3×|副|→清零副) |
| 4 | imu_filter `_on_imu` | FC指数平滑 b=0.5 (dv=0, IMU不参与) |
| 5 | imu_filter `_on_imu` | 交叉轴抑制 (二次, 保留) |

**关键决策**: IMU加速度不参与速度估计(dv=0), 消除倾斜重力泄漏。速度完全由FC提供+指数平滑。

### 4.3.1.2 偏航归零 (v4.0 新增)

FC磁力计每次上电偏航随机(-100°~-150°), 不能在系统中用作绝对参考。
- 启动后等2s让磁力计稳定, 取50采样平均为init_yaw
- 对每个FC四元数乘init_yaw逆旋转, 使初始yaw≈0°
- slam_ekf/nav_ekf launch不再依赖FC yaw
- AMCL从(0,0,0°)启动, 用户在RViz标一次初始位姿

### 4.3.2 推荐用法

```bash
# 建图 (EKF, 一键启动)
ros2 launch n10p_slam slam_ekf_launch.py

# 导航 (EKF, 一键启动)
ros2 launch n10p_nav nav_ekf_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml

# F5 下行测试 (需先启动导航)
cd /home/ylz/n10p_leishen
python3 send_slam_cur_f5.py --port /dev/ttyUSB0 --rate 10 --duration 30 --log-file logs/slam_cur_static.log

# AMCL 收敛监控
python3 /home/ylz/n10p_leishen/n10p_ws/scripts/amcl_convergence.py

# 速度诊断 (验证三层防御体系)
python3 /home/ylz/n10p_leishen/scripts/diag_velocity.py

# 导航跟踪诊断 (验证 AMCL+TF+扫描匹配)
python3 /home/ylz/n10p_leishen/scripts/diag_nav_tracking.py

# 自动串口检测
python3 /home/ylz/n10p_leishen/n10p_ws/scripts/auto_detect_serial.py

# 传统方案 (无飞控, 保留)
ros2 launch n10p_slam slam_launch.py scan_source:=wired

# 环境清理 (如有残留进程)
bash ~/n10p_leishen/scripts/clean_ros2.sh
```

### 4.4 7 个 ROS2 包

| 包 | 类型 | 功能 |
|----|------|------|
| `lslidar_msgs` | C++ (cmake) | 驱动自定义消息类型 |
| `lslidar_driver` | C++ (cmake) | N10P 官方驱动，解析串口→/scan |
| `n10p_bringup` | Python (ament_python) | 飞控解析、里程计(dummy/keyboard)、WiFi桥接、启动 |
| `n10p_slam` | Python | SLAM 配置 + launch |
| `n10p_nav` | Python | Nav2 导航配置 + launch |
| `n10p_gazebo` | Python | Gazebo 仿真（**树莓派不编译不运行**） |
| **`n10p_fusion`** | **Python (ament_python)** | **EKF 互补滤波，IMU+飞控融合→平滑里程计 TF** |

### 4.5 目录结构

```
/home/ylz/n10p_leishen/
├── CLAUDE.md                     # 本文件（最高指令）
├── user.md                       # 使用教程
├── env.md                        # 环境配置教程
├── learn.md                      # 学习笔记
├── n10p_knowledge_base/          # N10P 硬件/协议资料
├── n10p_reference_doc/           # 参考文档（00-13）
├── esp32_n10p_bridge/            # ESP32 固件工程（仅开发机编译）
├── maps/                         # SLAM 地图文件
├── scripts/                      # 工具脚本
├── n10p_ws/                      # ROS2 工作空间
│   ├── build/ install/ log/      # 编译产物
│   └── src/
│       ├── Lslidar_ROS2_driver/
│       │   ├── lslidar_msgs/
│       │   └── lslidar_driver/
│       ├── n10p_bringup/
│       ├── n10p_slam/
│       ├── n10p_nav/
│       └── n10p_gazebo/          # 树莓派不编译
```

---

## 5. 架构决策记录 (ADR)

> 来源：`n10p_reference_doc/05_architecture_decisions.md`

| ADR | 决策 | 原因 |
|-----|------|------|
| 1 | SLAM 用 slam-toolbox online async | 社区活跃，支持手持无里程计，资源可控 |
| 2 | 分阶段开发（Phase 0→7） | 每步可执行可验证，避免一次性部署困难 |
| 3 | 零侵入双路径（有线+无线） | 两路径完全隔离，通过 scan_source 参数切换，下游只订阅 /scan |
| 4 | 放弃 socat PTY → 独立 Python WiFi 节点 | lslidar_driver 的 tcsetattr 破坏 PTY 行规约 |
| 5 | 键盘里程计独立运行 | 桌面测试用，WASD 控制，不与 dummy_odom 同时运行 |
| 6 | AMCL 全向模型 + SmacPlanner2D + RPP | OmniMotionModel, MOORE 8方向, RegulatedPurePursuit |
| 7 | 全局 costmap 禁用 rolling_window | rolling_window + obstacle_layer → planner SIGSEGV，改用 static_layer + 空白 PGM |
| 8 | bootstrap 静态 TF（map→odom） | AMCL 激活前 map 不存在→RViz 死锁，用静态 TF 引导 |
| 9 | 自动串口检测 | USB-TTL 串口可能互换(/dev/ttyUSB0↔USB1)，根据 USB ID 自动匹配 CH340(飞控)/CP2102(雷达) |
| 10 | AMCL 自动初始姿态 | launch 启动时从飞控 0x04 四元数获取 yaw 作为 AMCL initial_pose，位置 (0,0) 由扫描匹配收敛 |
| 11 | 0x04 四元数符号修正 | 标准公式转欧拉后 pitch/yaw 符号与凌霄 0x03 直出帧相反（与飞控方对账确认），ano_bridge 已取反 |

---

## 6. 已知 Bug 与红线 ⚠️

> 来源：`n10p_reference_doc/07_bug_fixes_and_known_issues.md`

### 6.1 驱动层 Bug

| Bug | 现象 | 修复 | 文件位置 |
|-----|------|------|----------|
| angle_increment 错误 | SLAM 丢弃所有扫描 "1058 readings, expected 529" | `angle_increment = 2*PI/scan_num`（非 count_num） | lslidar_driver.cc:990 |
| double free + delete/delete[] | 启动崩溃 exit code -6 | 5处 delete→delete[]；删除子函数内重复 delete | lslidar_driver.cc:718,862,794,951,1379 |

### 6.2 SLAM 层 Bug

| Bug | 现象 | 修复 |
|-----|------|------|
| 旋转建图变形 | 直走正常，旋转后房间重叠 | correlation_search_space_dimension: 0.5→1.5 |
| launch 串口冲突 | slam_launch.py + bringup 同时启动→double free | 创建 slam_only_launch.py（无 driver/odom） |

### 6.3 Nav2 层 Bug

| Bug | 现象 | 修复 |
|-----|------|------|
| planner SIGSEGV | 全局 costmap rolling_window + obstacle_layer → 崩溃 | global_costmap: static_layer + inflation_layer + 空白 PGM |
| lifecycle 超时 | controller_server 配置超时 | bond_timeout→15s, service_timeout→15s |
| map 帧死锁 | "frame 'map' does not exist"，RViz 白屏 | launch 加 static_transform_publisher map→odom 全零 bootstrap |
| RemovePassedGoals 不存在 | bt_navigator 插件找不到 | 用系统 navigate_w_replanning_time.xml |

### 6.4 集成层 Bug

| Bug | 现象 | 修复 |
|-----|------|------|
| 两个里程计 TF 冲突 | 机器人定位飞到 44m 外 | dummy_odom 和 keyboard_odom 二选一，不同时运行 |
| wifi_bridge 发布太早 | costmap 队列爆满 | wifi_bridge 加 5 秒启动延迟 |
| Fast-DDS 共享内存僵尸 | "RTPS_TRANSPORT_SHM Error" | 启动前 `rm -f /dev/shm/fastrtps_*` |
| ament_python entry_points | scan_relay 找不到可执行文件 | 每次 build 后手动 cp（已废弃 scan_relay） |
| 0x04 四元数符号错误 | RViz 旋转方向与现实相反 | pitch/yaw 符号取反（与凌霄 0x03 对账确认） |
| 串口互换 | /dev/ttyUSB0↔USB1 顺序不定 | auto_detect_serial.py 根据 USB ID 自动匹配 |

### 6.5 已知坑点 (KI) 快速解决

| KI | 现象 | 快速解决 |
|----|------|----------|
| KI-001 | 串口 Permission denied | 用户在 dialout 组（已配置） |
| KI-002 | RViz 无点云 | LaserScan→Reliability 改 Best Effort |
| KI-003 | TF extrapolation 报错 | Frame Rate 改 10Hz |
| KI-004 | 有数据无显示 | 发布 static TF base_link→laser_frame |
| KI-005 | 驱动 double free | 确认 install/ 中为修复后版本 |
| KI-006 | 找不到 /dev/ttyACM0 | lsusb 查芯片型号，改 lsx10.yaml serial_port |

### 6.6 绝对不可触犯的红线

1. ❌ **全局 costmap 绝对不能用 `rolling_window: true + obstacle_layer`** → 必 SIGSEGV
2. ❌ **所有 Nav2 launch 必须有 map→odom 静态 TF bootstrap** → 否则死锁
3. ❌ **dummy_odom 和 keyboard_odom 绝不能同时运行** → TF 冲突
4. ❌ **N10P 帧解析：距离用 `<H`(小端)，角度用 `>H`(大端)** → 不可混用
5. ❌ **驱动源码 delete 必须为 delete[]**（已修复，编译前确认）
6. ❌ **不许用 `pkill -f "ros2"` 或 `killall ros2`** → 只杀自己的 PID
7. ❌ **不许用 `colcon build` 不带 `--parallel-workers 2`** → 会 OOM
8. ❌ **N10P 扫描方向 `idx=(360-deg)*1058/360`** — CW→CCW 反转不可删除 → Y轴镜像
9. ❌ **odom 协方差(姿态)不能改回 1.0** — 四元数 A 级可信, covariance=0.001
10. ❌ **TF yaw 不能改** — 雷达箭头朝机头前方, yaw=0（2026-07-16 双验证通过）
11. ❌ **未授权不准改代码** — 先说明改什么/为什么/影响，等用户明确说"改"
12. ❌ **用户回退 git 后必须立即更新记忆文件** — 不要记忆错乱，把回退前的状态当成当前状态

---

## 7. 开发方法论 ⭐

### 7.1 小步验证原则

**不一次性部署全部功能。每个步骤必须是：可执行 → 可验证 → 确定性的。**

```
前一步验证通过 ✓
    ↓
当前步执行
    ↓
当前步验证通过 ✓
    ↓
下一步执行
```

**严禁**在上一阶段未验证通过的情况下，开始下一阶段的工作。

### 7.2 计划体系

使用 `TodoWrite` 工具动态管理当前会话的任务计划。
每个计划项必须有：
- `content`：要做什么（祈使句）
- `activeForm`：正在做的状态描述（进行时）

### 7.3 测试驱动的验证

每完成一个功能节点，必须给出可执行的验证命令。
驱动编译完成后 → `ros2 topic list | grep scan` 确认话题存在。

### 7.4 阶段完成交付清单 ⭐ (硬约束)

**每完成一个 Phase 或子功能后，必须向用户罗列以下三项：**

1. **运行方法**：用户需要执行什么命令来启动这个功能（可复制的一行或多行命令）
2. **功能讲解**：这个节点/模块做了什么，关键参数含义，输入输出是什么
3. **测试预期效果**：用户应该看到什么才算功能正常（话题名、频率、终端日志等）

**反例（不允许）**：只说"编译通过"或"在这里改了一行"，不给出完整的运行+测试指引。

### 7.5 进程清理 (硬约束) ⚠️

**任何自动化测试启动的进程，必须在测试结束后立即清理。只杀自己启动的，不伤及无辜。**

安全原则：
1. **启动时记录 PID** — 每个后台进程启动后，立刻记下它的 PID
2. **只杀自己的 PID** — `kill <pid>` 精准终止
3. **禁止大范围模糊杀进程** — 绝对不允许 `pkill -f "ros2"` 之类通配符杀法
4. **杀之前确认** — `ps -p <pid>` 确认是自己启动的
5. **清理后确认** — 确认 PID 已退出

```bash
# ✅ 正确
TEST_PID=$!
kill $TEST_PID 2>/dev/null

# ❌ 禁止
pkill -f "ros2"
pkill -f "python3"
killall ros2
```

---

## 8. 代码规范

### 8.1 通用原则

- **改代码前必须先 Read 相关文件**（不要凭记忆）
- **最小化 diff**（局部 edit / 补缺失逻辑），禁止重写整个文件
- 不改与当前任务无关的文件
- 不擅自"优化"或"重构"已有代码
- 修复 bug 时必须分析**根因**（读了哪行、为什么冲突），不只说"我改了"
- 代码可读性优先 — 只写"为什么这样做"的注释
- 不过早抽象 — 三行类似代码比一个函数好
- 不引入未使用的依赖

### 8.2 ROS2 特定

- 节点名：`snake_case`
- 话题名：`/` 前缀 `snake_case`，如 `/laser_scan`
- launch 文件：Python 格式（`.launch.py`），不要 XML
- config 文件：YAML
- frame_id 遵循 REP-105：`base_link` → `laser_frame`
- 包名：`ros2_<功能>` 或 `<功能>_ros2`

---

## 9. 文件写入约束 (最高优先级) ⚠️

**所有文件只允许写在 `/home/ylz/n10p_leishen/` 内，绝对禁止写任何文件到项目目录之外。**

- ❌ 禁止写 `~/.claude/`、`/tmp/`、`/home/ylz/` 等外部路径
- ❌ 禁止在项目外创建记忆文件、日志、临时文件
- ✅ 所有产出（记忆、脚本、配置、编译产物）均在 `/home/ylz/n10p_leishen/` 内

### 9.1 记忆文件位置

```
/home/ylz/n10p_leishen/memory/
```

### 9.2 记忆文件清单

| 文件 | 类型 | 用途 |
|------|------|------|
| `workspace_state.md` | project | 当前开发阶段、编译状态 |
| `known_issues.md` | project | 已知坑点清单 |
| `env_config.md` | reference | 环境配置快照 |

### 9.3 记忆操作规则

- 发现新坑点 → 写入 `memory/known_issues.md`
- 环境变化 → 更新 `memory/env_config.md`
- 阶段推进 → 更新 `memory/workspace_state.md`
- 出问题解决 → 追加 `memory/workspace_state.md`
- 新功能/新节点 → 更新项目根目录 `user.md`

---

## 10. 安全规范

- 不在代码或日志中硬编码密钥/密码/Token
- 不执行 `sudo` 命令
- 不修改 `/opt/ros/` 下的系统文件
- 修改 Udev 规则前提醒用户备份
- 不对 `/dev/` 做危险操作（如 `dd`）

---

## 11. 回复质量要求（硬约束）⭐

**禁止简短回复。每个回答必须包含充分的技术细节、原理分析和上下文解释。**

### 11.1 必须包含的内容

每次技术讨论或方案建议必须涵盖以下结构：

1. **背景/现象**：当前发生了什么，用户看到了什么，为什么这是问题
2. **根因分析**：为什么会发生，涉及的底层原理是什么，代码层面哪里出了问题
3. **方案对比**：至少给出 2-3 种可行方案，对比它们的优缺点、资源消耗、实施难度
4. **推荐方案**：明确推荐哪个方案，解释为什么它最适合当前场景
5. **实施细节**：具体的代码改动位置、参数含义、预期效果、验证方法
6. **风险提示**：方案可能引入的副作用、需要注意的边界条件

### 11.2 禁止行为

- ❌ 一行结论就结束（如"修了 X 行"）
- ❌ 没有解释原理就说"改成 X"
- ❌ 不对比方案就直接改代码
- ❌ 不说明为什么这样改、有什么影响

### 11.3 问题分析模板

发现问题时使用以下结构：
1. **现象**：描述发生了什么、如何复现、影响范围
2. **根因**：从硬件→驱动→ROS2→算法逐层追溯，给出具体文件+行号
3. **方案对比**：列出每种方案的成本、收益、风险
4. **修复步骤**：具体的修改内容、编译命令、验证方法
5. **关联影响**：修改会不会影响其他功能、需要同步更新的地方

### 11.4 正常交互约定

- 修改文件后不需要重复读取确认
- 并行处理独立操作（批量发出）
- 需要确认时：列出选项和利弊，等待用户决策
- 普通任务：一行结论 + 做了什么
- 禁止："当然！/好问题！/没问题！"开头、结尾客套、复述问题原文

---

## 12. 当前任务：树莓派 × 飞控 0xF5 下行通信

### 12.1 终极目标

树莓派作为感知中枢：SLAM定位(当前位置) + K230视觉(目标偏移) → 融合打包
→ 0xF5自定义帧(31B) → 串口 → STM32飞控 → PID位置控制。
双模式运行：航点导航模式 + 视觉伺服模式，模式切换完全在树莓派侧处理。

### 12.2 数据流架构 (doc: 树莓派飞控对接文档.md)

```
┌─────────────────── 树莓派 ───────────────────┐
│  N10P → AMCL → cur_x/y/z (飞机当前坐标, cm) │
│  K230 → 目标检测 → dx/dy/dz (目标相对偏移)   │
│         └── tar = cur + dx/dy/dz            │
│                    ↓                         │
│     ano_bridge_node: 融合 + 打包 0xF5 帧     │
│     (cur+tar+flags, 31B, 50Hz)              │
└──────────────────┬───────────────────────────┘
                   │ UART 串口 (500000 baud)
                   │ GPIO14(TXD)→PD6(UART2 RX)
                   ↓
┌────────────── STM32F407 飞控 ──────────────┐
│  接收 0xF5 → PID(goal=tar, obs=cur)        │
│  → 0x41 实时控制帧 → 凌霄IMU → 电机        │
└─────────────────────────────────────────────┘
```

### 12.3 0xF5 帧格式 (31 字节)

```
[0]=0xAA [1]=0x61 [2]=0xF5 [3]=0x19
[4-7] cur_x s32 LE cm   [8-11] cur_y  [12-15] cur_z
[16-19] tar_x s32 LE cm  [20-23] tar_y [24-27] tar_z
[28] flags (bit0=SLAM_VALID, bit1=TARGET_VALID, bit2=VISUAL_MODE)
[29] SC [30] AC  — 校验覆盖 [0]~[28] 共 29 字节
```

### 12.4 已完成阶段

| Phase | 内容 | 状态 |
|-------|------|:--:|
| 6.0 | 环境验证 | ✅ |
| 6.1 | 编译验证 (5包arm64) | ✅ |
| 6.2 | 凌霄飞控串口驱动 (三层架构) | ✅ |
| 6.3 | 有线雷达驱动 (scan修复: 双回波同角度/固定1058/强度过滤) | ✅ (2026-07-20 修正180°镜像认知错误) |
| 6.4 | SLAM建图 (参数回退原始好配置) | ✅ |
| 6.5 | 建图质量 (直走OK, 旋转优化) | ✅ (EKF已解决) |
| 6.6 | Nav2导航 (AMCL+成本图+BT) | ✅ (基本通过) |
| 6.7 | 性能调优 (odom 50Hz, AMCL 2000/500粒) | ✅ |
| 7.0 | EKF 互补滤波集成 | ✅ |
| 7.1 | 坐标系验证 (N10P+飞控+ROS REP-105) | ✅ |
| 7.2 | AMCL 自动初始姿态 (yaw_util, 2026-07-24 修正公式) | ✅ |
| 7.3 | 自动串口检测 | ✅ |
| 7.4 | 0xF5 固定帧联调 (步骤1-5) | ✅ |
| 8.0 | **0xF5 接入真实 SLAM 数据** | 🔄 当前攻坚 |

### 12.5 当前攻坚任务：F5 接入真实 SLAM 数据 (Phase 8)

**目标**：`send_slam_cur_f5.py` 从 AMCL `/amcl_pose` 取实时坐标→打包 0xF5→串口→飞控
**当前 Bug**：cur=(None,None,None)，AMCL pose 数据未正确获取
**频率**：50Hz (当前测试 10Hz)

### 12.5 开发原则

- 全自动化验证流程, 每一步编译→运行→检验→修复→再检验
- 不投机取巧, 不跳过验证, 不急于宣告成功
- 遇到问题先分析根因, 参考权威开源方案和项目资料
- 不动系统文件/配置, 新依赖用venv隔离
| 编译 | -j16 | `--parallel-workers 2` |
| 地图分辨率 | 0.05m | **0.1m** |
| SLAM 线程 | 4 | **2** |
| AMCL 粒子 | 2000 | **1000** |
| Gazebo | ✅ | ❌ 不装不编译 |
| ESP-IDF | ✅ 编译固件 | ❌ 不需要 |
| 串口路径 | 开发机 USB ID | **需实际 ls 确认** |

---

## 13. 版本记录

| 日期 | 版本 | 变更 |
|------|------|------|
| 2026-05-27 | v1.0 | 初始创建（开发机 x86） |
| 2026-05-27 | v1.1 | 新增 user.md + 代码可读性规范 |
| 2026-05-30 | v1.2 | 新增进程清理硬约束 |
| 2026-06-04 | v2.0-pi | 树莓派适配：更新路径/内存/性能红线/已知Bug/ADR/禁用Gazebo |
| 2026-07-13 | v3.0-ekf | EKF 互补滤波集成：imu_filter_node + 7包编译验证 + 旋转建图通过 |
| 2026-07-19 | v3.1-f5 | 坐标系验证通过 + AMCL自动初始姿态 + 自动串口检测 + 0xF5联调步骤1-5通过 |
| 2026-07-24 | v3.2-baseline | 导航跟踪基线验证通过: 三层速度防御, AMCL阈值0.01 |
| 2026-07-26 | v4.0-nofcyaw | **FC偏航解耦+速度纯FC平滑+AMCL增强**: 输入端偏航归零, IMU dv=0, b=0.5, 交叉轴双层, AMCL参数增强, launch清除FC yaw依赖 |
