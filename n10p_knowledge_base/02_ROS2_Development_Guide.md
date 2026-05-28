# 02 — ROS2 开发环境全指南 (ROS2 Development Guide)

> 从零开始在 Ubuntu 22.04 + ROS2 Humble 下搭建 N10P 驱动开发环境
>
> 最后更新：2026-05-27

---

## 1. 先决条件

### 1.1 操作系统

| 项目 | 推荐配置 |
|------|----------|
| 操作系统 | **Ubuntu 22.04 LTS** (Jammy Jellyfish) |
| ROS2 发行版 | **Humble Hawksbill** |
| 架构 | amd64 (x86_64) |
| 内核版本 | Linux 5.15+ |

> N10P 驱动也支持 ROS2 Jazzy (Ubuntu 24.04)，但需要少量源码修改（见 4.4 节）。ROS2 Foxy/Galactic 理论上也可用，但社区反馈较少。

### 1.2 ROS2 Humble 安装

以下命令**仅作记录**，暂不执行：

```bash
# 确保系统 UTF-8 locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加 ROS2 APT 源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装 ROS2 Humble 桌面版
sudo apt update
sudo apt install ros-humble-desktop

# 安装 colcon 编译工具
sudo apt install python3-colcon-common-extensions
```

> 更推荐使用 "小鱼一键安装"（`wget http://fishros.com/install -O fishros && bash fishros`），对国内网络更友好。

---

## 2. 官方驱动仓库

### 2.1 仓库地址

| 仓库 | URL | 说明 |
|------|-----|------|
| **官方主仓库** | https://github.com/Lslidar/Lslidar_ROS2_driver | 镭神官方 ROS2 驱动 |
| N10 专用分支 (社区) | https://github.com/bjoernellens1/Lslidar_ROS2_driver/tree/N10_V1.0 | 可能有 N10P 专用优化 |
| 轮趣科技版 | 从淘宝/京东购买渠道获取 | 商家配套 ROS 包 |

### 2.2 支持的雷达型号

根据官方 README，该驱动支持的单线激光雷达型号包括：

- M10 / M10GPS / M10P
- **N10 / N10Plus（即 N10P）**
- N301 系列
- L10 等

### 2.3 代码仓库结构概览

```
Lslidar_ROS2_driver/
├── lslidar_driver/          # 核心驱动功能包
│   ├── src/                 # C++ 解析与发布源码
│   │   ├── lslidar_driver.cpp   # 主节点
│   │   ├── packet.cpp        # 数据包解析
│   │   └── ...
│   └── launch/              # ROS2 launch 启动文件
│       ├── lsn10p_launch.py         # N10P 串口启动
│       ├── lslidar_net.launch.py    # 网口版启动
│       ├── lslidar_serial.launch.py # 通用串口启动
│       └── viewer_scan_launch.py    # RViz2 可视化
├── lslidar_msgs/            # 自定义 ROS2 消息接口
│   └── msg/
│       └── LslidarPacket.msg       # 雷达原始数据包格式
├── README.md
└── ...
```

---

## 3. 系统依赖

### 3.1 APT 依赖

在编译 N10P 驱动前，需要安装以下系统依赖：

```bash
# libpcap: 用于网口通信（即使只用串口，编译时也需此依赖）
sudo apt install libpcap-dev

# ROS2 功能包依赖
sudo apt install ros-humble-diagnostic-updater

# 如果使用 ROS2 Jazzy
# sudo apt install ros-jazzy-diagnostic-updater

# 常规编译工具（通常已安装）
sudo apt install build-essential cmake
```

### 3.2 依赖清单汇总

| 依赖包 | 用途 | 必需？ |
|--------|------|--------|
| `libpcap-dev` | 网口数据包捕获库 | ✅ 编译必需 |
| `ros-humble-diagnostic-updater` | ROS2 诊断状态上报 | ✅ |
| `ros-humble-sensor-msgs` | LaserScan 消息类型 | ✅（随 ROS2 桌面版安装） |
| `ros-humble-rclcpp` | ROS2 C++ 客户端库 | ✅（随 ROS2 桌面版安装） |
| `yaml-cpp` | YAML 配置文件解析 | ✅（通常系统自带） |
| `pcl-ros` | 点云处理（可选） | ❌（N10P 只需 LaserScan） |

---

## 4. 编译步骤

### 4.1 创建工作空间

```bash
# 创建 ROS2 工作空间目录
mkdir -p ~/lidar_ws/src

# 进入 src 目录，克隆官方驱动
cd ~/lidar_ws/src
git clone https://github.com/Lslidar/Lslidar_ROS2_driver.git
```

### 4.2 安装依赖后编译

```bash
# 返回工作空间根目录
cd ~/lidar_ws

# 安装 rosdep 依赖（如有）
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install

# source 环境
source install/setup.bash
```

> `--symlink-install` 选项：配置文件更改后无需重新编译，直接生效。

### 4.3 常见编译问题与修复

| 现象 | 原因 | 修复方法 |
|------|------|----------|
| `fatal error: pcap.h: No such file or directory` | 缺少 libpcap-dev | `sudo apt install libpcap-dev` |
| `fatal error: diagnostic_updater/...` | 缺少 ROS2 diagnostic 包 | `sudo apt install ros-humble-diagnostic-updater` |
| `delete packet_bytes` 报错 (Jazzy) | 源码使用了 `delete` 而非 `delete []` | 修改源码：`delete packet_bytes;` → `delete [] packet_bytes;` |
| `cmake warning` 针对某些宏 | GCC 版本较新 | 可忽略，不影响编译产物 |

### 4.4 Jazzy (Ubuntu 24.04) 适配注意事项

如果使用 ROS2 Jazzy，除上述 `delete` 修复外：

- 社区反馈在树莓派 5 + Ubuntu 24.04 + Jazzy 上可以成功编译和运行
- 官方驱动仓库 `main` 分支主要是为 Foxy/Galactic/Humble 设计的，Jazzy 下会出现部分 C++ 标准更严格的警告
- **推荐方案**：在 Ubuntu 24.04 用户可考虑使用 Humble 的 Docker 容器，或等待官方发布 Jazzy 兼容版本

---

## 5. Linux 权限配置（重要！）

### 5.1 串口权限问题

N10P 通过 USB 串口挂载，Linux 默认只允许 `root` 和 `dialout` 组用户访问。

**临时方案**（每次重插 USB 后需重做）：
```bash
sudo chmod 666 /dev/ttyACM0   # 或 ttyUSB0
```

**永久方案** — 将用户加入 `dialout` 组：
```bash
sudo usermod -a -G dialout $USER
# 重要：执行后必须 重启/注销 或至少新开一个终端才能生效
# 验证是否生效：groups $USER | grep dialout
```

### 5.2 Udev 规则（推荐）

创建 udev 规则，每次插入 N10P 时自动设置权限并固定设备名：

```bash
# 先确认 N10P 的 USB Vendor 和 Product ID
lsusb
# 查找 CH9102 或 CP210x 设备的 ID（如 1a86:55d4 或 10c4:ea60）

# 创建 udev 规则文件
sudo tee /etc/udev/rules.d/99-lslidar-n10p.rules << 'EOF'
# 镭神 N10P 激光雷达 - CH9102 芯片版本
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", MODE="0666", SYMLINK+="n10p_lidar"
# 镭神 N10P 激光雷达 - CP210x 芯片版本
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666", SYMLINK+="n10p_lidar"
EOF

# 重载 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger
```

配置成功后，每次插入雷达都会在 `/dev/n10p_lidar` 创建固定符号链接，且权限自动为 0666。

> ⚠️ Vendor/Product ID 因批次而异，请通过 `lsusb` 确认实际值后再写入。

### 5.3 确认雷达已成功识别

```bash
# 方法1：查看 USB 设备
lsusb | grep -i -E 'ch9102|cp210|silicon|qinheng'

# 方法2：查看 tty 设备
ls -l /dev/ttyACM* /dev/ttyUSB* /dev/ttyCH343* 2>/dev/null

# 方法3：查看内核日志
sudo dmesg | grep -i tty | tail -10
```

---

## 6. 驱动运行

### 6.1 启动命令

```bash
# 环境准备
cd ~/lidar_ws
source install/setup.bash

# 方式一：N10P 专用 launch（推荐）
ros2 launch lslidar_driver lsn10p_launch.py

# 方式二：通用串口 launch（需确保 YAML 中 lidar_name 为 N10P）
ros2 launch lslidar_driver lslidar_serial.launch.py

# 方式三：同时启动 RViz2 可视化
ros2 launch lslidar_driver viewer_scan_launch.py
```

### 6.2 关键配置参数

在 YAML 配置文件中（如 `config/lsn10p.yaml`），可以调整以下参数：

```yaml
lslidar_driver_node:
  ros__parameters:
    # 基本信息
    lidar_name: "N10P"               # 雷达型号 N10P / N10
    frame_id: "laser_frame"          # 发布数据的坐标系名
    interface_selection: "serial"    # 接口类型 serial / net

    # 串口配置
    serial_port: "/dev/ttyACM0"      # 串口设备路径（按实际情况修改）
    baud_rate: 460800                # 波特率

    # 话题设置
    pubScan: true                    # 是否发布 LaserScan
    scan_topic: "/scan"              # LaserScan 话题名
    pubPointCloud2: false            # N10P 可关闭
    pointcloud_topic: "/lslidar_point_cloud"

    # 测距范围
    min_range: 0.02                  # 最小距离 (m)
    max_range: 12.0                  # 最大距离 (m)
    angle_min: 0.0                   # 扫描起始角度 (rad)
    angle_max: 6.2831853             # 扫描结束角度 (rad) 即 360°
```

### 6.3 验证数据

```bash
# 查看话题列表
ros2 topic list

# 查看 /scan 话题详细信息（类型、QoS 等）
ros2 topic info /scan

# 以文本形式查看雷达数据
ros2 topic echo /scan

# 查看发布频率
ros2 topic hz /scan
```

---

## 7. 关键话题与消息类型

| 话题名 | 消息类型 | QoS 策略 | 说明 |
|--------|----------|----------|------|
| `/scan` | `sensor_msgs/msg/LaserScan` | 通常 Best Effort | 2D 激光扫描数据（主要使用） |
| `/lslidar_packets` | `lslidar_msgs/msg/LslidarPacket` | — | 原始数据包（调试用） |
| `/diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | — | 雷达状态信息 |
| `/parameter_events` | `rcl_interfaces/msg/ParameterEvent` | — | 参数变更通知 |

---

## 8. 数据流架构（通俗理解）

```
N10P 雷达 (旋转电机 + 激光测距模块)
    │
    │ UART TTL 串口 (460800 bps)
    ▼
CH9102 USB 转串口芯片
    │
    │ USB 2.0
    ▼
Ubuntu 主机 (/dev/ttyACM0)
    │
    │ lslidar_driver 节点 读取并解析
    │  - 帧同步 (找 A5 5A... FA FB)
    │  - 解析转速、角度、距离
    │  - 组装为 sensor_msgs/LaserScan
    ▼
ROS2 话题 /scan
    │
    ▼
下游消费：RViz2 可视化 / SLAM / 导航避障
```

---

## 9. 关键信息来源

| 来源 | 链接 |
|------|------|
| 官方 GitHub 主仓库 | https://github.com/Lslidar/Lslidar_ROS2_driver |
| N10 专用分支 (bjoernellens1) | https://github.com/bjoernellens1/Lslidar_ROS2_driver/tree/N10_V1.0 |
| ROS2 Jazzy 保姆教程 (CSDN) | https://blog.csdn.net/dqsh06/article/details/149247904 |
| ROS1 踩坑指南 (CSDN) | https://blog.csdn.net/2301_81315771/article/details/151064512 |
| N10P 测试记录 (博客园) | https://www.cnblogs.com/cjl520/p/17528259.html |
| 小鱼 ROS2 工具站 | http://fishros.com/ |
