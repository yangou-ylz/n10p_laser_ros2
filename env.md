# N10P ROS2 SLAM — 环境配置教程 (保姆级)

> 版本: v1.0 | 更新: 2026-05-27
> 目标平台: Ubuntu 22.04 LTS (开发机) + 树莓派 4B (部署机, Raspberry Pi OS)
> 配套文件: `requirements.txt` (依赖清单)

---

## 目录

1. [操作系统安装](#1-操作系统安装)
2. [ROS2 Humble 安装](#2-ros2-humble-安装)
3. [编译工具链安装](#3-编译工具链安装)
4. [项目依赖安装](#4-项目依赖安装)
5. [串口权限配置](#5-串口权限配置)
6. [ROS2 环境快捷命令](#6-ros2-环境快捷命令)
7. [创建工作空间](#7-创建工作空间)
8. [编译驱动](#8-编译驱动)
9. [验证](#9-验证)
10. [树莓派特别说明](#10-树莓派特别说明)

---

## 1. 操作系统安装

### 1.1 开发机 (x86_64)

- 系统: Ubuntu 22.04.5 LTS (Jammy Jellyfish)
- 下载: https://releases.ubuntu.com/22.04/
- 安装方式: USB 启动盘或双系统
- 建议磁盘空间: ≥ 50GB
- 建议内存: ≥ 8GB

### 1.2 树莓派 4B (arm64)

- 系统: Raspberry Pi OS (64-bit, based on Debian Bookworm)
- 下载: https://www.raspberrypi.com/software/operating-systems/
- 烧录工具: Raspberry Pi Imager
- 注意事项:
  - 选择 **64-bit** 版本 (ROS2 Humble 在 32-bit 上不可用)
  - 树莓派官方镜像不含 Ubuntu，需通过 Docker 或手动编译方式安装 ROS2
  - **详细步骤见第 10 节**

---

## 2. ROS2 Humble 安装

### 2.1 开发机 (Ubuntu 22.04)

Ubuntu 22.04 官方支持 ROS2 Humble，直接通过 apt 安装。

```bash
# 1. 设置 locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 2. 添加 ROS2 仓库
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 3. 安装 ROS2 Humble 桌面版 (全量安装)
sudo apt update
sudo apt install -y ros-humble-desktop

# 4. 安装开发工具
sudo apt install -y python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
```

### 2.2 验证 ROS2 安装

```bash
# 在新终端中执行
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker   # 另一个终端: ros2 run demo_nodes_py listener
```

---

## 3. 编译工具链安装

```bash
sudo apt install -y build-essential cmake git python3-pip
```

验证:
```bash
gcc --version      # 应显示 11.x
cmake --version    # 应显示 3.22+
git --version      # 应显示 2.34+
```

---

## 4. 项目依赖安装

### 4.1 一键安装所有依赖

项目根目录的 `requirements.txt` 列出了全部依赖。复制粘贴以下命令一键安装:

```bash
# 系统库
sudo apt install -y build-essential cmake git python3-pip python3-colcon-common-extensions
sudo apt install -y libpcap-dev libboost-system-dev libboost-thread-dev libyaml-cpp-dev libeigen3-dev libpcl-dev

# ROS2 包 (ros-humble-desktop 已含大部分，以下为额外确认)
sudo apt install -y ros-humble-pcl-conversions ros-humble-diagnostic-updater ros-humble-diagnostic-msgs

# 确认 rosdep 依赖
rosdep update
```

### 4.2 分步验证

```bash
# 每安装一组后验证
dpkg -l | grep libpcap-dev          # 应显示 ii
dpkg -l | grep libpcl-dev            # 应显示 ii
dpkg -l | grep ros-humble-pcl-conversions  # 应显示 ii
```

### 4.3 当前环境实际安装记录

| 日期 | 操作 | 涉及包 |
|------|------|--------|
| 2026-05-27 | 初始安装 | 系统已有 331 个 ros-humble 包 |
| 2026-05-27 | 补充安装 | libpcap-dev (1.10.1) |
| 2026-05-27 | 补充安装 | libpcl-dev, ros-humble-pcl-conversions |

---

## 5. 串口权限配置

N10P 雷达通过 USB 转串口连接，需要读写权限。

### 5.1 一次性权限 (插拔后失效)

```bash
sudo chmod 666 /dev/ttyACM0
```

### 5.2 永久权限 (推荐)

```bash
# 将当前用户加入 dialout 组
sudo usermod -a -G dialout $USER

# 重新登录使权限生效 (或重启)
# 验证:
groups $USER | grep dialout
```

### 5.3 识别雷达串口

```bash
# 插上雷达后查看
lsusb | grep -iE "ch9102|cp210|ch340|qinheng"
# N10P 常见输出: QinHeng Electronics USB Single Serial

ls /dev/ttyACM* /dev/ttyUSB*
# N10P 常见输出: /dev/ttyACM0
```

如果找不到设备，尝试:
```bash
dmesg | tail -20  # 查看内核日志中的 USB 设备识别信息
```

---

## 6. ROS2 环境快捷命令

### 6.1 创建 ros2env 别名 (推荐)

在 `~/.bashrc` 末尾添加以下函数。此方法会自动清除可能冲突的 Conda 环境变量。

```bash
ros2env() {
    # 清除 Conda 环境
    if command -v conda &> /dev/null; then
        conda deactivate 2>/dev/null
    fi
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
    unset PYTHONPATH LD_LIBRARY_PATH

    # 清理 PATH 中的 conda 路径
    export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')

    # 加载 ROS2
    source /opt/ros/humble/setup.bash

    # 如果有 Gazebo
    if [ -f /usr/share/gazebo/setup.sh ]; then
        source /usr/share/gazebo/setup.sh
    fi

    echo "ROS2 环境已加载（Humble）"
    echo "当前 Python 路径: $(which python3)"
    echo "Python 版本: $(python3 --version)"
}
```

使用方式:
```bash
ros2env    # 激活 ROS2 环境
```

### 6.2 工作空间快捷激活

```bash
# 在 ~/.bashrc 中再添加一个函数
n10p_env() {
    ros2env
    source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash
    echo "N10P 工作空间已加载"
}
```

---

## 7. 创建工作空间

```bash
# 创建目录结构
mkdir -p ~/ROS2/n10p_leishen/n10p_ws/src

# 克隆 N10P 驱动
cd ~/ROS2/n10p_leishen/n10p_ws/src
git clone https://github.com/Lslidar/Lslidar_ROS2_driver.git
cd Lslidar_ROS2_driver
git checkout M10P/N10P    # 切换到 N10P 专用分支

# 回到工作空间根目录
cd ~/ROS2/n10p_leishen/n10p_ws
```

---

## 8. 编译驱动

### 8.1 修改配置文件 (编译前必做)

编辑 `src/Lslidar_ROS2_driver/lslidar_driver/params/lsx10.yaml`:

```yaml
/lslidar_driver_node:
  ros__parameters:
    lidar_name: N10_P                     # 改为 N10_P
    interface_selection: serial           # 保持 serial
    serial_port_: /dev/ttyACM0            # 改为实际设备名 (可能是 ttyACM0 或 ttyUSB0)
    pubScan: true
    pubPointCloud2: false
    scan_topic: /scan
    frame_id: laser_link
    min_range: 0.02                       # N10P 实际最小量程
    max_range: 12.0                       # N10P 实际最大量程 (室内), 室外放宽到 12.0
```

### 8.2 编译

```bash
# 激活 ROS2 环境
ros2env

# 安装 rosdep 依赖 (自动解决 ROS2 包依赖)
cd ~/ROS2/n10p_leishen/n10p_ws
rosdep install --from-paths src --ignore-src -r -y

# 编译
colcon build --symlink-install
```

编译成功标志: 看到 `Summary: 2 packages finished` (lslidar_msgs + lslidar_driver)

### 8.3 常见编译错误

| 错误 | 原因 | 解决 |
|------|------|------|
| `Could not find ament_cmake` | 未加载 ROS2 环境 | 执行 `ros2env` |
| `Could not find PCL` | libpcl-dev 未安装 | `sudo apt install -y libpcl-dev` |
| `Could not find pcl_conversions` | ros-humble-pcl-conversions 未安装 | `sudo apt install -y ros-humble-pcl-conversions` |
| `delete` 语法报错 (Jazzy 用户) | Jazzy 编译器更严格 | 改为 `delete []` (Humble 不受影响) |

---

## 9. 验证

### 9.1 无硬件验证 (检查编译结果)

```bash
source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash
ros2 pkg list | grep lslidar    # 应看到 lslidar_driver 和 lslidar_msgs
ros2 pkg executables lslidar_driver  # 应看到 lslidar_driver_node
```

### 9.2 连接雷达验证

```bash
# 确认雷达设备存在
ls /dev/ttyACM0

# 启动驱动
source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash
ros2 launch lslidar_driver lslidar_launch.py

# 新终端，检查话题
source /opt/ros/humble/setup.bash
ros2 topic list | grep scan     # 应看到 /scan
ros2 topic hz /scan             # 应显示约 10Hz
ros2 topic echo /scan --once    # 查看一帧数据
```

### 9.3 RViz2 可视化验证

```bash
# 新终端
ros2env
rviz2
# 在 RViz2 中: Add → By topic → /scan → LaserScan
# Reliability Policy 改为 Best Effort
```

---

## 10. 树莓派特别说明

> 树莓派部分在 Phase 6 正式移植时详细补充，以下为预备信息。

### 10.1 硬件差异

| 项目 | 开发机 (x86_64) | 树莓派 4B (arm64) |
|------|----------------|-------------------|
| 架构 | amd64 | aarch64 (arm64) |
| 系统 | Ubuntu 22.04 | Raspberry Pi OS (Debian) |
| ROS2 安装 | apt (官方仓库) | 需手动编译或 Docker |
| 内存 | 30GB | 4GB 或 8GB |
| 存储 | SSD | microSD (IO 较慢) |

### 10.2 树莓派 ROS2 Humble 安装方案

Raspberry Pi OS 基于 Debian Bookworm，ROS2 Humble 官方仅支持到 Debian Bullseye。
推荐方案 (Phase 6 时验证):
1. **方案 A: Docker 容器** — 在 RPi OS 上运行 Ubuntu 22.04 Docker 容器，容器内安装 ROS2 Humble
2. **方案 B: 刷 Ubuntu Server 22.04** — 树莓派直接安装 Ubuntu 22.04 arm64 版
3. **方案 C: 从源码编译** — 在 RPi OS 上从源码编译 ROS2 Humble (耗时数小时)

具体选哪个方案在 Phase 6 根据实际测试确定。

### 10.3 性能注意事项 (移植时必读)

- 编译时使用 `-j1` 或 `-j2` 限制并行数 (树莓派 4 核，编译大型 C++ 项目易 OOM)
- microSD 卡 IO 慢，考虑使用 USB SSD
- 关闭不必要的系统服务节省内存
- SLAM 地图分辨率适当降低 (0.05m → 0.1m)

---

## 附录 A: 文件清单

本项目环境相关的文件:

| 文件 | 用途 |
|------|------|
| `requirements.txt` | 所有软件包依赖清单 |
| `env.md` (本文件) | 环境配置教程 |
| `n10p_ws/src/Lslidar_ROS2_driver/lslidar_driver/params/lsx10.yaml` | 雷达参数配置 |
| `~/.bashrc` | ros2env / n10p_env 函数定义 |

## 附录 B: 更新记录

| 日期 | 变更 |
|------|------|
| 2026-05-27 | 初始创建，记录开发机完整配置 |
| 2026-05-27 | Phase 1 编译验证：驱动启动成功，/scan 10.05Hz，360° 数据正常 |
| | 待补充: 树莓派实际安装步骤 (Phase 6) |

## 附录 C: 开发机实测记录 (2026-05-27)

### 硬件识别

```
$ lsusb | grep QinHeng
Bus 003 Device 009: ID 1a86:55d4 QinHeng Electronics USB Single Serial
$ ls /dev/ttyACM0
/dev/ttyACM0    # CH9102 USB 转串口芯片
```

### 编译结果

```
Starting >>> lslidar_msgs      [4.06s]   ✅
Starting >>> lslidar_driver    [12.6s]   ✅
Summary: 2 packages finished [16.8s]
```

### 驱动启动

```
[INFO] [lslidar_driver_node]: Lidar is N10_P
port = /dev/ttyACM0, baud_rate = 460800
open_port /dev/ttyACM0 OK !
[INFO] [lslidar_driver_node]: Successfully initialize driver...
```

### /scan 话题验证

```
$ ros2 topic hz /scan
average rate: 10.051    ← 目标 10Hz ✅
$ ros2 topic echo /scan --once
frame_id: laser_frame    ← 坐标系 ✅
angle_min: 0.0           ← 0°
angle_max: 6.283         ← 360° ✅
range_min: 0.02          ← 最近量程
range_max: 12.0          ← 最远量程
ranges: [1.41, 1.37, ... 0.14, ... 5.18]  ← 实测有数据 ✅
intensities: [220, 219, ... 0, ... 231]     ← 强度正常 ✅
```
