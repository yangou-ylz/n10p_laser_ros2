# 03 — 环境配置

## 开发机环境 (x86_64 Ubuntu 22.04)

### ROS2 Humble 安装

```bash
# 设置locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# 添加ROS2仓库
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
```

### 系统依赖

```bash
sudo apt install -y build-essential cmake git python3-pip
sudo apt install -y libpcap-dev libboost-system-dev libboost-thread-dev libyaml-cpp-dev libeigen3-dev libpcl-dev
sudo apt install -y ros-humble-pcl-conversions ros-humble-diagnostic-updater ros-humble-diagnostic-msgs
```

### ros2env 函数 (~/.bashrc)

```bash
ros2env() {
    if command -v conda &> /dev/null; then conda deactivate 2>/dev/null; fi
    unset CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_PROMPT_MODIFIER
    unset PYTHONPATH LD_LIBRARY_PATH
    export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')
    source /opt/ros/humble/setup.bash
    if [ -f /usr/share/gazebo/setup.sh ]; then source /usr/share/gazebo/setup.sh; fi
    echo "ROS2 环境已加载（Humble）"
}
```

## 树莓派环境 (arm64 Ubuntu 22.04.05 Server)

树莓派已烧录 Ubuntu 22.04.05 LTS Server (arm64)，无图形界面。
ROS2 Humble 对 Ubuntu 22.04 arm64 有官方 apt 支持。

```bash
# 同样的安装命令，但不需要 ros-humble-desktop（server无GUI）
sudo apt install -y ros-humble-ros-base  # 替代ros-humble-desktop
# 其他依赖相同
```

### 存储分配

- TF卡 64G: 系统 + ROS2 + 代码（避免频繁写）
- SSD 512G: 地图数据、rosbag录制、编译中间产物
- 建议：将 `n10p_ws/` 放在SSD上，减少TF卡IO

### 树莓派特殊配置

- 编译限制并行数: `colcon build --parallel-workers 2`（避免OOM）
- 关闭swap或限制swap使用（减少TF卡写入）
- SLAM参数: `map_resolution: 0.1`（降低分辨率节省内存）
- SLAM求解器线程: `ceres_num_threads: 2`
- Gazebo **不在树莓派上安装**（无GPU，不需要仿真）

## ESP-IDF 环境（仅开发机编译固件用）

- ESP-IDF v5.3.2, 目标芯片 esp32s3
- 激活: `source ~/esp/esp-idf/export.sh`
- 编译: `cd esp32_n10p_bridge && idf.py build`
- 烧录: `idf.py -p /dev/ttyUSB0 flash monitor`
- 国内镜像: `export IDF_GITHUB_ASSETS="dl.espressif.cn/github_assets"`
