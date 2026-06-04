# 11 — 树莓派4B 迁移指南

> **当前任务**：将项目从 x86_64 开发机完整迁移到树莓派4B arm64。

## 目标硬件

- 树莓派4B, 4GB/8GB RAM
- Ubuntu 22.04.05 LTS Server (arm64, 无图形界面)
- TF卡 64G (系统+ROS2+代码) + SSD 512G (地图+数据+编译产物)
- 用户: ubuntu22 (与开发机相同)

## 迁移步骤

### Step 1: 系统基础配置

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 创建用户（如需要）
sudo usermod -a -G dialout ubuntu22

# 挂载SSD
sudo mkdir -p /mnt/ssd
sudo mount /dev/sda1 /mnt/ssd  # 根据实际设备名调整
# 写入/etc/fstab实现开机自动挂载

# 设置locale
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8
```

### Step 2: 安装 ROS2 Humble (arm64)

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
# Server版用ros-base替代ros-desktop（无GUI不需要桌面组件）
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions python3-rosdep
sudo rosdep init && rosdep update
```

### Step 3: 安装系统依赖

```bash
# 编译工具链
sudo apt install -y build-essential cmake git python3-pip

# 驱动编译依赖
sudo apt install -y libpcap-dev libboost-system-dev libboost-thread-dev
sudo apt install -y libyaml-cpp-dev libeigen3-dev libpcl-dev

# ROS2额外包
sudo apt install -y ros-humble-pcl-conversions ros-humble-diagnostic-updater
sudo apt install -y ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install -y ros-humble-tf2-ros ros-humble-tf2-sensor-msgs
```

### Step 4: 复制项目代码

```bash
# 从开发机复制整个项目
scp -r ubuntu22@<开发机IP>:/home/ubuntu22/ROS2/n10p_leishen /mnt/ssd/

# 或在树莓派上git clone（如已推送到git）
# git clone <repo_url> /mnt/ssd/n10p_leishen
```

### Step 5: 配置环境

```bash
# 编辑 ~/.bashrc，添加ros2env函数
cat >> ~/.bashrc << 'EOF'
ros2env() {
    source /opt/ros/humble/setup.bash
    echo "ROS2 环境已加载（Humble）"
}

n10p_env() {
    ros2env
    source /mnt/ssd/n10p_leishen/n10p_ws/install/setup.bash
    echo "N10P 工作空间已加载"
}
EOF

source ~/.bashrc
```

### Step 6: 编译项目

```bash
cd /mnt/ssd/n10p_leishen/n10p_ws

# 清理旧的x86编译产物
rm -rf build/ install/ log/

# 安装rosdep依赖
ros2env
rosdep install --from-paths src --ignore-src -r -y

# 编译（限制并行数，避免OOM）
colcon build --parallel-workers 2 --symlink-install
```

### Step 7: 树莓派SLAM参数调优

降低资源消耗以适配树莓派4B:

```yaml
# n10p_slam/config/mapper_params_online_async.yaml 修改
map_resolution: 0.1            # 0.05→0.1, 地图更粗糙但省内存
ceres_num_threads: 2           # 4→2, 用2个核
```
```yaml
# n10p_nav/config/nav2_params_n10p.yaml 修改
amcl:
  max_particles: 1000          # 2000→1000
  min_particles: 250           # 500→250
controller_server:
  controller_frequency: 10.0   # 20.0→10.0
```

### Step 8: 验证

```bash
# 验证环境
ros2env
ros2 pkg list | grep -E 'lslidar|n10p'

# 有线-仅雷达
ros2 launch lslidar_driver lslidar_launch.py
ros2 topic hz /scan     # 应约10Hz

# 无线-仅雷达
ros2 run n10p_bringup n10p_wifi_bridge_node
ros2 topic hz /scan

# SLAM手持建图
ros2 launch n10p_slam slam_launch.py

# Nav2导航（有线）
ros2 launch n10p_nav nav_launch.py
```

## 树莓派 vs 开发机差异

| 项目 | 开发机 | 树莓派 |
|------|--------|--------|
| 架构 | x86_64 | arm64 |
| ROS2包 | ros-humble-desktop | ros-humble-ros-base |
| 可视化 | RViz2本地 | 需远程（X11转发/SSH -X） |
| 编译并行 | -j16 | --parallel-workers 2 |
| 地图分辨率 | 0.05m | 0.1m |
| SLAM线程 | 4 | 2 |
| AMCL粒子 | 2000 | 1000 |
| Gazebo | ✅ | ❌不安装 |
| ESP-IDF | ✅ | ❌不需要 |

## 远程RViz2方案（无GUI问题）

树莓派是Server版无桌面，RViz2显示有两种方案：

**方案A: SSH X11转发**
```bash
ssh -X ubuntu22@<树莓派IP>
ros2env
rviz2 -d /mnt/ssd/n10p_leishen/n10p_ws/src/n10p_slam/config/n10p_slam.rviz
```

**方案B: 开发机运行RViz2**（推荐，性能更好）
```bash
# 开发机终端
export ROS_MASTER_URI=http://<树莓派IP>:11311
rviz2
```
树莓派和开发机必须连同一网络，且DDS配置正确（默认单播即可）。

## 性能红线

- SLAM内存 < 1GB, 单节点 < 100MB
- 不在回调中阻塞I/O或sleep
- 雷达10Hz, 下游处理不超过此频率
- QoS: Best Effort + Keep Last 队列5~10
- 日志: INFO级别 - DEBUG仅在排错时开启
