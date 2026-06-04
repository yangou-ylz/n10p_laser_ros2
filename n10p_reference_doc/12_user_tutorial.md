# 12 — 完整使用教程（精简版）

> 每步按顺序执行。所有命令在树莓派上把路径 `/home/ubuntu22/ROS2/n10p_leishen` 换成 `/mnt/ssd/n10p_leishen`。

## 0. 硬件准备

| 设备 | 串口 |
|------|------|
| N10P 雷达 | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EB011256-if00`, 460800 |
| 凌霄飞控 | `/dev/serial/by-id/usb-ANO_TC_ANO_RadioLink-if00`, 921600 |
| ESP32 WiFi | 192.168.0.184:8888 |

## 1. 环境激活

```bash
ros2env                                    # 激活ROS2
source ~/ROS2/n10p_leishen/n10p_ws/install/setup.bash  # 加载工作空间
```

## 2. 编译

```bash
cd ~/ROS2/n10p_leishen/n10p_ws
colcon build --packages-select lslidar_msgs lslidar_driver n10p_bringup n10p_slam n10p_nav
source install/setup.bash
```

## 3. 仅看雷达点云

```bash
# 有线
ros2 launch lslidar_driver lslidar_launch.py
# 无线
ros2 launch n10p_slam slam_launch.py scan_source:=wireless

# 验证
ros2 topic hz /scan    # 应约10Hz
```

## 4. 雷达+飞控全开

```bash
# 有线
ros2 launch n10p_bringup n10p_bringup_launch.py
# 无线
ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wireless
```

## 5. SLAM 建图

### 手持（无飞控）
```bash
# 有线
ros2 launch n10p_slam slam_launch.py
# 无线
ros2 launch n10p_slam slam_launch.py scan_source:=wireless
```
手持雷达缓慢走动，走一圈后回到起点（回环检测）。

### 飞控在线
```bash
# 终端1: 传感器
ros2 launch n10p_bringup n10p_bringup_launch.py
# 终端2: SLAM
ros2 launch n10p_slam slam_only_launch.py
```

## 6. 保存地图

```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/path/to/maps/n10p_map'}}"
```
result=0表示成功。生成 n10p_map.yaml + n10p_map.pgm。

## 7. 查看地图

```bash
python3 scripts/map_viewer.py maps/n10p_map.yaml
```

## 9. Nav2 导航

```bash
# 有线
ros2 launch n10p_nav nav_launch.py map:=/path/to/map.yaml
# 无线
ros2 launch n10p_nav nav_launch.py map:=/path/to/map.yaml scan_source:=wireless
```
RViz中: 2D Pose Estimate设位姿→AMCL收敛→2D Goal Pose设目标。

## 10. Gazebo仿真（仅开发机）

```bash
bash scripts/start_simulation.sh
```
启动前必做: `rm -f /dev/shm/fastrtps_*`

## 11. 桌面测试

```bash
# 终端1
ros2 run n10p_bringup keyboard_odom_node
# 终端2
ros2 launch n10p_nav desktop_test_launch.py scan_source:=wireless
```
WASD/QE控制移动，R重置位置，Ctrl+C退出。
