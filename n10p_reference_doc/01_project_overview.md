# 01 — 项目总览

## 项目目标

基于镭神智能 N10P 单线激光雷达的 ROS2 SLAM 建图与定位项目。最终部署目标是**树莓派4B 无人机机载计算机**。

## 当前进度

| Phase | 名称 | 状态 |
|-------|------|:---:|
| 0 | 环境验证 | ✅ |
| 1 | N10P 驱动编译与数据验证 | ✅ |
| 2 | RViz2 可视化联调 | ✅ |
| 2.5 | 凌霄飞控串口解析 (ano_bridge) | ✅ |
| 3 | SLAM 建图 (slam-toolbox) | ✅ |
| 4 | Nav2 导航 | ✅ |
| 5 | Gazebo 仿真集成 | ✅ |
| 5.5 | 桌面测试模式 (键盘里程计) | ✅ |
| 5.6 | ESP32 WiFi 无线雷达接入 | ✅ |
| 6 | **树莓派4B 移植** ← 当前 | ⬜ |
| 7 | 无人机 MAVROS 集成 | ⬜ |

## 总体数据流

```
N10P 原始数据 ───┬── 有线: USB串口 → lslidar_driver ──────→ /scan ──→ SLAM/Nav2/RViz2
                 │
                 └── 无线: ESP32 WiFi TCP → n10p_wifi_bridge_node → /scan (同上)
```

两条路径输出完全相同的 `/scan`（LaserScan, frame_id=laser_frame, 10Hz），下游无感知。

## TF 树（坐标系）

```
map (AMCL发布, map→odom) → odom (里程计) → base_link → laser_frame (静态)
```

手持建图模式（dummy_odom/全零里程计）下，odom→base_link为全零，slam-toolbox通过扫描匹配估计运动，map→odom由slam-toolbox发布。

## 核心组件清单

| 包 | 类型 | 功能 |
|----|------|------|
| Lslidar_ROS2_driver | C++ (cmake) | N10P 官方驱动，解析串口数据→/scan |
| n10p_bringup | Python (ament_python) | 飞控解析、里程计、WiFi桥接、启动配置 |
| n10p_slam | Python | SLAM 配置 + launch 文件 |
| n10p_nav | Python | Nav2 导航配置 + launch 文件 |
| n10p_gazebo | Python | Gazebo 仿真（URDF+世界+配置） |
| lslidar_msgs | C++ (cmake) | 驱动自定义消息类型 |
