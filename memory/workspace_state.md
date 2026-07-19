---
name: workspace-state
description: 当前开发阶段、工作空间编译状态
metadata:
  type: project
---

# 工作空间状态

**更新**: 2026-07-14

## 当前阶段

Phase 8 — 导航验证 + 建图验证通过，待飞控下行硬件接线

## ⛔ 绝对红线（不可再犯）

1. **N10P 扫描方向**: `idx = (360-deg) * 1058 / 360` — 必须保留 CW→CCW 反转
   - 原因: N10P 电机顺时针旋转, ROS 约定逆时针。去掉反转 = Y轴镜像。
   - 2026-07-14 实测验证: 正前方=X+, 左侧=Y+, 方向正确
2. **后半圈角度**: `scan_points_[idx+3000].degree = point_deg + 180.0` — 不可删除
3. **scan_num 固定 1058** — 不可改回 count_num*2
4. **强度过滤 intensity>0** — 不可删除, 否则噪声点重新出现
5. **odom 协方差 0.001** — 飞控四元数 A 级可信, 不可改回 1.0
6. **TF laser_frame Z=+0.05** — 雷达在飞控上方 5cm
7. **静态 TF yaw=0** — 雷达箭头朝机头前方, 不需要旋转

## 7 ROS2 包

| 包 | 状态 |
|----|:--:|
| lslidar_msgs | ✅ |
| lslidar_driver | ✅ (CW→CCW反转+180°偏移+强度过滤+固定1058) |
| n10p_bringup | ✅ (ano_bridge: 50Hz, IMU限速100Hz, 协方差0.001, xyz=0) |
| n10p_slam | ✅ (slam_ekf_launch, minimum_laser_range=0.2) |
| n10p_nav | ✅ (nav_ekf_launch, AMCL自动初始位姿) |
| **n10p_fusion** | ✅ (imu_filter: 重力去除+死区+Z固定+自适应alpha+50Hz) |
| n10p_gazebo | ✅ (树莓派不编译) |

## 当前推荐用法

```bash
# 建图 (EKF)
ros2 launch n10p_slam slam_ekf_launch.py

# 导航 (EKF)
ros2 launch n10p_nav nav_ekf_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml
```

## 硬件

- 树莓派 4B, 8GB, Ubuntu 22.04.5 Server arm64
- N10P: /dev/serial/by-id/usb-1a86_USB_Single_Serial_58EB011256-if00, 460800bps
- 飞控: USB-TTL CH340, /dev/ttyUSB0, 500000bps

## 关键参数速查

| 参数 | 值 | 位置 |
|------|-----|------|
| scan_num | 1058 固定 | lslidar_driver.cc |
| 角度映射 | `idx=(360-deg)*1058/360` | lslidar_driver.cc |
| 后半圈 | degree+180° | lslidar_driver.cc data_processing_2() |
| 强度过滤 | intensity>0 | lslidar_driver.cc pubScan() |
| TF Z | +0.05 | bringup/slam/nav launch |
| TF yaw | 0 | bringup/slam/nav launch |
| odom 协方差 | 0.001 | ano_bridge_node.py |
| EKF alpha_ori | 0.02 (自适应) | ekf.yaml |
| EKF alpha_vel | 0.05 | ekf.yaml |
| EKF publish_rate | 50Hz | ekf.yaml |
| ano_bridge pub_rate | 50Hz | ano_bridge.yaml |
| IMU 限速 | 100Hz | ano_bridge + imu_filter |
| 重力去除 | 四元数旋转法 | imu_filter_node.py |
| 加速度死区 | 0.05 m/s² | imu_filter_node.py |
| Z轴 | 固定为0 | imu_filter_node.py |
