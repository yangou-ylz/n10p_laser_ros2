---
name: workspace-state
description: 当前开发阶段、工作空间编译状态
metadata:
  type: project
---

# 工作空间状态

**更新**: 2026-07-13

## 当前阶段

Phase 7 — EKF 互补滤波集成 ✅  (建图验证通过)
Phase 8 — 优化整合 (待开始)

## 关键成果

- **EKF 互补滤波**: 旋转建图地图无变形, 显著优于原始方案
- **方案**: Python imu_filter_node (互补滤波, alpha=0.02, 100Hz)
- **robot_localization**: ARM64 二进制有 NaN bug, 已放弃

## 7 ROS2 包

| 包 | 状态 |
|----|:--:|
| lslidar_msgs | ✅ |
| lslidar_driver | ✅ |
| n10p_bringup | ✅ (已加 use_ekf 参数) |
| n10p_slam | ✅ |
| n10p_nav | ✅ |
| n10p_gazebo | ✅ (树莓派不编译) |
| **n10p_fusion** | ✅ (新增, EKF 互补滤波) |

## 当前推荐用法

```bash
# 建图
ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wired use_ekf:=true
ros2 launch n10p_slam slam_only_launch.py

# 导航
ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wired use_ekf:=true
ros2 launch n10p_nav nav_only_launch.py map:=/path/to/map.yaml

# 传统方案 (无飞控, 仍可用)
ros2 launch n10p_slam slam_launch.py scan_source:=wired
```

## 硬件

- 树莓派 4B, 8GB, Ubuntu 22.04.5 Server arm64
- N10P 雷达: /dev/serial/by-id/usb-1a86_USB_Single_Serial_58EB011256-if00
- 飞控: USB-TTL CH340, /dev/ttyUSB0, 500000bps, 已校准 acc_scale=0.007198
