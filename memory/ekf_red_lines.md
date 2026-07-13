---
name: ekf-red-lines
description: EKF 互补滤波方案 — 当前状态 + 红线约束
metadata:
  type: project
---

# EKF 融合方案状态

> 更新: 2026-07-13 19:30 | 状态: ✅ **Phase 3 建图验证通过, 推荐使用**

## 方案

- **节点**: `imu_filter_node` (Python 互补滤波器, ~180行)
- **算法**: IMU 陀螺仪积分 (高频预测) + 飞控四元数修正 (低频, alpha=0.02)
- **输入**: `/imu` + `/odom` (来自 ano_bridge_node)
- **输出**: `/odometry/filtered` + `odom→base_link` TF
- **验证**: 旋转建图测试 — 滤波方案地图无变形, 效果显著优于原始方案

## 使用方式

```bash
# 推荐 (有飞控): 传感器+滤波
ros2 launch n10p_bringup n10p_bringup_launch.py scan_source:=wired use_ekf:=true

# 配合 SLAM
ros2 launch n10p_slam slam_only_launch.py

# 配合 Nav2
ros2 launch n10p_nav nav_only_launch.py map:=/path/to/map.yaml

# 传统 (无飞控): 不使用滤波
ros2 launch n10p_slam slam_launch.py scan_source:=wired
```

## 红线

1. **不删除传统方案** (slam_launch.py, nav_launch.py, desktop_test_launch.py 全保留)
2. **不修改 ano_bridge_node.py** (零侵入)
3. **位置不融合** (FC 位置不可靠, AMCL 负责)
4. **use_ekf 目前仍是可选参数** (默认 false, 后续计划改 true)
5. **无需飞控时自动回退传统方案** (dummy_odom / keyboard_odom)

## 文件清单

| 文件 | 用途 |
|------|------|
| `n10p_fusion/n10p_fusion/imu_filter_node.py` | Python 互补滤波器 |
| `n10p_fusion/config/ekf.yaml` | 滤波参数 |
| `n10p_fusion/launch/ekf_odom_launch.py` | 独立启动 |
| `n10p_bringup/launch/n10p_bringup_launch.py` | 已加 use_ekf 参数 |
| `scripts/verify_ekf.sh` | 自动化验证 |
| `scripts/clean_ros2.sh` | 环境清理 |
| `scripts/pgm2png.py` | PGM→PNG 转换 |
| `rviz/n10p_nav_ekf.rviz` | RViz2 预设配置 |
