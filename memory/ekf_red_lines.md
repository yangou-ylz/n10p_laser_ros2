---
name: ekf-red-lines
description: EKF 互补滤波方案 — 当前基线状态 + 红线约束
metadata:
  type: project
---

# EKF 融合方案状态

> 更新: 2026-07-24 | 状态: ✅ **导航跟踪基线验证通过**

## 方案

- **节点**: `imu_filter_node` (Python 互补滤波器, ~280行)
- **算法**: IMU 陀螺仪积分 (高频预测) + 飞控四元数修正 (低频, 自适应 alpha=0.005~0.05)
  + IMU 加速度积分 (高频) + FC 速度修正 (低频, b=0.9静止/0.05运动)
- **输入**: `/imu` + `/odom` (来自 ano_bridge_node)
- **输出**: `/odometry/filtered` + `odom→base_link` TF (100Hz)
- **验证**: 2026-07-24 导航跟踪基线验证通过，三层速度防御体系确认有效

## 当前参数 (基线，不可随意改动)

| 参数 | 值 | 说明 |
|------|-----|------|
| alpha_ori | 0.02 (自适应 0.005~0.05) | 姿态互补系数，旋转越快越信IMU |
| alpha_vel | 0.05 (运动时) | 速度互补系数，移动时5%信任FC |
| **b (静止)** | **0.9** | FC速度<0.03时触发，速度0.5秒内归零 |
| **DEAD_ZONE** | **0.10 m/s²** | IMU加速度死区，低于此值归零 |
| publish_rate | 100Hz | TF + /odometry/filtered 发布频率 |
| FC_VEL_DEAD_ZONE | 0.02 m/s | ano_bridge 中 FC 速度死区（第1层防御） |

## 三层速度防御体系

```
[第1层] ano_bridge FC_VEL_DEAD_ZONE=0.02
          FC 速度 |v|<0.02 → 0，从源头切断噪声
             ↓
[第2层] imu_filter DEAD_ZONE=0.10
          IMU 加速度 |a|<0.10 → 0，消除传感器零偏
             ↓
[第3层] imu_filter b=0.9 (FC速度<0.03时)
          v_new = 0.1*(v_old+dv_imu) + 0.9*v_fc
          静止时 90% 信任 FC (报0)，速度快速归零
```

**迭代历史**:
- b=0.5 时: 运动后漂移 0.91 cm/s，速度归零需 ~10秒
- b=0.9 时: 运动后漂移 0.15 cm/s，速度归零 ~2-3秒
- **b=0.9+FC死区**: 运动后漂移 ≈0 cm/s，静止时位置冻结 ✅

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
2. **不修改 ano_bridge_node.py** (零侵入 — 注: FC死区和vy取反除外，这两处是有意修改)
3. **位置不融合** (FC 位置不可靠, AMCL 负责)
4. **use_ekf 目前仍是可选参数** (默认 false, 后续计划改 true)
5. **无需飞控时自动回退传统方案** (dummy_odom / keyboard_odom)
6. **三层速度防御参数不可回退** — DEAD_ZONE=0.10, b(静止)=0.9, FC_VEL_DEAD_ZONE=0.02 已验证

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
| `scripts/diag_velocity.py` | 速度数据流诊断 |
| `scripts/diag_nav_tracking.py` | 导航跟踪诊断 |
| `rviz/n10p_nav_ekf.rviz` | RViz2 预设配置 |
