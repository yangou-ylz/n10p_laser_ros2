---
name: ekf-red-lines
description: EKF 互补滤波方案 — v4.0 基线参数 + 红线约束
metadata:
  type: project
---

# EKF 融合方案状态

> 更新: 2026-07-26 | 状态: ✅ **v4.0 基线 — FC偏航解耦+速度纯FC平滑+AMCL增强**

## 当前架构

- **节点**: `imu_filter_node` (Python)
- **姿态**: 互补滤波 — 陀螺仪积分 + FC四元数修正 (输入端偏航归零, alpha_ori=0.50慢转)
- **速度**: 纯FC指数平滑 — IMU加速度不参与 (dv=0, b=0.5)
- **位置**: 从滤波速度积分, AMCL负责绝对修正
- **输入**: `/imu` + `/odom` (来自 ano_bridge)
- **输出**: `/odometry/filtered` + `odom→base_link` TF (100Hz)

## 当前参数 (v4.0基线，不可随意改动)

| 参数 | 值 | 说明 |
|------|-----|------|
| alpha_ori (慢转) | 0.50 | 静止/慢转时50%信任FC四元数 |
| alpha_ori (快转) | 0.005 | 快速旋转时更信陀螺仪 |
| 速度 b | 0.50 | 纯FC指数平滑 |
| dv_x, dv_y | 0.0 | IMU加速度不参与速度估计 |
| IMU DEAD_ZONE | 0.10 | 仅用于重力检查, 不影响dv (dv=0) |
| X_DOMINANT | 3.0 | 交叉轴抑制阈值 |
| init_yaw 延迟 | 2.0s + 50采样 | 启动后等磁力计稳定取平均 |

## 数据流

```
FC → ano_bridge → /odom (vx_sign/vy_sign + 死区 + 交叉轴)
                        │
                        ▼
                  imu_filter
                        │
     FC四元数: 偏航归零(init_yaw) → 互补滤波(alpha=0.50/0.005) → 姿态
     FC速度:   交叉轴抑制 → 指数平滑(b=0.5) → vel_filt → 积分→位置
                        │
                        ▼
            /odometry/filtered + odom→base_link TF (yaw≈0基准)
```

## 红线

1. **FC yaw不进入系统** — 偏航归零在输入端完成, launch不调用yaw_util
2. **IMU加速度不用于速度** — dv_x=dv_y=0, 永久保持
3. **交叉轴抑制保留** — ano_bridge+imu_filter双层不删
4. **不改当前vx_sign/vy_sign** — +1.0/+1.0已确认
5. **改YAML必须colcon build** — launch读的是install/目录
