---
name: workspace-state
description: 当前开发阶段、工作空间编译状态
metadata:
  type: project
---

# 工作空间状态

**更新**: 2026-07-26

## ⭐ 当前基线 (v4.0-nofcyaw | 2026-07-26)

**导航跟踪与建图系统经过全面重构，已达成新的稳定基线。所有关键问题已确认修复，后续开发以本基线为准。**

### 本次重构解决的核心问题

1. **FC 偏航随机漂移** — FC 磁力计每次上电 yaw 不同 (-103° ~ -147°)，导致建图坐标系歪斜、AMCL 初始位姿错误
2. **速度轴间耦合** — FC 单轴飞行时另一轴有泄漏 (3-7 cm/s)，导致 RViz 斜线运动
3. **IMU 重力泄漏** — 倾斜时重力投影到水平轴，滤波速度被反向污染
4. **速度方向频繁反转** — FC 每次上电 yaw 不同导致的坐标系错乱
5. **AMCL 收敛漂移** — 粒子团过度紧密，扫描匹配无法纠正缓慢 odom 漂移
6. **调试效率** — 每次改参数需重编代码，YAML/build 混淆

### 解决方案总览

| 问题 | 方案 | 位置 |
|------|------|------|
| FC偏航随机漂移 | 输入端偏航归零: 启动后等2s取均值的init_yaw, 对FC四元数做Z轴旋转校正 | imu_filter |
| 轴间耦合 | 交叉轴抑制: 主轴>3×副轴时清零副轴 (ano_bridge+imu_filter双层) | ano_bridge, imu_filter |
| IMU重力泄漏 | IMU加速度不参与速度估计 (dv=0), 速度纯FC指数平滑 | imu_filter |
| 方向反转 | vx_sign/vy_sign YAML可配参数 (当前+1.0/+1.0) | ano_bridge |
| AMCL收敛漂移 | 似然场放宽(likelihood_max_dist=1.5)+粒子探索增强(alpha_slow=0.1) | nav2_params |
| 建图歪斜 | SLAM/nav启动不再依赖FC yaw, AMCL从(0,0,0°)启动 | slam_ekf + nav_ekf launch |
| YAML不生效 | 改YAML后必须colcon build, launch读的是install/目录 | 红线 |

## 当前数据流

```
N10P → lslidar_driver → /scan (1058点, 10Hz)
FC 串口 → ano_bridge:
  ├─ /fc_vel_raw (原始FC速度, 诊断用)
  ├─ /odom (vx_sign/vy_sign + 死区0.02 + 交叉轴抑制, 50Hz)
  └─ /imu (100Hz限速, FC四元数)
    │
    ▼
imu_filter:
  ├─ 输入端偏航归零 (init_yaw校正FC四元数)
  ├─ 姿态互补滤波 (alpha_ori=0.50慢转, 0.005快转)
  ├─ 速度纯FC指数平滑 (b=0.50)
  ├─ 交叉轴抑制 (二次)
  ├─ 位置积分 (从滤波速度)
  └─ 输出: /odometry/filtered + odom→base_link TF (100Hz)
    │
    ▼
AMCL: 似然场扫描匹配 + 粒子滤波
  └─ 输出: map→odom TF + /amcl_pose
```

## 当前推荐用法

```bash
# 建图
ros2 launch n10p_slam slam_ekf_launch.py

# 导航
ros2 launch n10p_nav nav_ekf_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml

# 诊断: 方向验证
python3 /home/ylz/n10p_leishen/scripts/diag_direction.py

# 诊断: 漂移根因
python3 /home/ylz/n10p_leishen/scripts/diag_drift_root.py
```

## ⛔ 绝对红线（不可再犯）

1. N10P 扫描方向: idx=(360-deg)*1058/360 (CW→CCW)
2. 双回波同角度, 非180°偏移
3. odom协方差(姿态)=0.001, (位置)=1.0
4. TF yaw(lidar)=0, Z=+0.05
5. 编译: --parallel-workers 2
6. **改YAML必须colcon build** — launch加载install/非src/
7. **不改FC yaw相关逻辑** — 偏航归零已在imu_filter输入端完成
8. **不改当前vx_sign/vy_sign=+1.0/+1.0** — FC坐标系已确认
9. **不对IMU加速度做速度积分** — dv=0, 速度纯靠FC平滑
10. **不删交叉轴抑制** — ano_bridge + imu_filter双层保留
11. **不改slam_ekf/nav_ekf launch** — FC yaw依赖已清除

## 7 ROS2包状态

| 包 | 状态 |
|----|:--:|
| lslidar_msgs | ✅ |
| lslidar_driver | ✅ (CW→CCW+双回波+强度过滤+1058点) |
| n10p_bringup | ✅ (vx/vy参数化+死区+交叉轴抑制+50Hz+fc_vel_raw) |
| n10p_slam | ✅ (不依赖FC yaw, slam_only_launch) |
| n10p_nav | ✅ (yaw=0°启动, AMCL参数增强, launch清理) |
| n10p_fusion | ✅ (偏航归零+FC平滑+alpha_ori=0.50+dv=0+交叉轴) |
| n10p_gazebo | ✅ (树莓派不编译) |

## 关键参数速查

| 参数 | 值 | 位置 |
|------|-----|------|
| vx_sign / vy_sign | +1.0 / +1.0 | ano_bridge.yaml |
| FC_VEL_DEAD_ZONE | 0.02 m/s | ano_bridge |
| X_DOMINANT | 3.0 (交叉轴抑制) | ano_bridge + imu_filter |
| init_yaw 延迟 | 2.0s + 50采样平均 | imu_filter |
| alpha_ori (慢转) | 0.50 | imu_filter |
| 速度 b (运动) | 0.50 | imu_filter |
| IMU dv | 0.0 (不积分) | imu_filter |
| AMCL max_particles | 1000 | nav2_params |
| AMCL alpha_slow | 0.1 | nav2_params |
| AMCL laser_likelihood_max_dist | 1.5m | nav2_params |
| AMCL sigma_hit | 0.4 | nav2_params |
| AMCL max_beams | 60 | nav2_params |
| AMCL recovery_alpha_fast | 0.8 | nav2_params |
| AMCL update_min_d/a | 0.01 | nav2_params |
| AMCL set_initial_pose | true (yaw=0) | nav2_params |
| SLAM ceres_num_threads | 2 | mapper_params |
| odom协方差(姿态) | 0.001 | imu_filter |
