---
name: workspace-state
description: 当前开发阶段、工作空间编译状态
metadata:
  type: project
---

# 工作空间状态

**更新**: 2026-07-20

## 当前阶段

Phase 8 — 飞控 0xF5 下行联调中。SLAM+Nav+EKF 全部验证通过，自动串口识别已完成。
**当前攻坚**: `send_slam_cur_f5.py` 接入真实 AMCL 数据发送 0xF5 帧给飞控。

## ⛔ 绝对红线（不可再犯）

1. **N10P 扫描方向**: `idx = (360-deg) * 1058 / 360` — 必须保留 CW→CCW 反转
   - 原因: N10P 电机顺时针旋转, ROS 约定逆时针。去掉反转 = Y轴镜像。
   - 2026-07-14 实测验证: 正前方=X+, 左侧=Y+, 方向正确
2. **N10P 是双回波(Dual Echo)，不是双棱镜！** — 同一激光脉冲两次反射，角度相同、距离不同。
   - 两个回波角度 offset **必须为 0°**（不是 180°）。
   - 2026-07-20 修复: 错误认知曾导致 180° 镜像对称幽灵障碍物。
3. **scan_num 固定 1058** — 不可改回 count_num*2
4. **强度过滤 intensity>0** — 不可删除, 否则噪声点重新出现
5. **odom 协方差 0.001** — 飞控四元数 A 级可信, 不可改回 1.0
6. **TF laser_frame Z=+0.05** — 雷达在飞控上方 5cm
7. **静态 TF yaw=0** — 雷达箭头朝机头前方, 不需要旋转（2026-07-16 建图+导航双验证通过）
8. **未授权不准改代码** — 必须先向用户说明要改什么、为什么、有什么影响，等用户明确说"改"
9. **回退到 git commit 后** — 必须立即更新记忆文件，说明当前真实状态，不要记忆错乱

## 7 ROS2 包

| 包 | 状态 |
|----|:--:|
| lslidar_msgs | ✅ |
| lslidar_driver | ✅ (CW→CCW反转+双回波同角度+强度过滤+固定1058) |
| n10p_bringup | ✅ (ano_bridge: 50Hz, IMU限速100Hz, 协方差0.001, xyz=0) |
| n10p_slam | ✅ (slam_ekf_launch, 自动串口识别, minimum_laser_range=0.3) |
| n10p_nav | ✅ (nav_ekf_launch, 自动串口识别, AMCL动态粒子收敛) |
| **n10p_fusion** | ✅ (imu_filter: 四元数符号修正+重力去除+死区+Z固定+自适应alpha+50Hz) |
| n10p_gazebo | ✅ (树莓派不编译) |

## 当前推荐用法

```bash
# 建图 (EKF, 一键启动)
ros2 launch n10p_slam slam_ekf_launch.py

# 导航 (EKF, 一键启动)
ros2 launch n10p_nav nav_ekf_launch.py map:=/home/ylz/n10p_leishen/maps/n10p_map.yaml

# F5 下行测试 (需先启动导航)
cd /home/ylz/n10p_leishen
python3 send_slam_cur_f5.py --port /dev/ttyUSB0 --rate 10 --duration 30 --log-file logs/slam_cur_static.log

# AMCL 收敛监控
python3 /home/ylz/n10p_leishen/n10p_ws/scripts/amcl_convergence.py

# 自动串口检测
python3 /home/ylz/n10p_leishen/n10p_ws/scripts/auto_detect_serial.py
```

## 硬件

- 树莓派 4B, 8GB, Ubuntu 22.04.5 Server arm64
- N10P: `/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EB011256-if00`, 460800bps
- 飞控: USB-TTL CH340, `/dev/ttyUSB0` 或 `/dev/ttyUSB1`, 500000bps
- 自动串口检测脚本会根据 USB ID 自动识别 `/dev/ttyUSB0` vs `/dev/ttyUSB1`

## 关键参数速查

| 参数 | 值 | 位置 |
|------|-----|------|
| scan_num | 1058 固定 | lslidar_driver.cc |
| 角度映射 | `idx=(360-deg)*1058/360` | lslidar_driver.cc |
| 双回波角度 | echo1/echo2 同角度 (非 180°) | lslidar_driver.cc data_processing_2() |
| 强度过滤 | intensity>0 | lslidar_driver.cc pubScan() |
| minimum_laser_range | 0.3m (过滤无人机本体30cm) | mapper_params_online_async.yaml |
| TF Z | +0.05 | bringup/slam/nav launch |
| TF yaw | 0 | bringup/slam/nav launch |
| odom 协方差(姿态) | 0.001 | ano_bridge_node.py |
| odom 协方差(位置) | 1.0 (交给AMCL) | ano_bridge_node.py |
| EKF alpha_ori | 0.02 (自适应) | ekf.yaml |
| EKF alpha_vel | 0.05 | ekf.yaml |
| EKF publish_rate | 50Hz | ekf.yaml |
| ano_bridge pub_rate | 50Hz | ano_bridge.yaml |
| IMU 限速 | 100Hz | ano_bridge + imu_filter |
| 0x04 四元数符号 | pitch/yaw 取反(与0x03对账确认) | ano_bridge_node.py |
| 重力去除 | 四元数旋转法 | imu_filter_node.py |
| 加速度死区 | 0.05 m/s² | imu_filter_node.py |
| Z轴 | 固定为0 | imu_filter_node.py |
| AMCL max_particles | 2000 (收敛后自动降为500) | nav2_params_n10p.yaml |
| AMCL min_particles | 500 | nav2_params_n10p.yaml |
| AMCL 收敛阈值 | σ<5cm且Yawσ<3°持续3秒 | amcl_convergence.py |

## 0xF5 下行联调状态

| 步骤 | 内容 | 状态 |
|------|------|:--:|
| 1 | 离线帧测试 (黄金帧校验) | ✅ |
| 2 | 串口模块 (linux_serial.py, send_f5.py) | ✅ |
| 3 | 固定帧发送 + STM32 0xA0 ACK | ✅ |
| 4 | 方向测试 X/Y/Z 三轴 | ✅ 全部ACK, 0丢帧 |
| 5 | flags 失效测试 | ✅ |
| 6 | 接入真实SLAM (send_slam_cur_f5.py) | ✅ 2026-07-19 19:46 测试通过 |
| 7 | 速率验证 (10/30/50Hz) | ✅ 2026-07-20 测试通过 |
| 8 | 移动测试 (验证各轴方向和单位) | ⏳ 待执行 |
| 9 | 自动串口识别 (send_slam_cur_f5.py) | ✅ 2026-07-20 已集成 |

### 步骤6 测试结果 (2026-07-19 19:46)

- **测试命令**: `python3 send_slam_cur_f5.py --port /dev/ttyUSB0 --rate 10 --duration 30`
- **发送帧数**: 298 帧, 飞控 ACK 100% 响应 (RK #1153→#1450), 0 丢帧
- **f=00 阶段**: TX #1~#49 (约 5 秒), AMCL 初始化中, 坐标 sentinel=-2147483648
- **f=01 阶段**: TX #50~#298 (约 25 秒), AMCL 收敛后
- **静态坐标 (f=01, 554 数据点)**:

| 轴 | min | max | mean | 波动 |
|-----|------|------|------|-----|
| X | -8 cm | -6 cm | -6.5 cm | 2 cm |
| Y | 50 cm | 63 cm | 57.6 cm | 13 cm |
| Z | 0 cm | 0 cm | 0.0 cm | 0 cm |

- **异常跳变**: 无, 无突然十几米的情况
- **ACK 形态**: `f=01 c=<x>,<y>,<z> t=<same>` — 符合期望

### 步骤7 速率测试结果 (2026-07-20)

**修复**: `time.sleep(interval)` → 绝对时刻调度 `next_t += interval`，补偿工作时间。

| 目标 | 修复前 | 修复后 |
|------|--------|--------|
| 10Hz | ~10.1 Hz | — |
| 30Hz | ~29.4 Hz | — (TX端正常，接收端显示10Hz为飞控侧问题) |
| 50Hz | ~48.6 Hz (-2.8%) | **~50.0 Hz (±0.4%)** |

**⚠️ 集成到导航时的红线**: 禁止 `time.sleep()` 做定时发送，必须用绝对时刻调度。详见 [[known-issues]]#42。

### 步骤8 待办 (移动测试)

飞控要求移动测试: 向前/左/上移动时验证哪个轴增加、单位是否接近真实 cm。
**飞控方原话**: "收到这一步真实 SLAM 日志前，不继续写 PID、误差干运行或控制输出。"

## 已知当前 Bug

1. **`send_slam_cur_f5.py` 启动时序**: 前 ~5 秒 `cur=(None,None,None)` — 因为脚本先于 AMCL 首条消息启动。属正常时序问题，不影响后续使用。AMCL 初始化约 2-5 秒后正常发布数据。
2. **AMCL 零协方差误判收敛**: `/amcl_pose` 初始协方差全零时 `is_slam_valid()` 返回 True（`sqrt(0) < 0.3`），但零协方差表示粒子未分散而非真正收敛。暂不影响测试结果。
3. **路径规划偶尔失败**: 待排查 Nav2 planner 配置。
