---
name: known-issues
description: 已知坑点清单（绝对红线）
metadata:
  type: project
---

# 已知坑点

## 开发铁律

**每次改代码前必须读完全部记忆文件和 CLAUDE.md。**
**用户没让改代码时只分析不修改。**
**改一处的后果必须追溯到所有下游节点/话题/TF。**
**用户回退 git 后必须立即更新记忆文件，不要记忆错乱。**
**不轻易修改基线参数 — 改动前需要用户明确同意。**

## 2026-07-26 重构经验教训 (⭐ 最重要)

### 教训1: FC yaw不可用作绝对参考 ✓ 已解决
- **根因**: FC磁力计每次上电yaw不同(-103°~-147°), 导致建图坐标系歪斜/AMCL初始位姿错误
- **症状**: 建图时map旋转100+度, 导航AMCL箭头偏移20+, 速度方向每次开机不同
- **解决**: imu_filter输入端做偏航归零: 启动等2s→取50采样平均→FC四元数乘Z轴逆旋转
- **红线**: 以后绝对不把FC yaw当绝对参考, 不保存到文件, launch不调用yaw_util

### 教训2: YAML参数≠直接生效 ✓ 已解决
- **根因**: launch加载`install/`目录的YAML副本, 非`src/`中的源文件
- **症状**: 改YAML参数两小时后发现完全没生效, 方向一直反
- **解决**: 改YAML后必须`colcon build --packages-select <包名>`
- **红线**: 以后绝不告诉用户"改YAML不用编译", 这是完全错误的

### 教训3: IMU加速度不能用于平移速度 ✓ 已解决
- **根因**: 无人机倾斜时重力投影到水平轴, 重力泄漏无法被DEAD_ZONE完全过滤
- **症状**: 前飞时滤波速度vy方向反复反转 (FVX反/FVY反), 运动方向混乱
- **解决**: dv_x=dv_y=0, 速度完全由FC提供, imu_filter只做指数平滑
- **红线**: 绝不恢复IMU加速度到速度滤波的计算

### 教训4: 不能通过换odom_topic来修复AMCL ✓ 已解决
- **根因**: AMCL对odometry消息有特定的帧约定, 直接指向/odometry/filtered会出错
- **症状**: 改odom_topic后方向完全混乱, 前后左右全反
- **解决**: 在源头ano_bridge做交叉轴抑制, AMCL继续读/odom

### 教训5: 交叉轴耦合来自FC传感器本身 ✓ 已解决
- **根因**: FC光流传感器有交叉轴灵敏度, 单轴飞行时另一轴有3-7cm/s泄漏
- **症状**: 前飞RViz显示前+右, 左飞显示左+后
- **解决**: 双层交叉轴抑制(ano_bridge+imu_filter), 主轴>3×副轴→清零副轴

### 教训6: AMCL收敛后粒子过紧导致漂移不纠正 ✓ 已缓解
- **根因**: 粒子团太紧密, scan matching对缓慢odom漂移无响应
- **解决**: 放宽似然场(likelihood_max_dist=1.5, sigma_hit=0.4), 增强探索(alpha_slow=0.1)

### 教训7: 不要盲目调参数做补丁 — 先诊断根因
- 每次出问题应该运行诊断脚本抓数据, 看具体哪一层出错
- 不要猜原因就改代码, 越改越乱
- 改方向前先跑diag_direction.py确认FC/odom/filt三层符号是否一致

## 绝对红线

1. global_costmap 绝不用 rolling_window + obstacle_layer → SIGSEGV
2. Nav2 launch 必须有 map→odom bootstrap 静态TF
3. dummy_odom 和 keyboard_odom 不能同时运行
4. N10P 帧解析: 距离用 `<H`(小端), 角度用 `>H`(大端)
5. 树莓派编译必须 `--parallel-workers 2`
6. 禁止 `pkill -f "ros2"` 或 `killall ros2`
7. **所有文件只写在 /home/ylz/n10p_leishen/ 内，绝不写外部**
8. **N10P 扫描方向**: `idx=(360-deg)*1058/360` — CW→CCW反转
9. **odom 协方差 0.001** — 四元数A级可信
10. **TF yaw=0** — 雷达箭头朝机头前方
11. **未授权不准改代码** — 先说明改什么/为什么/影响，等用户明确说"改"
12. **改YAML必须colcon build** — launch加载install/非src/
13. **不改FC yaw相关逻辑** — 偏航归零已在imu_filter输入端完成
14. **dv_x=dv_y=0** — IMU加速度不参与速度估计
15. **不删交叉轴抑制** — 双层(ano_bridge+imu_filter)
16. **slam_ekf/nav_ekf launch不改动** — FC yaw依赖已清除

## 诊断脚本

| 脚本 | 用途 |
|------|------|
| `scripts/diag_direction.py` | 方向验证: 比较FC/odom/filt三层符号 |
| `scripts/diag_drift_root.py` | 漂移根因: TF+AMCL+速度全链数据 |
| `scripts/diag_cross_axis.py` | 轴间耦合: FC原始耦合vs滤波后 |
| `scripts/diag_stop_overshoot.py` | 急停过冲: 10Hz采样捕捉速度反向瞬态 |
| `scripts/diag_nav_tracking.py` | 导航跟踪: AMCL+TF+扫描匹配 |
| `scripts/diag_velocity.py` | 速度数据流: odom→filt→TF |
