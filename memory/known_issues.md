---
name: known-issues
description: 已知坑点清单（绝对红线）
metadata:
  type: project
---

# 已知坑点

## 开发铁律 (2026-07-19)

**每次改代码前必须读完全部记忆文件和 CLAUDE.md。**
**用户没让改代码时只分析不修改。**
**改一处的后果必须追溯到所有下游节点/话题/TF。**
**四元数姿态相关的改动涉及 4 个位置：ano_bridge、imu_filter 输入、imu_filter 输出(TF+odom)、yaw_util。**
**用户回退 git 后必须立即更新记忆文件，不要记忆错乱。**

## ⭐ 导航跟踪基线铁律 (2026-07-24 新增)

**导航实时跟踪已于 2026-07-24 首次验证通过，以下参数和架构已经过三轮迭代验证，不可回退：**

### 三层速度防御体系（完整链路）

```
[第1层] ano_bridge: FC_VEL_DEAD_ZONE=0.02 → 静止时 /odom 速度严格为0
    ↓
[第2层] imu_filter: DEAD_ZONE=0.10 → IMU加速度噪声<0.10m/s²时归零
    ↓
[第3层] imu_filter: b=0.9(静止时) → 速度在0.5秒内衰减到FC参考值
```

**迭代历史（不可倒退）**:
- 第1轮: DEAD_ZONE=0.05, b=0.05 → 静止漂移 0.5 cm/s ❌
- 第2轮: DEAD_ZONE=0.10, b=0.5(静止) → 运动后漂移 0.91 cm/s ❌
- 第3轮: DEAD_ZONE=0.10, b=0.9(静止) → 运动后漂移 0.15 cm/s ⚠️
- **第4轮 (基线): DEAD_ZONE=0.10, b=0.9, FC_VEL_DEAD_ZONE=0.02 → 运动后漂移 ≈0 cm/s ✅**

### 怀疑优先级（出问题时按此顺序排查）

1. 先怀疑传感器数据（FC速度、IMU加速度是否有异常）
2. 再怀疑参数漂移（DEAD_ZONE、b值是否需要微调）
3. 然后怀疑 AMCL 扫描匹配（update_min_d/a 是否需要调整）
4. **最后才怀疑根本架构**（三层防御体系本身）

**绝不怀疑的根本逻辑**: FC速度→ano_bridge→/odom→imu_filter→/odometry/filtered→TF 这条数据流是正确的。

## 绝对红线

1. global_costmap 绝不用 rolling_window + obstacle_layer → SIGSEGV
2. Nav2 launch 必须有 map→odom bootstrap 静态TF
3. dummy_odom 和 keyboard_odom 不能同时运行
4. N10P 帧解析: 距离用 `<H`(小端), 角度用 `>H`(大端)
5. 树莓派编译必须 `--parallel-workers 2`
6. 禁止 `pkill -f "ros2"` 或 `killall ros2`
7. **所有文件只写在 /home/ylz/n10p_leishen/ 内，绝不写外部**
8. **N10P 扫描方向**: `idx=(360-deg)*1058/360` — CW→CCW反转，不可改成 `deg*1058/360`
9. **odom 协方差 0.001** — 四元数A级可信, 不可改回1.0
10. **TF yaw=0** — 雷达箭头朝机头前方, 不需要旋转（2026-07-16 双验证通过）
11. **未授权不准改代码** — 先说明改什么/为什么/影响，等用户说"改"
12. **三层速度防御不可删除或弱化** — DEAD_ZONE=0.10, b=0.9, FC_VEL_DEAD_ZONE=0.02 已验证
13. **改 YAML 参数后必须 `colcon build` !!** — launch 加载的是 `install/` 里的文件，不是 `src/` 里的。改完源文件不构建等于没改。构建完用 `grep` 确认 install 里的文件内容。

## 飞控串口相关

12. 树莓派 UART 必须配置 `dtoverlay=disable-bt` 释放 ttyAMA0
13. 重启后 `/dev/ttyAMA0` 权限可能变回 `0620`，需 `sudo chmod 0666 /dev/ttyAMA0`
14. 飞控串口波特率是 **500000**（不是 921600），文档有误但实测确认
15. 0x02 帧 mag_sta 在偏移12，baro_sta 在偏移13（文档部分版本顺序有歧义）

## 飞控数据特点

16. **0x04 四元数 200Hz** — 是首选姿态来源，不要用 0x03 欧拉角（仅 2Hz）
    - 2026-07-12 实测: Yaw偏差 0.85°@90°, σ<0.03° → **A级可信, covariance=0.001**
    - **2026-07-19 重要修正**: 0x04 四元数转欧拉后，pitch 和 yaw 符号与凌霄 0x03 直出帧相反（与飞控方对账确认）。ano_bridge_node.py 已修正符号：pitch/yaw 取反。
17. **0x01 IMU 原始数据 501Hz** — 需要 acc_scale/gyr_scale 转换为物理单位
18. **0x08 XY_Pos 需要外部定位传感器才有效** — 无 GPS/UWB 时数值不可靠
    - ano_bridge 已将 odom→base TF 的 xyz 置零, 平移交给 AMCL 扫描匹配
19. 0x06 FC_Status 仅 3.4Hz — 不适合做高频状态判断
20. 电池 0x0D 显示 0.01V — 当前飞控无电池供电

## odom 协方差策略 (2026-07-12 修正, 2026-07-16 再次确认)

- **位置 (x/y/z): 1.0** — 飞控无外部定位, xyz 已置零, 完全交给 AMCL
- **姿态 (roll/pitch/yaw): 0.001 (±1.8°)** — 四元数实测 A 级可信, AMCL 直接信任
- 历史: 曾因一次不严谨的手持旋转测试误判为"飞控不可信", 设为 1.0 (±57°),
  导致 AMCL 完全忽略飞控旋转信息。已修正。

## 坐标系相关 (2026-07-16 最终确认)

21. **N10P 雷达箭头 = 0° 方向 = 飞控 X 正方向 = 无人机机头前方**
22. **ROS 约定: X=前, Y=左, Z=上 (REP-105)**
23. **TF static_transform base_link→laser_frame: z=+0.05, 无旋转(yaw=0)**
24. RViz 中: 红色=X轴(前), 绿色=Y轴(左), 蓝色=Z轴(上)
25. 建图时 RViz 显示的"镜像感"是正常的 — RViz 默认视角从上往下看，配合 REP-105 坐标系实际正确。

## AMCL 初始姿态 (2026-07-19)

26. **初始姿态公式 (2026-07-24 修正)**: `nav_ekf_launch.py` 通过 `yaw_util.py` 获取初始姿态。
    - **旧公式 (错误)**: `initial_yaw = nav_yaw - slam_yaw` — 计算的是 FC 漂移量
    - **新公式 (正确)**: `initial_yaw = nav_yaw` — 直接用 FC 当前 yaw
    - **根因**: slam_toolbox 建图以 odometry 位姿为起点（已包含 FC yaw），
      地图中机器人 yaw ≈ FC 当前值，不是 0°。减去 slam_yaw 反而把正确朝向减掉了。
    - 位置默认 (0,0)，由 AMCL 扫描匹配自动收敛
27. **收敛监控**: `amcl_convergence.py` 实时显示粒子数、σ(X/Y/Yaw)、收敛状态
    - 收敛标准: σ<5cm 且 Yawσ<3° 持续 3 秒
28. **不要用暴力粒子数(10000+)** — 树莓派跑不动，已回退。max_particles=500, min=100

## 路径规划 (2026-07-19)

29. 路径规划偶尔失败 — 待排查。可能原因: costmap 配置、障碍物检测范围、planner 参数。
30. minimum_laser_range 设为 0.3m 过滤无人机本体（30cm×30cm 正方形）。

## 串口权限

31. ttyAMA0 默认权限 `0620` (group=tty, write-only)，ylz 需在 tty 组且需读权限
32. 永久修复方案：添加 udev 规则 `KERNEL=="ttyAMA0", MODE="0666"`
33. USB-TTL 串口可能互换 — 已实现自动串口检测 `auto_detect_serial.py`

## 自动串口检测 (2026-07-19 新增)

34. `/dev/ttyUSB0` 和 `/dev/ttyUSB1` 可能互换（取决于上电顺序）
35. `auto_detect_serial.py` 根据 USB ID 自动匹配:
    - CH340 → 飞控串口
    - CP2102 → N10P 雷达
36. SLAM 和 Nav 的 launch 文件已集成自动检测

## F5 下行联调 (2026-07-19)

37. 帧格式: 31B, AA 61 F5 19, 6×s32 LE cm + flags + SC/AC
38. 速率: 10Hz 测试通过, 目标 50Hz
39. `send_slam_cur_f5.py` 启动时序: 前 ~5 秒 cur=(None,None,None) 是 AMCL 初始化延迟, 非 bug。约 5 秒后自动恢复正常。
40. **AMCL 零协方差误判**: 初始 pose 协方差全零时 `sqrt(0) < 0.3` → `is_slam_valid()` 返回 True, 但零协方差表示粒子未分散, 非真正收敛。暂不阻塞测试, 后续可考虑加 `cov > 0` 判断。
41. 2026-07-19 静态测试: 298 帧 100% ACK, f=00→f=01 转换正常, 坐标波动 X±2cm Y±13cm Z=0, 无异常跳变。

## 定时发送速率控制 (2026-07-20 新增)

42. **禁止 `time.sleep(interval)` 做固定频率发送**。`sleep` 不补偿 `send_frame()` 等工作耗时，50Hz 实际只有 ~48.6Hz（-2.8%）。正确做法: 按绝对时刻调度 `next_t += interval` + `sleep(max(0, next_t - now()))`，落后超一个周期时重置基准防止追赶螺旋。
43. 50Hz 测试通过: 修复后 50.0Hz±0.4%, 每个 50 帧间隔恰好 1.000 秒。30Hz TX 端 ~29.4Hz 正常，接收端若显示 10Hz 是飞控侧采样频率问题。

## N10P 雷达驱动认知修正 (2026-07-20)

44. **N10P 是双回波(Dual Echo)，不是双棱镜！** 这是之前最严重的认知错误。N10P 采用传统旋转电机单头扫描，两个距离值是同一激光脉冲打在**同一角度**先后收到的两次反射（如先打到窗户→再打到墙），不是两个对向安装的棱镜。
45. **180° 偏移是致命 bug**: 原代码 `scan_points_[idx+3000].degree = point_deg + 180.0` 将 echo2 放到相反方向，造成扫描点云 180° 镜像对称，SLAM 建图产生幽灵 L 形障碍物。修复: echo2 角度 = echo1 角度（同方向），保留近距离优先。
46. **echo1/echo2 独立验证**: 原代码只检查 echo1 是否 0xFFFF，echo1 无效时连有效的 echo2 也丢弃，导致 ~50% 扫描点丢失（交替 inf 模式）。修复: 两个回波各自独立判断有效性。
47. **历史教训**: 代码注释和记忆文件中"双棱镜"、"后半圈"、"前后半圈"等词都是错误认知的产物。看到这些词应立即联想到本次 bug。

## AMCL 初始姿态公式修正 (2026-07-24)

48. **`initial_yaw = nav_yaw - slam_yaw` 是错误公式！** 原以为 `initial_yaw` 是"偏移量"（FC 漂移了多少），实际 AMCL 的 `initial_pose.yaw` 接收的是机器人在 map 帧里的**绝对 yaw**。
49. **为什么旧公式错**: slam_toolbox 建图时以**当前 odometry 位姿**为起点，地图建立时机器人 yaw 已经包含了 FC 的 yaw 值（~113°）。`nav_yaw - slam_yaw` 把 FC 的 yaw 减掉了，只留下 FC 漂移的那一点点（~-1°），相当于告诉 AMCL"机器人只转了 1°"，实际机器人转了 113°。
50. **正确公式**: `initial_yaw = nav_yaw` — 直接用 FC 当前 yaw 值。建图和导航时机器人物理朝向不变 → FC 报的 yaw 也基本不变 → 直接用 nav_yaw 即对齐。
51. **验证**: 硬编码 116.6°（≈FC 当时的 yaw）完美吻合，证明 nav_yaw 就是正确值。
52. **教训**: 改任何公式前必须先搞清楚变量的物理含义——`initial_pose.yaw` 是绝对朝向，不是偏移量。

## 导航跟踪基线 — 速度漂移修复全记录 (2026-07-24)

53. **原始问题**: 无人机物理移动时 RViz 几乎不动，点云往后退。原因: ano_bridge 故意把 odom→base_link 平移置零，平移跟踪完全依赖 AMCL 扫描匹配（~1Hz），跟不上实际移动。
54. **修复策略**: 让 imu_filter 用 IMU 加速度积分 + FC 速度修正来提供高频平移估计，而非仅依赖 AMCL。
55. **第1次诊断**: FC 速度能到达 imu_filter，但 odom_vel 初始静止段全零（QoS 不匹配）。
56. **第2次诊断 (DEAD_ZONE=0.10, b=0.5)**: 运动后漂移 0.91 cm/s，filt_vel 运动后 10 秒仍未归零。
57. **第3次诊断 (DEAD_ZONE=0.10, b=0.9)**: 运动后漂移降至 0.15 cm/s（6倍改善），但 FC 自身有 0.01-0.02 m/s 噪声。
58. **第4次诊断 (DEAD_ZONE=0.10, b=0.9, FC_VEL_DEAD_ZONE=0.02)**: 静止时 odom_vel 和 filt_vel 均为全零，位置冻结，漂移 ≈0 cm/s。**基线确立。**
59. **AMCL 阈值同步调整**: update_min_d 0.02→0.01, update_min_a 0.02→0.01，对微小运动灵敏度翻倍。
60. **vy 方向确认**: ano_bridge `_on_velocity` 中 `vel_y = -d['vel_y_cms']*0.01`，光流传感器校正后方向正确，已确认为正式逻辑（非临时补丁）。

## 交叉轴耦合抑制 (2026-07-25)

61. **原始问题**: 单轴飞行时 FC 原始速度存在轴间耦合（前飞时 vy=0.03-0.07, 左右飞时 vx=0.03-0.06），导致 RViz 显示对角线漂移。
62. **根因**: FC 光流传感器存在交叉轴灵敏度，飞控用闭源算法抑制了此耦合，但我们的 ROS2 滤波链未做处理。
63. **修复**: imu_filter 中新增交叉轴抑制——当 |vx| > 3×|vy| 时清零 FC vy 参考值，反之亦然。阈值 3:1 保证真正的斜向飞行不受影响。
64. **效果 (飞行实测)**: 前飞 Y 偏 1.7cm→0.1cm (17倍), 右飞 X 偏 10.2cm→1.3cm (7.8倍), 左飞 X 偏 7.6cm→2.1cm (3.6倍)。
65. **诊断方法**: 脚本 `diag_cross_axis.py` 抓取 `/fc_vel_raw` (死区前) 和滤波后数据, 计算 coupling_ratio 实时显示轴间耦合。
