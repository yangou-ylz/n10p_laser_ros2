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

26. **初始位置**: `nav_ekf_launch.py` 通过 `yaw_util.py` 自动从飞控获取 yaw 作为初始姿态
    - 订阅 `/odom` 话题，取飞控四元数→欧拉→yaw
    - 位置默认 (0,0)，由 AMCL 扫描匹配自动收敛
    - 如飞控未连接(yaw=0)，AMCL 也能通过扫描匹配收敛（但较慢）
27. **收敛监控**: `amcl_convergence.py` 实时显示粒子数、σ(X/Y/Yaw)、收敛状态
    - 收敛标准: σ<5cm 且 Yawσ<3° 持续 3 秒
28. **不要用暴力粒子数(10000+)** — 树莓派跑不动，已回退。max_particles=2000, min=500

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
39. `send_slam_cur_f5.py` 当前 cur=(None,None,None) — AMCL pose 数据未正确获取，待修复
