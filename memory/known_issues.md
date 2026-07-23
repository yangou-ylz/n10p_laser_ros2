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
