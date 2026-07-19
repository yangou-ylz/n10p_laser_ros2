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
10. **TF yaw=0** — 雷达箭头朝机头前方, 不需要旋转

## 飞控串口相关

8. 树莓派 UART 必须配置 `dtoverlay=disable-bt` 释放 ttyAMA0
9. 重启后 `/dev/ttyAMA0` 权限可能变回 `0620`，需 `sudo chmod 0666 /dev/ttyAMA0`
10. 飞控串口波特率是 **500000**（不是 921600），文档有误但实测确认
11. 0x02 帧 mag_sta 在偏移12，baro_sta 在偏移13（文档部分版本顺序有歧义）

## 飞控数据特点

12. **0x04 四元数 200Hz** — 是首选姿态来源，不要用 0x03 欧拉角（仅 2Hz）
    - 2026-07-12 实测: Yaw偏差 0.85°@90°, σ<0.03° → **A级可信, covariance=0.001**
13. **0x01 IMU 原始数据 501Hz** — 需要 acc_scale/gyr_scale 转换为物理单位
14. **0x08 XY_Pos 需要外部定位传感器才有效** — 无 GPS/UWB 时数值不可靠
    - ano_bridge 已将 odom→base TF 的 xyz 置零, 平移交给 AMCL 扫描匹配
15. 0x06 FC_Status 仅 3.4Hz — 不适合做高频状态判断
16. 电池 0x0D 显示 0.01V — 当前飞控无电池供电

## odom 协方差策略 (2026-07-12 修正)

- **位置 (x/y/z): 1.0** — 飞控无外部定位, xyz 已置零, 完全交给 AMCL
- **姿态 (roll/pitch/yaw): 0.001 (±1.8°)** — 四元数实测 A 级可信, AMCL 直接信任
- 历史: 曾因一次不严谨的手持旋转测试误判为"飞控不可信", 设为 1.0 (±57°),
  导致 AMCL 完全忽略飞控旋转信息。已修正。

## 串口权限

17. ttyAMA0 默认权限 `0620` (group=tty, write-only)，ylz 需在 tty 组且需读权限
18. 永久修复方案：添加 udev 规则 `KERNEL=="ttyAMA0", MODE="0666"`
