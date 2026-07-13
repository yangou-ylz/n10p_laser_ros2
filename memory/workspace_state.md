---
name: workspace-state
description: 当前开发阶段、工作空间编译状态
metadata:
  type: project
---

# 工作空间状态

**更新**: 2026-07-12

## 当前阶段

Phase 7 — 飞控 0xF5 下行通信 (树莓派→STM32)

## 目标

树莓派融合 SLAM 定位 + K230 视觉 → 打包为 0xF5 自定义帧(31B)
→ 串口发飞控 → STM32 PID 位置控制

## 已完成

| Phase | 名称 | 状态 |
|-------|------|:--:|
| 6.0 | 环境验证 | ✅ |
| 6.1 | 编译验证 | ✅ |
| 6.2 | 凌霄飞控串口驱动 | ✅ |
| 6.3 | 有线雷达 (scan修复) | ✅ |
| 6.4 | SLAM建图 | ⚠️ |
| 6.5  | Nav2导航 (含nav_only_launch.py) | ✅ |
| 6.6 | **odom协方差修正** (实测0.85°@90°→0.001) | ✅ |
| 7.0 | 0xF5 下行帧 | ✅ (暂停, 待硬件接线) |

## 0xF5 帧格式 (31 字节)

```
[0]=0xAA [1]=0x61 [2]=0xF5 [3]=0x19
[4-7] cur_x s32 LE cm   [8-11] cur_y  [12-15] cur_z
[16-19] tar_x s32 LE cm  [20-23] tar_y [24-27] tar_z
[28] flags (bit0=SLAM_VALID bit1=TARGET_VALID bit2=VISUAL_MODE)
[29] SC [30] AC — 校验覆盖 [0]~[28]
```

## 硬件串口

- 树莓派 GPIO14(TXD) → STM32 PD6(UART2 RX)
- 波特率: 500000
- 校验: SC/AC 算法

## 树莓派环境

- 型号: Raspberry Pi 4B, 8GB
- 系统: Ubuntu 22.04.5 LTS Server (arm64)
- ROS2: Humble 244包
- 用户: ylz
