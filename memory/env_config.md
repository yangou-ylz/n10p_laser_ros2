---
name: env-config
description: 环境配置快照
metadata:
  type: reference
---

# 环境配置

| 项目 | 值 |
|------|-----|
| 型号 | Raspberry Pi 4B Rev 1.5 |
| 架构 | aarch64 |
| 内存 | ~1.8GB |
| 系统 | Ubuntu 22.04.5 LTS Server |
| ROS2 | Humble, 244 包, /opt/ros/humble/ |
| 用户 | ylz |
| 项目根 | /home/ylz/n10p_leishen/ |
| 工作空间 | /home/ylz/n10p_leishen/n10p_ws/ |

## 环境激活
```bash
source /opt/ros/humble/setup.bash
source /home/ylz/n10p_leishen/n10p_ws/install/setup.bash
```

## 串口配置

| 端口 | 设备 | 波特率 | 用途 |
|------|------|--------|------|
| `/dev/ttyAMA0` | 树莓派硬件 UART (GPIO14/15) | 500000 | 凌霄飞控 |
| `/dev/ttyUSB0` | CH340/CP2102 USB 转串口 | 460800 | N10P 雷达（有线） |

### 飞控串口接线
```
飞控 PD2 (UART5_RX) → 树莓派 GPIO15 (RXD, 物理引脚10)
飞控 GND → 树莓派 GND (物理引脚6/9/14/20/25/30/34/39)
```

### 串口权限修复（每次重启后）
```bash
sudo chmod 0666 /dev/ttyAMA0
```

## /boot/firmware/config.txt 关键配置
```ini
dtoverlay=disable-bt
enable_uart=1
```
