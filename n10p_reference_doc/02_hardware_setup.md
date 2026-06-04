# 02 — 硬件配置

## 设备清单

| 设备 | 型号 | 接口 | 识别方式 |
|------|------|------|----------|
| 激光雷达 | 镭神智能 N10P | USB串口 (CH9102芯片) | `/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EB011256-if00` |
| 匿名数传（凌霄飞控） | ANO RadioLink | USB串口 | `/dev/serial/by-id/usb-ANO_TC_ANO_RadioLink-if00` |
| ESP32-S3 WiFi桥接 | 开发板 N16R8 | USB串口 + WiFi | 192.168.0.184:8888 (TCP) |
| 开发机 | x86_64 Ubuntu 22.04 | — | RTX 5060, 30GB RAM, 16核 |
| 目标机（树莓派4B） | arm64 | — | 4/8GB RAM, TF卡64G, SSD 512G |

## N10P 雷达规格

- 类型: 360° 单线激光雷达
- 量程: 0.02m ~ 12m
- 扫描频率: 10Hz（400ms/圈电机转速）
- 角分辨率: ~0.34°（1058点/圈）
- 数据接口: 串口 460800 bps
- 数据帧: 108 字节/帧, 16点/帧, ~332 fps
- 帧头: A5 5A
- CRC: 累加和取低8位（N10_CalCRC8）
- USB芯片: CH9102（部分批次为CP210x）

## 飞控协议（匿名协议V7）

| 帧ID | 内容 | 映射到ROS2话题 |
|------|------|---------------|
| 0x01 | IMU（加速度+陀螺仪） | /imu |
| 0x04 | 四元数（姿态） | /odom pose.orientation |
| 0x05 | 高度 | /odom pose.position.z |
| 0x07 | 速度 | /odom twist.linear |
| 0x08 | 位置 | /odom pose.position.x, y |

## ESP32 硬件连接

```
N10P 雷达 CH9102 模块 (TTL侧)          ESP32-S3 开发板
─────────────────────────────          ────────────────
TX (N10P 数据输出)  ───────────────→   GPIO18 (UART1 RX)
GND                  ───────────────→   GND
```

- UART1: IO18(RX), IO17(TX), 460800-8N1
- 固件: 776KB (1MB分区), 自动连接WiFi+启动TCP Server(8888)
- 数据: 透传108字节N10P原始帧, TCP流式传输, 无外层包装

## 串口注意事项

- 三个USB设备同时插入时 `/dev/ttyACM0` 会冲突，用 `lsusb` + `dmesg` 区分
- 永久权限：`sudo usermod -a -G dialout $USER`
- ESP32烧录按住BOOT→点按RST→松开BOOT进入烧录模式
