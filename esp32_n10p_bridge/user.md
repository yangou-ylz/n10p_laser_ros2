# N10P 雷达 WiFi 无线转发 — 使用教程

> 最后更新: 2026-05-31

---

## 一、硬件接线

```
N10P 雷达 CH9102 模块 (TTL侧)     ESP32-S3 开发板
────────────────────────────     ──────────────
TX (数据输出) ────────────────→  IO18 (GPIO18)
GND          ────────────────→  GND
```

**只需要接 2 根线：TX 和 GND。**

- N10P 用自己的 USB 线供电（插电脑或充电头）
- ESP32 用自己的 USB 线供电（插电脑）

---

## 二、ESP32 固件烧录

**只需烧录一次，之后上电自动运行。**

```bash
# 1. 激活环境
source ~/esp/esp-idf/export.sh

# 2. 编译 + 烧录
cd /home/ubuntu22/ROS2/n10p_leishen/esp32_n10p_bridge
rm -rf build
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyACM0 flash
```

烧录时出现 `Connecting...` → 按住 ESP32 的 **BOOT** 键不放 → 点按 **RST** → 松开 BOOT。

---

## 三、ESP32 启动

烧录完成后 ESP32 自动重启。监视器确认状态：

```bash
idf.py -p /dev/ttyACM0 monitor
```

**预期输出**：
```
WiFi 已连接! IP: 192.168.0.184
TCP Server 监听端口 8888, 等待客户端连接...
=== 系统就绪 ===
UART: total=332 valid=332 (332fps) TCP发送=0 队列剩余=32
```

按 `Ctrl+]` 退出监视器。

---

## 四、电脑端运行

```bash
# 终端 1: 启动无线桥接节点
ros2env
python3 /home/ubuntu22/ROS2/n10p_leishen/esp32_n10p_bridge/n10p_wifi_bridge.py
```

**预期输出**：
```
已连接 ESP32 192.168.0.184:8888
帧: total=338 valid=338 fps=331 缓点=23
```

---

## 五、验证数据

```bash
ros2env
ros2 topic hz /scan
```
预期：`average rate: 10.000`

```bash
ros2 topic echo /scan --once --field ranges | head -5
```
预期：看到真实距离值，如 `5.888, 8.706, 1.794, 3.842`

---

## 六、RViz2 可视化

```bash
ros2env
rviz2
```

加载激光显示插件，选择话题 `/scan`，Fixed Frame 选 `laser_frame`。

---

## 七、有线/无线切换

| 模式 | 命令 |
|------|------|
| **无线** (ESP32) | `python3 esp32_n10p_bridge/n10p_wifi_bridge.py` |
| **有线** (原串口) | `ros2 launch lslidar_driver lslidar_launch.py` |

**两个不能同时运行**——都发布 `/scan` 话题会冲突。

---

## 八、故障排查

| 问题 | 检查 |
|------|------|
| ESP32 监视器无 N10P 数据 | TX 是否接 IO18, GND 是否连通, N10P 电机是否在转 |
| 桥接节点连不上 | ESP32 WiFi 是否连上 (看监视器), IP 是否正确 |
| /scan 无效距离全 inf | 雷达前方是否有物体 (最近 0.02m, 最远 12m) |
| 帧率偏低 (<300fps) | WiFi 信号是否弱 (RSSI < -60), 距离路由器是否太远 |
