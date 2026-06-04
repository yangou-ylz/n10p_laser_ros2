# 13 — ESP32 WiFi 桥接完整指南

## 硬件连接

```
N10P雷达 CH9102模块 (TTL侧)         ESP32-S3
─────────────────────────────       ───────────
TX (雷达→电脑) ─────────────────→  GPIO18 (UART1 RX)
GND            ─────────────────→  GND
```

- N10P USB插电脑或充电头（仅供电，不通信）
- ESP32-S3 通过 USB 插电脑供电和烧录
- 固件自动连WiFi + 启动TCP Server(8888)

## 固件编译与烧录（仅开发机）

```bash
source ~/esp/esp-idf/export.sh
cd ~/ROS2/n10p_leishen/esp32_n10p_bridge
idf.py build    # 编译
idf.py -p /dev/ttyUSB0 flash monitor  # 烧录+监视
```

烧录后ESP32上电自动运行，无需再编译。固件特性:
- 776KB/1MB分区，23%余量
- Core0: WiFi+TCP, Core1: UART接收
- 透传108字节N10P原始帧，TCP流式，无外层包装
- 帧率~332fps，CPU远未满载

## WiFi桥接节点使用

### 独立运行
```bash
ros2env
python3 n10p_wifi_bridge.py                     # 默认 192.168.0.184:8888
python3 n10p_wifi_bridge.py --host 192.168.1.5  # 指定IP
ros2 run n10p_bringup n10p_wifi_bridge_node     # ROS2方式
```

### 集成到Launch
```bash
ros2 launch n10p_slam slam_launch.py scan_source:=wireless
```

## N10P帧格式解析（关键参数）

| 字段 | 字节位置 | 字节序 | 类型 | 说明 |
|------|---------|--------|------|------|
| 帧头 | 0-1 | — | 0xA5 0x5A | 固定帧头 |
| 数据长度 | 2-3 | LE | uint16 | 108字节 |
| 起始角度 | 5-6 | **BE** | uint16 | 单位0.01° |
| 结束角度 | 105-106 | **BE** | uint16 | 单位0.01° |
| 点数据(×16) | 7-102 | — | 每点6字节 | dist(LE uint16 mm) + conf(LE uint16) |
| CRC | 107 | — | uint8 | 累加和取低8位 |

**关键**: 距离用小端 `<H`，角度用大端 `>H`。不可全用同一种字节序。

## 已验证性能指标

| 指标 | 实测值 |
|------|--------|
| TCP连接 | 192.168.0.184:8888 |
| 帧接收 | ~332 fps, CRC 100% |
| /scan发布 | 稳定 10.000Hz |
| 距离数据 | 0.27m ~ 11.02m |
| 帧格式 | frame_id=laser_frame, 360°完整 |
| 无线延时 | 比有线多2-5ms, 不影响SLAM |

## ESP-IDF环境搭建（仅开发机，国内网络）

```bash
# 克隆
mkdir -p ~/esp && cd ~/esp
git clone --depth 1 --branch v5.3.2 https://github.com/espressif/esp-idf.git

# 安装（国内镜像）
export IDF_GITHUB_ASSETS="dl.espressif.cn/github_assets"
cd ~/esp/esp-idf && bash install.sh esp32s3

# 子模块镜像
git config --global url."https://jihulab.com/esp-mirror/espressif/esp-idf".insteadOf "https://github.com/espressif/esp-idf"
git config --global url."https://jihulab.com/esp-mirror/espressif/esp32-wifi-lib".insteadOf "https://github.com/espressif/esp32-wifi-lib.git"
git config --global url."https://jihulab.com/esp-mirror/kmackay/micro-ecc".insteadOf "https://github.com/kmackay/micro-ecc.git"

# micro-ecc修复
git -c submodule."components/bootloader/subproject/components/micro-ecc/micro-ecc".update=none submodule update --init --recursive
rm -rf components/bootloader/subproject/components/micro-ecc/micro-ecc
git clone https://jihulab.com/esp-mirror/kmackay/micro-ecc components/bootloader/subproject/components/micro-ecc/micro-ecc
cd components/bootloader/subproject/components/micro-ecc/micro-ecc
git checkout 24c60e243580c7868f4334a1ba3123481fe1aa48
```
