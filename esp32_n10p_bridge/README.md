# ESP32-S3 N10P 雷达无线转发固件

## 硬件连接

```
N10P 雷达 TTL 串口        ESP32-S3 开发板
─────────────────        ────────────────
TX ────────────────────→ IO18 (UART1 RX)
RX ────────────────────→ IO17 (UART1 TX, 可选)
GND ───────────────────→ GND
5V  ───────────────────→ Vin (或 USB 独立供电)
```

## 编译

```bash
source ~/esp/esp-idf/export.sh
idf.py set-target esp32s3
idf.py build
```

## 烧录

```bash
idf.py -p /dev/ttyUSB0 flash monitor
```

/dev/ttyUSB0 是 ESP32-S3 开发板自带的 USB 转串口（用于烧录和监视器），
不是 N10P 雷达的串口。

## 监视器输出

烧录后 `idf.py monitor` 会显示：
- 每秒帧率统计（理论值约 1250 帧/秒）
- 有效帧率（通过 CRC8 校验的帧数）
- 帧同步丢失次数
- 最新一帧的头部字节

## Phase 计划

| Phase | 状态 | 说明 |
|-------|------|------|
| 1 | 开发中 | UART 接收验证 |
| 2 | 待开始 | WiFi TCP Server + 数据转发 |
| 3 | 待开始 | 电脑端 socat PTY + 驱动联调 |
| 4 | 待开始 | 端到端集成测试 + 性能对比 |
