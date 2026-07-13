---
name: downlink-progress
description: 0xF5 下行通道开发进度 — 暂停中
metadata:
  type: project
---

# 0xF5 下行通道开发进度

**状态**: 暂停 (2026-07-12)  
**暂停原因**: 用户需要先处理其他事务  
**恢复时从这里继续**: Phase 7 — 硬件接线 + AMCL 建图后启用下行

## 已完成的代码

| 文件 | 变更 | 测试结果 |
|------|------|:--:|
| `rpi_pos_frame.py` | 0xF5 31B 帧构造/校验/解析/双模式 | 10/10 单元测试 |
| `ano_transport.py` | `send_raw()` 线程安全串口写入 | - |
| `ano_bridge_node.py` | AMCL 订阅, 双模式下行 (航点+视觉占位) | 编译通过 |
| `ano_bridge.yaml` | 下行启用/模式/航点参数 | - |
| `CLAUDE.md` §12 | 架构文档 | - |
| `scripts/test_f5_pressure.py` | 50Hz 压力测试 | 10000帧 PASS |

## 50Hz 压力测试结果

- 10000 帧 @ 50.0Hz, 200 秒持续发送
- 抖动 <0.8ms, 零丢帧, 零校验错误
- 结论: 树莓派完全胜任 50Hz 0xF5 下行

## 恢复时需要做的事

1. 硬件接线: GPIO14(TXD)→飞控PD6(UART2 RX), GND共地
2. 持雷达直走建图 → 保存地图
3. 启动 bringup + Nav2 (AMCL 定位) → 确认 `/amcl_pose` 有输出
4. 改 `ano_bridge.yaml`: `pos_downlink_enable: true`
5. 重启 bringup → 飞控侧监控接收帧
6. K230 接入后: 改 `pos_downlink_mode: visual` + 订阅 `/k230_detection`
