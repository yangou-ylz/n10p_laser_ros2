---
name: workspace-state
description: 当前开发阶段、工作空间编译状态
metadata:
  type: project
---

# 工作空间状态

**更新**: 2026-06-22

## 当前阶段

Phase 6 — 树莓派 4B 移植（进行中）

| 子任务 | 名称 | 状态 | 日期 |
|--------|------|:---:|------|
| 6.0 | 树莓派环境验证 | ✅ | 2026-06-04 |
| 6.1 | 编译验证 | ✅ | 2026-06-14 |
| 6.2 | **凌霄飞控串口驱动** | ✅ | **2026-06-22** |
| 6.3 | 有线雷达 | ⬜ | — |
| 6.4 | SLAM 建图 | ⬜ | — |
| 6.5 | 建图质量对比 | ⬜ | — |
| 6.6 | Nav2 导航 | ⬜ | — |
| 6.7 | 性能调优 | ⬜ | — |
| 6.8 | 文档更新 | ⬜ | — |

## 6.2 飞控串口驱动成果

### 三层架构（新建）

| 文件 | 行数 | 职责 |
|------|------|------|
| `ano_protocol.py` | 585 | 纯协议层：19 种帧描述符、SC/AC 校验、帧构建/解码 |
| `ano_transport.py` | 409 | 传输层：串口管理、后台线程、跳字节重同步、回调分发 |
| `ano_data_logger.py` | 398 | 独立调试工具：终端实时显示 + CSV 记录 |

### 重构文件

| 文件 | 变更 |
|------|------|
| `ano_bridge_node.py` | 删除 ~150 行内联代码，用 SerialTransport + 回调替代 |
| `dummy_odom_node.py` | 171→132 行，用 SerialTransport 替代内联串口代码 |
| `setup.py` | 新增 `ano_data_logger` entry_point |

### 修复的 Bug

- 0x02 帧 mag_sta/baro_sta 顺序颠倒 → 修正
- 校验失败丢弃整帧 → 跳 1 字节重同步
- 缺失 0x06 飞控状态、0x0D 电池 → 已添加
- 默认波特率 921600→500000（匹配文档）

### 实测验证

- 串口: `/dev/ttyAMA0` @ 500000 bps
- 需要 `dtoverlay=disable-bt` + `sudo chmod 0666 /dev/ttyAMA0`
- 10 秒内 14,607 帧，16 种帧类型，0 校验错误
- 核心数据: 四元数 200Hz, IMU 501Hz, 高度 50Hz, 速度 50Hz

## 树莓派环境

- 型号: Raspberry Pi 4 Model B Rev 1.5
- 内存: ~1.8GB (2GB)
- 存储: microSD 59GB
- 系统: Ubuntu 22.04.5 LTS Server (arm64)
- ROS2: Humble 244 包
- 用户: ylz
- **所有文件路径均在 /home/ylz/n10p_leishen/ 内**
