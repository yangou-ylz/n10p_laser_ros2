---
name: downlink-progress
description: 0xF5 下行通道联调进度 — 树莓派↔STM32 同步开发
metadata:
  type: project
---

# 0xF5 下行通道联调进度

**更新**: 2026-07-19

## 架构

```
树莓派 AMCL (/amcl_pose) → send_slam_cur_f5.py → 0xF5帧(31B) → /dev/ttyUSB0 → STM32 → 0xA0 ACK
```

## 已完成

| 步骤 | 内容 | 状态 |
|------|------|:--:|
| 1 | 离线帧测试 (黄金帧校验) | ✅ |
| 2 | 串口模块 (linux_serial.py, send_f5.py) | ✅ |
| 3 | 固定帧发送 + STM32 0xA0 ACK | ✅ |
| 4 | 方向测试 X/Y/Z 三轴每轴 30 帧 | ✅ 全部ACK, 0丢帧 |
| 5 | flags 失效测试 (SLAM_INV/TARGET_INV) | ✅ |
| 6 | 接入真实SLAM (send_slam_cur_f5.py) | ✅ 2026-07-19 测试通过 |
| 7 | 移动测试 (验证各轴方向和单位) | ⏳ 待执行 |
| 8 | 50Hz 速率验证 (send_slam_cur_f5.py) | ✅ 2026-07-20 测试通过 |

## 步骤8 速率测试结果 (2026-07-20)

修复了 `time.sleep(interval)` 不补偿工作时间的 bug。

| 目标速率 | 修复前 | 修复后 | 方法 |
|----------|--------|--------|------|
| 10Hz | ~10.1 Hz | — | 误差可忽略 |
| 30Hz | ~29.4 Hz | — | 误差可忽略 |
| 50Hz | ~48.6 Hz (-2.8%) | **~50.0 Hz (±0.4%)** | 绝对时刻调度 |

**根因**: `time.sleep(interval)` 忽略了 `send_frame()` 耗时（~0.5ms），累积后 50Hz 慢 2.8%。

**修复**: 改为按绝对时刻 `next_t += interval` 调度，补偿工作时间：
```python
next_t += interval
sleep_time = next_t - time.monotonic()
if sleep_time > 0: time.sleep(sleep_time)
elif sleep_time < -interval: next_t = time.monotonic() + interval  # 防追赶螺旋
```

**⚠️ 集成到导航时必须用相同方式**，不可用固定 `time.sleep()`。

## 联调结果 (步骤 3-5)

- 帧格式: 31B, AA 61 F5 19, 6×s32 LE cm + flags + SC/AC
- 速率: 10Hz 发送, 100% ACK
- 坐标方向: X=前 Y=左 Z=上, 单位 cm
- flags: 0x03=正常(SLAM+TARGET都有效), 0x02=SLAM失效, 0x01=目标失效

## 当前问题: cur=(None,None,None) — 已定位

**现象**: `send_slam_cur_f5.py` 启动后前 ~5 秒 TX 日志显示 `cur=(None,None,None)`，飞控端收到 sentinel 值 `-2147483648` (0x80000000)。

**根因**: 脚本启动时 AMCL 尚未发布第一条 `/amcl_pose` 消息（AMCL 初始化需约 2-5 秒）。这是正常启动时序问题，非 bug。

**实际表现**: 约 5 秒后 AMCL 开始发布数据，cur 自动切换为有效坐标，flags 从 0x00 切换为 0x01。飞控侧正确看到 f=00→f=01 转换。

## 步骤6 测试结果 (2026-07-19 19:46)

- 298 帧发送, 100% ACK, 0 丢帧
- f=00 阶段约 5 秒, f=01 阶段约 25 秒
- 静态坐标波动: X 2cm, Y 13cm, Z 0cm
- 无异常跳变, 坐标单位 cm 正确
- 详见 [[workspace-state]]

## 下一步: 步骤7 移动测试

**步骤7 (移动测试)** — 飞控要求:
- 慢慢向前/向左/向上移动，观察 c.x/c.y/c.z 哪个轴增加
- 验证单位是否接近真实厘米
- 飞控方原话: "收到这一步真实 SLAM 日志前，不继续写 PID、误差干运行或控制输出"

**步骤8 (飞控侧待办)**:
- 基于真实 SLAM 数据实现 PID 位置控制闭环

## 相关文件

| 文件 | 用途 |
|------|------|
| `send_slam_cur_f5.py` | 从 AMCL 取数据→打包 0xF5→串口发送 |
| `send_f5.py` | 固定坐标测试发送 |
| `test_f5_frame.py` | 帧格式单元测试 |
| `ano_protocol.py` | 凌霄协议定义 |
| `linux_serial.py` | 串口封装 |
| `logs/slam_cur_static.log` | 联调日志 |
