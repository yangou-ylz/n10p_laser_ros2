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
| 6 | 接入真实SLAM (send_slam_cur_f5.py) | 🔄 cur=(None,None,None) 待修复 |

## 联调结果 (步骤 3-5)

- 帧格式: 31B, AA 61 F5 19, 6×s32 LE cm + flags + SC/AC
- 速率: 10Hz 发送, 100% ACK
- 坐标方向: X=前 Y=左 Z=上, 单位 cm
- flags: 0x03=正常(SLAM+TARGET都有效), 0x02=SLAM失效, 0x01=目标失效

## 当前问题: cur=(None,None,None)

**现象**: `send_slam_cur_f5.py` 启动后, TX 日志显示 `cur=(None,None,None)`，飞控端收到的坐标是 sentinel 值 `-2147483648` (0x80000000)。

**根因分析**: 脚本订阅 `/amcl_pose` 话题，但回调未收到有效数据。可能原因：
1. AMCL 节点未激活或未收敛 → 不发布 `/amcl_pose`
2. 话题名不匹配
3. 消息字段路径错误

**排查方法**:
```bash
# 确认 AMCL 是否在发布 pose
ros2 topic echo /amcl_pose --once
# 查看话题列表
ros2 topic list | grep amcl
```

## 下一步: 步骤6 修复 + 步骤7

**步骤6 修复**: 确保 AMCL 收敛后 `/amcl_pose` 有有效数据，脚本正确提取 cur_x/cur_y/cur_z。

**步骤7 (飞控侧待办)**: 
- 收到 cur 后通过 0xA0 回显坐标值
- 验证单位 cm、无跳变
- 实现 PID 位置控制闭环

## 相关文件

| 文件 | 用途 |
|------|------|
| `send_slam_cur_f5.py` | 从 AMCL 取数据→打包 0xF5→串口发送 |
| `send_f5.py` | 固定坐标测试发送 |
| `test_f5_frame.py` | 帧格式单元测试 |
| `ano_protocol.py` | 凌霄协议定义 |
| `linux_serial.py` | 串口封装 |
| `logs/slam_cur_static.log` | 联调日志 |
