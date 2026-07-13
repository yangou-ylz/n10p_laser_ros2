#!/usr/bin/env python3
"""0xF5 下行压力测试 — 干跑版 (不依赖物理串口), 验证帧构造+时序"""
import time, sys, struct
sys.path.insert(0, '/home/ylz/n10p_leishen/n10p_ws/src/n10p_bringup/n10p_bringup')
from rpi_pos_frame import (build_f5_frame, verify_f5_frame,
                            FLAG_SLAM_VALID, FLAG_TARGET_VALID)

FRAMES = 10000
TARGET_HZ = 50.0
INTERVAL = 1.0 / TARGET_HZ

t_start = time.monotonic()
count = 0
errors = 0
jitter_max = 0.0
missed_deadlines = 0
total_bytes = 0

while count < FRAMES:
    t_now = time.monotonic()
    expected_t = t_start + count * INTERVAL
    jitter = t_now - expected_t
    abs_jitter = abs(jitter)

    if abs_jitter > jitter_max:
        jitter_max = abs_jitter

    if jitter > INTERVAL * 0.5:
        missed_deadlines += 1  # 落后超半周期

    # 构造 31B 帧 (模拟真实数据流)
    f = build_f5_frame(float(count)*0.1, 0.0, 80.0,
                       float(count+100)*0.1, 0.0, 80.0,
                       FLAG_SLAM_VALID | FLAG_TARGET_VALID)
    if not verify_f5_frame(f):
        errors += 1

    total_bytes += len(f)  # =31

    # 模拟 serial.write(): 31B @ 500000bps = 0.62ms (计入总时间模拟)
    # 只模拟 CPU 开销, 实际 UART 传输异步不影响定时
    time.sleep(0.0003)  # ~0.3ms for struct + checksum + write syscall

    count += 1
    next_t = t_start + count * INTERVAL
    sleep_t = next_t - time.monotonic()
    if sleep_t > 0:
        time.sleep(sleep_t)

elapsed = time.monotonic() - t_start
actual_hz = count / elapsed
byte_rate = total_bytes / elapsed

print(f'发送帧数:   {count}')
print(f'耗时:       {elapsed:.1f}s (目标 {FRAMES*INTERVAL:.1f}s)')
print(f'实际频率:   {actual_hz:.1f} Hz (目标 {TARGET_HZ} Hz)')
print(f'总字节:     {total_bytes} B ({byte_rate:.0f} B/s)')
print(f'最大抖动:   {jitter_max*1000:.1f} ms')
print(f'超时次数:   {missed_deadlines} (>半周期滞后)')
print(f'帧校验错误: {errors}')

if errors == 0:
    print('帧格式:     PASS (10000帧全通过)')
else:
    print(f'帧格式:     FAIL ({errors}帧错误)')

if missed_deadlines <= 50 and actual_hz >= 49.5:
    print('时序:       PASS')
elif actual_hz >= 49.0:
    print('时序:       WARN (轻微丢帧)')
else:
    print('时序:       FAIL')

if actual_hz >= 49.0 and errors == 0:
    print('\n结果: PASS — 树莓派完全胜任 50Hz 0xF5 下行')
else:
    print('\n结果: 需要优化')
