#!/usr/bin/env python3
"""
飞控四元数 Yaw 精度测试
========================
连接飞控 → 稳定后提示旋转 → 检测>2°开始记录
→ 用户按Enter停止 → 对比旋转前后Yaw → 评估偏差

用法: python3 test_yaw_accuracy.py [--port /dev/ttyUSB0] [--baud 500000]
"""
import sys, time, math, argparse, threading

sys.path.insert(0, '/home/ylz/n10p_leishen/n10p_ws/src/n10p_bringup/n10p_bringup')
from ano_transport import SerialTransport


def quat_to_euler(w, x, y, z):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', default='/dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=500000)
    args = parser.parse_args()

    transport = SerialTransport(args.port, args.baud)
    if not transport.start():
        print(f'FAIL: 无法打开 {args.port}')
        return 1
    print(f'串口已连接: {args.port} @ {args.baud} bps')

    latest = {'w': 1.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}
    def cb(d):
        if 'error' not in d:
            for k in 'wxyz':
                latest[k] = d[k]
    transport.register_callback(0x04, cb)

    # ═══════════════════════════════════════════════════════
    # 阶段1: 等待姿态稳定 (5秒)
    # ═══════════════════════════════════════════════════════
    print('\n[阶段1] 等待姿态数据稳定 (5秒, 请勿移动飞控)...')
    time.sleep(1.0)
    pre_samples = []
    t0 = time.monotonic()
    while time.monotonic() - t0 < 5.0:
        _, _, y = quat_to_euler(latest['w'], latest['x'], latest['y'], latest['z'])
        pre_samples.append(y)
        time.sleep(0.05)

    pre_yaw = sum(pre_samples) / len(pre_samples)
    pre_std = (sum((yy - pre_yaw)**2 for yy in pre_samples) / len(pre_samples))**0.5
    print(f'基准 Yaw: {pre_yaw:.2f}° (噪声 σ={pre_std:.2f}°)')
    print(f'四元数: w={latest["w"]:.4f} x={latest["x"]:.4f} y={latest["y"]:.4f} z={latest["z"]:.4f}')

    # ═══════════════════════════════════════════════════════
    # 阶段2: 提示旋转, 检测>2°变化后开始连续记录
    # ═══════════════════════════════════════════════════════
    input('\n[阶段2] 请将飞控绕Z轴旋转约90度。准备好了按 Enter 开始监测...')

    # 等待旋转触发 (>2° 偏离基准)
    print('         等待旋转中...')
    triggered = False
    t_wait = time.monotonic()
    while not triggered:
        _, _, y = quat_to_euler(latest['w'], latest['x'], latest['y'], latest['z'])
        diff = abs(y - pre_yaw)
        if diff > 180:
            diff = 360 - diff
        if diff > 2.0:
            triggered = True
            break
        if time.monotonic() - t_wait > 120:
            print('         超时 (120s), 未检测到旋转')
            transport.stop()
            return 1
        time.sleep(0.05)

    print(f'         已检测到旋转 (偏离基准 {diff:.1f}°)')

    # ═══════════════════════════════════════════════════════
    # 阶段3: 持续记录, 等待用户按 Enter 停止
    # ═══════════════════════════════════════════════════════
    print('\n[阶段3] 正在连续记录四元数...')
    print('        旋转完成后, 保持飞控静止, 然后按 Enter 终止记录')

    # 后台线程持续采集
    post_samples = []
    recording = True

    def recorder():
        while recording:
            _, _, y = quat_to_euler(latest['w'], latest['x'], latest['y'], latest['z'])
            post_samples.append(y)
            time.sleep(0.02)  # 50Hz 采样

    rec_thread = threading.Thread(target=recorder, daemon=True)
    rec_thread.start()

    input()  # 阻塞等待用户按 Enter
    recording = False
    rec_thread.join(timeout=1.0)

    if len(post_samples) < 20:
        print('FAIL: 采样点不足, 请重试')
        transport.stop()
        return 1

    # 取最后 1 秒的稳定数据作为旋转后 Yaw
    stable_window = min(50, len(post_samples))  # 约 1 秒
    post_yaw = sum(post_samples[-stable_window:]) / stable_window
    post_std = (sum((yy - post_yaw)**2 for yy in post_samples[-stable_window:]) / stable_window)**0.5

    # ═══════════════════════════════════════════════════════
    # 分析
    # ═══════════════════════════════════════════════════════
    raw_change = abs(post_yaw - pre_yaw)
    if raw_change > 180:
        raw_change = 360 - raw_change
    deviation = raw_change - 90.0

    print(f'\n{"="*55}')
    print(f'  旋转前 Yaw:  {pre_yaw:8.2f}°  (σ={pre_std:.2f}°)')
    print(f'  旋转后 Yaw:  {post_yaw:8.2f}°  (σ={post_std:.2f}°)')
    print(f'  实际变化:    {raw_change:8.2f}°')
    print(f'  目标变化:      90.00°')
    print(f'  偏差:        {deviation:8.2f}°  ({abs(deviation)/90*100:.1f}%)')
    print(f'  采样点数:    {len(post_samples)} 帧')
    print(f'{"-"*55}')

    if abs(deviation) < 2:
        grade = 'A | 飞控高度可信, covariance→0.001 (±1.8°)'
    elif abs(deviation) < 5:
        grade = 'B | 飞控可信, covariance→0.005 (±4.0°)'
    elif abs(deviation) < 10:
        grade = 'C | 基本可信, covariance→0.01 (±5.7°)'
    elif abs(deviation) < 20:
        grade = 'D | 偏差较大, covariance→0.05 (±12.8°), scan match 主导'
    else:
        grade = 'F | 不可信, 建议仅用 scan matching'

    print(f'  等级: {grade}')
    print(f'{"="*55}')

    transport.stop()
    return 0


if __name__ == '__main__':
    sys.exit(main())
