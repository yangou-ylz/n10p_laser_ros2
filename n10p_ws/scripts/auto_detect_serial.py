#!/usr/bin/env python3
"""自动识别串口设备 — 根据数据内容判断设备身份。

识别规则:
  雷达 (N10P):  460800 baud, 数据含 0xA5 帧头 → /dev/ttyACM*
  FC 数据 (IMU): 500000 baud, 数据含 0xAA 帧头 (ANO 协议) → /dev/ttyUSB*
  下行模块:      500000 baud, 静默或仅 ACK → 排除前两者后的剩余 /dev/ttyUSB*
"""
import sys, time, glob
try:
    import serial
except ImportError:
    print("pip install pyserial")
    sys.exit(1)


def probe(port, baud, marker_byte, timeout=2.0):
    """打开串口读取, 检查是否包含 marker_byte。返回 (found, bytes_read)"""
    try:
        s = serial.Serial(port, baud, timeout=0.1)
        t0 = time.monotonic()
        data = b''
        while time.monotonic() - t0 < timeout and len(data) < 500:
            chunk = s.read(256)
            if chunk:
                data += chunk
        s.close()
        return marker_byte in data, len(data)
    except Exception:
        return False, 0


def detect():
    result = {'radar': None, 'fc_data': None, 'downlink': None}

    # 1. 找雷达 (ttyACM*, 460800, 0xA5)
    for port in sorted(glob.glob('/dev/ttyACM*')):
        ok, n = probe(port, 460800, 0xA5)
        if ok:
            result['radar'] = port
            print(f"  雷达:  {port} (0xA5, {n}B)")
            break

    # 2. 找 FC 数据 (ttyUSB*, 500000, 0xAA)
    usb_ports = sorted(glob.glob('/dev/ttyUSB*'))
    for port in usb_ports:
        ok, n = probe(port, 500000, 0xAA)
        if ok:
            result['fc_data'] = port
            print(f"  FC数据: {port} (0xAA, {n}B)")
            break

    # 3. 剩余 ttyUSB* → 下行模块
    for port in usb_ports:
        if port != result['fc_data'] and port != result['radar']:
            result['downlink'] = port
            print(f"  下行:   {port} (静默/ACK)")
            break

    return result


if __name__ == '__main__':
    print("串口识别中...")
    r = detect()
    if not any(r.values()):
        print("❌ 未找到任何设备")
        sys.exit(1)
    # 输出环境变量格式，方便脚本引用
    for k, v in r.items():
        if v:
            print(f"export N10P_{k.upper()}={v}")
