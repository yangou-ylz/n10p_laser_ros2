#!/usr/bin/env python3
"""
TCP-to-PTY 桥接: 连接 ESP32 TCP Server → 创建 PTY 虚拟串口 → 写入数据
用法: python3 tcp2pty.py 192.168.0.184 8888 /tmp/n10p_esp32
"""
import sys, os, socket, pty, time

def main():
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.0.184"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 8888
    link = sys.argv[3] if len(sys.argv) > 3 else "/tmp/n10p_esp32"

    # 连接 ESP32 TCP
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    print(f"[tcp2pty] 已连接 {host}:{port}")

    # 创建 PTY 虚拟串口
    master_fd, slave_fd = pty.openpty()
    os.system(f"ln -sf {os.ttyname(slave_fd)} {link}")
    slave_name = os.ttyname(slave_fd)
    print(f"[tcp2pty] PTY 已创建: {link} → {slave_name}")

    # 设 PTY 从端为 raw (让驱动能直接读)
    os.system(f"stty -F {slave_name} raw -echo -icanon min 0 time 1 2>/dev/null")

    total = 0
    try:
        while True:
            data = sock.recv(4096)
            if not data:
                print("[tcp2pty] TCP 连接断开")
                break
            os.write(master_fd, data)
            total += len(data)
            if total % (108 * 500) == 0:  # 每 500 帧报一次
                print(f"[tcp2pty] 已转发 {total} 字节")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        os.close(master_fd)
        os.close(slave_fd)
        os.system(f"rm -f {link}")
        print(f"[tcp2pty] 已关闭, 总计转发 {total} 字节")

if __name__ == "__main__":
    main()
