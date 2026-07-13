#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rpi_pos_frame.py — 树莓派→STM32 0xF5 位置下行帧构造模块
========================================================

实现文档《树莓派飞控对接文档.md》第三节定义的 0xF5 通信帧格式。

帧格式 (31 字节):
  [0]    0xAA      帧头
  [1]    0x61      目标地址 = STM32
  [2]    0xF5      帧ID = 自定义位置帧
  [3]    0x19      数据长度 = 25 字节
  [4~7]  cur_x     s32 小端序, 飞机当前X坐标, cm
  [8~11] cur_y     s32 小端序, 飞机当前Y坐标, cm
  [12~15] cur_z    s32 小端序, 飞机当前Z坐标, cm
  [16~19] tar_x    s32 小端序, 目标X坐标, cm
  [20~23] tar_y    s32 小端序, 目标Y坐标, cm
  [24~27] tar_z    s32 小端序, 目标Z坐标, cm
  [28]    flags    u8  状态标志
  [29]    SC       u8  和校验 (覆盖 [0]~[28])
  [30]    AC       u8  附加校验

flags 定义:
  bit0 (0x01): SLAM_VALID   — 1=SLAM定位正常
  bit1 (0x02): TARGET_VALID — 1=目标坐标有效
  bit2 (0x04): VISUAL_MODE  — 1=视觉模式 (K230), 0=航点模式
  bit3~7: 预留, 填0

两工作模式:
  模式A (航点): flags=0x03,  tar=预设航点
  模式B (视觉): flags=0x07,  tar=cur+视觉偏移,  视觉丢失时 tar=cur 悬停

无效值: 某轴不可用时填充 0x80000000, 飞控收到后停止该轴控制

用法:
    from rpi_pos_frame import build_f5_frame, FLAG_SLAM_VALID, FLAG_TARGET_VALID
    frame = build_f5_frame(0, 0, 80, 100, 0, 80, FLAG_SLAM_VALID | FLAG_TARGET_VALID)
    serial.write(frame)

测试:
    python3 -m n10p_bringup.rpi_pos_frame
"""

import struct

# ── 帧常量 ────────────────────────────────────────────────────────────
FRAME_HEADER = 0xAA
DEST_STM32   = 0x61
CMD_POS      = 0xF5       # 自定义位置帧 ID
DATA_LEN     = 0x19       # 25 字节 (6 × s32 + 1 × u8)
FRAME_SIZE   = 31         # 总帧长: 4(头)+25(数据)+2(校验)
CHKSUM_LEN   = 29         # 校验覆盖范围: [0]~[28]

INVALID_S32  = 0x80000000            # s32 最小值, 无符号表示
INVALID_S32_SIGNED = -2147483648     # 有符号表示

# ── flags 位定义 ──────────────────────────────────────────────────────
FLAG_SLAM_VALID   = 0x01   # SLAM 定位正常
FLAG_TARGET_VALID = 0x02   # 目标坐标有效
FLAG_VISUAL_MODE  = 0x04   # 视觉模式 (K230 提供目标)


def _to_s32_le(value_cm) -> int:
    """
    将位置值转换为 s32 小端序的无符号表示。

    转换规则:
      - None / NaN → INVALID_S32
      - 正常值 → round() → 钳制 s32 范围 → 返回无符号 32-bit 表示
    """
    if value_cm is None:
        return INVALID_S32
    if isinstance(value_cm, float) and value_cm != value_cm:
        return INVALID_S32
    val = int(round(value_cm))
    if val > 0x7FFFFFFF:
        val = 0x7FFFFFFF
    elif val < -2147483648:
        val = -2147483648
    return val & 0xFFFFFFFF


def _to_signed(raw: int) -> int:
    """将 32-bit 无符号表示转回有符号整数"""
    return raw if raw < 0x80000000 else raw - 0x100000000


def build_f5_frame(cur_x=None, cur_y=None, cur_z=None,
                   tar_x=None, tar_y=None, tar_z=None,
                   flags: int = 0) -> bytes:
    """
    构造 0xF5 自定义位置帧 (31 字节)。

    参数:
        cur_x/y/z: 飞机当前位置 (cm), float 或 None。None/NaN 表示无效。
        tar_x/y/z: 目标位置 (cm), float 或 None。
        flags:     状态标志字节。使用 FLAG_SLAM_VALID | FLAG_TARGET_VALID 等组合。

    返回:
        31 字节 bytes 对象，可直接写入串口。

    示例:
        >>> f = build_f5_frame(0, 0, 80, 100, 0, 80, 0x03)
        >>> f[0], f[2], f[3], len(f)
        (170, 245, 25, 31)
    """
    frame = bytearray(FRAME_SIZE)
    frame[0] = FRAME_HEADER
    frame[1] = DEST_STM32
    frame[2] = CMD_POS
    frame[3] = DATA_LEN

    # 6 个 s32: cur_x, cur_y, cur_z, tar_x, tar_y, tar_z
    raw = [
        _to_s32_le(cur_x), _to_s32_le(cur_y), _to_s32_le(cur_z),
        _to_s32_le(tar_x), _to_s32_le(tar_y), _to_s32_le(tar_z),
    ]
    signed = [_to_signed(v) for v in raw]
    struct.pack_into('<iiiiii', frame, 4, *signed)

    # flags
    frame[28] = flags & 0xFF

    # SC/AC 校验 (覆盖 [0]~[28] 共 29 字节)
    sc = 0
    ac = 0
    for i in range(CHKSUM_LEN):
        sc = (sc + frame[i]) & 0xFF
        ac = (ac + sc) & 0xFF
    frame[29] = sc
    frame[30] = ac

    return bytes(frame)


def build_hover_frame(cur_x, cur_y, cur_z) -> bytes:
    """
    构造悬停帧: 目标=当前位置, 飞控收到后悬停。

    用于:
      - 视觉丢失时的安全回退
      - SLAM 刚收敛但未收到航点指令时
    """
    return build_f5_frame(cur_x, cur_y, cur_z,
                          cur_x, cur_y, cur_z,
                          FLAG_SLAM_VALID | FLAG_TARGET_VALID)


def build_waypoint_frame(cur_x, cur_y, cur_z,
                         wp_x, wp_y, wp_z) -> bytes:
    """
    构造航点模式帧 (模式A): tar=预设航点, flags=0x03
    """
    return build_f5_frame(cur_x, cur_y, cur_z,
                          wp_x, wp_y, wp_z,
                          FLAG_SLAM_VALID | FLAG_TARGET_VALID)


def build_visual_frame(cur_x, cur_y, cur_z,
                       dx, dy, dz=0.0, target_valid=True) -> bytes:
    """
    构造视觉伺服模式帧 (模式B): tar = cur + (dx, dy, dz), flags=0x07

    参数:
        cur_x/y/z:   飞机当前位置 (cm)
        dx, dy, dz:  目标相对飞机的偏移 (cm), K230 输出
        target_valid: True=视觉正常, False=视觉丢失 (tar=cur 悬停)
    """
    flags = FLAG_SLAM_VALID | FLAG_VISUAL_MODE
    if target_valid:
        tar_x = cur_x + dx
        tar_y = cur_y + dy
        tar_z = cur_z + dz
        flags |= FLAG_TARGET_VALID
    else:
        # 视觉丢失: 原地悬停
        tar_x, tar_y, tar_z = cur_x, cur_y, cur_z
        flags |= FLAG_TARGET_VALID  # 目标有效 (=当前位置, 悬停)
    return build_f5_frame(cur_x, cur_y, cur_z,
                          tar_x, tar_y, tar_z, flags)


def build_invalid_frame() -> bytes:
    """构造全无效帧 (SLAM 未就绪时发送)"""
    return build_f5_frame(None, None, None,
                          None, None, None, 0x00)


def verify_f5_frame(frame: bytes) -> bool:
    """校验 0xF5 帧完整性"""
    if len(frame) != FRAME_SIZE:
        return False
    if frame[0] != FRAME_HEADER or frame[2] != CMD_POS:
        return False
    sc = ac = 0
    for i in range(CHKSUM_LEN):
        sc = (sc + frame[i]) & 0xFF
        ac = (ac + sc) & 0xFF
    return frame[29] == sc and frame[30] == ac


def parse_f5_frame(frame: bytes) -> dict:
    """解析 0xF5 帧, 返回可读字典 (用于调试)"""
    if not verify_f5_frame(frame):
        return {'valid': False}
    vals = struct.unpack_from('<iiiiii', frame, 4)
    flags = frame[28]
    return {
        'valid': True,
        'cur_x': vals[0] if vals[0] != INVALID_S32_SIGNED else None,
        'cur_y': vals[1] if vals[1] != INVALID_S32_SIGNED else None,
        'cur_z': vals[2] if vals[2] != INVALID_S32_SIGNED else None,
        'tar_x': vals[3] if vals[3] != INVALID_S32_SIGNED else None,
        'tar_y': vals[4] if vals[4] != INVALID_S32_SIGNED else None,
        'tar_z': vals[5] if vals[5] != INVALID_S32_SIGNED else None,
        'flags': flags,
        'slam_valid': bool(flags & FLAG_SLAM_VALID),
        'target_valid': bool(flags & FLAG_TARGET_VALID),
        'visual_mode': bool(flags & FLAG_VISUAL_MODE),
    }


# ── 自测 ─────────────────────────────────────────────────────────────
if __name__ == '__main__':
    import sys

    # 测试1: 航点模式正常帧
    f1 = build_f5_frame(0, 0, 80, 100, 0, 80,
                        FLAG_SLAM_VALID | FLAG_TARGET_VALID)
    assert len(f1) == 31, f"帧长错误: {len(f1)}"
    assert f1[0] == 0xAA and f1[1] == 0x61 and f1[2] == 0xF5, "帧头/地址/ID错误"
    assert f1[3] == 0x19, f"DLEN错误: {f1[3]:02X}"
    assert verify_f5_frame(f1), "校验失败"
    p1 = parse_f5_frame(f1)
    assert p1['cur_z'] == 80 and p1['tar_x'] == 100, f"解析错误: {p1}"
    assert p1['slam_valid'] and p1['target_valid'] and not p1['visual_mode']
    print(f"  ✓ 航点模式: {f1.hex()}")

    # 测试2: 无效帧
    f2 = build_invalid_frame()
    assert verify_f5_frame(f2)
    p2 = parse_f5_frame(f2)
    assert p2['cur_x'] is None and p2['tar_x'] is None
    assert not p2['slam_valid']
    print(f"  ✓ 全无效帧: {f2.hex()}")

    # 测试3: 视觉模式帧 (目标前方 50cm, 左 10cm)
    f3 = build_visual_frame(100, 200, 80, 50, -10, 0)
    p3 = parse_f5_frame(f3)
    assert p3['visual_mode'] and p3['target_valid']
    assert p3['tar_x'] == 150 and p3['tar_y'] == 190  # cur+dx, cur+dy
    print(f"  ✓ 视觉模式: cur=(100,200,80) dx=(50,-10,0) → tar_x=150 tar_y=190")

    # 测试4: 视觉丢失悬停
    f4 = build_visual_frame(100, 200, 80, 0, 0, 0, target_valid=False)
    p4 = parse_f5_frame(f4)
    assert p4['tar_x'] == 100 and p4['tar_y'] == 200  # tar = cur
    print(f"  ✓ 视觉丢失悬停: tar == cur")

    # 测试5: 悬停帧
    f5 = build_hover_frame(150, -30, 80)
    p5 = parse_f5_frame(f5)
    assert p5['cur_x'] == p5['tar_x'] == 150
    assert p5['cur_y'] == p5['tar_y'] == -30
    print(f"  ✓ 悬停帧: tar == cur")

    # 测试6: 校验完整性
    f6 = bytearray(f1)
    f6[8] ^= 0x01
    assert not verify_f5_frame(bytes(f6)), "篡改帧应校验失败"
    print("  ✓ 篡改检测")

    # 测试7: NaN/None 混合
    f7 = build_f5_frame(float('nan'), None, 80, 100, 0, 80, 0x03)
    p7 = parse_f5_frame(f7)
    assert p7['cur_x'] is None and p7['cur_y'] is None
    assert p7['cur_z'] == 80 and p7['tar_x'] == 100
    print("  ✓ NaN/None 混合处理")

    # 测试8: 连续 2000 帧高速校验 (模拟 40 秒 @ 50Hz)
    for i in range(2000):
        f = build_f5_frame(float(i)*0.5, 0.0, 80.0,
                           float(i+10)*0.5, 0.0, 80.0, 0x03)
        assert verify_f5_frame(f), f"第{i}帧校验失败"
    print("  ✓ 连续 2000 帧全部通过")

    # 测试9: SC/AC 手工验算 (文档示例)
    # cur=(0,0,80), tar=(100,0,80), flags=0x03
    f9 = build_f5_frame(0, 0, 80, 100, 0, 80, 0x03)
    sc_manual = ac_manual = 0
    for i in range(29):
        sc_manual = (sc_manual + f9[i]) & 0xFF
        ac_manual = (ac_manual + sc_manual) & 0xFF
    assert f9[29] == sc_manual and f9[30] == ac_manual, \
        f"SC/AC 验算失败: ({sc_manual:02X},{ac_manual:02X}) vs ({f9[29]:02X},{f9[30]:02X})"
    print(f"  ✓ SC/AC 验算: SC={sc_manual:02X} AC={ac_manual:02X}")

    # 测试10: flags 位组合
    f10 = build_f5_frame(0, 0, 80, 100, 0, 80,
                         FLAG_SLAM_VALID | FLAG_TARGET_VALID | FLAG_VISUAL_MODE)
    assert f10[28] == 0x07
    print(f"  ✓ flags=0x07 (SLAM+TARGET+VISUAL)")

    print(f"\n全部测试通过 (10/10)")

    # 帧格式参考
    print("\n── 帧格式参考 ──")
    print(f"航点模式: {f1.hex()}")
    print(f"无效帧:   {f2.hex()}")
    sys.exit(0)
