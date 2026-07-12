#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ano_protocol.py — 凌霄匿名协议 V7 纯协议层
===========================================

职责：帧描述符定义、校验计算、帧构建、帧解码。
特点：
  - 零外部依赖（仅 stdlib struct + dataclasses + math）
  - 所有函数为纯函数，无副作用，无状态
  - 描述符驱动解码：新增帧类型只需加一个 FrameDef 条目
  - 不包含任何 I/O 或线程操作

协议帧结构:
  [0xAA] [DEST] [CMD] [LEN] [DATA...] [SC] [AC]
    1B      1B     1B    1B    n B       1B   1B

校验覆盖范围: 帧头 0xAA 到 DATA 最后一个字节，共 LEN+4 字节
  SC = sum(覆盖字节) & 0xFF
  AC = cumulative_sum(SC) & 0xFF（每次累加 SC 的当前值）
"""

import struct
import math
from dataclasses import dataclass, field
from typing import Tuple, Dict, List, Optional, Callable

# ═══════════════════════════════════════════════════════════════════════
# 帧头常量
# ═══════════════════════════════════════════════════════════════════════

FRAME_HEAD = 0xAA  # 帧头标识，所有帧以 0xAA 开头

# ═══════════════════════════════════════════════════════════════════════
# 地址定义
# ═══════════════════════════════════════════════════════════════════════

ADDR_BROADCAST = 0xFF  # 广播地址（所有设备接收）
ADDR_IMU       = 0x60  # 凌霄 IMU 模块
ADDR_STM32     = 0x61  # STM32 飞控板
ADDR_PC        = 0xAF  # 上位机 / 地面站
ADDR_OPTICAL   = 0x22  # 匿名光流模块
ADDR_UWB       = 0x30  # 匿名 UWB 模块

ADDR_NAME = {
    0xFF: '广播',
    0x60: 'IMU',
    0x61: 'STM32',
    0xAF: '上位机',
    0x22: '光流',
    0x30: 'UWB',
}

# ═══════════════════════════════════════════════════════════════════════
# 帧字段描述符
# ═══════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class FrameField:
    """
    帧字段描述符 — 描述 DATA 区中一个字段的位置、类型和物理含义。

    属性:
        name:   字段名（snake_case，解码后字典的 key）
        offset: 在 DATA 区的字节偏移（从 0 开始）
        fmt:     struct 格式字符: 'b'=s8, 'B'=u8, 'h'=s16, 'H'=u16, 'i'=s32, 'I'=u32
        scale:   缩放因子 — 解码后的整数值乘以此系数得到物理值
        unit:    物理单位字符串（如 '°', 'cm', 'cm/s', 'V'）
        desc:    中文说明
    """
    name: str
    offset: int
    fmt: str
    scale: float = 1.0
    unit: str = ''
    desc: str = ''


@dataclass(frozen=True)
class FrameDef:
    """
    帧类型描述符 — 定义一个帧类型（CMD）的完整元数据。

    属性:
        cmd:        帧功能码（0x00 ~ 0xFF）
        name:       帧英文名（PascalCase，如 'Quaternion'）
        fields:     字段描述符列表（按 DATA 区字节顺序排列）
        min_len:    DATA 区最小字节数（用于长度校验）
        direction:  数据方向，如 'IMU→STM32' 或 'STM32→IMU'
        freq_hz:    典型发送频率（Hz），0 表示不定频
        desc:       中文说明
        post_decode: 可选的后处理函数，签名为 (dict) -> dict，用于计算派生字段

    内部缓存（预计算，加速解码）:
        _struct_fmt:  预计算的 struct 格式字符串，如 '<hhhhB'
        _field_names:  预提取的字段名列表，用于 zip 映射
    """
    cmd: int
    name: str
    fields: Tuple[FrameField, ...]
    min_len: int
    direction: str = ''
    freq_hz: float = 0.0
    desc: str = ''
    post_decode: Optional[Callable[[dict], dict]] = None

    # 预计算缓存 — 在 __post_init__ 中通过 object.__setattr__ 设置
    _struct_fmt: str = field(init=False, default='')
    _field_names: Tuple[str, ...] = field(init=False, default=())

    def __post_init__(self):
        fmt = '<' + ''.join(f.fmt for f in self.fields)
        names = tuple(f.name for f in self.fields)
        object.__setattr__(self, '_struct_fmt', fmt)
        object.__setattr__(self, '_field_names', names)


# ═══════════════════════════════════════════════════════════════════════
# 后处理钩子 — 用于计算派生字段
# ═══════════════════════════════════════════════════════════════════════

def _post_decode_quaternion(d: dict) -> dict:
    """0x04 帧后处理：从四元数 w,x,y,z 计算欧拉角 roll/pitch/yaw（单位：度）"""
    w, x, y, z = d['w'], d['x'], d['y'], d['z']
    # Roll: 绕 X 轴旋转
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    # Pitch: 绕 Y 轴旋转
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))  # 夹紧防浮点溢出
    pitch = math.degrees(math.asin(sinp))
    # Yaw: 绕 Z 轴旋转
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp))
    d['roll_deg'] = roll
    d['pitch_deg'] = pitch
    d['yaw_deg'] = yaw
    return d


def _post_decode_fc_status(d: dict) -> dict:
    """0x06 帧后处理：添加飞行模式可读名称"""
    mode_map = {0: '姿态', 1: '定高', 2: '定点', 3: '程控'}
    d['mode_str'] = mode_map.get(d['mode'], f'未知({d["mode"]})')
    return d


def _post_decode_module_status(d: dict) -> dict:
    """0x0E 帧后处理：添加状态码可读名称"""
    sta_str = {0: '无数据', 1: '不可用', 2: '正常', 3: '良好'}
    for key in ('sta_gvel', 'sta_gpos', 'sta_gps', 'sta_alt'):
        if key in d:
            d[key + '_str'] = sta_str.get(d[key], '?')
    return d


# ═══════════════════════════════════════════════════════════════════════
# 帧注册表 — 新增帧类型只需在此字典中添加一行
# ═══════════════════════════════════════════════════════════════════════

FRAME_REGISTRY: Dict[int, FrameDef] = {}

def _register(fd: FrameDef) -> FrameDef:
    """将帧描述符注册到全局注册表"""
    FRAME_REGISTRY[fd.cmd] = fd
    return fd


# ── 0x01 惯性传感器原始数据 ─────────────────────────────────────────
_register(FrameDef(
    cmd=0x01, name='IMU_Raw', min_len=13,
    direction='IMU→STM32', freq_hz=100.0, desc='加速度+陀螺仪原始量化值',
    fields=(
        FrameField('acc_x', 0, 'h', desc='X轴加速度（原始值）'),
        FrameField('acc_y', 2, 'h', desc='Y轴加速度（原始值）'),
        FrameField('acc_z', 4, 'h', desc='Z轴加速度（原始值）'),
        FrameField('gyr_x', 6, 'h', desc='X轴角速度（原始值）'),
        FrameField('gyr_y', 8, 'h', desc='Y轴角速度（原始值）'),
        FrameField('gyr_z', 10, 'h', desc='Z轴角速度（原始值）'),
        FrameField('shock', 12, 'B', desc='振动标志（0=正常）'),
    ),
))

# ── 0x02 气压计 + 磁力计 ─────────────────────────────────────────────
_register(FrameDef(
    cmd=0x02, name='Baro_Mag', min_len=14,
    direction='IMU→STM32', freq_hz=20.0, desc='磁力计+气压高度+温度',
    fields=(
        FrameField('mag_x', 0, 'h', desc='磁力计X'),
        FrameField('mag_y', 2, 'h', desc='磁力计Y'),
        FrameField('mag_z', 4, 'h', desc='磁力计Z'),
        FrameField('baro_alt_cm', 6, 'i', scale=1.0, unit='cm', desc='气压高度'),
        FrameField('temp_c', 10, 'h', scale=0.1, unit='°C', desc='温度'),
        FrameField('mag_sta', 12, 'B', desc='磁力计状态'),
        FrameField('baro_sta', 13, 'B', desc='气压计状态'),
    ),
))

# ── 0x03 姿态欧拉角（低频 0.67Hz，勿用于实时控制）─────────────────
_register(FrameDef(
    cmd=0x03, name='Euler_Angle', min_len=7,
    direction='IMU→STM32', freq_hz=0.67, desc='姿态欧拉角（低频！勿用于控制）',
    fields=(
        FrameField('roll_deg', 0, 'h', scale=0.01, unit='°', desc='横滚角'),
        FrameField('pitch_deg', 2, 'h', scale=0.01, unit='°', desc='俯仰角'),
        FrameField('yaw_deg', 4, 'h', scale=0.01, unit='°', desc='偏航角'),
        FrameField('fusion_sta', 6, 'B', desc='融合状态（0=正常）'),
    ),
))

# ── 0x04 姿态四元数（推荐！~67Hz）───────────────────────────────────
_register(FrameDef(
    cmd=0x04, name='Quaternion', min_len=9,
    direction='IMU→STM32', freq_hz=67.0, desc='姿态四元数（首选姿态来源，67Hz）',
    fields=(
        FrameField('w', 0, 'h', scale=0.0001, desc='四元数W分量'),
        FrameField('x', 2, 'h', scale=0.0001, desc='四元数X分量'),
        FrameField('y', 4, 'h', scale=0.0001, desc='四元数Y分量'),
        FrameField('z', 6, 'h', scale=0.0001, desc='四元数Z分量'),
        FrameField('fusion_sta', 8, 'B', desc='融合状态（0=正常）'),
    ),
    post_decode=_post_decode_quaternion,
))

# ── 0x05 融合高度 ────────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x05, name='Fused_Alt', min_len=9,
    direction='IMU→STM32', freq_hz=50.0, desc='融合高度（气压+激光/超声波补偿）',
    fields=(
        FrameField('alt_fused_cm', 0, 'i', scale=1.0, unit='cm', desc='融合高度'),
        FrameField('alt_add_cm', 4, 'i', scale=1.0, unit='cm', desc='附加高度（激光/超声波补偿后）'),
        FrameField('sta', 8, 'B', desc='高度融合状态'),
    ),
))

# ── 0x06 飞控状态 ────────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x06, name='FC_Status', min_len=5,
    direction='IMU→STM32', freq_hz=20.0, desc='飞控模式+解锁状态+当前CMD',
    fields=(
        FrameField('mode', 0, 'B', desc='飞行模式：0=姿态 1=定高 2=定点 3=程控'),
        FrameField('unlocked', 1, 'B', desc='解锁状态：0=上锁 1=解锁'),
        FrameField('cmd_cid', 2, 'B', desc='当前执行CMD的CID'),
        FrameField('cmd_0', 3, 'B', desc='CMD_0参数'),
        FrameField('cmd_1', 4, 'B', desc='CMD_1参数'),
    ),
    post_decode=_post_decode_fc_status,
))

# ── 0x07 飞行速度 ────────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x07, name='Velocity', min_len=6,
    direction='IMU→STM32', freq_hz=50.0, desc='机体坐标系飞行速度',
    fields=(
        FrameField('vel_x_cms', 0, 'h', scale=1.0, unit='cm/s', desc='X方向速度（前为正）'),
        FrameField('vel_y_cms', 2, 'h', scale=1.0, unit='cm/s', desc='Y方向速度（右为正）'),
        FrameField('vel_z_cms', 4, 'h', scale=1.0, unit='cm/s', desc='Z方向速度（上为正）'),
    ),
))

# ── 0x08 XY 位移 ─────────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x08, name='XY_Pos', min_len=8,
    direction='IMU→STM32', freq_hz=20.0, desc='相对起飞点的XY位移（需外部定位传感器）',
    fields=(
        FrameField('pos_x_cm', 0, 'i', scale=1.0, unit='cm', desc='X位移'),
        FrameField('pos_y_cm', 4, 'i', scale=1.0, unit='cm', desc='Y位移'),
    ),
))

# ── 0x09 风速估计 ────────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x09, name='Wind', min_len=4,
    direction='IMU→STM32', freq_hz=0.0, desc='风速估计（机体坐标系）',
    fields=(
        FrameField('wind_x_cms', 0, 'h', scale=1.0, unit='cm/s', desc='X方向风速'),
        FrameField('wind_y_cms', 2, 'h', scale=1.0, unit='cm/s', desc='Y方向风速'),
    ),
))

# ── 0x0A 目标姿态 ────────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x0A, name='Target_Att', min_len=7,
    direction='IMU→STM32', freq_hz=0.0, desc='飞控内部目标姿态（格式同0x03）',
    fields=(
        FrameField('roll_deg', 0, 'h', scale=0.01, unit='°', desc='目标横滚角'),
        FrameField('pitch_deg', 2, 'h', scale=0.01, unit='°', desc='目标俯仰角'),
        FrameField('yaw_deg', 4, 'h', scale=0.01, unit='°', desc='目标偏航角'),
        FrameField('fusion_sta', 6, 'B', desc='融合状态'),
    ),
))

# ── 0x0D 电池信息 ────────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x0D, name='Battery', min_len=4,
    direction='IMU→STM32', freq_hz=1.0, desc='电池电压+电流',
    fields=(
        FrameField('voltage_v', 0, 'H', scale=0.01, unit='V', desc='电池电压'),
        FrameField('current_a', 2, 'H', scale=0.01, unit='A', desc='电池电流'),
    ),
))

# ── 0x0E 外接模块状态 ────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x0E, name='Module_Status', min_len=4,
    direction='IMU→STM32', freq_hz=2.0, desc='外接传感器模块状态',
    fields=(
        FrameField('sta_gvel', 0, 'B', desc='通用速度传感器状态'),
        FrameField('sta_gpos', 1, 'B', desc='通用位置传感器状态'),
        FrameField('sta_gps', 2, 'B', desc='GPS模块状态'),
        FrameField('sta_alt', 3, 'B', desc='辅助高度传感器状态'),
    ),
    post_decode=_post_decode_module_status,
))

# ── 0x20 电机 PWM ────────────────────────────────────────────────────
# 变长帧：每 2 字节一个电机的 PWM 值，min_len=2（至少1个电机）
_register(FrameDef(
    cmd=0x20, name='Motor_PWM', min_len=2,
    direction='IMU→STM32', freq_hz=0.0, desc='电机PWM输出值（变长，每电机2字节）',
    fields=(),  # 变长帧无固定字段描述，由解码器特殊处理
))

# ── 0x21 姿态控制量 ──────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x21, name='Att_Ctrl', min_len=8,
    direction='IMU→STM32', freq_hz=0.0, desc='飞控姿态控制量',
    fields=(
        FrameField('ctrl_roll', 0, 'h', desc='Roll控制量'),
        FrameField('ctrl_pitch', 2, 'h', desc='Pitch控制量'),
        FrameField('ctrl_yaw', 4, 'h', desc='Yaw控制量'),
        FrameField('ctrl_thr', 6, 'h', desc='油门控制量'),
    ),
))

# ── 0x40 遥控器数据 ──────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x40, name='RC_Data', min_len=20,
    direction='STM32→IMU', freq_hz=50.0, desc='遥控器10通道数据（单位μs）',
    fields=(
        FrameField('ch_roll', 0, 'H', unit='μs', desc='CH1 Roll'),
        FrameField('ch_pitch', 2, 'H', unit='μs', desc='CH2 Pitch'),
        FrameField('ch_throttle', 4, 'H', unit='μs', desc='CH3 Throttle'),
        FrameField('ch_yaw', 6, 'H', unit='μs', desc='CH4 Yaw'),
        FrameField('ch_aux1', 8, 'H', unit='μs', desc='CH5/AUX1 飞行模式切换'),
        FrameField('ch_aux2', 10, 'H', unit='μs', desc='CH6/AUX2'),
        FrameField('ch_aux3', 12, 'H', unit='μs', desc='CH7/AUX3'),
        FrameField('ch_aux4', 14, 'H', unit='μs', desc='CH8/AUX4'),
        FrameField('ch_aux5', 16, 'H', unit='μs', desc='CH9/AUX5'),
        FrameField('ch_aux6', 18, 'H', unit='μs', desc='CH10/AUX6'),
    ),
))

# ── 0x41 实时控制帧 ──────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x41, name='RT_Ctrl', min_len=14,
    direction='STM32→IMU', freq_hz=50.0, desc='程控模式实时控制指令',
    fields=(
        FrameField('roll_deg', 0, 'h', scale=0.01, unit='°', desc='目标横滚角'),
        FrameField('pitch_deg', 2, 'h', scale=0.01, unit='°', desc='目标俯仰角'),
        FrameField('thr_pct', 4, 'h', scale=0.1, unit='%', desc='油门百分比'),
        FrameField('yaw_rate', 6, 'h', scale=1.0, unit='°/s', desc='偏航角速度'),
        FrameField('vel_x_cms', 8, 'h', unit='cm/s', desc='X速度'),
        FrameField('vel_y_cms', 10, 'h', unit='cm/s', desc='Y速度'),
        FrameField('vel_z_cms', 12, 'h', unit='cm/s', desc='Z速度'),
    ),
))

# ── 0xA0 日志字符串 ──────────────────────────────────────────────────
# 变长帧：第1字节=颜色，后续=GBK编码文本
_register(FrameDef(
    cmd=0xA0, name='Log_String', min_len=1,
    direction='IMU→STM32', freq_hz=0.0, desc='字符串日志（GBK编码）',
    fields=(),  # 变长帧，由解码器特殊处理
))

# ── 0xE0 CMD 命令帧 ──────────────────────────────────────────────────
_register(FrameDef(
    cmd=0xE0, name='CMD', min_len=3,
    direction='STM32→IMU', freq_hz=0.0, desc='CMD命令帧（解锁/上锁/起飞/降落）',
    fields=(
        FrameField('cid', 0, 'B', desc='命令类别'),
        FrameField('cmd_0', 1, 'B', desc='命令参数0'),
        FrameField('cmd_1', 2, 'B', desc='命令参数1'),
    ),
))

# ── 0x0F 系统心跳/状态 ──────────────────────────────────────────────
# 广播帧，~50Hz，4字节全零，飞控定期广播
_register(FrameDef(
    cmd=0x0F, name='Sys_Heartbeat', min_len=4,
    direction='IMU→STM32', freq_hz=50.0, desc='系统心跳/状态帧（4字节）',
    fields=(
        FrameField('status', 0, 'I', desc='状态字（32位，含义待确认）'),
    ),
))

# ── 0x30 外部传感器数据 ─────────────────────────────────────────────
# 24字节，~10Hz，包含外部定位传感器综合数据
_register(FrameDef(
    cmd=0x30, name='Ext_Sensor', min_len=24,
    direction='STM32→IMU', freq_hz=10.0, desc='外部传感器综合数据（24字节）',
    fields=(),  # 结构待确认，先保留原始数据
))

# ── 0x00 CK 应答帧 ───────────────────────────────────────────────────
_register(FrameDef(
    cmd=0x00, name='CK_Reply', min_len=3,
    direction='IMU→STM32', freq_hz=0.0, desc='对指令帧的CK应答',
    fields=(
        FrameField('for_cmd', 0, 'B', desc='被应答帧的CMD'),
        FrameField('sc', 1, 'B', desc='原始帧的SC'),
        FrameField('ac', 2, 'B', desc='原始帧的AC'),
    ),
))


# ═══════════════════════════════════════════════════════════════════════
# 帧名称快捷映射
# ═══════════════════════════════════════════════════════════════════════

FRAME_NAME: Dict[int, str] = {cmd: fd.name for cmd, fd in FRAME_REGISTRY.items()}


# ═══════════════════════════════════════════════════════════════════════
# 校验函数
# ═══════════════════════════════════════════════════════════════════════

def compute_checksum(data: bytes) -> Tuple[int, int]:
    """
    计算 SC/AC 双重校验和。

    参数:
        data: 覆盖范围字节序列（从 0xAA 到 DATA 最后字节，共 LEN+4 字节）

    返回:
        (sc, ac) 元组，每个值 0~255

    算法:
        SC = sum(覆盖字节) & 0xFF
        AC = cumulative_sum(SC) & 0xFF  — 每累加一个字节后的SC值再累加
    """
    sc = 0
    ac = 0
    for b in data:
        sc = (sc + b) & 0xFF
        ac = (ac + sc) & 0xFF
    return sc, ac


def verify_frame(frame: bytes) -> bool:
    """
    校验帧完整性。

    参数:
        frame: 完整帧字节序列（含帧头和校验字节）

    返回:
        True = 校验通过，False = 校验失败或长度不足

    算法流程:
        1. 检查最小长度 (6字节)
        2. 检查帧头是否为 0xAA
        3. 提取 LEN，检查总长度是否 ≥ LEN + 6
        4. 计算覆盖字节的 SC/AC
        5. 与帧末两个字节比较
    """
    if len(frame) < 6:
        return False
    if frame[0] != FRAME_HEAD:
        return False
    payload_len = frame[3]
    if len(frame) < payload_len + 6:
        return False
    # 校验覆盖: 0xAA 到 DATA 结束，共 payload_len + 4 字节
    covered = frame[:payload_len + 4]
    sc, ac = compute_checksum(covered)
    return sc == frame[payload_len + 4] and ac == frame[payload_len + 5]


# ═══════════════════════════════════════════════════════════════════════
# 帧构建函数
# ═══════════════════════════════════════════════════════════════════════

def build_frame(dest: int, cmd: int, payload: bytes = b'') -> bytes:
    """
    构造一帧完整的匿名协议帧（含校验字节）。

    参数:
        dest:    目标地址（ADDR_BROADCAST / ADDR_IMU / ADDR_STM32 等）
        cmd:     帧功能码
        payload: DATA 区字节数据

    返回:
        完整帧 bytes，可用于直接写入串口

    示例:
        >>> frame = build_frame(0xFF, 0xE0, bytes([0x10, 0x03, 0x00]))
        >>> frame.hex()
        'aaffe0031003002f67'
    """
    payload_len = len(payload)
    header = bytes([FRAME_HEAD, dest & 0xFF, cmd & 0xFF, payload_len & 0xFF])
    covered = header + payload
    sc, ac = compute_checksum(covered)
    return covered + bytes([sc, ac])


# ═══════════════════════════════════════════════════════════════════════
# 帧解码函数
# ═══════════════════════════════════════════════════════════════════════

def decode_frame(cmd: int, payload: bytes) -> dict:
    """
    将帧的 DATA 区字节解码为字典。

    解码流程:
        1. 查找 FRAME_REGISTRY 获取帧描述符
        2. 检查 payload 长度 ≥ min_len
        3. 一次性 struct.unpack 解出所有固定字段
        4. 将字段名与值 zip 成字典
        5. 对特殊帧（变长/字符串）做特殊处理
        6. 应用 post_decode 钩子（如有）

    参数:
        cmd:     帧功能码（用于查找帧描述符）
        payload: DATA 区字节数据

    返回:
        解码后的字典，字段名为 key，物理值（已缩放）为 value。
        未知帧返回 {'raw': hex字符串}。
        解码失败返回 {'error': 错误信息, 'raw': hex字符串}。

    示例:
        >>> data = bytes([0, 100, 0, 0, 0, 100, 0, 0, 0])
        >>> decode_frame(0x04, data)
        {'w': 0.0, 'x': 0.01, 'y': 0.0, 'z': 0.01, 'fusion_sta': 0,
         'roll_deg': 0.0, 'pitch_deg': 0.0, 'yaw_deg': 1.1459...}
    """
    # ── 查找帧描述符 ──────────────────────────────────────
    fd = FRAME_REGISTRY.get(cmd)
    if fd is None:
        return {'raw': payload.hex(), 'cmd': cmd}

    raw_hex = payload.hex()

    try:
        # ── 变长帧特殊处理 ──────────────────────────────────
        if cmd == 0x20:
            # 电机PWM：每 2 字节一个电机
            n_motors = len(payload) // 2
            motors = {}
            for i in range(n_motors):
                motors[f'm{i+1}'] = struct.unpack_from('<H', payload, i * 2)[0]
            return motors

        if cmd == 0xA0:
            # 日志字符串：第1字节颜色 + 后续GBK文本
            if len(payload) < 1:
                return {'error': '数据太短', 'raw': raw_hex}
            color_map = {0: '黑', 1: '红', 2: '绿'}
            color = color_map.get(payload[0], f'未知({payload[0]})')
            try:
                text = payload[1:].decode('gbk', errors='replace').rstrip('\x00')
            except Exception:
                text = payload[1:].decode('utf-8', errors='replace').rstrip('\x00')
            return {'color': color, 'text': text}

        # ── 长度检查 ────────────────────────────────────────
        if len(payload) < fd.min_len:
            return {'error': f'数据太短: 需要≥{fd.min_len}字节, 实际{len(payload)}字节',
                    'raw': raw_hex}

        # ── 固定字段帧：一次性 unpack ────────────────────────
        if fd.fields:
            values = struct.unpack(fd._struct_fmt, payload[:fd.min_len])
            result = {}
            for name, val in zip(fd._field_names, values):
                # 查找对应字段的 scale
                scale = 1.0
                unit = ''
                for f in fd.fields:
                    if f.name == name:
                        scale = f.scale
                        unit = f.unit
                        break
                if isinstance(val, float) or scale != 1.0:
                    result[name] = val * scale
                else:
                    result[name] = val
        else:
            result = {'raw': raw_hex}

        # ── 后处理钩子 ──────────────────────────────────────
        if fd.post_decode is not None:
            result = fd.post_decode(result)

        return result

    except Exception as e:
        return {'error': str(e), 'raw': raw_hex}
