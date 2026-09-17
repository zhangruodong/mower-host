"""割草机上下位机串口协议：11 字节发帧、24 字节收帧。

字段位置全部对着固件源码核实过，出处见 docs/协议.md：
    lawn-mower/BALANCE/uartx_callback.c      收帧分支（下位机侧）
    lawn-mower/BALANCE/data_task.c:98-137    回帧组包

上位机 core/protocol.py 只管组发帧、不解析，是这里的发帧子集。
"""
import math

FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D

# 发帧（上位机 → STM32）命令号。对照 uartx_callback.c 的分支，没有的别加
# （删过哪些、为什么删，见 docs/移除记录.md）。
CMD_VEL = 0         # 三轴速度，mm/s，16 位有符号大端
CMD_RECHARGE = 1    # 自动回充，同速度字段，走 charger.Up_Move*
CMD_DOCK = 3        # 红外对接，同速度字段，走 charger.Dock_Move*
CMD_SERVO = 4       # [3:4] = 脉宽 us，固件夹到 500~2500
CMD_SELFTEST = 5    # 无字段
CMD_EMERGENCY = 7   # 急停，锁存 —— 只能靠复位或 CMD=8 解
CMD_RELEASE = 8     # 解除急停

TX_LEN = 11
RX_LEN = 24
DIST_LEN = 19       # 超声波帧，固件无条件跟在底盘帧后；割草机没这硬件，不解析

SERVO_MIN_US = 500
SERVO_MAX_US = 2500

# IMU 原始值 → 物理单位。量程由固件配死：
#   lawn-mower/HARDWARE/ICM20948/ICM20948.c:107  REG_VAL_BIT_GYRO_FS_500DPS  → ±500 °/s
#   lawn-mower/HARDWARE/ICM20948/ICM20948.c:110  REG_VAL_BIT_ACCEL_FS_2g     → ±2 g
# 加速度那个除数固件里也有现成的，lawn-mower/BALANCE/balance_task.c:693。
GYROSCOPE_RATIO = 0.00026644    # ±500 °/s
ACCEL_RATIO = 1671.84           # ±2 g


def bcc(data, n):
    """前 n 字节逐字节异或。同固件 data_task.c:45 的 Check_BCC。"""
    v = 0
    for i in range(n):
        v ^= data[i]
    return v


def _i16(d, i):
    v = (d[i] << 8) | d[i + 1]
    return v - 0x10000 if v & 0x8000 else v


def _u16(d, i):
    return (d[i] << 8) | d[i + 1]


def _s16_bytes(v):
    """16 位有符号 → 大端两字节。负数靠 & 0xFF 落成补码，
    与固件 XYZ_Target_Speed_transition 的读法一致。"""
    v = max(-32768, min(32767, int(round(v))))
    return [(v >> 8) & 0xFF, v & 0xFF]


def build_frame(cmd, data=None):
    f = bytearray(TX_LEN)
    f[0] = FRAME_HEADER
    f[1] = cmd
    f[2] = 0
    for i in range(6):
        f[3 + i] = data[i] if data and i < len(data) else 0
    f[9] = bcc(f, 9)
    f[10] = FRAME_TAIL
    return bytes(f)


def build_vel_frame(vx, vy, vz):
    b = _s16_bytes(vx * 1000) + _s16_bytes(vy * 1000) + _s16_bytes(vz * 1000)
    return build_frame(CMD_VEL, b)


def build_servo_frame(pulse_us):
    pulse_us = max(SERVO_MIN_US, min(SERVO_MAX_US, int(pulse_us)))
    return build_frame(CMD_SERVO, _s16_bytes(pulse_us))


def build_selftest_frame():
    return build_frame(CMD_SELFTEST)


def build_emergency_frame():
    return build_frame(CMD_EMERGENCY)


def build_release_frame():
    return build_frame(CMD_RELEASE)


class UplinkParser:
    """24 字节底盘帧的滚动缓冲解析器。

    **不能**用「上一字节是帧尾，当前字节才是帧头」那种判据。固件 data_task.c:242
    在每帧底盘数据后面**无条件**再发 19 字节超声波帧（data_task.h:23-24，头 0xFA
    尾 0xFC），所以每个 0x7B 的前一字节是 0xFC —— 按那条判据 count 永远起不来，
    一帧都解不出来。

    改成扫 0x7B 取 24 字节、用帧尾 + BCC 双重校验，不通过就丢 1 字节重扫。
    假阳性约 1/65536，且不会失步。
    """

    def __init__(self):
        self._buf = bytearray()
        self.dropped = 0     # 丢了几个假帧头，观测串口质量用

    def feed(self, chunk):
        self._buf += chunk
        out = []
        while True:
            i = self._buf.find(FRAME_HEADER)
            if i < 0:
                self._buf.clear()
                break
            if i:
                del self._buf[:i]
            if len(self._buf) < RX_LEN:
                break
            if self._buf[RX_LEN - 1] == FRAME_TAIL and bcc(self._buf, RX_LEN - 2) == self._buf[RX_LEN - 2]:
                out.append(bytes(self._buf[:RX_LEN]))
                del self._buf[:RX_LEN]
            else:
                self.dropped += 1
                del self._buf[:1]
        return out


def parse_uplink(frame):
    """24 字节底盘帧 → dict。布局见 data_task.c:98-137。

    [1] 是 robot_control.FlagStop —— 源码里那句 //set aside //预留位 是错的，
    data_task.c:99 明确给它赋了值。
    """
    return {
        'flag_stop': frame[1],
        'vx': _i16(frame, 2) / 1000.0,
        'vy': _i16(frame, 4) / 1000.0,
        'vz': _i16(frame, 6) / 1000.0,
        'accel': tuple(_i16(frame, 8 + 2 * i) / ACCEL_RATIO for i in range(3)),
        'gyro': tuple(_i16(frame, 14 + 2 * i) * GYROSCOPE_RATIO for i in range(3)),
        'voltage': _u16(frame, 20) / 1000.0,
    }


class Mahony:
    """Mahony 互补滤波：用加速度计修陀螺零偏，出四元数。公开算法。

    kp=1.0 / ki=0 是这类滤波的常用起点。更新的步长用**实测 dt** 而不是写死
    1/20 —— 帧率一偏，写死的那个会发散，实测的只是滤波强弱变化。
    """

    def __init__(self, kp=1.0, ki=0.0):
        self.kp = kp
        self.ki = ki
        self.q = [1.0, 0.0, 0.0, 0.0]
        self._fb = [0.0, 0.0, 0.0]

    def update(self, gx, gy, gz, ax, ay, az, dt):
        q0, q1, q2, q3 = self.q
        if not (ax == 0.0 and ay == 0.0 and az == 0.0):
            n = math.sqrt(ax * ax + ay * ay + az * az)
            ax, ay, az = ax / n, ay / n, az / n
            halfvx = q1 * q3 - q0 * q2
            halfvy = q0 * q1 + q2 * q3
            halfvz = q0 * q0 - 0.5 + q3 * q3
            ex = ay * halfvz - az * halfvy
            ey = az * halfvx - ax * halfvz
            ez = ax * halfvy - ay * halfvx
            if self.ki > 0.0:
                self._fb = [self._fb[i] + self.ki * e * dt for i, e in enumerate((ex, ey, ez))]
            else:
                self._fb = [0.0, 0.0, 0.0]
            gx += self.kp * ex + self._fb[0]
            gy += self.kp * ey + self._fb[1]
            gz += self.kp * ez + self._fb[2]
        h = 0.5 * dt
        gx *= h
        gy *= h
        gz *= h
        self.q = [
            q0 + (-q1 * gx - q2 * gy - q3 * gz),
            q1 + (q0 * gx + q2 * gz - q3 * gy),
            q2 + (q0 * gy - q1 * gz + q3 * gx),
            q3 + (q0 * gz + q1 * gy - q2 * gx),
        ]
        n = math.sqrt(sum(c * c for c in self.q))
        self.q = [c / n for c in self.q]
        return self.q
