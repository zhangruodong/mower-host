"""协议层自检 —— 不依赖 ROS，直接 `python test/test_protocol.py` 就能跑。

每个断言都对着固件源码，出处见 docs/协议.md。
"""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from mower_bridge.protocol import (  # noqa: E402
    CMD_VEL, CMD_SERVO, CMD_EMERGENCY, CMD_RELEASE, DIST_LEN,
    UplinkParser, Mahony, bcc, build_frame, build_vel_frame, build_servo_frame,
    build_selftest_frame, build_emergency_frame, build_release_frame, parse_uplink,
)

# 固件 data_task.h:23-24
DIST_HEADER = 0xFA
DIST_TAIL = 0xFC


def uplink(vx=0, vy=0, vz=0, acc=(0, 0, 0), gyro=(0, 0, 0), volt=24000, flag=0):
    """照 data_task.c:98-137 组一个 24 字节底盘帧（原始单位）。"""
    b = bytearray(24)
    b[0] = 0x7B
    b[1] = flag
    for i, v in enumerate((vx, vy, vz)):
        b[2 + 2 * i] = (v >> 8) & 0xFF
        b[3 + 2 * i] = v & 0xFF
    for i, v in enumerate(acc):
        b[8 + 2 * i] = (v >> 8) & 0xFF
        b[9 + 2 * i] = v & 0xFF
    for i, v in enumerate(gyro):
        b[14 + 2 * i] = (v >> 8) & 0xFF
        b[15 + 2 * i] = v & 0xFF
    b[20] = (volt >> 8) & 0xFF
    b[21] = volt & 0xFF
    b[22] = bcc(b, 22)
    b[23] = 0x7D
    return bytes(b)


def distance_block(with_stray_header=False):
    """照 data_task.c:169-189：19 字节超声波帧，固件无条件跟在底盘帧后。
    割草机没这硬件，内容是全零 + BCC。"""
    b = bytearray(DIST_LEN)
    b[0] = DIST_HEADER
    if with_stray_header:
        b[3] = 0x7B          # 伪造一个帧头，考验重同步
    b[DIST_LEN - 2] = bcc(b, DIST_LEN - 2)
    b[DIST_LEN - 1] = DIST_TAIL
    return bytes(b)


def test_vel_frame_bytes():
    # +0.5 m/s → 500 = 0x01F4；BCC = 7B^01^F4 = 0x8E
    assert build_vel_frame(0.5, 0.0, 0.0) == bytes(
        [0x7B, 0x00, 0x00, 0x01, 0xF4, 0x00, 0x00, 0x00, 0x00, 0x8E, 0x7D])


def test_negative_velocity():
    """-0.5 m/s → 补码 0xFE0C。原来的实现 `vx_mm >> 8` 对负数得负数，
    塞进 bytearray 会直接抛 ValueError。"""
    f = build_vel_frame(-0.5, -1.0, -2.0)
    assert f[3:5] == bytes([0xFE, 0x0C])
    assert f[5:7] == bytes([0xFC, 0x18])       # -1000
    assert f[7:9] == bytes([0xF8, 0x30])       # -2000
    assert f[9] == bcc(f, 9)


def test_servo_clamp():
    # 固件自己也夹，这里夹是为了发出去的帧就是实的
    assert build_servo_frame(100)[3:5] == bytes([0x01, 0xF4])     # →500
    assert build_servo_frame(9999)[3:5] == bytes([0x09, 0xC4])    # →2500
    assert build_servo_frame(1500)[3:5] == bytes([0x05, 0xDC])


def test_bcc_and_shape():
    for f in (build_frame(CMD_VEL), build_selftest_frame(),
              build_emergency_frame(), build_release_frame(), build_servo_frame(1500)):
        assert len(f) == 11
        assert f[0] == 0x7B and f[10] == 0x7D and f[2] == 0
        assert f[9] == bcc(f, 9)
    assert build_emergency_frame()[1] == 7
    assert build_release_frame()[1] == 8       # 固件 uartx_callback.c:151


def test_parse_uplink_roundtrip():
    d = parse_uplink(uplink(vx=-1234, vy=567, vz=-89, volt=24500, flag=1))
    assert abs(d['vx'] + 1.234) < 1e-9
    assert abs(d['vy'] - 0.567) < 1e-9
    assert abs(d['vz'] + 0.089) < 1e-9
    assert abs(d['voltage'] - 24.5) < 1e-9
    assert d['flag_stop'] == 1


def test_parser_real_stream_shape():
    """固件每周期吐 [24 字节底盘帧][19 字节超声波帧]，所以每个 0x7B 的前一字节是
    0xFC —— 任何要求"前一字节是帧尾"的判据在这里都会一帧都解不出来。
    这条用例就是钉住这个形状的。"""
    p = UplinkParser()
    frames = p.feed((uplink(vx=100) + distance_block()) * 5)
    assert len(frames) == 5, frames
    assert p.dropped == 0


def test_parser_resyncs_mid_stream():
    p = UplinkParser()
    stream = (uplink() + distance_block()) * 4
    got = p.feed(stream[7:])                  # 从半路切进去，头一帧是残的
    assert len(got) == 3, got                 # 一个周期内重同步，后面的帧一帧不丢


def test_parser_survives_false_header():
    """超声波帧里混进一个 0x7B（真机上就是某个字段恰好等于帧头）。
    重同步必须走通，且真帧一帧都不能丢。"""
    p = UplinkParser()
    frames = p.feed((uplink(vx=200) + distance_block(with_stray_header=True)) * 4)
    assert len(frames) == 4, frames
    # 末周期那个假帧头后面剩的字节不足一帧，会被留到下次判，所以是 3 不是 4
    assert p.dropped >= 1, p.dropped


def test_parser_splits_across_reads():
    """串口按块读，帧会被切在两块中间。"""
    p = UplinkParser()
    stream = (uplink() + distance_block()) * 3
    got = []
    for i in range(0, len(stream), 7):
        got += p.feed(stream[i:i + 7])
    assert len(got) == 3, got


def test_mahony_static_stays_level():
    """静止时只有重力（+z），四元数应稳在单位附近。"""
    m = Mahony()
    for _ in range(200):
        q = m.update(0.0, 0.0, 0.0, 0.0, 0.0, 16384.0, 0.05)   # 1g
    assert q[0] > 0.99, q


def test_mahony_integrates_yaw():
    """绕 z 转 1 rad/s，1 秒后偏航角应接近 1 rad。"""
    m = Mahony()
    for _ in range(20):
        q = m.update(0.0, 0.0, 1.0, 0.0, 0.0, 16384.0, 0.05)
    import math
    yaw = math.atan2(2 * (q[0] * q[3] + q[1] * q[2]),
                     1 - 2 * (q[2] ** 2 + q[3] ** 2))
    assert 0.9 < yaw < 1.1, yaw


if __name__ == '__main__':
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('ok   %s' % name)
            except AssertionError as e:
                fails += 1
                print('FAIL %s: %s' % (name, e))
    print('\n%s' % ('全过' if not fails else '%d 个失败' % fails))
    sys.exit(1 if fails else 0)
