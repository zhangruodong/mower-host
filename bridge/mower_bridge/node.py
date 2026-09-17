"""chassis_bridge —— 底盘串口桥，mower-host 独占这个串口。

收发全包：订阅语义话题 → 组 11 字节帧 → 写串口；读串口 → 解 24 字节帧 →
发 /odom /imu/data_raw /PowerVoltage。

三件不能省的事：
  * 帧同步用滚动缓冲 + 帧尾/BCC 双校验。固件吐的字节流里每个 0x7B 前面是
    0xFC，任何要求"前一字节是帧尾"的判据都会一帧都解不出来
    （见 protocol.UplinkParser 的说明）。
  * **带 cmd_vel 超时看门狗**。固件其实有指令丢失检测（balance_task.c:58-64），
    但门控在 SysVal.SecurityLevel==0，而 sys.c:41 把默认值设成 1（"指令丢失后
    按最后一次速度指令运动"），全固件没有第二处赋值，上位机也改不了它
    （frame[2] 没人读）。所以那道保护永不生效 —— 停发速度指令车会按最后速度
    一直跑，只能靠这里兜。
  * 不写 TF。odom_combined → base_footprint 由 ekf_node 出（publish_tf: true）。
"""
import math
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, Int16, Empty

from .protocol import (
    Mahony, UplinkParser, parse_uplink,
    build_vel_frame, build_servo_frame, build_selftest_frame,
    build_emergency_frame, build_release_frame,
)

# 两套协方差：静止时更信编码器，运动时更信 IMU。
# 位置 xy 的方差静止时给 1e-9、动起来给 1e-3；姿态在 2D 下不可信，一律 1e6。
_COV_POSE_STILL = [1e-9, 0, 0, 0, 0, 0,
                   0, 1e-3, 1e-9, 0, 0, 0,
                   0, 0, 1e6, 0, 0, 0,
                   0, 0, 0, 1e6, 0, 0,
                   0, 0, 0, 0, 1e6, 0,
                   0, 0, 0, 0, 0, 1e-9]
_COV_POSE_MOVING = [1e-3, 0, 0, 0, 0, 0,
                    0, 1e-3, 0, 0, 0, 0,
                    0, 0, 1e3, 0, 0, 0,
                    0, 0, 0, 1e6, 0, 0,
                    0, 0, 0, 0, 1e6, 0,
                    0, 0, 0, 0, 0, 1e3]
_COV_TWIST_STILL = [1e-9, 0, 0, 0, 0, 0,
                    0, 1e-3, 1e-9, 0, 0, 0,
                    0, 0, 1e6, 0, 0, 0,
                    0, 0, 0, 1e6, 0, 0,
                    0, 0, 0, 0, 1e6, 0,
                    0, 0, 0, 0, 0, 1e-9]
_COV_TWIST_MOVING = [1e-3, 0, 0, 0, 0, 0,
                     0, 1e-3, 0, 0, 0, 0,
                     0, 0, 1e6, 0, 0, 0,
                     0, 0, 0, 1e6, 0, 0,
                     0, 0, 0, 0, 1e6, 0,
                     0, 0, 0, 0, 0, 1e3]

# 固件上电后 uartx_callback.c:186 会丢掉 CONTROL_DELAY 之内的所有命令，
# Time_count 在 100 Hz 的 balance_task 里自增，CONTROL_DELAY=1000 → 10 秒。
BOOT_SILENCE_SEC = 10.0


class ChassisBridge(Node):

    def __init__(self):
        super().__init__('chassis_bridge')

        p = self.declare_parameter
        self.port = p('port', '/dev/mower_chassis').value
        self.baud = p('baud', 115200).value
        self.cmd_vel_timeout = p('cmd_vel_timeout', 0.5).value
        self.odom_frame_id = p('odom_frame_id', 'odom_combined').value
        self.robot_frame_id = p('robot_frame_id', 'base_footprint').value
        self.gyro_frame_id = p('gyro_frame_id', 'gyro_link').value
        # 里程计修正系数。默认全 1.0 = **没标定过**，是留给实测的旋钮
        # （真实车轮直径、地面摩擦和标称总有偏差）。走 1 m 差多少按比例改。
        self.odom_x_scale = p('odom_x_scale', 1.0).value
        self.odom_y_scale = p('odom_y_scale', 1.0).value
        self.odom_z_scale_positive = p('odom_z_scale_positive', 1.0).value
        self.odom_z_scale_negative = p('odom_z_scale_negative', 1.0).value

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.volt_pub = self.create_publisher(Float32, '/PowerVoltage', 10)

        self.create_subscription(Empty, '/estop_release', self._on_release, 10)
        self.create_subscription(Empty, '/emergency_stop', self._on_emergency, 10)
        self.create_subscription(Empty, '/selftest_cmd', self._on_selftest, 10)
        self.create_subscription(Int16, '/servo_cmd', self._on_servo, 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

        self._parser = UplinkParser()
        self._ahrs = Mahony()
        self._lock = threading.Lock()
        self._x = self._y = self._yaw = 0.0
        self._last_rx = None
        self._last_cmd = None
        self._stopped = False
        self._volt_count = 0

        try:
            import serial
        except ImportError:
            raise SystemExit('缺 pyserial：pip install pyserial')
        try:
            self._ser = serial.Serial(self.port, self.baud, timeout=0.1)
        except Exception as e:
            # 开不了口就明确死掉。静默跑着什么都不做比崩了更危险 ——
            # 用户会以为桥在工作。
            raise SystemExit('打开串口 %s 失败: %s' % (self.port, e))

        self._running = True
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.create_timer(0.1, self._watchdog)
        # rclpy 没有一次性定时器 —— 不销毁它每 10 秒会再报一次"静默期结束"。
        self._boot_timer = self.create_timer(BOOT_SILENCE_SEC, self._boot_done)

        self.get_logger().info(
            'chassis_bridge 已起：%s@%d。固件上电后 %.0f 秒内不收命令，'
            '先别发指令。' % (self.port, self.baud, BOOT_SILENCE_SEC))

    # ---- 串口 ----

    def _read_loop(self):
        while self._running:
            try:
                chunk = self._ser.read(64)
            except Exception as e:
                self.get_logger().error('串口读失败，桥退出: %s' % e)
                self._running = False
                break
            if not chunk:
                continue
            for frame in self._parser.feed(chunk):
                self._on_frame(frame)

    def _write(self, frame):
        with self._lock:
            try:
                self._ser.write(frame)
            except Exception as e:
                self.get_logger().error('串口写失败: %s' % e)

    # ---- 收帧 → 话题 ----

    def _on_frame(self, frame):
        d = parse_uplink(frame)
        now = self.get_clock().now()
        # dt 用实测帧间隔，不用写死的 1/20 —— 帧率一偏，写死的那个就发散。
        dt = 0.0 if self._last_rx is None else (now - self._last_rx).nanoseconds * 1e-9
        self._last_rx = now
        dt = min(max(dt, 1e-4), 0.5)
        self._publish_odom(d, dt, now)
        self._publish_imu(d, dt, now)
        self._publish_voltage(d)

    def _publish_odom(self, d, dt, now):
        vx = d['vx'] * self.odom_x_scale
        vy = d['vy'] * self.odom_y_scale
        vz = d['vz'] * (self.odom_z_scale_positive if d['vz'] >= 0
                        else self.odom_z_scale_negative)
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        self._x += (vx * c - vy * s) * dt
        self._y += (vx * s + vy * c) * dt
        self._yaw += vz * dt

        m = Odometry()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.odom_frame_id
        m.child_frame_id = self.robot_frame_id
        m.pose.pose.position.x = self._x
        m.pose.pose.position.y = self._y
        # 偏航只进四元数，不进 position.z —— 把 yaw 塞进 z 会让 EKF 拿到假高度。
        m.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        m.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        m.twist.twist.linear.x = vx
        m.twist.twist.linear.y = vy
        m.twist.twist.angular.z = vz
        still = not (d['vx'] or d['vy'] or d['vz'])
        m.pose.covariance = _COV_POSE_STILL if still else _COV_POSE_MOVING
        m.twist.covariance = _COV_TWIST_STILL if still else _COV_TWIST_MOVING
        self.odom_pub.publish(m)

    def _publish_imu(self, d, dt, now):
        q0, q1, q2, q3 = self._ahrs.update(d['gyro'][0], d['gyro'][1], d['gyro'][2],
                                           d['accel'][0], d['accel'][1], d['accel'][2], dt)
        m = Imu()
        m.header.stamp = now.to_msg()
        m.header.frame_id = self.gyro_frame_id
        m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q1, q2, q3, q0
        # 只信 yaw：单 IMU 的 roll/pitch 靠加速度计，车体振动下不可信。
        # ekf.yaml 的 imu0_config 也只取了 yaw 和 yaw_vel。
        m.orientation_covariance[0] = 1e6
        m.orientation_covariance[4] = 1e6
        m.orientation_covariance[8] = 1e-6
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = d['gyro']
        m.angular_velocity_covariance[0] = 1e6
        m.angular_velocity_covariance[4] = 1e6
        m.angular_velocity_covariance[8] = 1e-6
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = d['accel']
        self.imu_pub.publish(m)

    def _publish_voltage(self, d):
        # 电压每 11 帧发一次就够 —— 20 Hz 下来是 1.8 Hz，电池不会那么快变。
        self._volt_count += 1
        if self._volt_count > 10:
            self._volt_count = 0
            m = Float32()
            m.data = d['voltage']
            self.volt_pub.publish(m)

    # ---- 话题 → 发帧 ----

    def _on_cmd_vel(self, msg):
        self._last_cmd = self.get_clock().now()
        self._stopped = False
        self._write(build_vel_frame(msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_servo(self, msg):
        self._write(build_servo_frame(msg.data))

    def _on_selftest(self, msg):
        self._write(build_selftest_frame())

    def _on_emergency(self, msg):
        self._write(build_emergency_frame())

    def _on_release(self, msg):
        self._write(build_release_frame())

    # ---- 看门狗 ----

    def _watchdog(self):
        if self._last_cmd is None or self._stopped:
            return
        idle = (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9
        if idle <= self.cmd_vel_timeout:
            return
        self.get_logger().warning(
            '/cmd_vel 停了 %.1fs（> %.2fs），发零速度停车'
            % (idle, self.cmd_vel_timeout))
        self._write(build_vel_frame(0.0, 0.0, 0.0))
        self._stopped = True

    def _boot_done(self):
        self._boot_timer.cancel()
        self.destroy_timer(self._boot_timer)
        self.get_logger().info('固件静默期结束，现在可以发指令了')

    # ---- 收尾 ----

    def shutdown(self):
        self._running = False
        if self._last_cmd is not None:
            # 退出前把速度归零，别让车带着最后一条指令跑。
            self._write(build_vel_frame(0.0, 0.0, 0.0))
            time.sleep(0.05)
        try:
            self._ser.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = ChassisBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
