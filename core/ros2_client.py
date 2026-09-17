"""ROS2 客户端 —— 启动时检测各组件就绪状态，逐个上报"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int16, Empty
from PyQt5.QtCore import QObject, pyqtSignal
import threading, random, time

_INIT_LOCK = threading.Lock()
_INITED = False

def _ensure_init():
    global _INITED
    with _INIT_LOCK:
        if not _INITED:
            rclpy.init()
            _INITED = True

class Ros2Client(QObject):
    sensor_updated = pyqtSignal(float, float, float, int)
    map_updated   = pyqtSignal(int, int, float, float, float, bytes)
    status_signal = pyqtSignal(str)
    error_signal  = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self._running = False
        self._node = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._ros_spin, daemon=True)
        self._thread.start()

    def open(self): self.start()
    def close(self): self.stop()

    def stop(self):
        self._running = False

    def _ros_spin(self):
        _ensure_init()
        self._node = _BridgeNode(self)
        executor = MultiThreadedExecutor()
        executor.add_node(self._node)
        while self._running and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

    def send(self, data: bytes):
        if len(data) < 2 or self._node is None: return
        cmd = data[1]
        if cmd == 0:
            vx = _i16(data, 3, 4) / 1000.0
            vy = _i16(data, 5, 6) / 1000.0
            vz = _i16(data, 7, 8) / 1000.0
            msg = Twist(); msg.linear.x = vx; msg.linear.y = vy; msg.angular.z = vz
            self._node.cmd_vel_pub.publish(msg)
        elif cmd == 4:
            p = (data[3]<<8)|data[4]; msg = Int16(); msg.data = p
            self._node.servo_pub.publish(msg)
        elif cmd == 5: self._node.selftest_pub.publish(Empty())
        elif cmd == 7: self._node.emergency_pub.publish(Empty())
        elif cmd == 8: self._node.release_pub.publish(Empty())


class _BridgeNode(Node):
    def __init__(self, parent):
        super().__init__('robot_ui_bridge_' + str(random.randint(1000,9999)))
        self._parent = parent
        qos_s = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cmd_vel_pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self.servo_pub     = self.create_publisher(Int16,  '/servo_cmd', 10)
        self.selftest_pub  = self.create_publisher(Empty,  '/selftest_cmd', 10)
        self.emergency_pub = self.create_publisher(Empty,  '/emergency_stop', 10)
        self.release_pub   = self.create_publisher(Empty,  '/estop_release', 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos_s)
        self.create_subscription(Float32, '/PowerVoltage', self._volt_cb, 10)
        self.create_subscription(OccupancyGrid, '/map', self._map_cb, 10)
        self._vx = 0.0; self._vz = 0.0; self._bat = 22.8

        # 延迟检查就绪状态（在单独线程里跑，每 3 秒一次，最多 5 轮）
        self._checked = set()
        self._status_timer = self.create_timer(3.0, self._check_ready)
        self._check_count = 0

    def _odom_cb(self, msg):
        self._vx = msg.twist.twist.linear.x
        self._vz = msg.twist.twist.angular.z
        self._parent.sensor_updated.emit(self._vx, self._vz, self._bat, 0)

    def _volt_cb(self, msg):
        self._bat = msg.data
        self._parent.sensor_updated.emit(self._vx, self._vz, self._bat, 0)

    def _map_cb(self, msg):
        self._parent.map_updated.emit(msg.info.width, msg.info.height,
            msg.info.resolution, msg.info.origin.position.x,
            msg.info.origin.position.y, bytes(msg.data))

    def _check_ready(self):
        self._check_count += 1
        try:
            topics = self.get_topic_names_and_types()
            topic_dict = {t[0] for t in topics}

            checks = [
                ('/odom',          '底盘桥已就绪 → /odom /imu /PowerVoltage'),
                ('/scan',          '激光雷达已就绪 → /scan (RPLidar C1)'),
                ('/odom_combined', 'EKF 融合已就绪 → /odom_combined'),
                ('/map',           'SLAM 建图已就绪 → /map (slam_toolbox)'),
                ('/cmd_vel',       '运动控制已就绪 → /cmd_vel'),
                ('/servo_cmd',     '舵机控制已就绪 → /servo_cmd'),
                ('/amcl_pose',     'AMCL 定位已就绪 → /amcl_pose'),
                ('/plan',          '路径规划已就绪 → /plan (Nav2 Planner)'),
            ]
            for topic, label in checks:
                if topic not in self._checked and topic in topic_dict:
                    self._checked.add(topic)
                    self._parent.status_signal.emit(label)

            # 「凑够 4 个就算全就绪」原来会对着 9 项的清单谎报，改成数够为止。
            # 建图/Nav2 没在跑的时候 /map /amcl_pose /plan 本来就不该在，这时
            # 与其静默停下，不如把缺的报出来。
            if len(self._checked) == len(checks):
                self._parent.status_signal.emit('全部组件就绪')
                self._status_timer.cancel()
            elif self._check_count >= 10:
                missing = [label for topic, label in checks if topic not in self._checked]
                self._parent.status_signal.emit('未就绪: ' + '; '.join(missing))
                self._status_timer.cancel()
        except:
            pass


def _i16(d, h, l):
    v = (d[h]<<8)|d[l]; return (v-0x10000) if v&0x8000 else v
