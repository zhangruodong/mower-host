"""简单雷达跟随 — 只在有目标时才发 /cmd_vel，不抢蓝牙"""
import math
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarFollower(Node):
    """雷达跟随节点：找前方最近物体，保持距离"""

    SAFE_DIST = 0.30      # 跟随目标距离 (m)
    MAX_LINEAR = 0.25      # 最大前进速度 (m/s)
    MAX_ANGULAR = 0.4      # 最大转向速度 (rad/s)
    FRONT_ANGLE = 30       # 只看前方 ±30°
    MIN_POINTS = 3         # 最少有效点数

    def __init__(self):
        super().__init__('simple_lidar_follower')
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(LaserScan, '/scan', self._cb, qos)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._running = True
        self.get_logger().info('Simple lidar follower ready')

    def stop(self):
        self._running = False

    def _cb(self, msg: LaserScan):
        if not self._running:
            return

        ranges = msg.ranges
        if not ranges:
            return

        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
        half = math.radians(self.FRONT_ANGLE / 2)

        # 收集前方有效点
        pts = []
        for i, r in enumerate(ranges):
            if not (msg.range_min < r < msg.range_max):
                continue
            a = angle_min + i * angle_inc
            if -half <= a <= half:
                pts.append((r, a))

        # 不够点数 → 不发指令（蓝牙可接管）
        if len(pts) < self.MIN_POINTS:
            return

        # 找最近的点
        closest = min(pts, key=lambda p: p[0])
        dist, angle = closest

        # 太远 (> 3m) 或太近 (< 0.1m) → 不发
        if dist > 3.0 or dist < 0.1:
            return

        # 计算控制量
        cmd = Twist()
        err = dist - self.SAFE_DIST

        # 距离控制
        if err > 0.05:    # 太远 → 前进
            cmd.linear.x = min(err * 0.6, self.MAX_LINEAR)
        elif err < -0.05:  # 太近 → 后退
            cmd.linear.x = max(err * 0.6, -0.15)

        # 角度修正
        if abs(angle) > 0.03:
            cmd.angular.z = angle * 1.5
            if abs(cmd.angular.z) > self.MAX_ANGULAR:
                cmd.angular.z = self.MAX_ANGULAR if cmd.angular.z > 0 else -self.MAX_ANGULAR

        self.pub.publish(cmd)


def spin_follower(node: LidarFollower):
    """在独立线程跑跟随节点"""
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while node._running and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
    finally:
        executor.remove_node(node)
        node.destroy_node()
