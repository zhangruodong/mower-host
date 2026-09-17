import os
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
os.environ.pop("QT_PLUGIN_PATH", None)

import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer
from core.ros2_client import Ros2Client
from core.protocol import Protocol
from core.dock_controller import DockController
from ui.main_window import MainWindow


def main():
    app = QApplication(sys.argv)

    serial_mgr = Ros2Client()
    # 先不 start —— 等窗口出来了再启动 ROS2
    protocol = Protocol()

    dock_ctrl = DockController(protocol)

    window = MainWindow(serial_mgr, protocol, dock_ctrl)
    window.show()

    # 窗口显示 500ms 后再启动 ROS2 后台检测
    window.log_send("--- 碧浪牧影 割草机器人控制面板 v2.0 ---")
    window.log_send("--- 硬件 ---")
    window.log_send("STM32F407VE+FreeRTOS已就绪")
    window.log_send("雷达: RPLidar C1 (10Hz, 16m)")
    window.log_send("相机: Orbbec 3D 深度相机")
    window.log_send("通信: 话题 → chassis_bridge → 11位元帧 UART3@115200 BCC校验")
    window.log_send("--- 软件栈 ---")
    window.log_send("UI: PyQt5 + OpenCV + pyqtgraph + numpy")
    window.log_send("ROS2: Humble + rclpy + nav_msgs + tf2")
    window.log_send("SLAM: slam_toolbox (CeresSolver, online_async)")
    window.log_send("EKF: robot_localization (/odom_combined)")
    window.log_send("Nav2: AMCL + SmacPlannerHybrid + MPPI")
    window.log_send("视觉: YOLOv8 + AprilTag 36h11")
    window.log_send("录像: OpenCV MJPG/AVI @15fps")
    window.log_send("--- 系统 ---")
    window.log_send("树莓派5B + Debian Bookworm")
    window.log_send("Docker + colcon + FastRTPS")
    window.log_send("正在准备，请稍后...")
    QTimer.singleShot(500, serial_mgr.start)
    window.log_send("后台检测启动中...")

    dock_ctrl.on_servo_cmd = lambda pulse: serial_mgr.send(
        protocol.build_servo_frame(pulse))

    serial_mgr.sensor_updated.connect(
        lambda l, r, batt, servo: (
            setattr(window, '_speed_left', l),
            setattr(window, '_speed_right', r)))
    serial_mgr.map_updated.connect(
        lambda w, h, res, ox, oy, data:
            window._mapping_widget.feed_map(w, h, res, ox, oy, data)
            if hasattr(window, '_mapping_widget') and window._mapping_widget else None)

    serial_mgr.status_signal.connect(window.log_send)

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
