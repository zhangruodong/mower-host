"""自动回充状态机（简化版）"""
from PyQt5.QtCore import QTimer


class DockController:
    def __init__(self, protocol):
        self.protocol = protocol
        self.on_servo_cmd = None
        # 摄像头看到 AprilTag 时置位。以前是个恒 False 的**类属性**，现在是实例属性。
        self.tag_seen = False
        self.on_tag_found = None
        self.on_done = None
        self._state = "IDLE"
        self._timer = None
        self._servo_pos = 1500
        self._direction = 1

    @property
    def servo_pos(self): return self._servo_pos

    @property
    def state(self): return self._state

    def start(self):
        self._state = "SWEEPING"
        if self._timer is None:
            self._timer = QTimer()
            self._timer.timeout.connect(self._tick)
        self._timer.start(30)

    def stop(self):
        if self._timer:
            self._timer.stop()
        self._state = "IDLE"

    def tag_detected(self):
        """CameraWidget 的 on_tag_seen 回调。

        以前 main_window.py:437/1135 挂的是这个方法名，但它不存在 —— 槽里抛出的
        异常会被 PyQt5 的 qFatal 接走，也就是**一看到 AprilTag 整个面板直接退出**。
        """
        self.tag_seen = True
        if self._state == "SWEEPING":
            # 找到了就不再左右扫，停下来对准。后续验证/旋转/退桩那段还没实现，
            # 状态会停在 VERIFYING —— 比假装走完整个流程诚实。
            self._state = "VERIFYING"
            if self.on_tag_found:
                self.on_tag_found()

    def _tick(self):
        if self._state == "SWEEPING":
            self._servo_pos += self._direction * 8
            if self._servo_pos >= 2400:
                self._servo_pos = 2400; self._direction = -1
            elif self._servo_pos <= 600:
                self._servo_pos = 600; self._direction = 1
            if self.on_servo_cmd:
                self.on_servo_cmd(self._servo_pos)
