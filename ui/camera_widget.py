import cv2, os
from datetime import datetime
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QImage, QPixmap
from core.tag_detector import TagDetector

SAVE_DIR = os.path.expanduser("~/Videos/割草机")


class CameraWidget(QLabel):

    def __init__(self):
        super().__init__()
        self.setAlignment(Qt.AlignCenter)
        self.setText("摄像头未开启")
        self.cap = None
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.tag_detector = TagDetector()
        self.on_tag_seen = None
        self.writer = None
        self.recording = False
        self.last_frame = None  # 供外部录像取帧
        self.detect_green = False  # 绿色检测默认关，割草时才开
        os.makedirs(SAVE_DIR, exist_ok=True)

    def start_camera(self):
        if self.cap is not None: return True
        # 自动搜索摄像头（0 不行就往后试）
        for idx in range(4):
            self.cap = cv2.VideoCapture(idx)
            if self.cap.isOpened():
                break
            self.cap = None
        if self.cap is None:
            self.setText("摄像头打开失败\n请检查连接"); return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.timer.start(30); return True

    def stop_camera(self):
        self.stop_recording(); self.timer.stop()
        if self.cap: self.cap.release()
        self.cap = None; self.clear(); self.setText("摄像头已关闭")

    def start_recording(self, label=""):
        if self.cap is None or not self.cap.isOpened() or self.recording: return None
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        pre = f"{label}_" if label else ""
        path = os.path.join(SAVE_DIR, f"{pre}{ts}.mp4")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        self.writer = cv2.VideoWriter(path, fourcc, 30.0, (640, 480))
        if not self.writer.isOpened(): self.writer = None; return None
        self.recording = True; print(f"[录像] 开始: {path}"); return path

    def stop_recording(self):
        if not self.recording: return
        self.recording = False
        if self.writer: self.writer.release(); self.writer = None; print("[录像] 已保存")

    def is_recording(self): return self.recording

    def update_frame(self):
        if self.tag_detector is None:
            return
        if self.cap is None: return
        ret, frame = self.cap.read()
        if not ret: return

        if self.recording and self.writer:
            cv2.circle(frame, (622, 18), 6, (0, 0, 255), -1)
            ts = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(frame, ts, (560, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            self.writer.write(frame)

        if self.tag_detector:
            self.tag_detector.detect(frame)
        if self.tag_detector.found and self.on_tag_seen:
            self.on_tag_seen()
        frame = self.tag_detector.draw(frame)
        if self.tag_detector.found:
            cv2.putText(frame, "TAG ID=0", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        # 绿色检测（草）
        if self.detect_green:
            frame = self._detect_green(frame)

        self.last_frame = frame.copy()  # 在检测之后存帧，录像带框

        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame.shape
        image = QImage(frame.data, w, h, ch * w, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(image)
        self.setPixmap(pixmap.scaled(self.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    def _detect_green(self, frame):
        """HSV 绿色检测 — 只检测画面下 1/3，框出绿草"""
        h, w = frame.shape[:2]
        roi = frame[int(h * 2 / 3):h, :]  # 只取下 1/3

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        green_mask = cv2.inRange(hsv, (35, 40, 40), (85, 255, 255))

        # 膨胀去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        offset_y = int(h * 2 / 3)
        grass_count = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 200:  # 忽略太小的
                continue
            x, y, cw, ch = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y + offset_y), (x + cw, y + ch + offset_y),
                          (0, 255, 0), 2)
            grass_count += 1

        if grass_count > 0:
            cv2.putText(frame, f"Grass: {grass_count}", (10, h - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return frame

    def closeEvent(self, event): self.stop_camera(); event.accept()
