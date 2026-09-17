from PyQt5.QtWidgets import QWidget
from PyQt5.QtCore import Qt, QRectF
from PyQt5.QtGui import QPainter, QColor, QFont, QPen
import math


class SpeedGauge(QWidget):

    def __init__(self, title="", max_speed=1.0, parent=None):
        super().__init__(parent)
        self.title = title
        self.value = 0.0
        self.max_speed = max_speed
        self.setMinimumSize(180, 140)

    def set_value(self, v):
        self.value = max(-self.max_speed, min(self.max_speed, v))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w // 2, h - 10
        r = min(w // 2 - 10, h - 20)

        arc = QRectF(cx - r, cy - r, 2 * r, 2 * r)
        p.setPen(QPen(QColor("#3d4552"), 10))
        p.drawArc(arc, 0 * 16, 180 * 16)

        needle_angle = 180 - int((self.value / self.max_speed) * 180 if self.max_speed > 0 else 0)
        needle_angle = max(0, min(180, needle_angle))
        rad = math.radians(needle_angle)
        nx = cx + int(r * 0.7 * math.cos(rad))
        ny = cy - int(r * 0.7 * math.sin(rad))
        p.setPen(QPen(QColor("#ffffff"), 3))
        p.drawLine(cx, cy, nx, ny)
        p.setBrush(QColor("#ffffff"))
        p.drawEllipse(cx - 5, cy - 5, 10, 10)

        p.setPen(QColor("#c0c8d0"))
        f = QFont("Arial", 12)
        p.setFont(f)
        p.drawText(QRectF(cx - r, cy - r + 10, 2 * r, 20), Qt.AlignCenter, self.title)
        val_str = f"{self.value:.2f} m/s"
        f2 = QFont("Arial", 16, QFont.Bold)
        p.setFont(f2)
        p.setPen(QColor("#4ade80"))
        p.drawText(QRectF(cx - r, cy - r + 30, 2 * r, 30), Qt.AlignCenter, val_str)

        p.setPen(QColor("#64748b"))
        f3 = QFont("Arial", 8); p.setFont(f3)
        p.drawText(QRectF(cx - r - 5, cy - 15, 30, 14), Qt.AlignRight, "0")
        p.drawText(QRectF(cx + r - 25, cy - 15, 30, 14), Qt.AlignLeft, f"{self.max_speed:.1f}")
        p.end()
