import math
import numpy as np
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt, QTimer
import pyqtgraph as pg


class MappingWidget(QWidget):
    """2D 建图 — 接 ROS2 /map"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(500, 500)
        self.resolution = 0.05
        self.grid_size = 600
        self.grid = np.full((self.grid_size, self.grid_size), -1, dtype=np.int8)
        self.x = self.grid_size / 2 * self.resolution
        self.y = self.grid_size / 2 * self.resolution
        self.theta = 0.0
        self._buf_vx = []; self._buf_vz = []
        self._init_ui()
        self._integrate_timer = QTimer()
        self._integrate_timer.timeout.connect(self._integrate); self._integrate_timer.start(50)
        self._draw_timer = QTimer()
        self._draw_timer.timeout.connect(self._redraw); self._draw_timer.start(200)

    def _init_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0); layout.setSpacing(2)
        self.setLayout(layout)
        info = QHBoxLayout()
        self.label_pos = QLabel("X: 0.00  Y: 0.00  O: 0")
        self.label_pos.setStyleSheet("color:#c0c8d0; font-size:13px; font-weight:bold; padding:4px;")
        self.label_pos.setAlignment(Qt.AlignCenter)
        info.addWidget(self.label_pos); layout.addLayout(info)
        self.plot = pg.PlotWidget()
        self.plot.setBackground('#1a1a2e')
        self.plot.showGrid(x=False, y=False)
        self.plot.setAspectLocked(True)
        self.plot.invertY(False)
        layout.addWidget(self.plot, 1)
        self.img = pg.ImageItem()
        self.img.setLookupTable(self._colormap())
        self.img.setLevels([0, 255])
        self.plot.addItem(self.img)
        self.robot_marker = self.plot.plot([], [], pen=None, symbol='o',
                                            symbolSize=20, symbolBrush='#00ff41', symbolPen='w')

    def _colormap(self):
        cmap = np.zeros((256, 4), dtype=np.uint8)
        cmap[0] = [30, 50, 80, 255]  # unknown=blue
        for i in range(1, 256):
            cmap[i] = [255, 255, 255, 255]  # free=white
        for i in range(180, 256):
            t = (i - 180) / 75.0
            cmap[i] = [int(50 + (1-t)*205), int(50*(1-t)), int(50*(1-t)), 255]  # obstacle=dark red
        cmap[255] = [0, 0, 0, 255]  # solid obstacle=black
        return cmap

    def feed_speed(self, vx, vz):
        self._buf_vx.append(vx); self._buf_vz.append(vz)

    def feed_map(self, width, height, resolution, ox, oy, data):
        self.resolution = resolution
        new_size = max(width, height)
        if new_size != self.grid_size:
            self.grid_size = new_size
            self.plot.removeItem(self.img)
            self.img = pg.ImageItem()
            self.img.setLookupTable(self._colormap())
            self.img.setLevels([0, 255])
            self.plot.addItem(self.img)
        raw = np.array(list(data), dtype=np.uint8)
        self.grid = raw.reshape((height, width))
        # 位置 = 地图原点（ox,oy）的相反数
        self.x = -ox + (width * resolution / 2)
        self.y = -oy + (height * resolution / 2)
        self._redraw()

    def _integrate(self):
        if self._buf_vx:
            n = len(self._buf_vx)
            vx = sum(self._buf_vx) / n; vz = sum(self._buf_vz) / n
            self._buf_vx.clear(); self._buf_vz.clear()
            dt = 0.05
            self.x += vx * math.cos(self.theta) * dt
            self.y += vx * math.sin(self.theta) * dt
            self.theta += vz * dt

    def _redraw(self):
        try: self.img.setImage(self.grid.T)
        except RuntimeError:
            self.img = pg.ImageItem()
            self.img.setLookupTable(self._colormap())
            self.img.setLevels([0, 255])
            self.plot.addItem(self.img)
            self.img.setImage(self.grid.T)
        self.img.setRect(pg.QtCore.QRectF(0, 0,
                          self.grid_size * self.resolution,
                          self.grid_size * self.resolution))
        self.robot_marker.setData([self.x], [self.y])
        self.plot.setXRange(self.x - 8, self.x + 8)
        self.plot.setYRange(self.y - 8, self.y + 8)
        deg = math.degrees(self.theta) % 360
        self.label_pos.setText(f"X: {self.x:.2f}  Y: {self.y:.2f}  O: {deg:.0f}")

    def reset(self):
        self.x = self.grid_size / 2 * self.resolution
        self.y = self.grid_size / 2 * self.resolution
        self.theta = 0.0
        self.grid.fill(0)
        self._buf_vx.clear(); self._buf_vz.clear()
        self._redraw()

    def closeEvent(self, event):
        self._integrate_timer.stop(); self._draw_timer.stop()
        event.accept()
