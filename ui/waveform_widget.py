import pyqtgraph as pg
from collections import deque
from PyQt5.QtCore import QTimer


class WaveformWidget(pg.GraphicsLayoutWidget):

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setBackground('#14161b')
        self._max = 300

        self.p1 = self.addPlot(row=0, col=0, title="前向速度 Vx (m/s)")
        self.p1.showGrid(x=True, y=True, alpha=0.3); self.p1.setLabel('left', 'm/s')
        self.curve_vx = self.p1.plot(pen=pg.mkPen('#4ade80', width=2))

        self.p2 = self.addPlot(row=1, col=0, title="旋转速度 Vz (rad/s)")
        self.p2.showGrid(x=True, y=True, alpha=0.3); self.p2.setLabel('left', 'rad/s')
        self.curve_vz = self.p2.plot(pen=pg.mkPen('#60a5fa', width=2))

        self.p3 = self.addPlot(row=2, col=0, title="电池电压 (V)")
        self.p3.showGrid(x=True, y=True, alpha=0.3); self.p3.setLabel('left', 'V')
        self.curve_b = self.p3.plot(pen=pg.mkPen('#facc15', width=2))

        self.d_vx = deque([0] * self._max, maxlen=self._max)
        self.d_vz = deque([0] * self._max, maxlen=self._max)
        self.db   = deque([0] * self._max, maxlen=self._max)

        self._buf_vx = []
        self._buf_vz = []
        self._buf_b  = []
        self.timer = QTimer()
        self.timer.timeout.connect(self._flush)
        self.timer.start(200)

    def update_values(self, vx, vz, batt, _servo=0):
        self._buf_vx.append(vx)
        self._buf_vz.append(vz)
        self._buf_b.append(batt)

    def _flush(self):
        if not self._buf_vx:
            return
        xs = list(range(self._max))
        a = sum(self._buf_vx) / len(self._buf_vx)
        b = sum(self._buf_vz) / len(self._buf_vz)
        c = sum(self._buf_b)  / len(self._buf_b)
        self._buf_vx.clear(); self._buf_vz.clear(); self._buf_b.clear()
        self.d_vx.append(a); self.d_vz.append(b); self.db.append(c)
        self.curve_vx.setData(xs, list(self.d_vx))
        self.curve_vz.setData(xs, list(self.d_vz))
        self.curve_b.setData(xs,  list(self.db))
