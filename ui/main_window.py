import cv2, json, os, glob, datetime
from ui.waveform_widget import WaveformWidget
from ui.mapping_widget import MappingWidget
import numpy as np
import pyqtgraph as pg
from collections import deque
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QTextEdit, QLabel,
    QStackedLayout, QSpacerItem, QSizePolicy, QButtonGroup, QRadioButton,
    QMessageBox, QListWidget, QListWidgetItem, QAbstractItemView
)
from PyQt5.QtCore import Qt, QTimer, QFileInfo
from PyQt5.QtGui import QTextCursor, QImage
from ui.camera_widget import CameraWidget
from ui.speed_gauge import SpeedGauge

CONFIG_FILE = os.path.expanduser("~/.割草机_config.json")
VIDEO_DIR = os.path.expanduser("~/Videos/割草机")

# 树莓派上的绝对路径，全写在这一次。地图由建图时的 map_saver_cli 写进 MAP_DIR，
# 导航和地图库都从这儿扫；换机器只改这三行。
ROS_WS = "/home/zhangruodong/ROS2"
MAP_DIR = "/home/zhangruodong"
RVIZ_CFG = ROS_WS + "/src/robot_nav2/rviz/mower.rviz"


class MainWindow(QWidget):

    # 手动控制速度。固件收 mm/s（uartx_callback.c 里 XYZ_Target_Speed_transition），
    # 这两个值远低于底盘上限，先用着安全。
    MANUAL_V = 0.4      # m/s
    MANUAL_W = 0.9      # rad/s

    def __init__(self, serial_mgr, protocol, dock_ctrl):
        super().__init__()
        self.serial_mgr = serial_mgr
        self.proto = protocol
        self.dock = dock_ctrl
        self._follow_on = False
        self._follow_node = None
        self.camera_widget = None
        self.camera_window = None
        self._mapping_widget = None
        self._speed_left = 0.0
        self._speed_right = 0.0
        self._manual_vx = 0.0
        self._manual_vz = 0.0
        self._keys_down = set()
        self.config = self._load_config()
        self.init_ui()
        # 手动控制：按住才发、松手归零，页面切走也归零
        self.setFocusPolicy(Qt.StrongFocus)
        self._manual_timer = QTimer(); self._manual_timer.timeout.connect(self._manual_tick)
        self.stack_layout.currentChanged.connect(self._on_page_changed)
        self._rec_vx_buf = deque(maxlen=120); self._rec_vz_buf = deque(maxlen=120); self._rec_b_buf = deque(maxlen=120)
        self.serial_mgr.sensor_updated.connect(
            lambda l, r, batt, servo: (self._waveform.update_values(l, r, batt, servo),
                                       self._rec_vx_buf.append(l), self._rec_vz_buf.append(r),
                                       self._rec_b_buf.append(batt),
                                       setattr(self, '_rec_servo', servo),
                                       self._set_rec_values(l, r, batt),
                                       self._push_sensor_to_camera(l, r, batt),
                                       self._feed_mapping(l, r))
        )
        self._rec_timer = QTimer(); self._rec_timer.timeout.connect(self._flush_rec_plot); self._rec_timer.start(200)
        self.dock.on_tag_found = self._on_tag_found
        self.dock.on_done = self._on_dock_done
        self._status_timer = QTimer(); self._status_timer.timeout.connect(self._update_status); self._status_timer.start(2000)
        self._speed_timer = QTimer(); self._speed_timer.timeout.connect(self._update_speed); self._speed_timer.start(1000)

    def _load_config(self):
        d = {"hw_mode": 0, "servo": True, "ultrasonic": False}
        if os.path.exists(CONFIG_FILE):
            try:
                with open(CONFIG_FILE) as f: d.update(json.load(f))
            except: pass
        return d

    def _save_config(self):
        with open(CONFIG_FILE, "w") as f: json.dump(self.config, f, indent=2)

    def show_settings(self):
        self.layout().setContentsMargins(11,11,11,11); self.setWindowTitle("设置"); self.stack_layout.setCurrentWidget(self.settings_widget)
    def show_hw_config(self):
        self.setWindowTitle("硬件配置"); self.hw_save_hint.setText(""); self.stack_layout.setCurrentWidget(self.hw_widget)
    def show_main(self):
        self.layout().setContentsMargins(11,11,11,11); self.setWindowTitle("割草机 控制面板"); self.stack_layout.setCurrentWidget(self.main_widget)
    def show_cache_page(self):
        self.setWindowTitle("清除缓存"); self.stack_layout.setCurrentWidget(self.cache_widget)
    def show_manual(self):
        self.setWindowTitle("手动控制"); self.stack_layout.setCurrentWidget(self.manual_widget)
    def show_video_page(self):
        self.setWindowTitle("视频管理"); self._refresh_video_list(); self.stack_layout.setCurrentWidget(self.video_widget)
    def show_speed_menu(self):
        self.setWindowTitle("速度显示"); self.stack_layout.setCurrentWidget(self.speed_menu_widget)
    def show_gauge_only(self):
        self.setWindowTitle("速度仪表盘"); self.stack_layout.setCurrentWidget(self.gauge_widget)
    def show_gauge_record(self):
        self.setWindowTitle("录制+显示"); self.layout().setContentsMargins(0,0,0,0); self.stack_layout.setCurrentWidget(self.gauge_rec_widget)
        self._ensure_rec_camera()
        from PyQt5.QtWidgets import QApplication
        QApplication.processEvents()
        w, h = self.width(), self.height()
        self.resize(w+1, h); self.resize(w, h)
    def show_waveform(self):
        self.setWindowTitle("实时波形"); self.stack_layout.setCurrentWidget(self.waveform_widget)

    def _push_sensor_to_camera(self, vl, vr, batt):
        """传感器数据推送给摄像头控件画曲线"""
        cw = getattr(self, 'camera_widget', None)
        if cw and hasattr(cw, 'set_sensor_data'):
            cw.set_sensor_data(vl, vr, batt)

    def _feed_mapping(self, vl, vr):
        """传感器速度推送给建图窗口"""
        if self._mapping_widget and getattr(self, '_mapping', False):
            self._mapping_widget.feed_speed(vl, vr)

    def _set_rec_values(self, vl, vr, batt):
        """传感器回调: 直接更新文字数值"""
        self.label_rec_values.setText(
            f"<span style='color:#4ade80'>Vx:{vl:.2f}</span>  "
            f"<span style='color:#60a5fa'>Vz:{vr:.2f}</span>  "
            f"<span style='color:#facc15'>电池:{batt:.1f}V</span>")

    def _flush_rec_plot(self):
        if hasattr(self, '_rec_curve_vx') and self._rec_curve_vx:
            xs = list(range(len(self._rec_vx_buf)))
            self._rec_curve_vx.setData(xs, list(self._rec_vx_buf))
            self._rec_curve_vz.setData(xs, list(self._rec_vz_buf))
            self._rec_curve_b.setData(xs, [b/25.0 for b in self._rec_b_buf])

    def _ensure_rec_camera(self):
        if not hasattr(self, 'rec_camera') or not self.rec_camera or not getattr(self.rec_camera, 'cap', None):
            self.rec_camera = CameraWidget()
            self.rec_camera.start_camera()
            if hasattr(self, 'rec_camera_container') and self.rec_camera_container:
                lay = self.rec_camera_container.layout()
                if lay is None:
                    lay = QVBoxLayout(); self.rec_camera_container.setLayout(lay)
                while lay.count(): lay.takeAt(0)
                lay.addWidget(self.rec_camera, 1)

    def init_ui(self):
        self.resize(1000, 500)
        self.setWindowTitle("割草机 控制面板")
        self.stack_layout = QStackedLayout()
        self.setLayout(self.stack_layout)

        self.main_widget = QWidget(); main_layout = QHBoxLayout(); self.main_widget.setLayout(main_layout)
        left_layout = QVBoxLayout(); left_layout.setAlignment(Qt.AlignTop)

        self.status_label = QLabel("状态:\n就绪"); self.status_label.setAlignment(Qt.AlignCenter); self.status_label.setStyleSheet("font-size:32px; font-weight:bold;")
        left_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Fixed)); left_layout.addWidget(self.status_label); left_layout.addStretch()

        self.btn_selftest = QPushButton("整机自检"); self.btn_mow = QPushButton("开始割草"); self.btn_follow = QPushButton("雷达跟随"); self.btn_dock = QPushButton("自动回充"); self.btn_manual = QPushButton("手动控制"); self.btn_stop = QPushButton("紧急停止"); self.btn_release = QPushButton("解除急停")
        for btn in [self.btn_selftest, self.btn_mow, self.btn_follow, self.btn_dock, self.btn_manual, self.btn_stop, self.btn_release]:
            btn.setMinimumHeight(40); btn.setStyleSheet("font-size:20px;"); left_layout.addWidget(btn)
        # 急停是锁存的（固件 uartx_callback.c:138），不解就是复位前一直停着 ——
        # 所以解除必须是主界面上一个够显眼的按钮，不能藏进菜单。
        self.btn_release.setStyleSheet("font-size:20px; background-color:#1e6b3b; color:white;")
        self.btn_selftest.clicked.connect(self.start_self_test); self.btn_mow.clicked.connect(self.start_mow); self.btn_follow.clicked.connect(self.toggle_follow); self.btn_dock.clicked.connect(self.start_dock); self.btn_manual.clicked.connect(self.show_manual); self.btn_stop.clicked.connect(self.stop_all); self.btn_release.clicked.connect(self.release_estop)

        self.btn_record = QPushButton("SLAM地图"); self.btn_record.setMinimumHeight(40); self.btn_record.setStyleSheet("font-size:20px;"); self.btn_record.clicked.connect(self.show_slam_page); left_layout.addWidget(self.btn_record)
        self.btn_settings = QPushButton("设置"); self.btn_settings.setMinimumHeight(40); self.btn_settings.setStyleSheet("font-size:20px;"); self.btn_settings.clicked.connect(self.show_settings); left_layout.addWidget(self.btn_settings)
        main_layout.addLayout(left_layout, 1)

        center_layout = QVBoxLayout(); center_layout.addWidget(QLabel("发送日志")); self.send_log = QTextEdit(); self.send_log.setReadOnly(True); self.send_log.document().setMaximumBlockCount(500); center_layout.addWidget(self.send_log, stretch=1); self.btn_clear_send = QPushButton("清空发送日志"); self.btn_clear_send.clicked.connect(self.clear_send_log); center_layout.addWidget(self.btn_clear_send); main_layout.addLayout(center_layout, 1)
        right_layout = QVBoxLayout(); right_layout.addWidget(QLabel("接收日志")); self.recv_log = QTextEdit(); self.recv_log.setReadOnly(True); self.recv_log.document().setMaximumBlockCount(500); right_layout.addWidget(self.recv_log, stretch=1); self.btn_clear_recv = QPushButton("清空接收日志"); self.btn_clear_recv.clicked.connect(self.clear_recv_log); right_layout.addWidget(self.btn_clear_recv); main_layout.addLayout(right_layout, 1)
        self.stack_layout.addWidget(self.main_widget)

        self.settings_widget = self._page("设置", [
            ("切换外设 >", self.show_hw_config), ("打开摄像头", self.open_camera_window),
            ("速度显示 >", self.show_speed_menu), ("清除缓存 >", self.show_cache_page),
            ("返回主界面", self.show_main),
        ]); self.stack_layout.addWidget(self.settings_widget)

        self.hw_widget = QWidget(); hwl = QVBoxLayout(); self.hw_widget.setLayout(hwl)
        t = QLabel("切换外设"); t.setAlignment(Qt.AlignCenter); t.setStyleSheet("font-size:28px; font-weight:bold;"); hwl.addWidget(t); hwl.addWidget(QLabel("选择当前连接的传感器配置"))
        self.rb_normal = QRadioButton("摄像头(普通)"); self.rb_depth = QRadioButton("双目+激光雷达"); self.rb_auto = QRadioButton("自动识别")
        self.hw_group = QButtonGroup(); self.hw_group.addButton(self.rb_normal,0); self.hw_group.addButton(self.rb_depth,1); self.hw_group.addButton(self.rb_auto,2)
        m = self.config.get("hw_mode",0); b = self.hw_group.button(m)
        if b: b.setChecked(True)
        for rb in [self.rb_normal, self.rb_depth, self.rb_auto]: rb.setStyleSheet("font-size:18px;"); hwl.addWidget(rb)
        self.hw_save_hint = QLabel(""); self.hw_save_hint.setStyleSheet("color:green;"); hwl.addWidget(self.hw_save_hint)
        sb = QPushButton("保存配置"); sb.setMinimumHeight(40); sb.clicked.connect(self._save_hw); hwl.addWidget(sb)
        bb = QPushButton("← 返回设置"); bb.setMinimumHeight(40); bb.clicked.connect(self.show_settings); hwl.addWidget(bb)
        self.stack_layout.addWidget(self.hw_widget)

        self.cache_widget = self._page("清除缓存", [("清除视频 >", self.show_video_page), ("清除配置文件", self._clear_config), ("← 返回设置", self.show_settings)]); self.stack_layout.addWidget(self.cache_widget)

        self.video_widget = QWidget(); vl = QVBoxLayout(); self.video_widget.setLayout(vl)
        vt = QLabel("视频管理"); vt.setAlignment(Qt.AlignCenter); vt.setStyleSheet("font-size:28px; font-weight:bold;"); vl.addWidget(vt)
        top = QHBoxLayout(); self.video_count_label = QLabel("共 0 个视频"); top.addWidget(self.video_count_label); top.addStretch()
        self.btn_select_all = QPushButton("全选"); self.btn_select_all.setMinimumHeight(35); self.btn_select_all.clicked.connect(self._toggle_select_all); top.addWidget(self.btn_select_all)
        self.btn_sort_video = QPushButton("时间▼"); self.btn_sort_video.setMinimumHeight(35)
        self.btn_sort_video.clicked.connect(lambda: (setattr(self, '_video_sort_time', not getattr(self, '_video_sort_time', True)), self._refresh_video_list())); top.addWidget(self.btn_sort_video); vl.addLayout(top)
        self.video_list = QListWidget(); self.video_list.setStyleSheet("font-size:16px; QListWidget::indicator { width: 24px; height: 24px; }"); self.video_list.setSelectionMode(QAbstractItemView.NoSelection); self.video_list.itemDoubleClicked.connect(self._play_video); vl.addWidget(self.video_list)
        btns = QHBoxLayout(); self.btn_delete_videos = QPushButton("删除所选"); self.btn_delete_videos.setMinimumHeight(40); self.btn_delete_videos.setStyleSheet("background-color:#8b0000; color:white; font-size:18px;"); self.btn_delete_videos.clicked.connect(self._delete_selected_videos); btns.addWidget(self.btn_delete_videos)
        bb2 = QPushButton("← 返回"); bb2.setMinimumHeight(40); bb2.clicked.connect(self.show_cache_page); btns.addWidget(bb2); vl.addLayout(btns); self.stack_layout.addWidget(self.video_widget)

        self.speed_menu_widget = self._page("速度显示", [("仅显示速度 >", self.show_gauge_only), ("记录并显示 >", self.show_gauge_record), ("波形显示 >", self.show_waveform), ("← 返回设置", self.show_settings)]); self.stack_layout.addWidget(self.speed_menu_widget)

        self.gauge_widget = QWidget(); gl = QVBoxLayout(); self.gauge_widget.setLayout(gl)
        gt = QLabel("速度仪表盘"); gt.setAlignment(Qt.AlignCenter); gt.setStyleSheet("font-size:28px; font-weight:bold;"); gl.addWidget(gt)
        gr = QHBoxLayout(); self.gauge_left = SpeedGauge("前向 Vx", 1.0); self.gauge_right = SpeedGauge("旋转 Vz", 1.0); gr.addWidget(self.gauge_left); gr.addWidget(self.gauge_right); gl.addLayout(gr)
        gbb = QPushButton("← 返回"); gbb.setMinimumHeight(40); gbb.clicked.connect(self.show_speed_menu); gl.addWidget(gbb); self.stack_layout.addWidget(self.gauge_widget)

        self.gauge_rec_widget = QWidget(); rl = QVBoxLayout(); rl.setContentsMargins(0,0,0,0); rl.setSpacing(0); self.gauge_rec_widget.setLayout(rl)
        rr = QHBoxLayout(); rr.setSpacing(0); rr.setContentsMargins(0,0,0,0)
        self.rec_camera_container = QWidget(); self.rec_camera_container.setLayout(QVBoxLayout());
        self.rec_camera_container.layout().setContentsMargins(0,0,0,0); self.rec_camera_container.layout().setSpacing(0)
        rr.addWidget(self.rec_camera_container, 2)
        # 右边: 上=微型波形, 下=文字数据
        rv = QVBoxLayout(); rv.setSpacing(0); rv.setContentsMargins(0,0,0,0)
#        import pyqtgraph as pg  # lazy import
        self._rec_plot = pg.PlotWidget(); self._rec_plot.setBackground('#14161b'); self._rec_plot.showGrid(x=True, y=True, alpha=0.3)
        self._rec_plot.setLabel('left', '')
        self._rec_curve_vx = self._rec_plot.plot(pen=pg.mkPen('#4ade80', width=2), name='Vx')
        self._rec_curve_vz = self._rec_plot.plot(pen=pg.mkPen('#60a5fa', width=2), name='Vz')
        self._rec_curve_b  = self._rec_plot.plot(pen=pg.mkPen('#facc15', width=1), name='Bat')
        rv.addWidget(self._rec_plot, 1)
        self.label_rec_data = QLabel("<span style='color:#4ade80'>● 前向</span>  <span style='color:#60a5fa'>● 旋转</span>  <span style='color:#facc15'>● 电池</span>"); self.label_rec_data.setStyleSheet("font-size:14px; font-weight:bold; padding:2px;"); self.label_rec_data.setAlignment(Qt.AlignCenter); self.label_rec_data.setTextFormat(Qt.RichText); self.label_rec_data.setMaximumHeight(22)
        rv.addWidget(self.label_rec_data, 0)
        self.label_rec_values = QLabel("Vx: --  Vz: --  电池: --"); self.label_rec_values.setStyleSheet("font-size:18px; font-weight:bold; padding:4px;"); self.label_rec_values.setAlignment(Qt.AlignCenter); self.label_rec_values.setTextFormat(Qt.RichText)
        rv.addWidget(self.label_rec_values, 0)
        rv.addStretch(1)
        rr.addLayout(rv, 1)
        rl.addLayout(rr, 1)
        rbb = QPushButton("← 返回"); rbb.setMaximumHeight(36); rbb.setMinimumHeight(36); rbb.clicked.connect(self.show_speed_menu); rl.addWidget(rbb); self.stack_layout.addWidget(self.gauge_rec_widget)

        self.waveform_widget = QWidget(); wl = QVBoxLayout(); self.waveform_widget.setLayout(wl)
        from ui.waveform_widget import WaveformWidget
        self._waveform = WaveformWidget(); wl.addWidget(self._waveform)
        wbb = QPushButton("← 返回"); wbb.setMinimumHeight(40); wbb.clicked.connect(self.show_speed_menu); wl.addWidget(wbb)
        self.stack_layout.addWidget(self.waveform_widget)
        self.slam_widget = QWidget(); sl2 = QVBoxLayout(); self.slam_widget.setLayout(sl2)
        st2 = QLabel("SLAM 建图"); st2.setAlignment(Qt.AlignCenter); st2.setStyleSheet("font-size:28px;font-weight:bold;")
        sl2.addWidget(st2); sl2.addSpacing(10)
        self._btn_start_slam = QPushButton("开始建图"); self._btn_start_slam.setMinimumHeight(45)
        self._btn_start_slam.setStyleSheet("font-size:20px;background-color:#1e6b3b;color:white;")
        self._btn_start_slam.clicked.connect(self.start_mapping); sl2.addWidget(self._btn_start_slam); sl2.addSpacing(8)
        self._btn_save_map = QPushButton("保存地图"); self._btn_save_map.setMinimumHeight(45)
        self._btn_save_map.setStyleSheet("font-size:20px;"); self._btn_save_map.clicked.connect(self._save_map); sl2.addWidget(self._btn_save_map)
        sl2.addSpacing(8)
        self._btn_nav = QPushButton("Nav2 导航"); self._btn_nav.setMinimumHeight(45)
        self._btn_nav.setStyleSheet("font-size:20px;background-color:#e67e22;color:white;")
        self._btn_nav.clicked.connect(self.start_nav); sl2.addWidget(self._btn_nav)
        sl2.addSpacing(8)
        self._btn_stop_slam = QPushButton("停止建图"); self._btn_stop_slam.setMinimumHeight(45)
        self._btn_stop_slam.setStyleSheet("font-size:20px;background-color:#8b0000;color:white;")
        self._btn_stop_slam.clicked.connect(self._stop_slam); sl2.addWidget(self._btn_stop_slam)
        sl2.addSpacing(8)
        self._btn_map_gallery = QPushButton("地图库"); self._btn_map_gallery.setMinimumHeight(45)
        self._btn_map_gallery.setStyleSheet("font-size:20px;background-color:#1565c0;color:white;")
        self._btn_map_gallery.clicked.connect(self.show_map_gallery); sl2.addWidget(self._btn_map_gallery)
        sl2.addStretch()
        bb2 = QPushButton("返回主界面"); bb2.setMinimumHeight(40); bb2.clicked.connect(self.show_main); sl2.addWidget(bb2)
        self.stack_layout.addWidget(self.slam_widget)
        # ---- 手动控制 ----
        self.manual_widget = QWidget(); ml = QVBoxLayout(); self.manual_widget.setLayout(ml)
        mt = QLabel("手动控制"); mt.setAlignment(Qt.AlignCenter); mt.setStyleSheet("font-size:32px; font-weight:bold;")
        ml.addWidget(mt)
        mh = QLabel("按住 ↑↓←→ 或 WASD 驱动，松手停车。\n"
                    "Nav2 在跑时别用手动 —— 两边都往 /cmd_vel 发。")
        mh.setAlignment(Qt.AlignCenter); mh.setStyleSheet("font-size:14px; color:#888;"); ml.addWidget(mh)
        ml.addSpacing(20)

        def _mbtn(text, vx, vz):
            # NoFocus 是关键：否则点完按钮焦点留在按钮上，方向键会变成在按钮间挪焦点
            b = QPushButton(text); b.setMinimumSize(80, 60)
            b.setStyleSheet("font-size:24px;"); b.setFocusPolicy(Qt.NoFocus)
            b.pressed.connect(lambda: self._manual_set(vx, vz))
            b.released.connect(lambda: self._manual_set(0.0, 0.0))
            return b

        r1 = QHBoxLayout(); r1.addStretch(); r1.addWidget(_mbtn("↑", self.MANUAL_V, 0.0)); r1.addStretch(); ml.addLayout(r1)
        r2 = QHBoxLayout(); r2.addStretch()
        for w in (_mbtn("←", 0.0, self.MANUAL_W), _mbtn("停", 0.0, 0.0), _mbtn("→", 0.0, -self.MANUAL_W)):
            r2.addWidget(w)
        r2.addStretch(); ml.addLayout(r2)
        r3 = QHBoxLayout(); r3.addStretch(); r3.addWidget(_mbtn("↓", -self.MANUAL_V, 0.0)); r3.addStretch(); ml.addLayout(r3)
        ml.addSpacing(20)
        mbb = QPushButton("← 返回主界面"); mbb.setMinimumHeight(40); mbb.clicked.connect(self.show_main); ml.addWidget(mbb)
        self.stack_layout.addWidget(self.manual_widget)

        self.map_gallery_widget = QWidget(); self.map_gallery_widget.setLayout(QVBoxLayout())
        self.stack_layout.addWidget(self.map_gallery_widget)

    def _page(self, title, buttons):
        w = QWidget(); l = QVBoxLayout(); w.setLayout(l)
        t = QLabel(title); t.setAlignment(Qt.AlignCenter); t.setStyleSheet("font-size:32px; font-weight:bold;"); l.addWidget(t); l.addSpacing(15)
        for text, handler in buttons:
            btn = QPushButton(text); btn.setMinimumHeight(42); btn.setStyleSheet("font-size:20px;"); btn.clicked.connect(handler); l.addWidget(btn)
            if text != buttons[-1][0]: l.addSpacing(6)
        return w

    def _refresh_video_list(self):
        self.video_list.clear()
        os.makedirs(VIDEO_DIR, exist_ok=True)
        all_files = glob.glob(os.path.join(VIDEO_DIR, "*.avi")) + glob.glob(os.path.join(VIDEO_DIR, "*.mp4"))
        # 过滤空壳文件（< 100KB 就是摄像头没开时录的）
        all_files = [f for f in all_files if os.path.getsize(f) > 100 * 1024]
        if getattr(self, '_video_sort_time', True):
            files = sorted(all_files, key=lambda f: os.path.getmtime(f), reverse=True)
            self.btn_sort_video.setText("时间▼")
        else:
            files = sorted(all_files, reverse=True)
            self.btn_sort_video.setText("名称▼")
        self.video_count_label.setText(f"共 {len(files)} 个视频")
        for f in files:
            sz = os.path.getsize(f) / 1048576.0
            ts = datetime.datetime.fromtimestamp(os.path.getmtime(f)).strftime("%Y-%m-%d %H:%M:%S")
            item = QListWidgetItem(f"{os.path.basename(f)}    {sz:.1f}MB    {ts}")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable); item.setCheckState(Qt.Unchecked); item.setData(Qt.UserRole, f)
            self.video_list.addItem(item)

    def _play_video(self, item):
        """双击播放视频 — OpenCV 读 MJPG/AVI（Docker 兼容）"""
        path = item.data(Qt.UserRole)
        if not path or not os.path.exists(path):
            return
        import cv2
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel
        from PyQt5.QtGui import QImage, QPixmap

        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            self.status_label.setText(f"状态:\n视频:\n{os.path.basename(path)}")
            self.log_send("视频文件: " + path)
            return

        fps = cap.get(cv2.CAP_PROP_FPS) or 15.0
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 960)
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 480)

        dlg = QDialog(self); dlg.setWindowTitle(os.path.basename(path))
        dlg.resize(min(w, 960), min(h, 540))
        lbl = QLabel(); lbl.setAlignment(Qt.AlignCenter)
        lay = QVBoxLayout(); lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(lbl); dlg.setLayout(lay)

        timer = QTimer()
        def _next():
            ret, frame = cap.read()
            if not ret:
                cap.release(); timer.stop(); return
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            hf, wf = frame.shape[:2]
            qimg = QImage(frame.data, wf, hf, wf * 3, QImage.Format_RGB888)
            lbl.setPixmap(QPixmap.fromImage(qimg).scaled(
                lbl.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        timer.timeout.connect(_next); timer.start(int(1000 / fps))
        dlg.finished.connect(lambda: (cap.release(), timer.stop()))
        dlg.exec_()

    def _toggle_select_all(self):
        checked = self.btn_select_all.text() == "全选"
        state = Qt.Checked if checked else Qt.Unchecked
        for i in range(self.video_list.count()): self.video_list.item(i).setCheckState(state)
        self.btn_select_all.setText("取消全选" if checked else "全选")

    def _delete_selected_videos(self):
        to_del = [self.video_list.item(i).data(Qt.UserRole) for i in range(self.video_list.count()) if self.video_list.item(i).checkState() == Qt.Checked]
        if not to_del: QMessageBox.information(self, "提示", "未选择任何视频"); return
        if QMessageBox.question(self, "确认删除", f"确定删除 {len(to_del)} 个视频？", QMessageBox.Yes|QMessageBox.No) == QMessageBox.Yes:
            for f in to_del:
                try: os.remove(f)
                except: pass
            self._refresh_video_list(); self.btn_select_all.setText("全选"); self.log_send(f"删除 {len(to_del)} 个视频")

    def _clear_config(self):
        if QMessageBox.question(self, "确认", "确定清除所有配置？", QMessageBox.Yes|QMessageBox.No) == QMessageBox.Yes:
            if os.path.exists(CONFIG_FILE): os.remove(CONFIG_FILE)
            self.config = {"hw_mode":0, "servo":True, "ultrasonic":False}; self._save_config()
            QMessageBox.information(self, "完成", "配置已清除"); self.log_send("配置已清除")

    def _update_speed(self):
        vl = self._speed_left if abs(self._speed_left) > 0.001 else 0.0
        vr = self._speed_right if abs(self._speed_right) > 0.001 else 0.0
        if hasattr(self, 'gauge_left'): self.gauge_left.set_value(vl); self.gauge_right.set_value(vr)
        try:
            b = self._rec_b_buf[-1] if self._rec_b_buf else 0
            self.label_rec_values.setText(
                f"<span style='color:#4ade80'>Vx:{vl:.2f}</span>  "
                f"<span style='color:#60a5fa'>Vz:{vr:.2f}</span>  "
                f"<span style='color:#facc15'>电池:{b:.1f}V</span>")
        except Exception:
            pass

    def _save_hw(self):
        from core.hw_detector import has_usb_camera, has_depth_camera, has_lidar
        mode = self.hw_group.checkedId()
        if mode < 0: mode = 0
        missing = []
        if mode == 0 and not has_usb_camera(): missing.append("USB 摄像头")
        elif mode == 1:
            if not has_depth_camera(): missing.append("深度相机")
            if not has_lidar(): missing.append("激光雷达")
        if missing and mode != 2:
            cur = self._detect_mode_name(2)
            QMessageBox.warning(self, "硬件检测警告", f"未检测到: {', '.join(missing)}\n\n已自动切换到「自动识别」模式\n当前模式: {cur}")
            mode = 2; self.rb_auto.setChecked(True)
        else:
            from core.hw_detector import get_device_names
            devs = get_device_names()
            devs_text = "\n".join([f"  · {d}" for d in devs])
            if "未识别" in devs[0]:
                QMessageBox.warning(self, "硬件检测",
                    f"未识别设备，请检查连接\n\n当前模式: {self._detect_mode_name(mode)}")
            else:
                QMessageBox.information(self, "保存成功",
                    f"已检测到设备:\n{devs_text}\n\n当前模式: {self._detect_mode_name(mode)}")
        self.config["hw_mode"] = mode; self._save_config(); self.hw_save_hint.setText(f"已保存 ({self._detect_mode_name()})")

    def _detect_mode_name(self, mode=None):
        from core.hw_detector import has_depth_camera, has_lidar
        if mode is None: mode = self.config.get("hw_mode", 0)
        if mode == 0: return "摄像头(普通)"
        elif mode == 1:
            parts = []; has_depth_camera() and parts.append("深度相机"); has_lidar() and parts.append("雷达")
            return "双目+雷达: " + (" + ".join(parts) if parts else "未检测到")
        elif mode == 2:
            from core.hw_detector import auto_detect; names = {0:"普通", 1:"深度+雷达", -1:"无设备"}; return f"自动识别: {names.get(auto_detect(), '未知')}"
        return "未知"

    def clear_send_log(self): self.send_log.clear()
    def clear_recv_log(self): self.recv_log.clear()
    def log_send(self, label=""): self.send_log.append(f"[{label}]"); self.send_log.moveCursor(QTextCursor.End)
    def log_recv(self, text): self.recv_log.append(text); self.recv_log.moveCursor(QTextCursor.End)

    def _release_camera(self):
        self._stop_grab_record()
        if self.camera_widget: self.camera_widget.stop_recording(); self.camera_widget.stop_camera(); self.camera_widget = None
        if self.camera_window: self.camera_window.close(); self.camera_window = None

    def _on_camera_window_close(self, event): self._release_camera(); event.accept()
    def closeEvent(self, event):
        self._lidar_running = False
        self._release_camera()
        if hasattr(self, 'rec_camera') and self.rec_camera: self.rec_camera.stop_recording(); self.rec_camera.stop_camera()
        import subprocess
        subprocess.run(
            ["bash", "-c",
             self.ROS_SETUP + " && timeout 3 ros2 service call /stop_motor std_srvs/srv/Empty '{}' 2>/dev/null"],
            env=dict(os.environ, DISPLAY=":0"))
        subprocess.run(["pkill", "-f", "slam_toolbox|slam_gmapping|cartographer_node|rplidar|lslidar|x10|rviz2|lasertracker|laserfollower|base_to_laser|static_transform|nav2_bringup"])
        # 停串口节点，释放串口，防止下次启动崩溃
        subprocess.run(["pkill", "-f", "chassis_bridge"])
        self.serial_mgr.close(); event.accept()

    # ============ 录屏（预览窗 + OpenCV 录像，Docker 兼容） ============
    def _start_grab_record(self, label):
        if getattr(self, '_grab_writer', None): return
        import cv2

        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(VIDEO_DIR, f"{label}_{ts}.avi")
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self._grab_writer = cv2.VideoWriter(path, fourcc, 15.0, (960, 480))
        if not self._grab_writer.isOpened():
            self._grab_writer = None; return

        # 弹 Qt 预览窗（CameraWidget 内部会打开摄像头）
        self._open_composite_window()
        self._grab_timer = QTimer(); self._grab_timer.timeout.connect(self._grab_frame)
        self._grab_timer.start(66)  # ~15fps
        self.status_label.setText(f"状态:\n录制中\n{label}")
        self.log_send(f"录像: {path}")

    def _open_composite_window(self):
        """预览窗: 摄像头 + 曲线 + 文字（纯 Qt 显示）"""
        self._composite_win = QWidget()
        self._composite_win.setWindowTitle("录制中")
        self._composite_win.resize(960, 480)
        lay = QHBoxLayout(); lay.setContentsMargins(0, 0, 0, 0); lay.setSpacing(0)
        self._composite_win.setLayout(lay)

        # 左: 摄像头 (640x480)
        cam_container = QWidget()
        cam_container.setFixedSize(640, 480)
        cam_container.setLayout(QVBoxLayout())
        cam_container.layout().setContentsMargins(0, 0, 0, 0)
        self._rec_cam_widget = CameraWidget()
        self._rec_cam_widget.start_camera()
        self._rec_cam_widget.on_tag_seen = lambda: self.dock.tag_detected()
        cam_container.layout().addWidget(self._rec_cam_widget)
        lay.addWidget(cam_container, 2)

        # 右: 曲线 + 文字
        rv = QVBoxLayout(); rv.setSpacing(2); rv.setContentsMargins(0, 0, 0, 0)
        self._comp_plot = pg.PlotWidget(); self._comp_plot.setBackground('#14161b')
        self._comp_plot.showGrid(x=True, y=True, alpha=0.3)
        self._comp_curve_vx = self._comp_plot.plot(pen=pg.mkPen('#4ade80', width=2), name='Vx')
        self._comp_curve_vz = self._comp_plot.plot(pen=pg.mkPen('#60a5fa', width=2), name='Vz')
        self._comp_curve_b = self._comp_plot.plot(pen=pg.mkPen('#facc15', width=1), name='Bat')
        rv.addWidget(self._comp_plot, 1)
        self._comp_values = QLabel("Vx:-- Vz:-- 电池:--")
        self._comp_values.setStyleSheet("font-size:16px; font-weight:bold; padding:4px;")
        self._comp_values.setAlignment(Qt.AlignCenter)
        rv.addWidget(self._comp_values)
        rv.addStretch(1)
        lay.addLayout(rv, 1)

        self._composite_win.closeEvent = lambda e: self._stop_grab_record()
        self._composite_win.show()

        # 预览曲线刷新
        self._comp_timer = QTimer(); self._comp_timer.timeout.connect(self._flush_comp_plot)
        self._comp_timer.start(200)

    def _flush_comp_plot(self):
        if not hasattr(self, '_comp_curve_vx') or not self._comp_curve_vx: return
        vl = self._speed_left; vr = self._speed_right
        xs = list(range(len(self._rec_vx_buf)))
        self._comp_curve_vx.setData(xs, list(self._rec_vx_buf))
        self._comp_curve_vz.setData(xs, list(self._rec_vz_buf))
        self._comp_curve_b.setData(xs, [b / 25.0 for b in self._rec_b_buf])
        b = self._rec_b_buf[-1] if self._rec_b_buf else 0
        self._comp_values.setText(
            f"<span style='color:#4ade80'>Vx:{vl:.2f}</span>  "
            f"<span style='color:#60a5fa'>Vz:{vr:.2f}</span>  "
            f"<span style='color:#facc15'>电池:{b:.1f}V</span>")

    def _grab_frame(self):
        """OpenCV 合成录像（从 CameraWidget 缓存取帧，不抢摄像头）"""
        import cv2
        cw = getattr(self, '_rec_cam_widget', None)
        cam = cw.last_frame if cw and hasattr(cw, 'last_frame') else None
        wr = getattr(self, '_grab_writer', None)
        if cam is None or wr is None:
            return
        cam = cv2.resize(cam, (640, 480))

        canvas = np.zeros((480, 960, 3), dtype=np.uint8)
        canvas[:, :640] = cam
        canvas[:, 640:] = [20, 22, 30]

        vl = self._speed_left; vr = self._speed_right
        b = self._rec_b_buf[-1] if self._rec_b_buf else 0
        servo = getattr(self, '_rec_servo', 1500)

        y = 35
        for text, color in [
            (f"Vx: {vl:+.3f} m/s", (74, 222, 128)),
            (f"Vz: {vr:+.3f} m/s", (96, 165, 250)),
            (f"Battery: {b:.1f} V", (250, 204, 21)),
            (f"Servo: {servo} us", (192, 192, 192)),
        ]:
            cv2.putText(canvas, text, (655, y), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, color, 1, cv2.LINE_AA)
            y += 28

        # 波形区（浅色背景 + 网格 + 三线 Vx/Vz/Battery）
        plot_x, plot_y, plot_w, plot_h = 650, 160, 300, 200
        cv2.rectangle(canvas, (plot_x, plot_y), (plot_x + plot_w, plot_y + plot_h),
                      (40, 42, 50), -1)
        # 网格线
        for gy in range(plot_y + 50, plot_y + plot_h, 50):
            cv2.line(canvas, (plot_x, gy), (plot_x + plot_w, gy), (60, 62, 70), 1)
        for gx in range(plot_x + 50, plot_x + plot_w, 50):
            cv2.line(canvas, (gx, plot_y), (gx, plot_y + plot_h), (60, 62, 70), 1)

        # 图例
        cv2.putText(canvas, "Vx", (plot_x + 5, plot_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (74, 222, 128), 1)
        cv2.putText(canvas, "Vz", (plot_x + 35, plot_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (96, 165, 250), 1)
        cv2.putText(canvas, "Bat", (plot_x + 65, plot_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (250, 204, 21), 1)

        # 画 Vx 线
        vx_buf = list(self._rec_vx_buf)
        n = max(len(vx_buf), 1)
        mid_y = plot_y + plot_h // 2
        for i in range(n - 1):
            x1 = plot_x + int(i * plot_w / n)
            x2 = plot_x + int((i + 1) * plot_w / n)
            y1 = mid_y - int(vx_buf[i] * 120)
            y2 = mid_y - int(vx_buf[i + 1] * 120)
            cv2.line(canvas, (x1, max(plot_y, min(plot_y + plot_h, y1))),
                     (x2, max(plot_y, min(plot_y + plot_h, y2))), (74, 222, 128), 1)
        # 画 Vz 线
        vz_buf = list(self._rec_vz_buf)
        for i in range(n - 1):
            x1 = plot_x + int(i * plot_w / n)
            x2 = plot_x + int((i + 1) * plot_w / n)
            y1 = mid_y - int(vz_buf[i] * 120)
            y2 = mid_y - int(vz_buf[i + 1] * 120)
            cv2.line(canvas, (x1, max(plot_y, min(plot_y + plot_h, y1))),
                     (x2, max(plot_y, min(plot_y + plot_h, y2))), (96, 165, 250), 1)
        # 画 Battery 线（缩放到 0~1 范围）
        if self._rec_b_buf:
            b_max = max(self._rec_b_buf) or 1
            b_buf = [x / b_max for x in self._rec_b_buf]
            for i in range(len(b_buf) - 1):
                x1 = plot_x + int(i * plot_w / len(b_buf))
                x2 = plot_x + int((i + 1) * plot_w / len(b_buf))
                y1 = mid_y - int(b_buf[i] * 80)
                y2 = mid_y - int(b_buf[i + 1] * 80)
                cv2.line(canvas, (x1, max(plot_y, min(plot_y + plot_h, y1))),
                         (x2, max(plot_y, min(plot_y + plot_h, y2))), (250, 204, 21), 1)
        wr.write(canvas)

    def _stop_grab_record(self):
        if hasattr(self, '_grab_timer') and self._grab_timer:
            self._grab_timer.stop()
        if hasattr(self, '_comp_timer') and self._comp_timer:
            self._comp_timer.stop()
        if getattr(self, '_grab_writer', None):
            self._grab_writer.release(); self._grab_writer = None
            self.log_send("录像已保存")
        if hasattr(self, '_composite_win') and self._composite_win:
            self._composite_win.close(); self._composite_win = None
        if hasattr(self, '_rec_cam_widget') and self._rec_cam_widget:
            try: self._rec_cam_widget.stop_camera()
            except: pass
            self._rec_cam_widget = None

        # 关录制窗 = 停当前动作（只有真的停了动作才发急停帧）
        stopped_any = False
        if getattr(self, '_selftest_running', False):
            self._selftest_running = False; self.log_send("自检: 手动停止"); stopped_any = True
        if getattr(self, '_dock_running', False):
            self._dock_running = False; self.dock.stop(); self.log_send("回充: 手动停止"); stopped_any = True
        if stopped_any:
            self._speed_left = 0; self._speed_right = 0
            self.status_label.setText("状态:\n就绪")
            self.serial_mgr.send(self.proto.build_emergency_frame())

    ROS_SETUP = "source " + ROS_WS + "/install/setup.bash"

    # 我们自己的底盘桥，整个替掉了原来的底盘节点。
    # 路径按本文件位置推，换机器/换克隆目录都不用改。
    BRIDGE_SETUP = (ROS_SETUP + " && source "
                    + os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                   "bridge", "install", "setup.bash"))
    BRIDGE_LAUNCH = "ros2 launch mower_bridge chassis_bridge.launch.py"

    def start_mapping(self):
        import subprocess, os, time

        # --- 停止建图 ---
        if getattr(self, '_mapping', False):
            self._mapping = False
            self._btn_start_slam.setText("开始建图")
            self.status_label.setText("状态:\n就绪")
            self.log_send("建图: 已停止")
            subprocess.run(
                ["bash", "-c",
                 self.ROS_SETUP + " && timeout 3 ros2 service call /stop_motor std_srvs/srv/Empty '{}' 2>/dev/null"],
                env=dict(os.environ, DISPLAY=":0"))
            subprocess.run(["pkill", "-f", "slam_toolbox|slam_gmapping|cartographer_node|rplidar|lslidar|x10"])
            subprocess.run(["pkill", "rviz2"])
            time.sleep(2)
            # 把底盘桥拉回来（建图停了，控制得回来）
            subprocess.Popen(
                ["bash", "-c",
                 self.BRIDGE_SETUP + " && " + self.BRIDGE_LAUNCH],
                env=dict(os.environ, DISPLAY=":0"))
            return

        # --- 启动建图 ---
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QRadioButton, QDialogButtonBox, QButtonGroup
        dlg = QDialog()
        dlg.setWindowTitle("选择建图方式")
        lay = QVBoxLayout()
        dlg.setLayout(lay)
        group = QButtonGroup(dlg)
        opts = [
            ("slam_toolbox",  "slam_toolbox (推荐, 异步)"),
            ("gmapping",      "gmapping (粒子滤波)"),
            ("cartographer",  "cartographer (图优化)"),
        ]
        radios = []
        for val, label in opts:
            r = QRadioButton(label)
            lay.addWidget(r)
            group.addButton(r)
            r.val = val
            radios.append(r)
        radios[0].setChecked(True)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if not dlg.exec_():
            return

        slam_choice = next(r.val for r in radios if r.isChecked())
        self.log_send("建图: 启动 " + slam_choice)

        # 这三个是树莓派工作空间里装好的包名，不是路径 —— 拼错一个建图就起不来。
        launch_pkg = {
            "slam_toolbox": "slam_toolbox online_async_launch.py",
            "gmapping":     "slam_gmapping slam_gmapping.launch.py",
            "cartographer": "cartographer_ros cartographer.launch.py",
        }
        rviz_cfg = RVIZ_CFG

        # 1. 彻底清理旧进程，让 EKF 从零开始（避免残留影响建图质量）
        subprocess.run(["pkill", "-f", "chassis_bridge|ekf_node|rplidar"])
        time.sleep(2)

        # 2. 启动建图（source 完整 overlay 环境）
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && ros2 launch " + launch_pkg[slam_choice]],
            env=dict(os.environ, DISPLAY=":0"))

        # 3. 启动 rviz2
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && rviz2 -d " + rviz_cfg],
            env=dict(os.environ, DISPLAY=":0"))

        self._mapping = True
        self._btn_start_slam.setText("停止建图")
        self.status_label.setText("状态:\n建图中(" + slam_choice + ")")
        self.log_send("建图: " + slam_choice + " 已启动")
    def show_slam_page(self):
        self.setWindowTitle("SLAM 地图"); self.stack_layout.setCurrentWidget(self.slam_widget)

    def _save_map(self):
        from datetime import datetime
        import subprocess, os, threading
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        fname = os.path.join(MAP_DIR, "mymap_" + ts)
        self.log_send("保存地图: " + fname); self.status_label.setText("状态:\n保存地图中...")
        self._btn_save_map.setEnabled(False)

        def _run():
            try:
                r = subprocess.run(
                    ["bash", "-c",
                     self.ROS_SETUP + " && ros2 run nav2_map_server map_saver_cli -f " + fname],
                    env=dict(os.environ, DISPLAY=":0"), capture_output=True, text=True, timeout=35)
                err = r.stderr.strip() or r.stdout.strip()
                ok = r.returncode == 0 or os.path.exists(fname + ".pgm")
            except Exception as e:
                err = str(e); ok = False
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(0, lambda: self._on_save_done(ok, fname, err[:80] if err else ""))

        threading.Thread(target=_run, daemon=True).start()

    def _on_save_done(self, ok, fname, err):
        self._btn_save_map.setEnabled(True)
        if ok:
            self.log_send("地图保存成功: " + fname); self.status_label.setText("状态:\n地图已保存")
        else:
            self.log_send("保存失败: " + (err or "未知错误"))
            self.status_label.setText("状态:\n保存失败")

    def start_nav(self):
        """Nav2 导航 — 点一下启动选地图，再点停止恢复控制"""
        import subprocess, os, time, glob

        # --- 停止导航 ---
        if getattr(self, '_nav_on', False):
            self._nav_on = False
            self._btn_nav.setText("Nav2 导航")
            self.status_label.setText("状态:\n就绪")
            self.log_send("导航: 停止")
            # 杀 Nav2 launch、rviz2、激光、停电机
            subprocess.run(["pkill", "-f", "nav2_bringup"])
            subprocess.run(["pkill", "rviz2"])
            subprocess.run(["pkill", "-f", "rplidar|lslidar|x10"])
            subprocess.run(
                ["bash", "-c",
                 self.ROS_SETUP + " && timeout 3 ros2 service call /stop_motor std_srvs/srv/Empty '{}' 2>/dev/null"],
                env=dict(os.environ, DISPLAY=":0"))
            time.sleep(2)
            # 恢复底盘控制
            subprocess.Popen(
                ["bash", "-c",
                 self.BRIDGE_SETUP + " && " + self.BRIDGE_LAUNCH],
                env=dict(os.environ, DISPLAY=":0"))
            return

        # --- 启动导航 ---
        maps = sorted(glob.glob(os.path.join(MAP_DIR, "mymap*.yaml")), reverse=True)
        if not maps:
            self.log_send("导航: 没有已保存的地图，请先建图")
            return

        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QListWidget, QDialogButtonBox, QLabel
        dlg = QDialog(self)
        dlg.setWindowTitle("选择地图")
        dlg.setMinimumSize(400, 300)
        lay = QVBoxLayout(); dlg.setLayout(lay)
        lay.addWidget(QLabel("选择导航用的地图:"))
        lst = QListWidget()
        for m in maps:
            lst.addItem(os.path.basename(m))
        lst.setCurrentRow(0)
        lay.addWidget(lst)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept); bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if not dlg.exec_():
            return
        map_file = maps[lst.currentRow()]
        self.log_send("导航: 加载 " + os.path.basename(map_file))

        # 停建图 + 旧进程
        if getattr(self, '_mapping', False):
            subprocess.run(["pkill", "-f", "slam_toolbox|slam_gmapping|cartographer_node"])
            subprocess.run(["pkill", "rviz2"])
            self._mapping = False
            time.sleep(1)
        subprocess.run(["pkill", "-f", "chassis_bridge"])
        time.sleep(2)

        # 启 Nav2
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && ros2 launch nav2_bringup bringup_launch.py map:=" + map_file],
            env=dict(os.environ, DISPLAY=":0"))
        time.sleep(3)

        # 启 rviz2
        rviz_cfg = RVIZ_CFG
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && rviz2 -d " + rviz_cfg],
            env=dict(os.environ, DISPLAY=":0"))

        self._nav_on = True
        self._btn_nav.setText("停止导航")
        self.status_label.setText("状态:\n导航中")
        self.log_send("导航: 已启动，在 rviz2 用 2D Goal Pose 点目标")

    def _stop_slam(self):
        import subprocess, time
        # 先通过 ROS2 服务正常停止雷达电机
        subprocess.run(
            ["bash", "-c",
             self.ROS_SETUP + " && timeout 3 ros2 service call /stop_motor std_srvs/srv/Empty '{}' 2>/dev/null"],
            env=dict(os.environ, DISPLAY=":0"))
        time.sleep(1)
        subprocess.run(["pkill", "-f", "slam_toolbox|slam_gmapping|cartographer_node|rplidar|lslidar|x10"])
        subprocess.run(["pkill", "rviz2"])
        time.sleep(2)
        # 把底盘桥拉回来
        subprocess.Popen(
            ["bash", "-c",
             self.BRIDGE_SETUP + " && " + self.BRIDGE_LAUNCH],
            env=dict(os.environ, DISPLAY=":0"))
        self._mapping = False
        self._btn_start_slam.setText("开始建图")
        self.status_label.setText("就绪")
        self.log_send("建图: 已停止, 控制已恢复")



    def show_map_gallery(self):
        self.setWindowTitle("地图库"); self._refresh_gallery(); self.stack_layout.setCurrentWidget(self.map_gallery_widget)

    def _refresh_gallery(self):
        import glob, os
        lay = self.map_gallery_widget.layout()
        # 首次创建完整布局
        if not getattr(self, '_gallery_init', False):
            self._gallery_init = True
            # Header
            hdr = QHBoxLayout()
            t = QLabel("地图库"); t.setStyleSheet("font-size:28px;font-weight:bold;")
            hdr.addWidget(t); hdr.addStretch()
            btn_select = QPushButton("全选"); btn_select.setMinimumHeight(35)
            btn_select.clicked.connect(lambda: (
                [self._map_list.item(i).setCheckState(
                    Qt.Unchecked if btn_select.text() == "取消全选" else Qt.Checked)
                 for i in range(self._map_list.count())],
                btn_select.setText("全选" if btn_select.text() == "取消全选" else "取消全选")
            ))
            hdr.addWidget(btn_select)
            btn_refresh = QPushButton("刷新"); btn_refresh.clicked.connect(self._refresh_gallery)
            hdr.addWidget(btn_refresh)
            btn_back = QPushButton("返回"); btn_back.clicked.connect(self.show_slam_page)
            hdr.addWidget(btn_back)
            lay.addLayout(hdr)
            # QListWidget
            self._map_list = QListWidget()
            self._map_list.setStyleSheet("font-size:16px; QListWidget::indicator { width: 24px; height: 24px; }")
            self._map_list.setSelectionMode(QAbstractItemView.NoSelection)
            self._map_list.itemDoubleClicked.connect(self._map_card_click)
            lay.addWidget(self._map_list, 1)
            # 底部删除按钮
            btn_del = QPushButton("删除所选"); btn_del.setMinimumHeight(45)
            btn_del.setStyleSheet("background-color:#8b0000; color:white; font-size:20px;")
            btn_del.clicked.connect(self._delete_selected_maps)
            lay.addWidget(btn_del)
        else:
            self._map_list.clear()

        files = sorted(glob.glob(os.path.join(MAP_DIR, "mymap*.yaml")), reverse=True)
        for yf in files:
            base = yf.replace(".yaml", ""); pgm = base + ".pgm"
            if not os.path.exists(pgm): continue
            name = os.path.basename(base); sz = os.path.getsize(pgm)
            item = QListWidgetItem(f"{name}    {sz/1024:.0f}KB")
            item.setFlags(item.flags() | Qt.ItemIsUserCheckable)
            item.setCheckState(Qt.Unchecked)
            item.setData(Qt.UserRole, name)
            self._map_list.addItem(item)

    def _map_card_click(self, item):
        """双击地图 → 弹出操作菜单"""
        name = item.data(Qt.UserRole)
        self._card_click_dialog(name)

    def _delete_selected_maps(self):
        """删除所有勾选的地图"""
        to_del = [self._map_list.item(i).data(Qt.UserRole)
                  for i in range(self._map_list.count())
                  if self._map_list.item(i).checkState() == Qt.Checked]
        if not to_del:
            QMessageBox.information(self, "提示", "未选择任何地图")
            return
        if QMessageBox.question(self, "确认删除",
                                f"确定删除 {len(to_del)} 个地图？",
                                QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
        import os
        for name in to_del:
            for ext in [".pgm", ".yaml", ".ppm"]:
                p = os.path.join(MAP_DIR, name + ext)
                if os.path.exists(p):
                    os.remove(p)
        self._refresh_gallery()
        self.log_send(f"已删除 {len(to_del)} 个地图")

    def _card_click_dialog(self, name):
        """地图操作弹窗"""
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QPushButton
        dlg = QDialog(self); dlg.setWindowTitle(f"打开地图: {name}")
        dlg.setMinimumSize(320, 250)
        lay = QVBoxLayout(); dlg.setLayout(lay)
        btn_r = QPushButton("rviz2 查看"); btn_r.setMinimumHeight(40)
        btn_r.clicked.connect(lambda: (self._open_map_rviz(name), dlg.accept()))
        lay.addWidget(btn_r)
        btn_p = QPushButton("图片查看"); btn_p.setMinimumHeight(40)
        btn_p.clicked.connect(lambda: (self._open_map_ppm(name), dlg.accept()))
        lay.addWidget(btn_p)
        btn_rn = QPushButton("重命名"); btn_rn.setMinimumHeight(40)
        btn_rn.clicked.connect(lambda: self._rename_map(name))
        lay.addWidget(btn_rn)
        btn_d = QPushButton("删除"); btn_d.setMinimumHeight(40)
        btn_d.setStyleSheet("color:red;")
        btn_d.clicked.connect(lambda: self._delete_map(name))
        lay.addWidget(btn_d)
        dlg.exec_()

    def _open_map_rviz(self, name):
        import subprocess, os, time
        yaml_file = os.path.join(MAP_DIR, name + ".yaml")
        subprocess.run(["pkill", "-f", "map_server"]); time.sleep(1)
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && ros2 run nav2_map_server map_server "
             "--ros-args -p yaml_filename:=" + yaml_file],
            env=dict(os.environ, DISPLAY=":0"))
        time.sleep(2)
        # 激活 lifecycle 节点（否则不发布 /map）
        subprocess.run(
            ["bash", "-c",
             self.ROS_SETUP + " && ros2 lifecycle set /map_server configure 2>/dev/null"
             " && ros2 lifecycle set /map_server activate 2>/dev/null"],
            env=dict(os.environ, DISPLAY=":0"))
        rviz_cfg = RVIZ_CFG
        subprocess.Popen(
            ["bash", "-c",
             self.ROS_SETUP + " && rviz2 -d " + rviz_cfg],
            env=dict(os.environ, DISPLAY=":0"))
        self.log_send("rviz2查看(已加载map_server): " + name)

    def _open_map_ppm(self, name):
        """用 Qt 弹窗显示地图图片，不依赖外部看图软件"""
        import numpy as np
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel
        from PyQt5.QtGui import QPixmap, QImage
        from PyQt5.QtCore import Qt

        pgm = os.path.join(MAP_DIR, name + ".pgm")

        # 解析 PGM 头: P5\nW H\n255\n
        with open(pgm, "rb") as f:
            header_lines = []
            for _ in range(10):
                line = f.readline()
                if not line.startswith(b'#'):
                    header_lines.append(line.strip())
                if len(header_lines) >= 3:
                    break
            raw = f.read()
        w, h = map(int, header_lines[1].split())
        raw = raw[:w * h]

        # 转 RGB 数组: 0=黑(障碍), 1~100=白(空地), 其余=灰(未知)
        img = np.zeros((h, w, 3), dtype=np.uint8)
        for i, b in enumerate(raw):
            y, x = divmod(i, w)
            if b == 0:         img[y, x] = [0, 0, 0]
            elif b <= 100:     img[y, x] = [255, 255, 255]
            else:              img[y, x] = [80, 80, 80]

        qimg = QImage(img.data, w, h, w * 3, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimg)

        dlg = QDialog(self)
        dlg.setWindowTitle(f"地图: {name}")
        lay = QVBoxLayout(); dlg.setLayout(lay)
        lbl = QLabel()
        lbl.setPixmap(pixmap.scaled(900, 700, Qt.KeepAspectRatio, Qt.SmoothTransformation))
        lay.addWidget(lbl)
        dlg.exec_()
        self.log_send("打开图片: " + name)


    def _rename_map(self, old):
        from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QComboBox,
                                      QDialogButtonBox, QLabel)
        dlg = QDialog(self)
        dlg.setWindowTitle("重命名")
        dlg.setMinimumWidth(350)
        lay = QVBoxLayout(); dlg.setLayout(lay)
        lay.addWidget(QLabel("选择或输入新名称:"))
        combo = QComboBox(); combo.setEditable(True); combo.setInsertPolicy(QComboBox.NoInsert)
        # 常用中文名
        presets = [
            old,  # 原名排第一
            "客厅", "卧室", "阳台", "厨房", "院子", "走廊",
            "一楼", "二楼", "三楼", "前院", "后院", "花园",
            "办公室", "仓库", "车库", "草坪", "测试区",
        ]
        for name in presets:
            combo.addItem(name)
        combo.setCurrentText(old)
        lay.addWidget(combo)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept); bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if not dlg.exec_():
            return
        new = combo.currentText().strip()
        if new and new != old:
            import os
            for ext in [".pgm", ".yaml", ".ppm"]:
                a = os.path.join(MAP_DIR, old + ext); b = os.path.join(MAP_DIR, new + ext)
                if os.path.exists(a): os.rename(a, b)
            self._refresh_gallery(); self.log_send("重命名 " + old + " -> " + new)

    def _delete_map(self, name):
        from PyQt5.QtWidgets import QMessageBox
        if QMessageBox.question(self, "确认删除", "删除地图 " + name + " ？", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            import os
            for ext in [".pgm", ".yaml", ".ppm"]:
                p = os.path.join(MAP_DIR, name + ext)
                if os.path.exists(p): os.remove(p)
            self._refresh_gallery(); self.log_send("已删除: " + name)

    def _cancel_tasks(self, for_task=""):
        """互斥: 取消当前所有运行中任务, 返回被停止的列表"""
        stopped = []
        if getattr(self, '_selftest_running', False):
            self._selftest_running = False; stopped.append("自检")
            if hasattr(self, '_selftest_timer') and self._selftest_timer: self._selftest_timer.stop()
        if getattr(self, '_dock_running', False) or (self.dock.state != "IDLE"):
            self._dock_running = False; self.dock.stop(); stopped.append("回充")
        # 手动速度一定停掉，但不计入 stopped —— 计入了下面就会补一帧**锁存**的急停，
        # 开个自检还得先按解除急停，太容易把人绕进去。
        if self._manual_vx or self._manual_vz:
            self._keys_down.clear(); self._manual_set(0.0, 0.0)
        if stopped:
            self.serial_mgr.send(self.proto.build_emergency_frame())
            self._release_camera()
            self.log_send(f"接收到新任务, 已停止 {', '.join(stopped)}")
        return stopped

    # ---------- 手动控制 ----------
    # 补的是一条**完全没有**的通道：build_vel_frame 原先只有一个调用点，挂在
    # DockController.on_vel_cmd 上，而 _tick() 从不调它 —— 手动速度是死代码。

    def _manual_keys(self):
        return (Qt.Key_Up, Qt.Key_Down, Qt.Key_Left, Qt.Key_Right,
                Qt.Key_W, Qt.Key_S, Qt.Key_A, Qt.Key_D)

    def _on_page_changed(self, _idx):
        if self.stack_layout.currentWidget() is self.manual_widget:
            self.setFocus()          # 焦点回到窗口，方向键才到得了 keyPressEvent
        else:
            self._keys_down.clear()
            self._manual_set(0.0, 0.0)   # 切走就停，别留着一个速度在跑

    def keyPressEvent(self, e):
        # 只在手动控制页接管方向键 —— 免得在别的页面误触把车开走
        if (self.stack_layout.currentWidget() is self.manual_widget
                and not e.isAutoRepeat() and e.key() in self._manual_keys()):
            self._keys_down.add(e.key()); self._apply_manual_keys(); e.accept(); return
        super().keyPressEvent(e)

    def keyReleaseEvent(self, e):
        if (self.stack_layout.currentWidget() is self.manual_widget
                and not e.isAutoRepeat() and e.key() in self._manual_keys()):
            self._keys_down.discard(e.key()); self._apply_manual_keys(); e.accept(); return
        super().keyReleaseEvent(e)

    def _apply_manual_keys(self):
        ks = self._keys_down

        def want(pos, neg):
            return bool(ks & pos) and not (ks & neg)

        fwd = want({Qt.Key_Up, Qt.Key_W}, {Qt.Key_Down, Qt.Key_S})
        back = want({Qt.Key_Down, Qt.Key_S}, {Qt.Key_Up, Qt.Key_W})
        left = want({Qt.Key_Left, Qt.Key_A}, {Qt.Key_Right, Qt.Key_D})
        right = want({Qt.Key_Right, Qt.Key_D}, {Qt.Key_Left, Qt.Key_A})
        self._manual_set(self.MANUAL_V * (1 if fwd else -1 if back else 0),
                         self.MANUAL_W * (1 if left else -1 if right else 0))

    def _manual_set(self, vx, vz):
        if (vx, vz) == (self._manual_vx, self._manual_vz):
            return
        self._manual_vx, self._manual_vz = vx, vz
        if vx or vz:
            self._manual_timer.start(100)
            self.log_send("手动: Vx=%+.2f Vz=%+.2f" % (vx, vz))
        else:
            self._manual_timer.stop()
            self.serial_mgr.send(self.proto.build_vel_frame(0.0, 0.0, 0.0))
            self.log_send("手动: 停")

    def _manual_tick(self):
        # 10 Hz 重发。桥的看门狗是 0.5 s，这个频率够不上超时；面板万一崩了，
        # 看门狗半秒后自己把车停下 —— 固件那道保护够不着（见 bridge/node.py）。
        self.serial_mgr.send(self.proto.build_vel_frame(self._manual_vx, 0.0, self._manual_vz))

    def release_estop(self):
        self.serial_mgr.send(self.proto.build_release_frame())
        self.status_label.setText("状态:\n急停已解除")
        self.log_send("解除急停")

    def start_self_test(self):
        self._cancel_tasks("自检"); self._release_camera()
        self.status_label.setText("状态:\n自检中..."); self.log_send("自检: 开始")
        self._start_grab_record("自检")
        if hasattr(self, '_rec_cam_widget') and self._rec_cam_widget:
            self._rec_cam_widget.detect_green = False
        self._selftest_running = True; self.serial_mgr.send(self.proto.build_selftest_frame())
        self._selftest_timer = QTimer(); self._selftest_timer.setSingleShot(True); self._selftest_timer.timeout.connect(self._selftest_done); self._selftest_timer.start(12000)

    def _selftest_done(self):
        if not getattr(self, '_selftest_running', False): return
        self._selftest_timer.stop()
        self._selftest_running = False; self.log_send("自检: 完成"); self.status_label.setText("状态:\n就绪"); self._release_camera()

    def start_mow(self):
        # 弹窗选地图（仅展示，不加载）
        import glob, os
        maps = sorted(glob.glob(os.path.join(MAP_DIR, "mymap*.yaml")), reverse=True)
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QLabel, QDialogButtonBox, QListWidget
        dlg = QDialog(self); dlg.setWindowTitle("割草区域(Nav2导航)")
        dlg.setMinimumSize(400, 280)
        lay = QVBoxLayout(); dlg.setLayout(lay)
        lay.addWidget(QLabel("选择割草地图:"))
        lst = QListWidget()
        lst.addItem("跳过 (不选地图)")
        for m in maps:
            lst.addItem(os.path.basename(m))
        lst.setCurrentRow(0); lay.addWidget(lst)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept); bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if not dlg.exec_():
            return
        if lst.currentRow() > 0:
            self.log_send("割草区域: " + os.path.basename(maps[lst.currentRow() - 1]))

        self._cancel_tasks("割草"); self._release_camera()
        # 这条路径不发刀片指令 —— 刀片控制已整套移除，原因见 docs/移除记录.md。
        self.status_label.setText("状态:\n割草中(无刀片控制)")
        self.log_send("割草: 开始 —— 刀片未接入, 只有录像和导航")
        self._start_grab_record("割草")
        if hasattr(self, '_rec_cam_widget') and self._rec_cam_widget:
            self._rec_cam_widget.detect_green = True

    def toggle_follow(self):
        if self._follow_on:
            self._follow_on = False
            self.btn_follow.setText("雷达跟随")
            self.status_label.setText("状态:\n就绪")
            self.log_send("雷达跟随: 已停止")
        else:
            self._follow_on = True
            self.btn_follow.setText("停止跟随")
            self.status_label.setText("状态:\n雷达启动中...")
            self.log_send("雷达跟随: 启动中...")
            QTimer.singleShot(2000, lambda: (
                self.status_label.setText("状态:\n跟随中"),
                self.log_send("雷达跟随: 已开启")
            ))

    def start_dock(self):
        self._cancel_tasks("回充"); self._release_camera()
        self.status_label.setText("状态:\n扫描中..."); self.log_send("自动回充: 开始")
        self._start_grab_record("回充")
        self._dock_running = True; self.dock.start()

    def _on_tag_found(self): self.status_label.setText("状态:\n对准中...")
    def _on_dock_done(self): self._dock_running = False; self.status_label.setText("状态:\n对接完成")

    def stop_all(self):
        stopped = []
        if getattr(self, '_selftest_running', False):
            self._selftest_running = False; stopped.append("自检")
            if hasattr(self, '_selftest_timer') and self._selftest_timer: self._selftest_timer.stop()
        if getattr(self, '_dock_running', False) or self.dock.state != "IDLE":
            self._dock_running = False; self.dock.stop(); stopped.append("回充")
        if self.camera_widget and self.camera_widget.is_recording():
            self.camera_widget.stop_recording(); stopped.append("录像")
        if self._follow_on:
            self._follow_on = False
            self.btn_follow.setText("雷达跟随"); stopped.append("跟随")
        if self._manual_vx or self._manual_vz:
            self._keys_down.clear(); self._manual_set(0.0, 0.0); stopped.append("手动")
        if getattr(self, '_mapping', False):
            self._lidar_running = False
            self._mapping = False; stopped.append("建图")
            # 原来这里写的是 self._mapping_window —— 全树没赋过值的属性，
            # 建图中按急停会在这一行抛异常，**下面那帧急停就发不出去**。
            if self._mapping_widget:
                self._mapping_widget.close()
                self._mapping_widget = None
        self.serial_mgr.send(self.proto.build_emergency_frame())
        self._speed_left = 0; self._speed_right = 0
        self.status_label.setText("状态:\n已停止")
        self.log_send("紧急停止: " + ", ".join(stopped) if stopped else "紧急停止"); self._release_camera()

    def _update_status(self):
        states = {'IDLE':'待机','SWEEPING':'扫描','VERIFYING':'验证','ROTATING':'旋转','UNDOCK':'退桩','DONE':'完成'}
        self.log_recv(f"[状态] 舵机:{self.dock.servo_pos}us  {states.get(self.dock.state, self.dock.state)}  "
                      f"标签:{'找到' if self.dock.tag_seen else '未检测'}")

    def open_camera_window(self):
        self.camera_window = QWidget(); self.camera_window.setWindowTitle("摄像头画面"); self.camera_window.resize(800, 600)
        layout = QVBoxLayout(); self.camera_window.setLayout(layout)
        self.camera_widget = CameraWidget(); layout.addWidget(self.camera_widget); self.camera_widget.start_camera()
        self.camera_widget.on_tag_seen = lambda: self.dock.tag_detected()
        self.camera_window.closeEvent = self._on_camera_window_close; self.camera_window.show()
