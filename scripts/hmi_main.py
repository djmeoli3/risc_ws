import sys
import os
import csv
import time
import cv2
import subprocess
import numpy as np
from PyQt6.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QListWidget, QPushButton, QLabel,
                             QTableWidget, QTableWidgetItem, QMessageBox,
                             QHeaderView, QTabWidget, QProgressBar, QGridLayout,
                             QSizePolicy, QFrame, QFileDialog)
from PyQt6.QtCore import Qt, pyqtSignal, QObject, QRect, QThread, QTimer
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtGui import QPainter, QColor, QImage, QPixmap, QFont, QTransform

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Bool

NAV_AND_FEED_STATE = 2

# ---------------------------------------------------------------------------
# cmd index constants
# ---------------------------------------------------------------------------
class CMD:
    STATE_REQUEST  = 0
    X_LEAD_TARGET  = 3
    ZL_TARGET      = 4
    ZE_TARGET      = 5
    GRIP_OPEN      = 10
    ADHESIVE_ON    = 13


# ---------------------------------------------------------------------------
# camera worker
# ---------------------------------------------------------------------------
class CameraWorker(QObject):
    frame_ready  = pyqtSignal(QImage)
    camera_error = pyqtSignal(str)

    def run(self):
        cap = None
        # skip index 0 (built-in), validate before format set
        for idx in range(1, 10):
            try:
                test = cv2.VideoCapture(idx, cv2.CAP_V4L2)
                if not test.isOpened():
                    test.release()
                    continue
                # validate before applying format
                ret, frame = test.read()
                if ret and frame is not None and frame.size > 0:
                    # default format -- MJPG crashes some cameras
                    test.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    test.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)
                    cap = test
                    print(f"Camera found: index={idx}")
                    break
                test.release()
            except Exception as e:
                print(f"Camera index {idx} error: {e}")
                try: test.release()
                except: pass
                continue

        if cap is None:
            self.camera_error.emit("No USB camera found (indices 1-9)")
            return

        self._running = True
        while self._running:
            ret, frame = cap.read()
            if not ret:
                self.camera_error.emit("Camera disconnected")
                break
            try:
                processed = self._detect_brick(frame)
                rgb = cv2.cvtColor(processed, cv2.COLOR_BGR2RGB)
                h, w, ch = rgb.shape
                qt_img = QImage(rgb.data, w, h, ch * w, QImage.Format.Format_RGB888)
                self.frame_ready.emit(qt_img.copy())
            except Exception as e:
                print(f"Camera frame error: {e}")
            time.sleep(0.033)   # ~30 fps cap, no CPU spin

        cap.release()

    def stop(self):
        self._running = False

    # --- CV TUNING PARAMS (adjust these without touching logic) ---
    CV_ROI_TOP    = 0.05   # ROI top    (fraction of frame height)
    CV_ROI_BOTTOM = 0.95   # ROI bottom
    CV_ROI_LEFT   = 0.05   # ROI left
    CV_ROI_RIGHT  = 0.95   # ROI right
    CV_AREA_MIN   = 3000   # min contour area in pixels
    CV_AREA_MAX   = 80000  # max contour area in pixels
    CV_CANNY_LO   = 30     # Canny lower threshold
    CV_CANNY_HI   = 90     # Canny upper threshold
    CV_SHARPEN    = True   # apply unsharp mask to fight lens blur

    def _detect_brick(self, frame):
        h, w = frame.shape[:2]
        r_y1 = int(h * self.CV_ROI_TOP)
        r_y2 = int(h * self.CV_ROI_BOTTOM)
        r_x1 = int(w * self.CV_ROI_LEFT)
        r_x2 = int(w * self.CV_ROI_RIGHT)
        roi  = frame[r_y1:r_y2, r_x1:r_x2]

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # unsharp mask -- fights vibration blur
        if self.CV_SHARPEN:
            blur   = cv2.GaussianBlur(gray, (0, 0), 3)
            gray   = cv2.addWeighted(gray, 1.8, blur, -0.8, 0)

        # canny -- robust to lighting
        edges = cv2.Canny(gray, self.CV_CANNY_LO, self.CV_CANNY_HI)

        # close edge gaps
        kernel = np.ones((4, 4), np.uint8)
        edges  = cv2.dilate(edges, kernel, iterations=1)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        detected = 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if self.CV_AREA_MIN < area < self.CV_AREA_MAX:
                rect = cv2.minAreaRect(cnt)
                box  = np.int0(cv2.boxPoints(rect))
                box[:, 0] += r_x1
                box[:, 1] += r_y1
                cv2.drawContours(frame, [box], 0, (0, 230, 80), 2)
                detected += 1

        # roi outline
        cv2.rectangle(frame, (r_x1, r_y1), (r_x2, r_y2), (0, 100, 230), 1)

        # brick count overlay
        cv2.putText(frame, f"BRICKS: {detected}", (r_x1 + 8, r_y1 + 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 230, 80), 2)
        return frame


# ---------------------------------------------------------------------------
# ros worker
# ---------------------------------------------------------------------------
class ROSWorker(QObject):
    state_update    = pyqtSignal(int)
    xaxis_update    = pyqtSignal(list)
    zl_update       = pyqtSignal(list)
    progress_update = pyqtSignal(int, int)
    node_ready      = pyqtSignal()

    def run(self):
        rclpy.init()
        self.node = Node('hmi_ros_worker')
        self.manual_cmd_pub = self.node.create_publisher(Float32MultiArray, 'hmi_manual_cmd', 10)
        self.trigger_pub    = self.node.create_publisher(Bool, 'hmi_start_trigger', 10)
        self.pause_pub      = self.node.create_publisher(Bool, 'hmi_pause_trigger', 10)
        self.cancel_pub     = self.node.create_publisher(Bool, 'hmi_cancel_trigger', 10)
        self.node.create_subscription(Float32MultiArray, 'coordinator_state',
                                      self._status_cb, 10)
        self.node.create_subscription(Float32MultiArray, 'xaxis_status',
                                      self._xaxis_cb, 10)
        self.node.create_subscription(Float32MultiArray, 'zl_status',
                                      self._zl_cb, 10)
        self.node_ready.emit()
        # spin_once loop -- keeps thread responsive
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def _status_cb(self, msg):
        # [state, placed, total]
        placed = int(msg.data[1]) if len(msg.data) > 1 else 0
        total  = int(msg.data[2]) if len(msg.data) > 2 else 0
        self.state_update.emit(int(msg.data[0]))
        self.progress_update.emit(placed, total)

    def _xaxis_cb(self, msg):
        self.xaxis_update.emit(list(msg.data))

    def _zl_cb(self, msg):
        self.zl_update.emit(list(msg.data))

    def publish_manual(self, d: list):
        msg = Float32MultiArray()
        msg.data = d
        self.manual_cmd_pub.publish(msg)

    def publish_start(self, val: bool):
        msg = Bool(); msg.data = val
        self.trigger_pub.publish(msg)

    def publish_pause(self, val: bool):
        msg = Bool(); msg.data = val
        self.pause_pub.publish(msg)

    def publish_cancel(self):
        msg = Bool(); msg.data = True
        self.cancel_pub.publish(msg)


# ---------------------------------------------------------------------------
# build map widget
# ---------------------------------------------------------------------------
class BuildMap(QWidget):
    def __init__(self):
        super().__init__()
        self.bricks = []
        self.active_idx = -1
        self.current_layer_z = -1.0
        self.setMinimumHeight(250)
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)

    def set_data(self, brick_list):
        self.bricks = brick_list
        self.active_idx = -1
        if self.bricks:
            self.current_layer_z = self.bricks[0]['z']
        self.update()

    def update_progress(self, current_idx, current_z=None):
        self.active_idx = current_idx
        if current_z is not None:
            self.current_layer_z = current_z
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(QColor(13, 13, 13))
        painter.setPen(QColor(30, 30, 30))
        painter.drawRect(self.rect())

        if not self.bricks:
            painter.setPen(QColor(80, 80, 80))
            painter.setFont(QFont("Arial", 12))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, "No build loaded")
            return

        layer_bricks = [b for b in self.bricks if b['z'] == self.current_layer_z]
        if not layer_bricks:
            return

        max_x = max(b['x'] for b in layer_bricks) or 1
        max_y = max(b['y'] for b in layer_bricks) or 1
        pad = 40
        scale = min((self.width() - pad*2) / (max_x + 130),
                    (self.height() - pad*2) / (max_y + 130))
        scale = max(scale, 0.1)
        x_off = pad
        y_off = pad

        for i, brick in enumerate(self.bricks):
            if brick['z'] != self.current_layer_z:
                continue
            b_w = int(60 * scale)
            b_h = int(120 * scale)
            x_p = int(x_off + brick['x'] * scale)
            y_p = int(y_off + brick['y'] * scale)

            painter.save()
            painter.translate(x_p + b_w / 2, y_p + b_h / 2)
            painter.rotate(brick.get('theta', 0.0))

            global_idx = self.bricks.index(brick)
            if global_idx < self.active_idx:
                painter.setBrush(QColor(27, 94, 32))     # placed - deep green
            elif global_idx == self.active_idx:
                painter.setBrush(QColor(230, 81, 0))     # active - orange
            else:
                painter.setBrush(QColor(38, 38, 38))     # pending - dark grey

            painter.setPen(QColor(60, 60, 60))
            painter.drawRect(int(-b_w/2), int(-b_h/2), b_w, b_h)
            painter.restore()

            painter.setPen(QColor(230, 230, 230))
            # Font scales with brick size -- fits ~3 chars inside the short side
            # min(b_w, b_h) / 3 gives a font that fills ~1/3 of the narrow dimension
            # clamped between 7pt (tiny bricks) and 36pt (huge bricks)
            font_size = max(7, min(36, int(min(b_w, b_h) / 3)))
            painter.setFont(QFont("Arial", font_size, QFont.Weight.Bold))
            painter.drawText(QRect(x_p, y_p, b_w, b_h),
                             Qt.AlignmentFlag.AlignCenter, str(global_idx + 1))


# ---------------------------------------------------------------------------
# main hmi window
# ---------------------------------------------------------------------------
class RISCHMI(QMainWindow):

    # FSM state name map for display
    STATE_NAMES = {
        0: "IDLE", 1: "HOMING", 2: "NAV & FEED", 3: "GRIP ENGAGE",
        4: "SWING TO PUMP", 5: "ROTATE TO TARGET", 6: "LOWER & PLACE",
        7: "RELEASE BRICK", 8: "LIFT UP", 9: "ROTATE RESET",
        10: "SWING UP RESET", 11: "X OVERSHOOT", 12: "PUMP DELIVERY",
        13: "SWING TO FINAL", 98: "PAUSED", 99: "SAFETY STOP"
    }

    def __init__(self):
        super().__init__()
        self.setWindowTitle("RiSC 1.0 — Autonomous Brick Layer")
        self.resize(1280, 800)

        self.script_dir        = os.path.dirname(os.path.abspath(__file__))
        self.ifc_dir           = os.path.join(self.script_dir, "ifc_files")
        self.output_csv_dir    = os.path.join(self.script_dir, "output_csvs")
        self.translator_script = os.path.join(self.script_dir, "ifc_translation.py")

        self.total_bricks  = 0
        self.placed_bricks = 0
        self.last_state    = 0
        self.brick_data    = []
        self.is_paused     = False
        self.pump_on       = False

        # Jog timer — fires every 100ms while a jog button is held
        self._jog_timer = QTimer(self)
        self._jog_timer.setInterval(100)
        self._jog_timer.timeout.connect(self._send_jog_cmd)
        self._jog_x = 0.0
        self._jog_z = 0.0


        self._setup_ui()
        self._start_ros()
        self._start_camera()

        # Ctrl+Alt+T -- open terminal (useful when HMI is fullscreen)
        QShortcut(QKeySequence("Ctrl+Alt+T"), self).activated.connect(
            lambda: subprocess.Popen(["x-terminal-emulator"]))
        # Ctrl+Alt+Q -- clean exit
        QShortcut(QKeySequence("Ctrl+Alt+Q"), self).activated.connect(
            lambda: QApplication.instance().quit())
        # Poll ROS topic list every 3s to update agent indicators
        self._agent_timer = QTimer()
        self._agent_timer.timeout.connect(self._check_agents)
        self._agent_timer.start(3000)

        # Load any existing CSV on startup
        self.load_csv_preview()

    # ------------------------------------------------------------------
    # UI Setup
    # ------------------------------------------------------------------
    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8)

        self.tabs = QTabWidget()
        self.tabs.setStyleSheet("""
            QTabWidget::pane { border: 1px solid #444; background: #1e1e1e; }
            QTabBar::tab { background: #2d2d2d; color: #aaa; padding: 8px 20px; }
            QTabBar::tab:selected { background: #1e1e1e; color: #fff; border-bottom: 2px solid #e65100; }
        """)
        root.addWidget(self.tabs)

        self._setup_manual_tab()
        self._setup_project_tab()
        self._setup_monitor_tab()
        self.refresh_file_list()

    def _btn(self, label, color=None, height=50, checkable=False):
        """Helper: styled QPushButton — industrial orange/charcoal theme."""
        b = QPushButton(label)
        b.setCheckable(checkable)
        b.setMinimumHeight(height)
        bg = color if color else "#2a2a2a"
        style = (f"font-size: 13px; font-weight: bold; border-radius: 4px; "
                 f"padding: 4px 12px; letter-spacing: 1px; "
                 f"background-color: {bg}; color: {'#fff' if color else '#ccc'}; "
                 f"border: 1px solid {'#555' if not color else 'transparent'};")
        b.setStyleSheet(style)
        return b

    def _setup_manual_tab(self):
        page   = QWidget()
        layout = QHBoxLayout(page)   # two-column layout
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)

        # ── LEFT COLUMN: status + jog ────────────────────────────────────────
        left_col = QWidget()
        left_col.setStyleSheet("background: #141414; border-right: 1px solid #1e1e1e;")
        left_layout = QVBoxLayout(left_col)
        left_layout.setSpacing(0)
        left_layout.setContentsMargins(8, 8, 8, 8)

        # ROS status banner
        self.status_label = QLabel("WAITING FOR ROS...")
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.status_label.setStyleSheet(
            "font-size: 13px; font-weight: bold; color: #ff9800; letter-spacing: 3px; "
            "background: #1a1a1a; padding: 12px; border-bottom: 2px solid #e65100;")
        left_layout.addWidget(self.status_label)

        left_layout.addSpacing(16)

        # Axis position readouts
        pos_frame = QFrame()
        pos_frame.setStyleSheet("""
            QFrame { background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 4px; }
        """)
        pos_layout = QHBoxLayout(pos_frame)
        pos_layout.setContentsMargins(12, 8, 12, 8)

        def _pos_readout(axis_label):
            w = QWidget()
            v = QVBoxLayout(w)
            v.setSpacing(2)
            v.setContentsMargins(0,0,0,0)
            lbl = QLabel(axis_label)
            lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            lbl.setStyleSheet("color: #e65100; font-size: 7px; letter-spacing: 1px; font-weight: bold;")
            val = QLabel("---")
            val.setAlignment(Qt.AlignmentFlag.AlignCenter)
            val.setStyleSheet("color: #fff; font-size: 16px; font-family: 'Courier New'; font-weight: bold;")
            v.addWidget(lbl)
            v.addWidget(val)
            return w, val

        x_box, self.x_pos_label = _pos_readout("X  (mm FROM START)")
        z_box, self.z_pos_label = _pos_readout("Z  (mm FROM START)")

        divider = QFrame()
        divider.setFrameShape(QFrame.Shape.VLine)
        divider.setStyleSheet("color: #2a2a2a;")

        pos_layout.addWidget(x_box)
        pos_layout.addWidget(divider)
        pos_layout.addWidget(z_box)
        left_layout.addWidget(pos_frame)

        left_layout.addSpacing(16)

        # Jog section label
        jog_title = QLabel("MOTION CONTROL")
        jog_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        jog_title.setStyleSheet(
            "color: #e65100; font-size: 11px; letter-spacing: 4px; font-weight: bold; padding: 4px 0;")
        left_layout.addWidget(jog_title)

        left_layout.addSpacing(4)

        # Jog buttons — cross layout
        jog_frame = QFrame()
        jog_frame.setStyleSheet("background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 4px;")
        jog_inner = QVBoxLayout(jog_frame)
        jog_inner.setContentsMargins(12, 12, 12, 12)
        jog_inner.setSpacing(6)

        grid = QGridLayout()
        grid.setSpacing(6)

        self.btn_z_up    = self._btn("▲  Z UP",    "#bf360c", height=70)
        self.btn_z_down  = self._btn("▼  Z DOWN",  "#bf360c", height=70)
        self.btn_x_left  = self._btn("◀  X LEFT",  "#bf360c", height=70)
        self.btn_x_right = self._btn("X RIGHT  ▶", "#bf360c", height=70)

        _jog_style = ("font-size: 20px; font-weight: bold; border-radius: 4px; "
                      "letter-spacing: 2px; background-color: #bf360c; color: #fff; "
                      "border: 1px solid transparent;")
        for _b in [self.btn_z_up, self.btn_z_down, self.btn_x_left, self.btn_x_right]:
            _b.setStyleSheet(_jog_style)

        for btn in [self.btn_z_up, self.btn_z_down, self.btn_x_left, self.btn_x_right]:
            btn.setEnabled(False)

        self.btn_x_left.pressed.connect(lambda: [self._jog_start(-1e6, 0.0), self._jog_press_style(self.btn_x_left)])
        self.btn_x_left.released.connect(lambda: [self._jog_stop(), self._jog_release_style(self.btn_x_left)])
        self.btn_x_right.pressed.connect(lambda: [self._jog_start(1e6, 0.0), self._jog_press_style(self.btn_x_right)])
        self.btn_x_right.released.connect(lambda: [self._jog_stop(), self._jog_release_style(self.btn_x_right)])
        self.btn_z_up.pressed.connect(lambda: [self._jog_start(0.0, 1e6), self._jog_press_style(self.btn_z_up)])
        self.btn_z_up.released.connect(lambda: [self._jog_stop(), self._jog_release_style(self.btn_z_up)])
        self.btn_z_down.pressed.connect(lambda: [self._jog_start(0.0, -1e6), self._jog_press_style(self.btn_z_down)])
        self.btn_z_down.released.connect(lambda: [self._jog_stop(), self._jog_release_style(self.btn_z_down)])

        grid.addWidget(self.btn_z_up,    0, 0, 1, 2)
        grid.addWidget(self.btn_x_left,  1, 0)
        grid.addWidget(self.btn_x_right, 1, 1)
        grid.addWidget(self.btn_z_down,  2, 0, 1, 2)
        jog_inner.addLayout(grid)
        left_layout.addWidget(jog_frame)

        left_layout.addSpacing(10)

        # Pump button -- below motion control, orange default, maroon when ON
        pump_section_label = QLabel("ADHESIVE PUMP")
        pump_section_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pump_section_label.setStyleSheet(
            "color: #e65100; font-size: 11px; letter-spacing: 4px; font-weight: bold; padding: 2px 0;")
        left_layout.addWidget(pump_section_label)

        left_layout.addSpacing(4)

        self.btn_pump = self._btn("● PUMP  OFF", "#bf360c", height=60, checkable=True)
        self.btn_pump.setEnabled(False)
        self.btn_pump.setStyleSheet(
            "font-size: 15px; font-weight: bold; letter-spacing: 2px; border-radius: 4px; "
            "background: #bf360c; color: #fff; border: 2px solid #e65100;")
        self.btn_pump.toggled.connect(self._on_pump_toggle)
        left_layout.addWidget(self.btn_pump)

        left_layout.addStretch()

        # ── RIGHT COLUMN: pump + system info ────────────────────────────────
        right_col = QWidget()
        right_col.setStyleSheet("background: #111111;")
        right_layout = QVBoxLayout(right_col)
        right_layout.setSpacing(0)
        right_layout.setContentsMargins(8, 8, 8, 8)

        # ── RiSC Logo ─────────────────────────────────────────────────────
        logo_frame = QFrame()
        logo_frame.setStyleSheet("background: transparent;")
        logo_layout = QVBoxLayout(logo_frame)
        logo_layout.setSpacing(2)
        logo_layout.setContentsMargins(0, 0, 0, 8)

        import os as _os
        _logo_path = _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "risc_logo.png")
        robot_label = QLabel()
        robot_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        _pix = QPixmap(_logo_path).scaledToHeight(
            100, Qt.TransformationMode.SmoothTransformation)
        robot_label.setPixmap(_pix)

        risc_label = QLabel("RiSC 1.0")
        risc_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        risc_label.setStyleSheet(
            "color: #e65100; font-size: 18px; font-weight: bold; "
            "font-family: 'Courier New'; letter-spacing: 2px;")

        sub_label = QLabel("ROBOTICS IN SMART CONSTRUCTION")
        sub_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        sub_label.setStyleSheet(
            "color: #7b1d1d; font-size: 8px; letter-spacing: 1px; font-weight: bold;")

        divider_line = QFrame()
        divider_line.setFrameShape(QFrame.Shape.HLine)
        divider_line.setStyleSheet("color: #7b1d1d; background: #7b1d1d; max-height: 1px;")

        logo_layout.addWidget(robot_label)
        logo_layout.addWidget(risc_label)
        logo_layout.addWidget(sub_label)
        logo_layout.addSpacing(8)
        logo_layout.addWidget(divider_line)
        right_layout.addWidget(logo_frame)

        right_layout.addSpacing(8)

        # Section title
        pump_title = QLabel("SYSTEM STATUS")
        pump_title.setAlignment(Qt.AlignmentFlag.AlignCenter)
        pump_title.setStyleSheet(
            "color: #e65100; font-size: 11px; letter-spacing: 2px; font-weight: bold; padding-bottom: 8px;")
        right_layout.addWidget(pump_title)

        # Pump state indicator
        pump_state_frame = QFrame()
        pump_state_frame.setStyleSheet(
            "background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 4px; padding: 8px;")
        pump_state_layout = QVBoxLayout(pump_state_frame)

        self.pump_debug_label = QLabel("SYSTEM IDLE")
        self.pump_debug_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.pump_debug_label.setStyleSheet(
            "font-size: 14px; color: #2e7d32; font-weight: bold; letter-spacing: 2px; "
            "font-family: 'Courier New';")
        pump_state_layout.addWidget(self.pump_debug_label)
        right_layout.addWidget(pump_state_frame)

        right_layout.addSpacing(16)

        # System info panel — static labels showing axis info
        info_frame = QFrame()
        info_frame.setStyleSheet(
            "background: #1a1a1a; border: 1px solid #2a2a2a; border-radius: 4px;")
        info_layout = QVBoxLayout(info_frame)
        info_layout.setContentsMargins(12, 10, 12, 10)
        info_layout.setSpacing(6)

        info_title = QLabel("SYSTEM INFO")
        info_title.setStyleSheet(
            "color: #e65100; font-size: 9px; letter-spacing: 2px; font-weight: bold;")
        info_layout.addWidget(info_title)

        for key, val in [("X RANGE",      "0 – ~700 mm"),
                         ("Z RANGE",      "0 – ~1000 mm"),
                         ("JOG SPEED",    "50% MAX"),
                         ("BRICK SIZE",   "127 × 59.18 × 38.1 mm"),
                         ("LAYER HEIGHT", "38.1 mm"),
                         ("PROTOCOL",     "ROS2 HUMBLE")]:
            row = QHBoxLayout()
            k = QLabel(key)
            k.setStyleSheet("color: #777; font-size: 10px; letter-spacing: 1px;")
            v = QLabel(val)
            v.setStyleSheet("color: #bbb; font-size: 10px; font-family: 'Courier New';")
            v.setAlignment(Qt.AlignmentFlag.AlignRight)
            row.addWidget(k)
            row.addWidget(v)
            info_layout.addLayout(row)

        # agent status -- lights up with udev rules
        info_layout.addSpacing(8)
        agents_title = QLabel("AGENT STATUS")
        agents_title.setStyleSheet(
            "color: #e65100; font-size: 9px; letter-spacing: 2px; font-weight: bold;")
        info_layout.addWidget(agents_title)

        self.agent_indicators = {}
        for agent_name in ["TOOLHEAD", "X-AXIS", "Z-EBOX", "Z-LIFT"]:
            row = QHBoxLayout()
            dot = QLabel("●")
            dot.setStyleSheet("color: #333; font-size: 14px;")
            lbl = QLabel(agent_name)
            lbl.setStyleSheet("color: #888; font-size: 10px; letter-spacing: 1px;")
            status = QLabel("OFFLINE")
            status.setStyleSheet("color: #555; font-size: 10px; font-family: 'Courier New';")
            status.setAlignment(Qt.AlignmentFlag.AlignRight)
            row.addWidget(dot)
            row.addSpacing(6)
            row.addWidget(lbl)
            row.addWidget(status)
            info_layout.addLayout(row)
            self.agent_indicators[agent_name] = (dot, status)


        right_layout.addWidget(info_frame)
        right_layout.addStretch()

        layout.addWidget(left_col, 5)
        layout.addWidget(right_col, 4)
        self.tabs.addTab(page, "Manual Control")

    def _setup_project_tab(self):
        page   = QWidget()
        layout = QHBoxLayout(page)
        layout.setSpacing(12)

        # Left: file list
        left = QVBoxLayout()
        left.setSpacing(6)
        # Header row: label + add button
        header_row = QHBoxLayout()
        left_label = QLabel("IFC FILES")
        left_label.setStyleSheet("color: #e65100; font-size: 16px; letter-spacing: 4px; font-weight: bold;")
        add_btn = QPushButton("+")
        add_btn.setFixedSize(36, 36)
        add_btn.setToolTip("Add IFC file")
        add_btn.setStyleSheet("""
            QPushButton {
                background: #e65100; color: white; font-size: 20px; font-weight: bold;
                border-radius: 18px; border: none;
            }
            QPushButton:hover { background: #f57c00; }
            QPushButton:pressed { background: #bf360c; }
        """)
        add_btn.clicked.connect(self.add_ifc_file)
        header_row.addWidget(left_label)
        header_row.addStretch()
        header_row.addWidget(add_btn)

        self.file_list = QListWidget()
        self.file_list.setStyleSheet(
            "background: #1a1a1a; color: #e0e0e0; border: 1px solid #2a2a2a; border-radius: 4px;")
        self.translate_btn = self._btn("⚙  GENERATE COORDINATES", "#e65100", height=44)
        self.translate_btn.clicked.connect(self.run_translation)
        self.delete_btn    = self._btn("🗑  DELETE SELECTED IFC",  "#4a0000", height=44)
        self.delete_btn.clicked.connect(self.delete_ifc)
        left.addLayout(header_row)
        left.addWidget(self.file_list)
        left.addWidget(self.translate_btn)
        left.addWidget(self.delete_btn)

        # Right: preview table + start button
        right = QVBoxLayout()
        right.setSpacing(6)
        right_label = QLabel("COORDINATE PREVIEW")
        right_label.setStyleSheet("color: #e65100; font-size: 13px; letter-spacing: 3px; font-weight: bold;")
        self.preview_table = QTableWidget()
        self.preview_table.setColumnCount(7)
        self.preview_table.setHorizontalHeaderLabels(
            ["Brick", "X", "Y", "Z", "Theta", "Drop Offset", "X Overshoot"])
        self.preview_table.horizontalHeader().setSectionResizeMode(
            QHeaderView.ResizeMode.Stretch)
        self.preview_table.setStyleSheet(
            "background: #1a1a1a; color: #e0e0e0; gridline-color: #2a2a2a; border: none;")
        self.start_build_btn = self._btn("▶  START BUILD", "#7b1d1d", height=64)
        self.start_build_btn.setVisible(False)
        self.start_build_btn.clicked.connect(self.start_robot_mission)
        right.addWidget(right_label)
        right.addWidget(self.preview_table)
        right.addWidget(self.start_build_btn)

        layout.addLayout(left, 1)
        layout.addLayout(right, 2)
        self.tabs.addTab(page, "Project Manager")

    def _setup_monitor_tab(self):
        page   = QWidget()
        layout = QHBoxLayout(page)
        layout.setSpacing(12)

        # --- Left panel: stats + build map ---
        left = QVBoxLayout()
        left.setSpacing(8)

        # Stats row
        stats_frame = QFrame()
        stats_frame.setStyleSheet("background: #1a1a1a; border-radius: 4px; border: 1px solid #2a2a2a;")
        stats_layout = QHBoxLayout(stats_frame)

        self.stat_state = self._stat_box("STATE", "IDLE")
        self.stat_layer = self._stat_box("LAYER", "—")
        self.stat_brick = self._stat_box("BRICK", "— / —")

        for w in [self.stat_state, self.stat_layer, self.stat_brick]:
            stats_layout.addWidget(w)

        left.addWidget(stats_frame)

        # Pause / Resume+Cancel button area
        self.pause_widget = QWidget()
        pause_layout = QHBoxLayout(self.pause_widget)
        pause_layout.setContentsMargins(0, 0, 0, 0)
        pause_layout.setSpacing(6)

        self.btn_pause = self._btn("⏸  PAUSE BUILD", "#7b1d1d", height=55)
        self.btn_pause.setEnabled(False)
        self.btn_pause.clicked.connect(self._on_pause_clicked)

        self.btn_resume = self._btn("▶  RESUME", "#2e7d32", height=55)
        self.btn_resume.setVisible(False)
        self.btn_resume.clicked.connect(self._on_resume_clicked)

        self.btn_cancel = self._btn("✕  CANCEL BUILD", "#4a0000", height=55)
        self.btn_cancel.setVisible(False)
        self.btn_cancel.clicked.connect(self._on_cancel_clicked)

        pause_layout.addWidget(self.btn_pause)
        pause_layout.addWidget(self.btn_resume)
        pause_layout.addWidget(self.btn_cancel)
        left.addWidget(self.pause_widget)

        # Progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setStyleSheet("""
            QProgressBar {
                background: #1a1a1a;
                border-radius: 3px;
                height: 22px;
                border: 1px solid #2a2a2a;
                text-align: center;
                color: #fff;
                font-size: 11px;
                font-weight: bold;
                letter-spacing: 2px;
            }
            QProgressBar::chunk {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #bf360c, stop:0.5 #e65100, stop:1 #f57c00);
                border-radius: 2px;
            }
        """)
        self.progress_bar.setFormat("%p%  ■ ■ ■")
        left.addWidget(self.progress_bar)

        # Build map
        map_label = QLabel("BUILD MAP  (CURRENT LAYER)")
        map_label.setStyleSheet("color: #e65100; font-size: 13px; letter-spacing: 3px; font-weight: bold;")
        self.build_map = BuildMap()
        self.build_map.setStyleSheet("background: #0d0d0d; border-radius: 4px; border: 1px solid #2a2a2a;")
        left.addWidget(map_label)
        left.addWidget(self.build_map)

        # --- Right panel: camera ---
        right = QVBoxLayout()
        right.setSpacing(6)
        cam_label = QLabel("CAMERA FEED")
        cam_label.setStyleSheet("color: #e65100; font-size: 13px; letter-spacing: 3px; font-weight: bold;")
        self.camera_label = QLabel()
        self.camera_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.camera_label.setMinimumSize(640, 360)
        self.camera_label.setStyleSheet(
            "background: #0d0d0d; color: #444; border-radius: 4px; font-size: 12px; border: 1px solid #2a2a2a; letter-spacing: 2px;")
        self.camera_label.setText("Initializing camera...")
        right.addWidget(cam_label)
        right.addWidget(self.camera_label)
        right.addStretch()

        layout.addLayout(left, 3)
        layout.addLayout(right, 2)
        self.tabs.addTab(page, "Build Monitor")

    def _stat_box(self, title, value):
        """Helper: small stat display widget."""
        frame = QFrame()
        frame.setStyleSheet("background: #141414; border-radius: 4px; padding: 6px; border: 1px solid #2a2a2a;")
        vbox = QVBoxLayout(frame)
        vbox.setSpacing(2)
        t = QLabel(title)
        t.setAlignment(Qt.AlignmentFlag.AlignCenter)
        t.setStyleSheet("color: #e65100; font-size: 11px; letter-spacing: 2px; font-weight: bold;")
        v = QLabel(value)
        v.setAlignment(Qt.AlignmentFlag.AlignCenter)
        v.setStyleSheet("color: #fff; font-size: 24px; font-weight: bold; font-family: 'Courier New', monospace;")
        vbox.addWidget(t)
        vbox.addWidget(v)
        frame._value_label = v
        return frame

    def _set_stat(self, frame, value):
        frame._value_label.setText(str(value))

    # ------------------------------------------------------------------
    # ROS + Camera startup (on main thread — no outer daemon thread)
    # ------------------------------------------------------------------
    def _start_ros(self):
        self.ros_worker = ROSWorker()
        self.ros_thread = QThread()
        self.ros_worker.moveToThread(self.ros_thread)
        self.ros_thread.started.connect(self.ros_worker.run)
        self.ros_worker.node_ready.connect(self._on_ros_ready)
        self.ros_worker.state_update.connect(self._on_fsm_update)
        self.ros_worker.xaxis_update.connect(self._on_xaxis_update)
        self.ros_worker.zl_update.connect(self._on_zl_update)
        self.ros_worker.progress_update.connect(self._on_progress_update)
        self.ros_thread.start()

    def _start_camera(self):
        self.cam_worker = CameraWorker()
        self.cam_thread = QThread()
        self.cam_worker.moveToThread(self.cam_thread)
        self.cam_thread.started.connect(self.cam_worker.run)
        self.cam_worker.frame_ready.connect(self._on_frame_ready)
        self.cam_worker.camera_error.connect(self._on_camera_error)
        self.cam_thread.start()

    # ------------------------------------------------------------------
    # Jog logic (hold-to-move via QTimer)
    # ------------------------------------------------------------------
    def _jog_start(self, x_val, z_val):
        self._jog_x = x_val
        self._jog_z = z_val
        self._send_jog_cmd()       # immediate first fire
        self._jog_timer.start()

    def _jog_stop(self):
        self._jog_timer.stop()
        if hasattr(self, 'ros_worker'):
            d = [0.0] * 17
            self.ros_worker.publish_manual(d)

    def _jog_press_style(self, btn):
        btn.setStyleSheet(btn.styleSheet().replace("#bf360c", "#6d1f07"))

    def _jog_release_style(self, btn):
        btn.setStyleSheet(btn.styleSheet().replace("#6d1f07", "#bf360c"))

    def _send_jog_cmd(self):
        if not hasattr(self, 'ros_worker'):
            return
        d = [0.0] * 17
        d[CMD.X_LEAD_TARGET] = self._jog_x
        d[CMD.ZL_TARGET]     = self._jog_z
        d[CMD.ZE_TARGET]     = self._jog_z

        self.ros_worker.publish_manual(d)

    # ------------------------------------------------------------------
    # Pump + Pause
    # ------------------------------------------------------------------
    def _on_pump_toggle(self, checked):
        """Toggle pump on/off. Firmware handles the 0.5s retract on turn-off."""
        self.pump_on = checked
        self.btn_pump.setText("● PUMP  ON" if checked else "● PUMP  OFF")
        self.btn_pump.setStyleSheet(
            "font-size: 15px; font-weight: bold; letter-spacing: 2px; border-radius: 4px; "
            + ("background: #7b1d1d; color: #ffccbc; border: 2px solid #bf360c;"
               if checked else
               "background: #bf360c; color: #fff; border: 2px solid #e65100;"))
        if hasattr(self, 'ros_worker'):
            d = [0.0] * 17
            d[CMD.ADHESIVE_ON] = 1.0 if checked else 0.0
            self.ros_worker.publish_manual(d)

    def _show_low_adhesive_toast(self):
        """Show a non-blocking LOW ADHESIVE warning in the bottom-right corner."""
        toast = QFrame(self)
        toast.setStyleSheet("""
            QFrame {
                background: #1a1a1a;
                border: 2px solid #7b1d1d;
                border-radius: 6px;
            }
        """)
        toast_layout = QVBoxLayout(toast)
        toast_layout.setContentsMargins(16, 12, 16, 12)
        toast_layout.setSpacing(8)

        title = QLabel("⚠  LOW ADHESIVE")
        title.setStyleSheet(
            "color: #e65100; font-size: 14px; font-weight: bold; letter-spacing: 2px;")
        title.setAlignment(Qt.AlignmentFlag.AlignCenter)

        msg = QLabel("Adhesive reservoir is running low.\nRefill before continuing the build.")
        msg.setStyleSheet("color: #aaa; font-size: 11px;")
        msg.setAlignment(Qt.AlignmentFlag.AlignCenter)

        ok_btn = QPushButton("OK")
        ok_btn.setStyleSheet(
            "background: #7b1d1d; color: #ffccbc; font-weight: bold; "
            "border-radius: 4px; padding: 6px 24px; border: 1px solid #bf360c;")
        ok_btn.clicked.connect(toast.deleteLater)

        toast_layout.addWidget(title)
        toast_layout.addWidget(msg)
        toast_layout.addWidget(ok_btn, alignment=Qt.AlignmentFlag.AlignCenter)

        toast.adjustSize()
        # Position bottom-right of main window
        tw, th = toast.sizeHint().width() + 32, toast.sizeHint().height() + 32
        toast.setFixedSize(max(tw, 280), max(th, 120))
        x = self.width()  - toast.width()  - 20
        y = self.height() - toast.height() - 20
        toast.move(x, y)
        toast.show()
        toast.raise_()

    def _on_pause_clicked(self):
        self.is_paused = True
        self.btn_pause.setVisible(False)
        self.btn_resume.setVisible(True)
        self.btn_cancel.setVisible(True)
        if hasattr(self, 'ros_worker'):
            self.ros_worker.publish_pause(True)

    def _on_resume_clicked(self):
        self.is_paused = False
        self.btn_resume.setVisible(False)
        self.btn_cancel.setVisible(False)
        self.btn_pause.setVisible(True)
        if hasattr(self, 'ros_worker'):
            self.ros_worker.publish_pause(False)

    def _on_cancel_clicked(self):
        reply = QMessageBox.question(
            self, "Cancel Build",
            "Cancel the current build and return to IDLE?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if reply == QMessageBox.StandardButton.Yes:
            self.is_paused = False
            self.btn_resume.setVisible(False)
            self.btn_cancel.setVisible(False)
            self.btn_pause.setVisible(True)
            self.placed_bricks = 0
            self._first_nav_seen = False
            self._update_monitor()
            if hasattr(self, 'ros_worker'):
                self.ros_worker.publish_cancel()

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------
    def _on_progress_update(self, placed: int, total: int):
        """Directly driven by coordinator brick counter -- no state-transition inference."""
        if total == 0:
            return
        
        if total != self.total_bricks:
            self.total_bricks = total
        if placed != self.placed_bricks:
            self.placed_bricks = placed
            self._update_monitor()

    def _on_zl_update(self, data):
        if len(data) > 1 and hasattr(self, 'z_pos_label'):
            self.z_pos_label.setText(f"{data[1]:.1f}")

    def _on_xaxis_update(self, data):
        if len(data) > 1 and hasattr(self, 'x_pos_label'):
            self.x_pos_label.setText(f"{data[1]:.1f}")
        # IR signal at index 11 -- 1.0 = low adhesive (sensor blocked)
        if len(data) > 11:
            ir_low = data[11] > 0.5
            if ir_low and not getattr(self, '_ir_alert_shown', False):
                self._ir_alert_shown = True
                self._show_low_adhesive_toast()
            elif not ir_low:
                self._ir_alert_shown = False
        if len(data) > 2:
            pump_state = int(data[2])
            labels = {0: "IDLE (stop)", 1: "FWD (latch+on)",
                      2: "RETRACT (active)", 10: "CB: saw ON",
                      20: "CB: saw OFF"}
            labels_style = {
                0: ("SYSTEM IDLE",    "#555"),
                1: ("PUMP RUNNING",   "#e65100"),
                2: ("RETRACTING...",  "#bf360c"),
                10: ("CMD: ON",       "#f57c00"),
                20: ("CMD: OFF",      "#888"),
            }
            text, color = labels_style.get(pump_state, (f"STATE {pump_state}", "#555"))
            self.pump_debug_label.setText(text)
            self.pump_debug_label.setStyleSheet(
                f"font-size: 14px; color: {color}; font-weight: bold; "
                f"letter-spacing: 2px; font-family: 'Courier New';"
            )

    def _check_agents(self):
        """Check which hardware nodes are publishing and update indicators."""
        if not getattr(self, '_ros_ready', False):
            return
        topic_map = {
            "TOOLHEAD": "toolhead_status",
            "X-AXIS":   "xaxis_status",
            "Z-EBOX":   "ze_status",
            "Z-LIFT":   "zl_status",
        }
        for name, topic in topic_map.items():
            dot, status_lbl = self.agent_indicators[name]
            # Check if we have received data on this topic recently
            alive = self.ros_worker.node.count_publishers(topic) > 0
            dot.setStyleSheet(f"color: {'#2e7d32' if alive else '#7b1d1d'}; font-size: 14px;")
            status_lbl.setText("ONLINE" if alive else "OFFLINE")
            status_lbl.setStyleSheet(
                f"color: {'#4caf50' if alive else '#555'}; font-size: 12px; font-family: 'Courier New';")

    def _on_ros_ready(self):
        self._ros_ready = True
        self.status_label.setText("ROS2 READY")
        self.status_label.setStyleSheet(
            "font-size: 15px; font-weight: bold; color: #e65100; letter-spacing: 3px; "
            "background: #1a1a1a; border-radius: 4px; padding: 10px; border: 1px solid #e65100;")
        for btn in [self.btn_x_left, self.btn_x_right,
                    self.btn_z_up, self.btn_z_down]:
            btn.setEnabled(True)
        self.btn_pump.setEnabled(True)
        self.btn_pump.setStyleSheet(
            "font-size: 15px; font-weight: bold; letter-spacing: 2px; border-radius: 4px; "
            "background: #bf360c; color: #fff; border: 2px solid #e65100;")
        self.btn_pause.setEnabled(True)
        self.btn_resume.setEnabled(True)
        self.btn_cancel.setEnabled(True)

    def _on_fsm_update(self, state: int):
        name = self.STATE_NAMES.get(state, f"STATE {state}")
        self.status_label.setText(name)
        self._set_stat(self.stat_state, name)
        self.last_state = state

    def _update_monitor(self):
        pct = int(self.placed_bricks / self.total_bricks * 100) if self.total_bricks else 0
        self.progress_bar.setValue(pct)
        self._set_stat(self.stat_brick, f"{self.placed_bricks} / {self.total_bricks}")

        # Current layer number
        if self.placed_bricks > 0 and self.brick_data:
            idx = min(self.placed_bricks - 1, len(self.brick_data) - 1)
            current_z = self.brick_data[idx]['z']
            unique_layers = sorted(set(b['z'] for b in self.brick_data))
            layer_num = unique_layers.index(current_z) + 1
            self._set_stat(self.stat_layer, f"{layer_num} / {len(unique_layers)}")
            self.build_map.update_progress(self.placed_bricks, current_z)  # active=placed_bricks, green=<placed_bricks

    def _on_frame_ready(self, qt_image: QImage):
        # Rotate 90° clockwise to correct camera mounting angle
        # Change to 0, 90, 180, 270 as needed
        transform = QTransform().rotate(90)
        rotated = qt_image.transformed(transform)
        pix = QPixmap.fromImage(rotated).scaled(
            self.camera_label.width(), self.camera_label.height(),
            Qt.AspectRatioMode.KeepAspectRatio,
            Qt.TransformationMode.SmoothTransformation)
        self.camera_label.setPixmap(pix)

    def _on_camera_error(self, msg: str):
        self.camera_label.setText(f"⚠  {msg}")
        self.camera_label.setStyleSheet(
            "background: #1a1a1a; color: #f44336; border-radius: 6px; "
            "font-size: 13px; border: 1px solid #f44336;")

    # ------------------------------------------------------------------
    # Project manager
    # ------------------------------------------------------------------
    def load_csv_preview(self):
        csv_path = os.path.join(self.output_csv_dir, "current_build.csv")
        if not os.path.exists(csv_path):
            return
        self.preview_table.setRowCount(0)
        self.brick_data = []
        try:
            with open(csv_path, 'r') as f:
                for row_idx, row in enumerate(csv.DictReader(f)):
                    self.preview_table.insertRow(row_idx)
                    bx, by, bz = float(row['x']), float(row['y']), float(row['z'])
                    self.brick_data.append(
                        {'x': bx, 'y': by, 'z': bz, 'theta': float(row['theta'])})
                    for col, key in enumerate(['brick_id','x','y','z','theta','drop_offset','x_overshoot']):
                        self.preview_table.setItem(
                            row_idx, col, QTableWidgetItem(row.get(key, '')))
            self.total_bricks = len(self.brick_data)
            self.start_build_btn.setVisible(True)
            self.build_map.set_data(self.brick_data)
            self.build_map.update_progress(0)  # show first brick as active (yellow)

            unique_layers = len(set(b['z'] for b in self.brick_data))
            self._set_stat(self.stat_brick, f"0 / {self.total_bricks}")
            self._set_stat(self.stat_layer, f"— / {unique_layers}")
        except Exception as e:
            print(f"CSV load error: {e}")

    def run_translation(self):
        selected = self.file_list.currentItem()
        if not selected:
            return
        input_path  = os.path.join(self.ifc_dir, selected.text())
        output_path = os.path.join(self.output_csv_dir, "current_build.csv")
        os.makedirs(self.output_csv_dir, exist_ok=True)
        try:
            subprocess.run(
                ["python3", self.translator_script, input_path, output_path],
                check=True)
            self.load_csv_preview()
        except Exception as e:
            QMessageBox.critical(self, "Translation Error", str(e))

    def start_robot_mission(self):
        if not getattr(self, '_ros_ready', False):
            QMessageBox.warning(self, "Not Ready", "ROS is not connected yet.")
            return
        if not self.brick_data:
            QMessageBox.warning(self, "No Build Loaded", "Please load and translate an IFC file first.")
            return
        self.placed_bricks = 0
        self.total_bricks  = 0
        self._update_monitor()
        self.ros_worker.publish_start(True)
        self.tabs.setCurrentIndex(2)   # jump to Build Monitor

    def add_ifc_file(self):
        """Open file picker and copy selected IFC files into ifc_files/."""
        paths, _ = QFileDialog.getOpenFileNames(
            self, "Add IFC Files", "", "IFC Files (*.ifc);;All Files (*)")
        if not paths:
            return
        os.makedirs(self.ifc_dir, exist_ok=True)
        added = 0
        for path in paths:
            dest = os.path.join(self.ifc_dir, os.path.basename(path))
            if os.path.abspath(path) == os.path.abspath(dest):
                continue  # already in the right place
            import shutil
            shutil.copy2(path, dest)
            added += 1
        if added:
            self.refresh_file_list()

    def delete_ifc(self):
        selected = self.file_list.currentItem()
        if not selected:
            return
        filename = selected.text()
        if QMessageBox.question(
                self, 'Confirm', f"Permanently delete {filename}?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        ) == QMessageBox.StandardButton.Yes:
            os.remove(os.path.join(self.ifc_dir, filename))
            self.refresh_file_list()
            self.preview_table.setRowCount(0)
            self.start_build_btn.setVisible(False)

    def refresh_file_list(self):
        self.file_list.clear()
        if os.path.exists(self.ifc_dir):
            files = [f for f in os.listdir(self.ifc_dir) if f.endswith('.ifc')]
            self.file_list.addItems(sorted(files))


# ---------------------------------------------------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setStyleSheet("""
        QMainWindow, QWidget   { background-color: #141414; color: #e0e0e0; font-family: 'Courier New', monospace; }
        QTabWidget::pane       { border: 1px solid #333; background: #141414; }
        QTabBar::tab           { background: #1e1e1e; color: #777; padding: 10px 24px;
                                 font-size: 11px; letter-spacing: 2px; font-weight: bold;
                                 border-bottom: 2px solid transparent; }
        QTabBar::tab:selected  { background: #141414; color: #e65100;
                                 border-bottom: 2px solid #e65100; }
        QTabBar::tab:hover     { color: #ccc; }
        QListWidget            { background: #1e1e1e; color: #e0e0e0;
                                 border: 1px solid #333; border-radius: 4px; }
        QListWidget::item:selected { background: #e65100; color: #fff; }
        QListWidget::item:hover    { background: #2a2a2a; }
        QTableWidget           { background: #1e1e1e; color: #e0e0e0; gridline-color: #2a2a2a;
                                 border: none; }
        QHeaderView::section   { background: #1a1a1a; color: #e65100; padding: 6px;
                                 border: none; font-size: 10px; letter-spacing: 1px;
                                 font-weight: bold; }
        QScrollBar:vertical    { background: #1e1e1e; width: 6px; }
        QScrollBar::handle:vertical { background: #e65100; border-radius: 3px; }
        QScrollBar:horizontal  { background: #1e1e1e; height: 6px; }
        QScrollBar::handle:horizontal { background: #e65100; border-radius: 3px; }
        QToolTip               { background: #1e1e1e; color: #e65100; border: 1px solid #e65100; }
    """)
    window = RISCHMI()
    window.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
    window.setGeometry(0, 0, 1024, 600)
    window.show()
    sys.exit(app.exec())