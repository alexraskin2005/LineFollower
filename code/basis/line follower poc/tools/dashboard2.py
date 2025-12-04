#!/usr/bin/env python3
"""
ESP32 PyQt5 Dashboard (TCP receive, HTTP commands, raw JSON tab)

- Receives telemetry via TCP (line-delimited JSON).
- Sends control commands via HTTP (start/stop/set/calibrate/enable/disable).
- Two tabs: Dashboard + Raw JSON.
"""

import sys
import socket
import json
import time
import requests
import datetime
import subprocess
import platform
import re

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QPainter, QColor, QPen
from PyQt5.QtCore import QThread, pyqtSignal
import pyqtgraph as pg

last_timestamp = None


class MotorBar(QtWidgets.QWidget):
    """
    Custom motor bar: 0 is center.
    Positive = fill left->right.
    Negative = fill right->left.
    Shows numeric value centered.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.value = 0
        self.setMinimumHeight(30)

    def setValue(self, v):
        try:
            v = int(v)
        except Exception:
            try:
                v = int(float(v))
            except Exception:
                v = 0
        v = max(-1000, min(1000, v))
        self.value = v
        self.update()

    def paintEvent(self, e):
        painter = QPainter(self)
        rect = self.rect()

        bg_color = QColor(211, 211, 211)
        fill_pos_color = QColor(60, 180, 75)
        fill_neg_color = QColor(220, 53, 69)
        pen_color = QColor(40, 40, 40)

        painter.fillRect(rect, bg_color)

        mid_x = rect.width() // 2
        height = rect.height()

        if self.value < 0:
            pct = abs(self.value) / 1000.0
            w = int(pct * mid_x)
            fill_rect = QtCore.QRect(mid_x - w, 0, w, height)
            painter.fillRect(fill_rect, fill_neg_color)
        elif self.value > 0:
            pct = self.value / 1000.0
            w = int(pct * mid_x)
            fill_rect = QtCore.QRect(mid_x, 0, w, height)
            painter.fillRect(fill_rect, fill_pos_color)

        pen = QPen(pen_color)
        pen.setWidth(1)
        painter.setPen(pen)
        painter.drawLine(mid_x, 0, mid_x, height)

        painter.setPen(pen)
        painter.drawText(rect, Qt.AlignCenter, str(self.value))


class TcpClientThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(dict, float)
    disconnected = QtCore.pyqtSignal()

    def __init__(self, ip, port=1234, parent=None):
        super().__init__(parent)
        self.ip = ip
        self.port = port
        self.running = True
        self.sock = None

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ip, self.port))
            self.sock.settimeout(1.0)
            buffer = ""

            while self.running:
                try:
                    data = self.sock.recv(4096).decode(errors="ignore")
                    if not data:
                        break

                    buffer += data

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()
                        if not line:
                            continue
                        try:
                            try:
                                obj = json.loads(line)
                            except json.JSONDecodeError:
                                alt = line.replace("'", '"')
                                obj = json.loads(alt)
                            ts = time.time()
                            self.data_received.emit(obj, ts)
                        except Exception:
                            pass

                except socket.timeout:
                    continue
                except Exception:
                    break

        except Exception:
            pass

        try:
            if self.sock:
                self.sock.close()
        except Exception:
            pass

        self.disconnected.emit()

    def stop(self):
        self.running = False
        try:
            if self.sock:
                self.sock.shutdown(socket.SHUT_RDWR)
                self.sock.close()
        except Exception:
            pass
        self.wait(2000)


class LanScannerDialog(QtWidgets.QDialog):
    def __init__(self, parent=None, base_subnet="192.168.1."):
        super().__init__(parent)
        self.setWindowTitle("LAN Scanner")
        self.setMinimumWidth(400)

        self.base_subnet = base_subnet
        self.selected_ip = None
        self.scan_thread = None

        layout = QtWidgets.QVBoxLayout(self)

        self.status_label = QtWidgets.QLabel("Click 'Scan Network' to start")
        layout.addWidget(self.status_label)

        self.list = QtWidgets.QListWidget()
        layout.addWidget(self.list)

        self.scan_button = QtWidgets.QPushButton("Scan Network")
        layout.addWidget(self.scan_button)

        buttons = QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel
        self.button_box = QtWidgets.QDialogButtonBox(buttons)
        layout.addWidget(self.button_box)

        self.scan_button.clicked.connect(self.start_scan)
        self.button_box.accepted.connect(self.on_accept)
        self.button_box.rejected.connect(self.reject)

    def start_scan(self):
        self.list.clear()
        self.status_label.setText("Scanning...")

        self.scan_thread = LanScanThread(self.base_subnet)
        self.scan_thread.device_found.connect(lambda text: self.list.addItem(text))
        self.scan_thread.progress.connect(lambda i: self.status_label.setText(f"Scanning {self.base_subnet}{i}..."))
        self.scan_thread.finished.connect(lambda: self.status_label.setText("Scan complete"))
        self.scan_thread.start()

    def on_accept(self):
        item = self.list.currentItem()
        if item:
            self.selected_ip = item.text().split()[0]
        if self.scan_thread:
            self.scan_thread.stop()
        self.accept()


class LanScanThread(QThread):
    device_found = pyqtSignal(str)
    progress = pyqtSignal(int)
    finished = pyqtSignal()

    def __init__(self, base_subnet):
        super().__init__()
        self.base_subnet = base_subnet
        self._is_running = True

    def run(self):
        for i in range(1, 255):
            if not self._is_running:
                break

            ip = f"{self.base_subnet}{i}"
            self.progress.emit(i)

            try:
                result = subprocess.call(
                    ["ping", "-c", "1", "-W", "50", ip] if not sys.platform.startswith("win")
                    else ["ping", "-n", "1", "-w", "50", ip],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            except:
                result = 1

            if result == 0:
                try:
                    hostname = socket.gethostbyaddr(ip)[0]
                except:
                    hostname = "Unknown"

                mac = self.get_mac(ip)
                text = f"{ip}\t{hostname}\t{mac}"
                self.device_found.emit(text)

        self.finished.emit()

    def stop(self):
        self._is_running = False

    def get_mac(self, ip):
        try:
            if platform.system() == "Windows":
                output = subprocess.check_output("arp -a", shell=True).decode()
                match = re.search(rf"{re.escape(ip)}\s+([-\w\.]+)", output)
                if match:
                    return match.group(1)

            elif platform.system() == "Linux":
                output = subprocess.check_output(["arp", "-n", ip]).decode()
                match = re.search(r"(([0-9a-f]{2}:){5}[0-9a-f]{2})", output, re.I)
                if match:
                    return match.group(1)

            elif platform.system() == "Darwin":
                output = subprocess.check_output(["arp", ip]).decode()
                match = re.search(r"(([0-9a-f]{2}:){5}[0-9a-f]{2})", output, re.I)
                if match:
                    return match.group(1)

        except:
            pass
        return "Unknown"


class ESP32Monitor(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Line Follower Dashboard (TCP)")
        self.resize(1400, 900)

        self.tabs = QtWidgets.QTabWidget()
        self.setCentralWidget(self.tabs)

        self.dashboard_tab = QtWidgets.QWidget()
        self.tabs.addTab(self.dashboard_tab, "Dashboard")

        self.raw_tab = QtWidgets.QWidget()
        self.tabs.addTab(self.raw_tab, "Raw JSON")

        self._build_dashboard_ui()
        self._build_raw_ui()

        self.tcp_thread = None
        self.connected = False
        self.history_graph = []

    def _build_dashboard_ui(self):
        layout = QtWidgets.QVBoxLayout(self.dashboard_tab)

        # --- Top connection row ---
        top_row = QtWidgets.QHBoxLayout()
        top_row.addWidget(QtWidgets.QLabel("ESP32 Host/IP:"))
        self.host_input = QtWidgets.QLineEdit("44-1d-64-e3-da-60")
        self.host_input.setMinimumWidth(200)
        top_row.addWidget(self.host_input)

        top_row.addWidget(QtWidgets.QLabel("TCP Port:"))
        self.port_spin = QtWidgets.QSpinBox()
        self.port_spin.setRange(1, 65535)
        self.port_spin.setValue(1234)
        top_row.addWidget(self.port_spin)

        self.explore_button = QtWidgets.QPushButton("Explore")
        top_row.addWidget(self.explore_button)
        self.mac_button = QtWidgets.QPushButton("Find by MAC")
        top_row.addWidget(self.mac_button)

        self.connect_button = QtWidgets.QPushButton("Connect TCP")
        top_row.addWidget(self.connect_button)
        self.disconnect_button = QtWidgets.QPushButton("Disconnect TCP")
        self.disconnect_button.setEnabled(False)
        top_row.addWidget(self.disconnect_button)

        top_row.addStretch()
        layout.addLayout(top_row)

        self.explore_button.clicked.connect(self.explore_network)
        self.mac_button.clicked.connect(self.MAC_TO_IP)
        self.connect_button.clicked.connect(self.connect_tcp)
        self.disconnect_button.clicked.connect(self.disconnect_tcp)

        # --- Command buttons (HTTP) ---
        cmd_row = QtWidgets.QHBoxLayout()
        self.start_btn = QtWidgets.QPushButton("Start")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.enable_btn = QtWidgets.QPushButton("Enable motors")
        self.disable_btn = QtWidgets.QPushButton("Disable motors")
        self.calib_white_btn = QtWidgets.QPushButton("Calibrate White")
        self.calib_black_btn = QtWidgets.QPushButton("Calibrate Black")
        self.calib_reset_btn = QtWidgets.QPushButton("reset Calib")

        cmd_row.addWidget(self.start_btn)
        cmd_row.addWidget(self.stop_btn)
        # Add new enable/disable next to start/stop
        cmd_row.addWidget(self.enable_btn)
        cmd_row.addWidget(self.disable_btn)

        cmd_row.addWidget(self.calib_white_btn)
        cmd_row.addWidget(self.calib_black_btn)
        cmd_row.addWidget(self.calib_reset_btn)
        cmd_row.addStretch()
        layout.addLayout(cmd_row)

        self.start_btn.clicked.connect(lambda: self.send_http_command("/start"))
        self.stop_btn.clicked.connect(lambda: self.send_http_command("/stop"))
        self.enable_btn.clicked.connect(lambda: self.send_http_command("/enable"))
        self.disable_btn.clicked.connect(lambda: self.send_http_command("/disable"))
        self.calib_white_btn.clicked.connect(lambda: self.send_http_command("/calibrate/white"))
        self.calib_black_btn.clicked.connect(lambda: self.send_http_command("/calibrate/black"))
        self.calib_reset_btn.clicked.connect(lambda: self.send_http_command("/calibrate/reset"))

        # --- Parameters & controls (kp, ki, kd, cycleTime, diff, defaultSpeed,
        # added: crossingsPerLap, maxTimeWithoutLine) ---
        params_group = QtWidgets.QGroupBox("ESP32 Parameters")
        params_layout = QtWidgets.QHBoxLayout(params_group)
        left_col = QtWidgets.QVBoxLayout()
        right_col = QtWidgets.QVBoxLayout()
        params_layout.addLayout(left_col)
        params_layout.addLayout(right_col)

        self.param_labels = {}
        self.param_inputs = {}
        self.param_set_buttons = {}

        def add_param(col_layout, name):
            row = QtWidgets.QHBoxLayout()
            lbl = QtWidgets.QLabel("N/A")
            lbl.setFixedWidth(120)
            inp = QtWidgets.QLineEdit()
            inp.setFixedWidth(120)
            btn = QtWidgets.QPushButton("Set")
            btn.setFixedWidth(60)
            row.addWidget(QtWidgets.QLabel(f"{name}:"))
            row.addWidget(lbl)
            row.addWidget(inp)
            row.addWidget(btn)
            col_layout.addLayout(row)
            self.param_labels[name] = lbl
            self.param_inputs[name] = inp
            self.param_set_buttons[name] = btn
            btn.clicked.connect(lambda _, n=name: self.send_set_command(n))

        # left column: PID + new crossingsPerLap
        for n in ("kp", "ki", "kd", "crossingsPerLap"):
            add_param(left_col, n)

        # right column: timing + speeds + new maxTimeWithoutLine
        for n in ("cycleTime", "diff", "defaultSpeed", "maxTimeWithoutLine"):
            add_param(right_col, n)

        layout.addWidget(params_group)

        # --- Running & Calc times ---
        status_row = QtWidgets.QHBoxLayout()
        status_row.addWidget(QtWidgets.QLabel("Running:"))
        self.running_label = QtWidgets.QLabel("N/A")
        status_row.addWidget(self.running_label)
        status_row.addSpacing(20)
        status_row.addWidget(QtWidgets.QLabel("Motors enabled:"))
        self.enabled_label = QtWidgets.QLabel("N/A")
        status_row.addWidget(self.enabled_label)
        status_row.addSpacing(20)
        status_row.addWidget(QtWidgets.QLabel("Lap time:"))
        self.laptime_label = QtWidgets.QLabel("N/A")
        status_row.addWidget(self.laptime_label)
        status_row.addSpacing(20)
        status_row.addWidget(QtWidgets.QLabel("Crossing count:"))
        self.crossing_label = QtWidgets.QLabel("N/A")
        status_row.addWidget(self.crossing_label)
        status_row.addSpacing(20)
        status_row.addWidget(QtWidgets.QLabel("calculation time:"))
        self.calc_label = QtWidgets.QLabel("N/A")
        status_row.addWidget(self.calc_label)
        status_row.addStretch()
        layout.addLayout(status_row)

        # --- Calibration white/black labels ---
        calib_vals_row = QtWidgets.QHBoxLayout()
        self.white_label = QtWidgets.QLabel("white=N/A")
        self.black_label = QtWidgets.QLabel("black=N/A")
        calib_vals_row.addWidget(self.white_label)
        calib_vals_row.addWidget(self.black_label)
        calib_vals_row.addStretch()
        layout.addLayout(calib_vals_row)

        # --- Mapped Sensors (8 vertical bars) ---
        sensors_group = QtWidgets.QGroupBox("Mapped Sensors")
        sens_layout = QtWidgets.QHBoxLayout(sensors_group)
        self.sensor_bars = []
        for i in range(8):
            bar = QtWidgets.QProgressBar()
            bar.setOrientation(Qt.Vertical)
            bar.setRange(0, 1000)
            bar.setFormat(f"S{i}\n%p%")
            bar.setFixedHeight(160)
            sens_layout.addWidget(bar)
            self.sensor_bars.append(bar)
        layout.addWidget(sensors_group)

        # --- line position and error ---
        error_row = QtWidgets.QHBoxLayout()
        error_row.addWidget(QtWidgets.QLabel("Line position:"))
        self.position_label = QtWidgets.QLabel("N/A")
        error_row.addWidget(self.position_label)
        error_row.addSpacing(20)
        error_row.addWidget(QtWidgets.QLabel("Error:"))
        self.error_label = QtWidgets.QLabel("N/A")
        error_row.addWidget(self.error_label)
        error_row.addSpacing(20)
        error_row.addWidget(QtWidgets.QLabel("Correction:"))
        self.correction_label = QtWidgets.QLabel("N/A")
        error_row.addWidget(self.correction_label)
        error_row.addStretch()
        layout.addLayout(error_row)

        # --- Motors (two rows: motor1 + motor2) ---
        motors_group = QtWidgets.QGroupBox("Motors")
        motor_layout = QtWidgets.QVBoxLayout(motors_group)

        self.motor1_widget = MotorBar()
        self.motor2_widget = MotorBar()

        motor_layout.addWidget(QtWidgets.QLabel("Motor 1"))
        motor_layout.addWidget(self.motor1_widget)

        motor_layout.addWidget(QtWidgets.QLabel("Motor 2"))
        motor_layout.addWidget(self.motor2_widget)

        layout.addWidget(motors_group)

        # --- Graph: error & correction (and optional motor overlays) ---
        graph_row = QtWidgets.QHBoxLayout()
        graph_row.addWidget(QtWidgets.QLabel("Graph window (s):"))
        self.graph_window = QtWidgets.QDoubleSpinBox()
        self.graph_window.setRange(1.0, 120.0)
        self.graph_window.setValue(15.0)
        self.graph_window.setDecimals(1)
        graph_row.addWidget(self.graph_window)
        graph_row.addStretch()
        layout.addLayout(graph_row)

        self.graph_widget = pg.PlotWidget(title="Error / Correction Over Time")
        self.graph_widget.showGrid(x=True, y=True)
        self.graph_widget.addLegend()
        self.graph_widget.setLabel("left", "Value")
        self.graph_widget.setLabel("bottom", "Time (s)")
        self.error_curve = self.graph_widget.plot(pen="r", name="error")
        #self.corr_curve = self.graph_widget.plot(pen="g", name="correction")
        layout.addWidget(self.graph_widget)

        # status bar
        self.status = self.statusBar()
        self.status.showMessage("Ready")

    # -----------------------
    # Incoming data handler
    # -----------------------
    def on_tcp_data(self, obj: dict, ts: float):
        """
        Update UI elements using incoming JSON.
        Expected JSON fields (examples):
          timeStamp, ssid, cycleTime, calculationTime, running,
          kp, ki, kd, linePosition, error, correction,
          motor1, motor2, defaultSpeed, diff,
          sensors (list of 8), blackValues (list), whiteValues (list),
          crossingsPerLap, maxTimeWithoutLine
        """
        # Update raw JSON tab (pretty-printed)
        try:
            pretty = json.dumps(obj, indent=2, ensure_ascii=False)
        except Exception:
            pretty = str(obj)
        self.raw_text.setPlainText(pretty)

        # log data
        try:
            with open("log.txt", "a", encoding="utf-8") as f:
                timestamp = datetime.datetime.now().isoformat()
                f.write(f"[{timestamp}] {str(obj)}\n")
        except Exception as e:
            print(f"Logging error: {e}")

        # Update parameter labels (including new parameters)
        for key in ("kp", "ki", "kd", "cycleTime", "defaultSpeed", "diff",
                    "crossingsPerLap", "maxTimeWithoutLine"):
            val = obj.get(key, "N/A")
            if key in self.param_labels:
                self.param_labels[key].setText(str(val))

        # running & calc time
        self.running_label.setText(str(obj.get("running", "N/A")))
        self.enabled_label.setText(str(obj.get("motorsEnabled", "N/A")))
        self.laptime_label.setText(str(obj.get("lapTime", "N/A")))
        self.crossing_label.setText(str(obj.get("crossingCount", "N/A")))
        self.calc_label.setText(str(obj.get("calculationTime", "N/A")))

        # white/black arrays (show as short text)
        w = obj.get("whiteValues", [])
        b = obj.get("blackValues", [])
        self.white_label.setText(f"white={w}")
        self.black_label.setText(f"black={b}")

        # sensors -> scale to 0..1000 for progress bars
        sensors = obj.get("sensors", [0] * len(self.sensor_bars))
        try:
            max_in = max(sensors) if sensors else 0
            scale = 1.0
            if max_in > 1000:
                scale = 1000.0 / float(max_in)
        except Exception:
            scale = 1.0

        for i, bar in enumerate(self.sensor_bars):
            try:
                val = int(round(sensors[i] * scale))
            except Exception:
                val = 0
            val = max(0, min(1000, val))
            bar.setValue(val)

        # line position and error
        self.position_label.setText(str(obj.get("linePosition", "N/A")))
        self.error_label.setText(str(obj.get("error", "N/A")))
        self.correction_label.setText(str(obj.get("correction", "N/A")))

        # Motors
        self.motor1_widget.setValue(obj.get("motor1", 0))
        self.motor2_widget.setValue(obj.get("motor2", 0))

        # graph: store error and correction
        now = time.time()
        self.history_graph.append({
            "time": now,
            "error": obj.get("error", 0) or 0,
            "correction": obj.get("correction", 0) or 0
        })
        # trim based on graph_window
        window = float(self.graph_window.value())
        cutoff = now - window
        self.history_graph = [h for h in self.history_graph if h["time"] >= cutoff]
        if self.history_graph:
            t0 = self.history_graph[0]["time"]
            times = [h["time"] - t0 for h in self.history_graph]
            errors = [h["error"] for h in self.history_graph]
            corrs = [h["correction"] for h in self.history_graph]
            self.error_curve.setData(times, errors)
            #self.corr_curve.setData(times, corrs)
            vals = errors + corrs
            if vals:
                ymax = max(10, max(abs(v) for v in vals))
                self.graph_widget.setYRange(-ymax * 1.1, ymax * 1.1)

        # Update status bar with brief info
        global last_timestamp
        current_ts = int(obj.get("timeStamp", 0))
        ts_readable2 = 0 if last_timestamp in (None, 0) else current_ts - last_timestamp
        last_timestamp = current_ts

        ts_readable = time.strftime("%H:%M:%S", time.localtime(ts))
        self.status.showMessage(f"Last update: {ts_readable}; time between updates: {ts_readable2} ms")


    # -----------------------
    # HTTP commands (start/stop/set/calibrate/enable/disable)
    # -----------------------
    def send_http_command(self, endpoint: str):
        host = self.host_input.text().strip()
        if not host:
            QtWidgets.QMessageBox.warning(self, "Error", "Enter host/IP to send commands.")
            return
        # resolve hostname to IP for HTTP if possible
        try:
            ip = socket.gethostbyname(host)
        except Exception:
            ip = host
        url = f"http://{ip}{endpoint}"
        try:
            requests.get(url, timeout=2.0)
            self.status.showMessage(f"Sent: {endpoint}")
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Command Error", f"Failed to send {endpoint}:\n{e}")

    def send_set_command(self, param_name: str):
        host = self.host_input.text().strip()
        if not host:
            QtWidgets.QMessageBox.warning(self, "Error", "Enter host/IP to send commands.")
            return
        val = self.param_inputs[param_name].text().strip()
        if not val:
            QtWidgets.QMessageBox.warning(self, "Error", f"No value entered for {param_name}.")
            return

        # Use UI param names directly (we're using crossingsPerLap / maxTimeWithoutLine)
        endpoint = f"/set?{param_name}={val}"
        self.send_http_command(endpoint)


    # -----------------------
    # Explorer (basic LAN check)
    # -----------------------
    def explore_network(self):
        base = self.host_input.text().strip()

        # If no base provided, detect automatically
        if not base or not base.endswith("."):
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                s.connect(("8.8.8.8", 80))
                ip = s.getsockname()[0]
                s.close()

                parts = ip.split(".")
                base = f"{parts[0]}.{parts[1]}.{parts[2]}."
            except:
                return None
            if not base:
                self.status.showMessage("Could not detect your LAN subnet")
                return
            self.host_input.setText(base)

        dlg = LanScannerDialog(self, base_subnet=base)
        dlg.setModal(True)
        result = dlg.exec_()

        if result == QtWidgets.QDialog.Accepted and dlg.selected_ip:
            self.host_input.setText(dlg.selected_ip)
            self.status.showMessage(f"Selected: {dlg.selected_ip}")
        else:
            self.status.showMessage("No selection made or cancelled")


    def MAC_TO_IP(self):
        """
        Scan the LAN for a specific MAC address and update the input field with the IP.
        """
        mac_input = self.host_input.text().strip()
        if not mac_input:
            self.status.showMessage("Enter a MAC address to search")
            return

        self.status.showMessage(f"Searching for {mac_input}...")
        QtWidgets.QApplication.processEvents()

        target_mac = mac_input.lower()
        # Normalize MAC: replace '-' or '.' with ':'
        target_mac = ''.join(':' if c in "-." else c for c in target_mac)

        # Auto-detect subnet
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        except:
            self.status.showMessage("Unable to detect local subnet")
            return
        finally:
            s.close()
        parts = ip.split(".")
        base_subnet = f"{parts[0]}.{parts[1]}.{parts[2]}."

        system_platform = platform.system()

        found_ip = None
        for i in range(1, 255):
            ip = f"{base_subnet}{i}"

            # Ping to populate ARP table
            ping_cmd = ["ping", "-n", "1", "-w", "50", ip] if system_platform.startswith("Windows") else ["ping", "-c", "1", "-W", "1", ip]
            subprocess.run(ping_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

            # Read MAC from ARP table
            mac = None
            try:
                if system_platform.startswith("Windows"):
                    output = subprocess.run("arp -a", shell=True, capture_output=True, text=True).stdout
                    match = re.search(rf"{re.escape(ip)}\s+([-\w\.]+)", output)
                    if match:
                        mac = match.group(1)
                elif system_platform.startswith("Linux"):
                    output = subprocess.run(["arp", "-n", ip], capture_output=True, text=True).stdout
                    match = re.search(r"(([0-9a-f]{2}:){5}[0-9a-f]{2})", output, re.I)
                    if match:
                        mac = match.group(1)
                elif system_platform.startswith("Darwin"):
                    output = subprocess.run(["arp", ip], capture_output=True, text=True).stdout
                    match = re.search(r"(([0-9a-f]{2}:){5}[0-9a-f]{2})", output, re.I)
                    if match:
                        mac = match.group(1)
            except:
                continue

            if mac and mac.lower() == target_mac:
                found_ip = ip
                break

        if found_ip:
            self.host_input.setText(found_ip)
            self.status.showMessage(f"Found {found_ip} for MAC {mac_input}")
        else:
            self.status.showMessage(f"No device found with MAC {mac_input}")

    # -----------------------
    # TCP Connect / Disconnect
    # -----------------------
    def connect_tcp(self):
        if self.connected:
            return

        host = self.host_input.text().strip()
        if not host:
            QtWidgets.QMessageBox.warning(self, "Error", "Enter host/IP to connect.")
            return
        try:
            ip = socket.gethostbyname(host)
        except Exception:
            ip = host

        port = self.port_spin.value()
        self.status.showMessage(f"Connecting to {ip}:{port} ...")

        self.tcp_thread = TcpClientThread(ip, port)
        self.tcp_thread.data_received.connect(self.on_tcp_data)
        self.tcp_thread.disconnected.connect(self.on_tcp_disconnected)
        self.tcp_thread.start()

        self.connected = True
        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)

    def disconnect_tcp(self):
        if self.tcp_thread:
            self.tcp_thread.stop()
            self.tcp_thread = None

        self.connected = False
        self.status.showMessage("TCP disconnected")
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)

    def on_tcp_disconnected(self):
        self.connected = False
        self.status.showMessage("TCP connection closed")
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)


    # -----------------------
    # Raw JSON Tab
    # -----------------------
    def _build_raw_ui(self):
        layout = QtWidgets.QVBoxLayout(self.raw_tab)

        self.raw_text = QtWidgets.QPlainTextEdit()
        self.raw_text.setReadOnly(True)
        layout.addWidget(self.raw_text)

        btn_row = QtWidgets.QHBoxLayout()
        save_btn = QtWidgets.QPushButton("Save Raw JSON to File")
        clear_btn = QtWidgets.QPushButton("Clear Raw")
        btn_row.addWidget(save_btn)
        btn_row.addWidget(clear_btn)
        btn_row.addStretch()
        layout.addLayout(btn_row)

        save_btn.clicked.connect(self.save_raw_json)
        clear_btn.clicked.connect(lambda: self.raw_text.clear())

    def save_raw_json(self):
        text = self.raw_text.toPlainText()
        if not text.strip():
            QtWidgets.QMessageBox.information(self, "Empty", "Raw JSON is empty.")
            return
        fn, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save Raw JSON", "raw.json", "JSON Files (*.json)")
        if fn:
            try:
                with open(fn, "w", encoding="utf-8") as f:
                    f.write(text)
                self.status.showMessage(f"Saved raw JSON to {fn}")
            except Exception as e:
                QtWidgets.QMessageBox.warning(self, "Error", f"Could not save file:\n{e}")


    # -----------------------
    # Window close cleanup
    # -----------------------
    def closeEvent(self, event):
        if self.tcp_thread:
            self.tcp_thread.stop()
        event.accept()


# -----------------------
# Main entry point
# -----------------------
def main():
    app = QtWidgets.QApplication(sys.argv)
    w = ESP32Monitor()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
