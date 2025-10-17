import sys
import requests
import socket
import json
import re
import time
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton,
    QLabel, QProgressBar, QGroupBox, QTableWidget, QTableWidgetItem,
    QDoubleSpinBox, QScrollArea, QSpinBox, QSizePolicy, QMessageBox
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg


class ESP32Monitor(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Line Follower Dashboard")
        self.resize(1600, 1000)

        # === Scrollable Layout ===
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.main_widget = QWidget()
        self.layout = QVBoxLayout(self.main_widget)
        scroll.setWidget(self.main_widget)
        main_layout = QVBoxLayout()
        main_layout.addWidget(scroll)
        self.setLayout(main_layout)

        # === Connection ===
        conn_layout = QHBoxLayout()
        conn_layout.addWidget(QLabel("ESP32 Host:"))
        self.ip_input = QLineEdit("linefollower.local")
        conn_layout.addWidget(self.ip_input)

        self.connect_button = QPushButton("Connect")
        conn_layout.addWidget(self.connect_button)

        conn_layout.addWidget(QLabel("Polling (ms):"))
        self.poll_spin = QSpinBox()
        self.poll_spin.setRange(100, 5000)
        self.poll_spin.setValue(500)
        conn_layout.addWidget(self.poll_spin)
        self.layout.addLayout(conn_layout)

        self.connect_button.clicked.connect(self.toggle_connection)

        # === Parameter Layout ===
        self.info_group = QGroupBox("ESP32 Parameters & Controls")
        info_layout = QHBoxLayout()
        self.info_group.setLayout(info_layout)
        self.layout.addWidget(self.info_group)

        left_col = QVBoxLayout()
        right_col = QVBoxLayout()
        info_layout.addLayout(left_col)
        info_layout.addLayout(right_col)

        self.param_fields = {}
        self.param_inputs = {}
        self.param_buttons = {}

        def add_param(layout, name):
            row = QHBoxLayout()
            val_label = QLabel("N/A")
            val_label.setFixedWidth(60)
            input_box = QLineEdit()
            input_box.setFixedWidth(80)
            set_btn = QPushButton("Set")
            set_btn.setFixedWidth(50)
            set_btn.clicked.connect(lambda _, n=name: self.send_set_command(n))
            row.addWidget(QLabel(f"{name}:"))
            row.addWidget(val_label)
            row.addWidget(input_box)
            row.addWidget(set_btn)
            container = QWidget()
            container.setLayout(row)
            layout.addWidget(container)
            self.param_fields[name] = val_label
            self.param_inputs[name] = input_box
            self.param_buttons[name] = set_btn

        # Left column: PID
        for f in ["kp", "ki", "kd"]:
            add_param(left_col, f)
        # Right column: Timing & speed
        for f in ["cycleTime", "maxSpeed", "defaultSpeed"]:
            add_param(right_col, f)

        # === Running control ===
        run_row = QHBoxLayout()
        run_row.addWidget(QLabel("Running:"))
        self.running_label = QLabel("N/A")
        run_row.addWidget(self.running_label)
        self.run_btn = QPushButton("Start/Stop")
        self.run_btn.clicked.connect(self.toggle_running)
        run_row.addWidget(self.run_btn)
        self.layout.addLayout(run_row)

        # === Calibration ===
        calib_row = QHBoxLayout()
        calib_row.addWidget(QLabel("Calibration:"))
        self.white_label = QLabel("white=N/A")
        self.black_label = QLabel("black=N/A")
        calib_row.addWidget(self.white_label)
        calib_row.addWidget(self.black_label)
        self.calib_white_btn = QPushButton("Calibrate White")
        self.calib_black_btn = QPushButton("Calibrate Black")
        self.reset_calib_btn = QPushButton("Reset Calibration")
        self.calib_white_btn.clicked.connect(lambda: self.send_simple_command("/calibrate/white"))
        self.calib_black_btn.clicked.connect(lambda: self.send_simple_command("/calibrate/black"))
        self.reset_calib_btn.clicked.connect(lambda: self.send_simple_command("/calibrate/reset"))
        calib_row.addWidget(self.calib_white_btn)
        calib_row.addWidget(self.calib_black_btn)
        calib_row.addWidget(self.reset_calib_btn)
        self.layout.addLayout(calib_row)

        # === Mapped Sensors ===
        self.mapped_group = QGroupBox("Mapped Sensors")
        mapped_layout = QHBoxLayout()
        self.mapped_group.setLayout(mapped_layout)
        self.mapped_bars = []
        for i in range(8):
            bar = QProgressBar()
            bar.setOrientation(Qt.Vertical)
            bar.setRange(0, 100)
            bar.setFormat(f"S{i}")
            bar.setFixedHeight(150)
            mapped_layout.addWidget(bar)
            self.mapped_bars.append(bar)
        self.layout.addWidget(self.mapped_group)

        # === Motors ===
        self.motor_group = QGroupBox("Motors")
        motor_layout = QVBoxLayout()
        self.motor_group.setLayout(motor_layout)
        self.m1_bar = QProgressBar()
        self.m2_bar = QProgressBar()
        for bar, label in [(self.m1_bar, "M1"), (self.m2_bar, "M2")]:
            bar.setRange(0, 100)
            bar.setFormat(f"{label}: %p%")
            motor_layout.addWidget(bar)
        self.layout.addWidget(self.motor_group)

        # === Graph ===
        graph_ctrl = QHBoxLayout()
        graph_ctrl.addWidget(QLabel("Graph window (seconds):"))
        self.graph_window_time = QDoubleSpinBox()
        self.graph_window_time.setRange(1.0, 120.0)
        self.graph_window_time.setValue(15.0)
        self.graph_window_time.setDecimals(1)
        graph_ctrl.addWidget(self.graph_window_time)
        self.layout.addLayout(graph_ctrl)

        self.graph_widget = pg.PlotWidget(title="Error / Correction Over Time")
        self.graph_widget.showGrid(x=True, y=True)
        self.graph_widget.addLegend()
        self.graph_widget.setLabel("left", "Value")
        self.graph_widget.setLabel("bottom", "Time (s)")
        self.graph_widget.setMinimumHeight(400)
        self.error_curve = self.graph_widget.plot(pen="r", name="error")
        self.correction_curve = self.graph_widget.plot(pen="g", name="correction")
        self.layout.addWidget(self.graph_widget)

        # === Table ===
        self.table = QTableWidget()
        self.table.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.table.setAlternatingRowColors(True)
        self.table.setMinimumHeight(400)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.layout.addWidget(self.table)

        # === Timer ===
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.esp_host = None
        self.esp_ip = None
        self.connected = False
        self.history_graph = []
        self.history_table = []

    # === CONNECTION ===
    def toggle_connection(self):
        if self.connected:
            self.disconnect_esp()
        else:
            self.connect_esp()

    def connect_esp(self):
        self.esp_host = self.ip_input.text().strip()
        interval = self.poll_spin.value()
        try:
            self.esp_ip = socket.gethostbyname(self.esp_host)
            self.timer.start(interval)
            self.connected = True
            self.connect_button.setText("Disconnect")
            print(f"Connected to {self.esp_ip}")
        except Exception as e:
            QMessageBox.warning(self, "Connection Error", str(e))

    def disconnect_esp(self):
        self.timer.stop()
        self.connected = False
        self.connect_button.setText("Connect")
        print("Disconnected.")

    # === POLLING ===
    def update_data(self):
        if not self.connected or not self.esp_ip:
            return
        try:
            url = f"http://{self.esp_ip}/debug"
            response = requests.get(url, timeout=1)
            text = re.sub(r"\b(nan|NaN)\b", "null", response.text)
            data = json.loads(text)
        except Exception as e:
            print("Error connecting:", e)
            return

        # Update labels
        for field, lbl in self.param_fields.items():
            lbl.setText(str(data.get(field, "N/A")))
        self.running_label.setText(str(data.get("running", "N/A")))
        self.white_label.setText(f"white={data.get('whiteValues', [])}")
        self.black_label.setText(f"black={data.get('blackValues', [])}")

        # Update sensors and motors
        mapped = data.get("mappedSensors", [0]*8)
        for i, v in enumerate(mapped):
            self.mapped_bars[i].setValue(v)
        self.m1_bar.setValue(max(0, min(100, int(data.get("m1", 0)))))
        self.m2_bar.setValue(max(0, min(100, int(data.get("m2", 0)))))

        # Update graph
        now = time.time()
        self.history_graph.append({
            "time": now,
            "error": data.get("error", 0),
            "correction": data.get("correction", 0)
        })
        window = self.graph_window_time.value()
        cutoff = now - window
        self.history_graph = [h for h in self.history_graph if h["time"] >= cutoff]
        if self.history_graph:
            t0 = self.history_graph[0]["time"]
            times = [h["time"] - t0 for h in self.history_graph]
            self.error_curve.setData(times, [h["error"] for h in self.history_graph])
            self.correction_curve.setData(times, [h["correction"] for h in self.history_graph])
            vals = [h["error"] for h in self.history_graph] + [h["correction"] for h in self.history_graph]
            if vals:
                ymax = max(100, abs(max(vals, key=abs)))
                self.graph_widget.setYRange(-ymax, ymax)

        # Update table (full history)
        exclude = {"ssid", "ip", "calculationTime", "sensors", "whiteValues", "blackValues"}
        flat = {}
        for k, v in data.items():
            if k in exclude:
                continue
            if isinstance(v, list):
                for i, val in enumerate(v):
                    flat[f"{k}[{i}]"] = val
            else:
                flat[k] = v

        self.history_table.insert(0, flat)
        if len(self.history_table) > 100:
            self.history_table.pop()

        headers = list(flat.keys())
        self.table.setColumnCount(len(headers))
        self.table.setHorizontalHeaderLabels(headers)
        self.table.setRowCount(len(self.history_table))
        for row_idx, row_data in enumerate(self.history_table):
            for col_idx, key in enumerate(headers):
                self.table.setItem(row_idx, col_idx, QTableWidgetItem(str(row_data.get(key, ""))))

    # === COMMANDS ===
    def send_simple_command(self, endpoint):
        if not self.connected:
            QMessageBox.warning(self, "Error", "Not connected to ESP32.")
            return
        try:
            url = f"http://{self.esp_ip}{endpoint}"
            requests.get(url, timeout=1)
            print("Sent:", url)
        except Exception as e:
            QMessageBox.warning(self, "Command Error", str(e))

    def send_set_command(self, param):
        if not self.connected:
            QMessageBox.warning(self, "Error", "Not connected to ESP32.")
            return
        val = self.param_inputs[param].text().strip()
        if not val:
            QMessageBox.warning(self, "Error", f"No value entered for {param}.")
            return
        endpoint = f"/set?{param}={val}"
        self.send_simple_command(endpoint)

    def toggle_running(self):
        if self.running_label.text() == "True":
            self.send_simple_command("/stop")
        else:
            self.send_simple_command("/start")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ESP32Monitor()
    window.show()
    sys.exit(app.exec_())
