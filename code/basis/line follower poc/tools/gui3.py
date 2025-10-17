import sys
import requests
import socket
import json
import re
import time
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QPushButton,
    QLabel, QProgressBar, QGroupBox, QTableWidget, QTableWidgetItem,
    QDoubleSpinBox, QSpinBox, QSizePolicy, QScrollArea, QMessageBox
)
from PyQt5.QtCore import QTimer, Qt
import pyqtgraph as pg


# ---------------- ESP Connection ----------------
class ESPConnection(QtCore.QObject):
    data_received = QtCore.pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.esp_host = None
        self.esp_ip = None
        self.connected = False
        self.timer = QTimer()
        self.timer.timeout.connect(self.poll_data)

    def connect_esp(self, host, interval_ms=500):
        self.esp_host = host.strip()
        try:
            self.esp_ip = socket.gethostbyname(self.esp_host)
            self.connected = True
            self.timer.start(interval_ms)
            return True
        except Exception as e:
            self.connected = False
            return False, str(e)

    def disconnect_esp(self):
        self.timer.stop()
        self.connected = False

    def poll_data(self):
        if not self.connected or not self.esp_ip:
            return
        try:
            url = f"http://{self.esp_ip}/debug"
            response = requests.get(url, timeout=1)
            text = re.sub(r"\b(nan|NaN)\b", "null", response.text)
            data = json.loads(text)
            self.data_received.emit(data)
        except:
            pass

    def send_command(self, endpoint):
        if not self.connected:
            return
        try:
            url = f"http://{self.esp_ip}{endpoint}"
            requests.get(url, timeout=1)
        except:
            pass

    def send_set(self, param, val):
        self.send_command(f"/set?{param}={val}")


# ---------------- Control & Monitor Tab ----------------
class ControlTab(QtWidgets.QWidget):
    def __init__(self, esp: ESPConnection):
        super().__init__()
        self.esp = esp
        self.esp.data_received.connect(self.on_data)
        self.history_graph = []
        self.history_table = []

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        # --- Scroll area ---
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        self.main_widget = QWidget()
        self.main_layout = QVBoxLayout(self.main_widget)
        scroll.setWidget(self.main_widget)
        layout.addWidget(scroll)

        # --- Connection ---
        conn_group = QGroupBox("Connection")
        conn_layout = QHBoxLayout(conn_group)
        self.ip_input = QLineEdit("linefollower.local")
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_connection)
        self.poll_spin = QSpinBox(); self.poll_spin.setRange(100, 5000); self.poll_spin.setValue(500)
        self.conn_status_label = QLabel("Disconnected")
        conn_layout.addWidget(QLabel("ESP32 IP:"))
        conn_layout.addWidget(self.ip_input)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(QLabel("Polling (ms):"))
        conn_layout.addWidget(self.poll_spin)
        conn_layout.addWidget(self.conn_status_label)
        conn_layout.addStretch()
        self.main_layout.addWidget(conn_group)

        # --- PID & Parameters ---
        param_group = QGroupBox("ESP32 Parameters & Controls")
        param_layout = QHBoxLayout(param_group)
        left_col = QVBoxLayout()
        right_col = QVBoxLayout()
        param_layout.addLayout(left_col)
        param_layout.addLayout(right_col)
        self.main_layout.addWidget(param_group)

        self.param_fields = {}
        self.param_inputs = {}
        self.param_buttons = {}

        def add_param(layout, name):
            row = QHBoxLayout()
            val_label = QLabel("N/A"); val_label.setFixedWidth(60)
            input_box = QLineEdit(); input_box.setFixedWidth(80)
            set_btn = QPushButton("Set"); set_btn.setFixedWidth(50)
            set_btn.clicked.connect(lambda _, n=name: self.send_set(n))
            row.addWidget(QLabel(f"{name}:"))
            row.addWidget(val_label)
            row.addWidget(input_box)
            row.addWidget(set_btn)
            container = QWidget(); container.setLayout(row)
            layout.addWidget(container)
            self.param_fields[name] = val_label
            self.param_inputs[name] = input_box
            self.param_buttons[name] = set_btn

        for f in ["kp", "ki", "kd"]:
            add_param(left_col, f)
        for f in ["cycleTime", "maxSpeed", "defaultSpeed"]:
            add_param(right_col, f)

        # --- Running ---
        run_row = QHBoxLayout()
        run_row.addWidget(QLabel("Running:"))
        self.running_label = QLabel("N/A")
        self.run_btn = QPushButton("Start/Stop")
        self.run_btn.clicked.connect(self.toggle_running)
        run_row.addWidget(self.running_label)
        run_row.addWidget(self.run_btn)
        self.main_layout.addLayout(run_row)

        # --- Calibration ---
        calib_row = QHBoxLayout()
        calib_row.addWidget(QLabel("Calibration:"))
        self.white_label = QLabel("white=N/A")
        self.black_label = QLabel("black=N/A")
        calib_row.addWidget(self.white_label)
        calib_row.addWidget(self.black_label)
        self.calib_white_btn = QPushButton("Calibrate White")
        self.calib_black_btn = QPushButton("Calibrate Black")
        self.reset_calib_btn = QPushButton("Reset Calibration")
        self.calib_white_btn.clicked.connect(lambda: self.esp.send_command("/calibrate/white"))
        self.calib_black_btn.clicked.connect(lambda: self.esp.send_command("/calibrate/black"))
        self.reset_calib_btn.clicked.connect(lambda: self.esp.send_command("/calibrate/reset"))
        calib_row.addWidget(self.calib_white_btn)
        calib_row.addWidget(self.calib_black_btn)
        calib_row.addWidget(self.reset_calib_btn)
        self.main_layout.addLayout(calib_row)

        # --- Mapped Sensors ---
        self.mapped_group = QGroupBox("Mapped Sensors")
        mapped_layout = QHBoxLayout(self.mapped_group)
        self.mapped_bars = []
        for i in range(8):
            bar = QProgressBar(); bar.setOrientation(Qt.Vertical); bar.setRange(0, 100); bar.setFormat(f"S{i}"); bar.setFixedHeight(150)
            mapped_layout.addWidget(bar)
            self.mapped_bars.append(bar)
        self.main_layout.addWidget(self.mapped_group)

        # --- Motors ---
        self.motor_group = QGroupBox("Motors")
        motor_layout = QVBoxLayout(self.motor_group)
        self.m1_bar = QProgressBar(); self.m2_bar = QProgressBar()
        for bar, label in [(self.m1_bar,"M1"),(self.m2_bar,"M2")]:
            bar.setRange(0,100); bar.setFormat(f"{label}: %p%")
            motor_layout.addWidget(bar)
        self.main_layout.addWidget(self.motor_group)
        self.motor_group.setLayout(motor_layout)

        # --- Graph ---
        graph_ctrl = QHBoxLayout()
        graph_ctrl.addWidget(QLabel("Graph window (s):"))
        self.graph_window_time = QDoubleSpinBox(); self.graph_window_time.setRange(1,120); self.graph_window_time.setDecimals(1); self.graph_window_time.setValue(15)
        graph_ctrl.addWidget(self.graph_window_time)
        self.main_layout.addLayout(graph_ctrl)

        self.graph_widget = pg.PlotWidget(title="Error / Correction Over Time")
        self.graph_widget.showGrid(x=True,y=True)
        self.graph_widget.addLegend()
        self.graph_widget.setLabel("left","Value"); self.graph_widget.setLabel("bottom","Time (s)")
        self.graph_widget.setMinimumHeight(300)
        self.error_curve = self.graph_widget.plot(pen="r", name="error")
        self.correction_curve = self.graph_widget.plot(pen="g", name="correction")
        self.main_layout.addWidget(self.graph_widget)

        # --- Table ---
        self.table = QTableWidget()
        self.table.setSizePolicy(QSizePolicy.Expanding,QSizePolicy.Expanding)
        self.table.setAlternatingRowColors(True)
        self.table.setMinimumHeight(200)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.main_layout.addWidget(self.table)

    # ---------------- Connection ----------------
    def toggle_connection(self):
        if self.esp.connected:
            self.esp.disconnect_esp()
            self.connect_btn.setText("Connect"); self.conn_status_label.setText("Disconnected")
        else:
            ok = self.esp.connect_esp(self.ip_input.text(), self.poll_spin.value())
            if ok is True:
                self.connect_btn.setText("Disconnect"); self.conn_status_label.setText("Connected")
            else:
                self.conn_status_label.setText("Error")

    # ---------------- Commands ----------------
    def send_set(self, param):
        val = self.param_inputs[param].text().strip()
        if val: self.esp.send_set(param, val)

    def toggle_running(self):
        if self.running_label.text() == "True":
            self.esp.send_command("/stop")
        else:
            self.esp.send_command("/start")

    # ---------------- Data Update ----------------
    @QtCore.pyqtSlot(dict)
    def on_data(self, data):
        if not data: return

        # Update labels
        for f,lbl in self.param_fields.items(): lbl.setText(str(data.get(f,"N/A")))
        self.running_label.setText(str(data.get("running","N/A")))
        self.white_label.setText(f"white={data.get('whiteValues',[])}")
        self.black_label.setText(f"black={data.get('blackValues',[])}")

        # Mapped sensors
        mapped = data.get("mappedSensors",[0]*8)
        for i,v in enumerate(mapped): self.mapped_bars[i].setValue(v)

        # Motors
        self.m1_bar.setValue(max(0,min(100,int(data.get("m1",0)))))
        self.m2_bar.setValue(max(0,min(100,int(data.get("m2",0)))))

        # Graph
        now = time.time()
        self.history_graph.append({"time":now,"error":data.get("error",0),"correction":data.get("correction",0)})
        cutoff = now - self.graph_window_time.value()
        self.history_graph = [h for h in self.history_graph if h["time"] >= cutoff]
        if self.history_graph:
            t0=self.history_graph[0]["time"]
            times = [h["time"]-t0 for h in self.history_graph]
            self.error_curve.setData(times,[h["error"] for h in self.history_graph])
            self.correction_curve.setData(times,[h["correction"] for h in self.history_graph])

        # Table
        exclude={"ssid","ip","calculationTime","sensors","whiteValues","blackValues"}
        flat={}
        for k,v in data.items():
            if k in exclude: continue
            if isinstance(v,list):
                for i,val in enumerate(v): flat[f"{k}[{i}]"]=val
            else: flat[k]=v
        self.history_table.insert(0,flat)
        if len(self.history_table)>100: self.history_table.pop()
        headers=list(flat.keys())
        self.table.setColumnCount(len(headers))
        self.table.setHorizontalHeaderLabels(headers)
        self.table.setRowCount(len(self.history_table))
        for row_idx,row_data in enumerate(self.history_table):
            for col_idx,key in enumerate(headers):
                self.table.setItem(row_idx,col_idx,QTableWidgetItem(str(row_data.get(key,""))))


# ---------------- Main App ----------------
class MainApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Line Follower Dashboard")
        self.resize(1200, 800)
        self.esp = ESPConnection()
        self.tabs = QtWidgets.QTabWidget()
        self.control_tab = ControlTab(self.esp)
        self.tabs.addTab(self.control_tab,"Control & Monitor")
        self.setCentralWidget(self.tabs)


# ---------------- Run ----------------
if __name__=="__main__":
    app = QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
