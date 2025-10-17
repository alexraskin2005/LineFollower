import sys, csv, json, threading, time
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import requests
from websocket import WebSocketApp
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton
from PyQt5.QtGui import QPalette, QColor
import sys



# ---------------- ESP Connection ----------------
class ESPConnection(QtCore.QObject):
    data_received = QtCore.pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.ip = None
        self.connected = False
        self.thread = None

    def connect(self, ip):
        self.ip = ip
        try:
            requests.get(f"http://{self.ip}/api/pid", timeout=2)
            self.connected = True
            self.thread = threading.Thread(target=self.ws_loop, daemon=True)
            self.thread.start()
            return True
        except Exception as e:
            print("Connection failed:", e)
            self.connected = False
            return False

    def ws_loop(self):
        url = f"ws://{self.ip}:81/"
        def on_message(ws, msg):
            try:
                j = json.loads(msg)
                self.data_received.emit(j)
            except:
                pass
        while self.connected:
            try:
                ws = WebSocketApp(url, on_message=on_message)
                ws.run_forever()
            except Exception as e:
                print("WS error:", e)
            time.sleep(2)

    def send_cmd(self, val):
        if not self.connected: return
        try:
            requests.get(f"http://{self.ip}/api/cmd", params={"val": val}, timeout=2)
        except: pass

    def set_pid(self, kp, ki, kd, max_motor):
        if not self.connected: return
        try:
            requests.get(f"http://{self.ip}/api/pid",
                         params={"kp":kp,"ki":ki,"kd":kd,"max":max_motor}, timeout=2)
        except: pass

# ---------------- Control Tab ----------------
class ControlTab(QtWidgets.QWidget):
    def __init__(self, esp:ESPConnection):
        super().__init__()
        self.esp = esp
        self.esp.data_received.connect(self.on_data)
        self.logging = False
        self.csv_file = None
        self.max_points = 500
        self.y_min = 0
        self.y_max = 0
        self.x_range_seconds = 5
        self.y_padding = 0.05

        layout = QtWidgets.QVBoxLayout(self)
        layout.setSpacing(10)

        # --- Connection ---
        conn_group = QtWidgets.QGroupBox("Connection")
        conn_group.setStyleSheet("QGroupBox{font-weight:bold;}")
        conn_layout = QtWidgets.QHBoxLayout(conn_group)
        self.ip_input = QtWidgets.QLineEdit("192.168.1.123")
        self.connect_btn = QtWidgets.QPushButton("Connect")
        self.connect_btn.clicked.connect(self.connect_esp)
        self.conn_status_label = QtWidgets.QLabel("Disconnected")
        self.conn_status_label.setStyleSheet("color: gray; font-size:11px")
        conn_layout.addWidget(QtWidgets.QLabel("ESP32 IP:"))
        conn_layout.addWidget(self.ip_input)
        conn_layout.addWidget(self.connect_btn)
        conn_layout.addWidget(self.conn_status_label)
        conn_layout.addStretch()
        layout.addWidget(conn_group)

        # --- PID Controls ---
        pid_group = QtWidgets.QGroupBox("PID Controller")
        pid_group.setStyleSheet("QGroupBox{font-weight:bold;}")
        pid_layout = QtWidgets.QVBoxLayout(pid_group)

        input_layout = QtWidgets.QHBoxLayout()
        self.kp_input = QtWidgets.QDoubleSpinBox(); self.kp_input.setDecimals(3); self.kp_input.setRange(0,100); self.kp_input.setValue(0.01); self.kp_input.setPrefix("Kp: ")
        self.ki_input = QtWidgets.QDoubleSpinBox(); self.ki_input.setDecimals(3); self.ki_input.setRange(0,100); self.ki_input.setValue(0.0); self.ki_input.setPrefix("Ki: ")
        self.kd_input = QtWidgets.QDoubleSpinBox(); self.kd_input.setDecimals(3); self.kd_input.setRange(0,100); self.kd_input.setValue(0.0); self.kd_input.setPrefix("Kd: ")
        self.max_input = QtWidgets.QSpinBox(); self.max_input.setRange(0,255); self.max_input.setValue(255); self.max_input.setPrefix("Max: ")
        self.pid_btn = QtWidgets.QPushButton("Set PID")
        self.pid_btn.clicked.connect(self.set_pid)
        input_layout.addWidget(self.kp_input)
        input_layout.addWidget(self.ki_input)
        input_layout.addWidget(self.kd_input)
        input_layout.addWidget(self.max_input)
        input_layout.addWidget(self.pid_btn)
        pid_layout.addLayout(input_layout)

        current_layout = QtWidgets.QHBoxLayout()
        self.kp_display = QtWidgets.QLabel("Current: ?")
        self.ki_display = QtWidgets.QLabel("Current: ?")
        self.kd_display = QtWidgets.QLabel("Current: ?")
        self.max_display = QtWidgets.QLabel("Current: ?")
        current_layout.addWidget(self.kp_display)
        current_layout.addWidget(self.ki_display)
        current_layout.addWidget(self.kd_display)
        current_layout.addWidget(self.max_display)
        pid_layout.addLayout(current_layout)
        layout.addWidget(pid_group)

        # --- Robot control and logging ---
        control_group = QtWidgets.QGroupBox("Robot Control")
        control_group.setStyleSheet("QGroupBox{font-weight:bold;}")
        ctrl_layout = QtWidgets.QHBoxLayout(control_group)
        self.start_btn = QtWidgets.QPushButton("Start")
        self.stop_btn = QtWidgets.QPushButton("Stop")
        self.log_btn = QtWidgets.QPushButton("Start Logging")
        self.start_btn.clicked.connect(lambda: self.esp.send_cmd("START"))
        self.stop_btn.clicked.connect(lambda: self.esp.send_cmd("STOP"))
        self.log_btn.clicked.connect(self.toggle_logging)
        ctrl_layout.addWidget(self.start_btn)
        ctrl_layout.addWidget(self.stop_btn)
        ctrl_layout.addWidget(self.log_btn)
        layout.addWidget(control_group)

        self.log_file_label = QtWidgets.QLabel("")
        layout.addWidget(self.log_file_label)

        # --- Graph Settings ---
        graph_group = QtWidgets.QGroupBox("Graph Settings")
        graph_group.setStyleSheet("QGroupBox{font-weight:bold;}")
        settings_layout = QtWidgets.QHBoxLayout(graph_group)
        self.x_range_input = QtWidgets.QDoubleSpinBox(); self.x_range_input.setRange(0.1,60); self.x_range_input.setValue(self.x_range_seconds); self.x_range_input.setSuffix(" s"); self.x_range_input.valueChanged.connect(lambda v: setattr(self,'x_range_seconds',v))
        self.y_min_input = QtWidgets.QDoubleSpinBox(); self.y_min_input.setRange(-1000,1000); self.y_min_input.setValue(0); self.y_min_input.valueChanged.connect(lambda v: setattr(self,'y_min',v))
        self.y_max_input = QtWidgets.QDoubleSpinBox(); self.y_max_input.setRange(-1000,1000); self.y_max_input.setValue(0); self.y_max_input.valueChanged.connect(lambda v: setattr(self,'y_max',v))
        self.y_padding_input = QtWidgets.QDoubleSpinBox(); self.y_padding_input.setRange(0.0,1.0); self.y_padding_input.setSingleStep(0.01); self.y_padding_input.setValue(self.y_padding); self.y_padding_input.setSuffix(" pad"); self.y_padding_input.valueChanged.connect(lambda v: setattr(self,'y_padding',v))
        settings_layout.addWidget(QtWidgets.QLabel("X-axis:")); settings_layout.addWidget(self.x_range_input)
        settings_layout.addWidget(QtWidgets.QLabel("Y-min:")); settings_layout.addWidget(self.y_min_input)
        settings_layout.addWidget(QtWidgets.QLabel("Y-max:")); settings_layout.addWidget(self.y_max_input)
        settings_layout.addWidget(QtWidgets.QLabel("Y-padding:")); settings_layout.addWidget(self.y_padding_input)
        layout.addWidget(graph_group)

        # --- Plot ---
        self.plot_widget = pg.PlotWidget(title="Live Data (Sensor / Correction / Error)")
        self.plot_widget.addLegend()
        self.plot_widget.showGrid(x=True,y=True)
        layout.addWidget(self.plot_widget)
        self.sensor_curve = self.plot_widget.plot(pen='w', name="Sensor")
        self.corr_curve = self.plot_widget.plot(pen='g', name="Correction")
        self.error_curve = self.plot_widget.plot(pen='y', name="Error")

        # --- Motors ---
        motor_group = QtWidgets.QGroupBox("Motors")
        motor_group.setStyleSheet("QGroupBox{font-weight:bold;}")
        motor_layout = QtWidgets.QVBoxLayout(motor_group)
        self.motor1_bar = QtWidgets.QProgressBar(); self.motor1_bar.setRange(0,255)
        self.motor1_label = QtWidgets.QLabel("Motor1: ?")
        self.motor2_bar = QtWidgets.QProgressBar(); self.motor2_bar.setRange(0,255)
        self.motor2_label = QtWidgets.QLabel("Motor2: ?")
        motor_layout.addWidget(self.motor1_label); motor_layout.addWidget(self.motor1_bar)
        motor_layout.addWidget(self.motor2_label); motor_layout.addWidget(self.motor2_bar)
        layout.addWidget(motor_group)

        # --- Data buffers ---
        self.data_buffer = {"t":[],"sensor":[],"correction":[],"error":[]}

        # Apply dark mode


        #app.setStyle("Fusion")

        # dark_palette = QPalette()
        # dark_palette.setColor(QPalette.Window, QColor(12, 12, 12))
        # dark_palette.setColor(QPalette.WindowText, QColor(255, 255, 255))
        # dark_palette.setColor(QPalette.Base, QColor(12, 12, 12))
        # dark_palette.setColor(QPalette.AlternateBase, QColor(12, 12, 12))
        # dark_palette.setColor(QPalette.ToolTipBase, QColor(255, 255, 255))
        # dark_palette.setColor(QPalette.ToolTipText, QColor(255, 255, 255))
        # dark_palette.setColor(QPalette.Text, QColor(255, 255, 255))
        # dark_palette.setColor(QPalette.Button, QColor(12, 12, 12))
        # dark_palette.setColor(QPalette.ButtonText, QColor(255, 255, 255))
        # dark_palette.setColor(QPalette.BrightText, QColor(255, 0, 0))
        # dark_palette.setColor(QPalette.Highlight, QColor(142, 45, 197))
        # dark_palette.setColor(QPalette.HighlightedText, QColor(255, 255, 255))

        # app.setPalette(dark_palette)


        # self.setStyleSheet("""
        #     QWidget { background-color: #121212; color: #e0e0e0; font-size:12px; }
        #     QGroupBox { border: 1px solid #444; margin-top:10px; }
        #     QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding:0 3px; }
        #     QSpinBox, QDoubleSpinBox, QLineEdit { background-color:#1e1e1e; color:#e0e0e0; border:1px solid #555; padding:2px; }
        #     QPushButton { background-color:#2c2c2c; border:1px solid #555; padding:4px; }
        #     QPushButton:hover { background-color:#3c3c3c; }
        #     QProgressBar { border:1px solid #555; text-align:center; }
        #     QProgressBar::chunk { background-color: #00aa00; }
        # """)

    # ---------------- Methods ----------------
    def connect_esp(self):
        ip = self.ip_input.text()
        if self.esp.connect(ip):
            self.conn_status_label.setText("Connected")
        else:
            self.conn_status_label.setText("Disconnected")

    def set_pid(self):
        kp=float(self.kp_input.value())
        ki=float(self.ki_input.value())
        kd=float(self.kd_input.value())
        max_motor=float(self.max_input.value())
        self.esp.set_pid(kp,ki,kd,max_motor)

    def toggle_logging(self):
        if not self.logging:
            fname,_ = QtWidgets.QFileDialog.getSaveFileName(self,"Save CSV","","CSV Files (*.csv)")
            if fname:
                self.csv_file = open(fname,"w",newline="")
                self.csv_writer = csv.writer(self.csv_file)
                self.csv_writer.writerow(["timestamp_ms","sensor","correction","error","motor1","motor2","Kp","Ki","Kd","max_motor"])
                self.logging = True
                self.log_btn.setText("Stop Logging")
                self.log_file_label.setText(fname.split('/')[-1])
        else:
            self.logging = False
            if self.csv_file: self.csv_file.close(); self.csv_file=None
            self.log_btn.setText("Start Logging"); self.log_file_label.setText("")

    @QtCore.pyqtSlot(dict)
    def on_data(self,j):
        if not j:
            self.conn_status_label.setText("Disconnected")
            self.kp_display.setText("Current: ?"); self.ki_display.setText("Current: ?")
            self.kd_display.setText("Current: ?"); self.max_display.setText("Current: ?")
            self.motor1_label.setText("Motor1: ?"); self.motor2_label.setText("Motor2: ?")
            return

        self.conn_status_label.setText("Connected")

        # PID display
        self.kp_display.setText(f"Current: {j.get('kp',0):.2f}")
        self.ki_display.setText(f"Current: {j.get('ki',0):.2f}")
        self.kd_display.setText(f"Current: {j.get('kd',0):.2f}")
        self.max_display.setText(f"Current: {j.get('max',0):.0f}")

        # Update buffers
        t=j['t']/1000.0
        self.data_buffer["t"].append(t)
        self.data_buffer["sensor"].append(j['s'])
        self.data_buffer["correction"].append(j['c'])
        self.data_buffer["error"].append(j['e'])
        for key in self.data_buffer:
            if len(self.data_buffer[key])>self.max_points: self.data_buffer[key].pop(0)

        self.sensor_curve.setData(self.data_buffer["t"], self.data_buffer["sensor"])
        self.corr_curve.setData(self.data_buffer["t"], self.data_buffer["correction"])
        self.error_curve.setData(self.data_buffer["t"], self.data_buffer["error"])

        # X/Y axis
        if self.data_buffer["t"]:
            t_max=self.data_buffer["t"][-1]
            t_min=max(0,t_max-self.x_range_seconds)
            self.plot_widget.setXRange(t_min,t_max)
        if self.y_min!=0 or self.y_max!=0:
            self.plot_widget.setYRange(self.y_min,self.y_max)
        else:
            vals=self.data_buffer["sensor"]+self.data_buffer["correction"]+self.data_buffer["error"]
            if vals:
                max_abs=max(abs(min(vals)),abs(max(vals)))
                pad=max_abs*self.y_padding
                self.plot_widget.setYRange(-max_abs-pad,max_abs+pad)

        # Motors
        m1=j.get("m1",0); m2=j.get("m2",0)
        self.motor1_bar.setValue(int(m1)); self.motor1_label.setText(f"Motor1: {int(m1)}")
        self.motor2_bar.setValue(int(m2)); self.motor2_label.setText(f"Motor2: {int(m2)}")

        # Logging
        if self.logging and self.csv_file:
            self.csv_writer.writerow([j['t'], j['s'], j['c'], j['e'], m1, m2, j['kp'], j['ki'], j['kd'], j['max']])

# ---------------- Autotune Tab ----------------
class AutotuneTab(QtWidgets.QWidget):
    def __init__(self, esp:ESPConnection):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel("Autotune tab - to be implemented"))

# ---------------- Predict Tab ----------------
class PredictTab(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(QtWidgets.QLabel("Predict PID tab - to be implemented"))

# ---------------- Main App ----------------
class MainApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Line Follower Client")
        self.resize(1000,650)
        self.esp = ESPConnection()
        self.tabs = QtWidgets.QTabWidget()
        self.control_tab = ControlTab(self.esp)
        self.autotune_tab = AutotuneTab(self.esp)
        self.predict_tab = PredictTab()
        self.tabs.addTab(self.control_tab,"Control & Monitor")
        self.tabs.addTab(self.autotune_tab,"Autotune")
        self.tabs.addTab(self.predict_tab,"Predict PID")
        self.setCentralWidget(self.tabs)

# ---------------- Run ----------------
if __name__=="__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainApp()
    window.show()
    sys.exit(app.exec_())
