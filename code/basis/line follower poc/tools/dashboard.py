# ESP32 PyQt5 Dashboard Template
# This file provides a full GUI template that:
# 1. Lets the user enter/find ESP32 IP
# 2. Connect/disconnect via TCP
# 3. Shows latest JSON data
# 4. Keeps a scrollable history with timestamps

import sys
import socket
import json
import threading
import time
from PyQt5 import QtCore, QtGui, QtWidgets

class TcpClientThread(QtCore.QThread):
    data_received = QtCore.pyqtSignal(dict, float)
    disconnected = QtCore.pyqtSignal()

    def __init__(self, ip, port=1234):
        super().__init__()
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
                    data = self.sock.recv(2048).decode()
                    if not data:
                        break
                    buffer += data
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        try:
                            obj = json.loads(line)
                            ts = time.time()
                            self.data_received.emit(obj, ts)
                        except json.JSONDecodeError:
                            pass
                except socket.timeout:
                    continue

        except Exception as e:
            pass

        self.disconnected.emit()

    def stop(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass

class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Dashboard")
        self.resize(900, 600)

        self.client_thread = None
        self.history = []  # list of (timestamp, dict)

        layout = QtWidgets.QVBoxLayout()

        # Top row: IP input + explore + connect
        top_row = QtWidgets.QHBoxLayout()
        self.ip_input = QtWidgets.QLineEdit()
        self.ip_input.setPlaceholderText("Enter ESP32 IP...")
        self.explore_button = QtWidgets.QPushButton("Explore")
        self.connect_button = QtWidgets.QPushButton("Connect")
        self.disconnect_button = QtWidgets.QPushButton("Disconnect")
        self.disconnect_button.setEnabled(False)

        top_row.addWidget(self.ip_input)
        top_row.addWidget(self.explore_button)
        top_row.addWidget(self.connect_button)
        top_row.addWidget(self.disconnect_button)

        layout.addLayout(top_row)

        # Live data display
        self.live_view = QtWidgets.QTextEdit()
        self.live_view.setReadOnly(True)
        layout.addWidget(QtWidgets.QLabel("Current Values:"))
        layout.addWidget(self.live_view)

        # Container
        container = QtWidgets.QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Signals
        self.explore_button.clicked.connect(self.explore_network)
        self.connect_button.clicked.connect(self.connect_to_esp)
        self.disconnect_button.clicked.connect(self.disconnect_from_esp)

    def explore_network(self):
        # Basic LAN scan for ESP32 on port 1234
        found_ips = []
        base_ip = self.ip_input.text().strip()
        if not base_ip:
            return
        for i in range(1, 255):
            ip = base_ip + str(i)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.1)
            try:
                s.connect((ip, 1234))
                found_ips.append(ip)
                s.close()
            except:
                pass
        if found_ips:
            self.ip_input.setText(found_ips[0])
        else:
            self.ip_input.setText("Not Found")

    def connect_to_esp(self):
        ip = self.ip_input.text().strip()
        if not ip:
            return

        self.client_thread = TcpClientThread(ip)
        self.client_thread.data_received.connect(self.handle_data)
        self.client_thread.disconnected.connect(self.handle_disconnect)
        self.client_thread.start()

        self.connect_button.setEnabled(False)
        self.disconnect_button.setEnabled(True)

    def disconnect_from_esp(self):
        if self.client_thread:
            self.client_thread.stop()
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)

    def handle_disconnect(self):
        self.connect_button.setEnabled(True)
        self.disconnect_button.setEnabled(False)

    def handle_data(self, obj, ts):
        # Update live view instantly
        self.live_view.setPlainText(json.dumps(obj, indent=2))








if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
