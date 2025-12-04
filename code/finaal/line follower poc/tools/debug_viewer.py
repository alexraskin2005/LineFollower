#!/usr/bin/env python3
"""
Simple PyQt5 GUI to poll the device /debug endpoint and display values.
Usage:
  python tools/debug_viewer.py --host 192.168.4.1 --port 80

Requirements: PyQt5, requests
"""
import sys
import argparse
import json
import threading
import time

import requests
from PyQt5 import QtWidgets, QtCore

POLL_INTERVAL = 1.0  # seconds


class DebugPoller(QtCore.QObject):
    updated = QtCore.pyqtSignal(dict)

    def __init__(self, url, interval=POLL_INTERVAL, parent=None):
        super().__init__(parent)
        self.url = url
        self.interval = interval
        self._running = False
        self._thread = None


    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=1)
            self._thread = None

    def _loop(self):
        while self._running:
            try:
                r = requests.get(self.url, timeout=2)
                r.raise_for_status()
                data = r.json()
            except Exception as e:
                data = {"error": str(e)}
            self.updated.emit(data)
            time.sleep(self.interval)


class DebugWindow(QtWidgets.QWidget):
    def __init__(self, url):
        super().__init__()
        self.url = url
        self.setWindowTitle('Line Follower Debug Viewer')
        self.resize(800, 600)
        self._build_ui()

        self.poller = DebugPoller(url)
        self.poller.updated.connect(self.on_update)
        self.poller.start()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)

        top = QtWidgets.QHBoxLayout()
        self.status_label = QtWidgets.QLabel('Not connected')
        top.addWidget(self.status_label)
        top.addStretch()
        layout.addLayout(top)

        grid = QtWidgets.QGridLayout()
        self.fields = {}

        labels = [
            'timestamp', 'cycleTime', 'ssid', 'ip', 'calculationTime',
            'defaultSpeed', 'maxSpeed', 'running', 'kp', 'ki', 'kd',
            'error', 'correction', 'm1', 'm2'
        ]
        for i, key in enumerate(labels):
            lbl = QtWidgets.QLabel(key)
            val = QtWidgets.QLabel('')
            val.setMinimumWidth(120)
            grid.addWidget(lbl, i, 0)
            grid.addWidget(val, i, 1)
            self.fields[key] = val

        layout.addLayout(grid)

        # sensors and mapped sensors table
        self.sensors_table = QtWidgets.QTableWidget(2, 8)
        self.sensors_table.setHorizontalHeaderLabels([f'S{i}' for i in range(8)])
        self.sensors_table.setVerticalHeaderLabels(['raw', 'mapped'])
        layout.addWidget(self.sensors_table)

        # calibration arrays
        cal_layout = QtWidgets.QHBoxLayout()
        self.white_label = QtWidgets.QLabel('white: -')
        self.black_label = QtWidgets.QLabel('black: -')
        cal_layout.addWidget(self.white_label)
        cal_layout.addWidget(self.black_label)
        cal_layout.addStretch()
        layout.addLayout(cal_layout)

        # raw JSON viewer
        self.json_view = QtWidgets.QPlainTextEdit()
        self.json_view.setReadOnly(True)
        layout.addWidget(self.json_view)

    @QtCore.pyqtSlot(dict)
    def on_update(self, data):
        if 'error' in data:
            self.status_label.setText('Error: ' + data.get('error'))
            return
        self.status_label.setText('OK')
        # update simple fields
        for k, lbl in self.fields.items():
            lbl.setText(str(data.get(k, '')))

        # sensors
        sensors = data.get('sensors', [])
        mapped = data.get('mappedSensors', [])
        for i in range(8):
            self.sensors_table.setItem(0, i, QtWidgets.QTableWidgetItem(str(sensors[i]) if i < len(sensors) else ''))
            self.sensors_table.setItem(1, i, QtWidgets.QTableWidgetItem(str(mapped[i]) if i < len(mapped) else ''))

        # calibration arrays
        white = data.get('whiteValues', [])
        black = data.get('blackValues', [])
        self.white_label.setText('white: ' + ','.join(str(x) for x in white))
        self.black_label.setText('black: ' + ','.join(str(x) for x in black))

        # raw json
        self.json_view.setPlainText(json.dumps(data, indent=2))


def main(argv):
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='linefollower.local', help='device IP or hostname')
    parser.add_argument('--port', default=80, type=int, help='device port')
    parser.add_argument('--interval', default=1.0, type=float, help='poll interval seconds')
    args = parser.parse_args(argv)

    url = f'http://{args.host}:{args.port}/debug'
    global POLL_INTERVAL
    POLL_INTERVAL = args.interval

    app = QtWidgets.QApplication([])
    w = DebugWindow(url)
    w.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main(sys.argv[1:])
