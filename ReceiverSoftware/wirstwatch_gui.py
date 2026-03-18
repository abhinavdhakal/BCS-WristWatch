import asyncio
import csv
import struct
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
from bleak import BleakClient, BleakScanner
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt6.QtWidgets import (
    QApplication,
    QComboBox,
    QFormLayout,
    QGroupBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

try:
    from ReceiverSoftware.hr_spo2 import compute_hr_spo2
except ModuleNotFoundError:
    from hr_spo2 import compute_hr_spo2


SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
IR_CHAR_UUID = "12345678-1234-1234-1234-123456789ab1"
RED_CHAR_UUID = "12345678-1234-1234-1234-123456789ab2"
TEMP_CHAR_UUID = "12345678-1234-1234-1234-123456789ab3"
IMU_CHAR_UUID = "12345678-1234-1234-1234-123456789ab4"
BATT_CHAR_UUID = "12345678-1234-1234-1234-123456789ab5"
DEVICE_SLEEP_MARKER = 0xFFFFFFFF


class SessionLogger:
    def __init__(self, base_dir: Path):
        self.base_dir = base_dir
        self.base_dir.mkdir(parents=True, exist_ok=True)
        self.path = None
        self.file = None
        self.writer = None

    def start(self):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = self.base_dir / f"session_{stamp}.csv"
        self.file = self.path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.file)
        self.writer.writerow(
            [
                "timestamp",
                "ir",
                "red",
                "temp_c",
                "battery_v",
                "battery_percent",
                "hr",
                "spo2",
                "signal_valid",
                "accel_x",
                "accel_y",
                "accel_z",
                "gyro_x",
                "gyro_y",
                "gyro_z",
            ]
        )
        return str(self.path)

    def write(self, values):
        if not self.writer:
            return
        self.writer.writerow(
            [
                datetime.now().isoformat(timespec="milliseconds"),
                values["IR"],
                values["Red"],
                f"{values['Temp_C']:.3f}",
                f"{values['Battery_V']:.3f}",
                f"{values['Battery_%']:.1f}",
                f"{values['HR']:.2f}",
                f"{values['SpO2']:.2f}",
                int(values["Signal_Valid"]),
                f"{values['Accel_X']:.4f}",
                f"{values['Accel_Y']:.4f}",
                f"{values['Accel_Z']:.4f}",
                f"{values['Gyro_X']:.4f}",
                f"{values['Gyro_Y']:.4f}",
                f"{values['Gyro_Z']:.4f}",
            ]
        )
        self.file.flush()

    def stop(self):
        if self.file:
            self.file.close()
        self.path = None
        self.file = None
        self.writer = None


class BleWorker(QObject):
    scan_done = pyqtSignal(list)
    connect_done = pyqtSignal(bool, str)
    disconnect_done = pyqtSignal(str)
    data_update = pyqtSignal(dict)
    status_update = pyqtSignal(str)
    log = pyqtSignal(str)
    error = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        self.client = None
        self.connected = False
        self.manual_disconnect = False
        self.disconnect_notified = False
        self.logger = SessionLogger(Path(__file__).resolve().parent / "sessions")

        self.sensor_values = {
            "IR": 0,
            "Red": 0,
            "Temp_C": 0.0,
            "Accel_X": 0.0,
            "Accel_Y": 0.0,
            "Accel_Z": 0.0,
            "Gyro_X": 0.0,
            "Gyro_Y": 0.0,
            "Gyro_Z": 0.0,
            "Battery_V": 0.0,
            "Battery_%": 0.0,
            "HR": 0.0,
            "SpO2": 0.0,
            "Signal_Valid": False,
        }

        self.ir_buffer = deque(maxlen=250)
        self.red_buffer = deque(maxlen=250)
        self.accel_x_series = deque(maxlen=250)
        self.accel_y_series = deque(maxlen=250)
        self.accel_z_series = deque(maxlen=250)
        self.gyro_x_series = deque(maxlen=250)
        self.gyro_y_series = deque(maxlen=250)
        self.gyro_z_series = deque(maxlen=250)
        self.last_hr_compute = 0.0

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _submit(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self.loop)

    def _log(self, message: str):
        ts = datetime.now().strftime("%H:%M:%S")
        self.log.emit(f"[{ts}] {message}")

    def scan(self, name_filter: str):
        self._submit(self._scan(name_filter))

    def connect(self, address: str):
        self._submit(self._connect(address))

    def disconnect(self, reason: str = "Disconnected"):
        self._submit(self._disconnect(reason))

    async def _scan(self, name_filter: str):
        self.status_update.emit("Scanning for matching BLE devices...")
        self._log(f"Scan started (filter='{name_filter.strip()}')")
        try:
            devices = await BleakScanner.discover(timeout=4.0, service_uuids=[SERVICE_UUID])
            found = []
            needle = name_filter.strip().lower()
            for d in devices:
                name = d.name or "Unknown"
                if needle and needle not in name.lower():
                    continue
                found.append({"name": name, "address": d.address})
            found.sort(key=lambda x: x["name"].lower())
            self._log(f"Scan complete: {len(found)} matching device(s)")
            self.scan_done.emit(found)
        except Exception as exc:
            self._log(f"Scan failed: {exc}")
            self.error.emit(f"Scan failed: {exc}")

    async def _connect(self, address: str):
        if self.connected:
            self.connect_done.emit(True, "Already connected")
            return

        self.status_update.emit(f"Connecting to {address}...")
        self._log(f"Connecting to {address}")
        try:
            self.manual_disconnect = False
            self.disconnect_notified = False
            self.client = BleakClient(address, timeout=8.0, disconnected_callback=self._on_disconnected)
            await self.client.connect(timeout=8.0)
            if not self.client.is_connected:
                self.connect_done.emit(False, "Connection failed")
                return

            self.ir_buffer.clear()
            self.red_buffer.clear()
            self.last_hr_compute = time.monotonic()

            session_path = self.logger.start()
            self._log(f"Session started: {session_path}")
            await self.client.start_notify(IR_CHAR_UUID, self._notification_handler)
            await self.client.start_notify(RED_CHAR_UUID, self._notification_handler)
            await self.client.start_notify(TEMP_CHAR_UUID, self._notification_handler)
            await self.client.start_notify(IMU_CHAR_UUID, self._notification_handler)
            await self.client.start_notify(BATT_CHAR_UUID, self._notification_handler)

            self.connected = True
            self.status_update.emit("Connected and streaming")
            self._log("Connected and notifications subscribed")
            self.connect_done.emit(True, session_path)
        except Exception as exc:
            self.connected = False
            if self.client and self.client.is_connected:
                await self.client.disconnect()
            self.client = None
            self._log(f"Connect failed: {exc}")
            self.connect_done.emit(False, f"{exc}")

    async def _disconnect(self, reason: str = "Disconnected"):
        self.manual_disconnect = True
        self._log(f"Disconnect requested: {reason}")
        try:
            self.connected = False
            if self.client and self.client.is_connected:
                for uuid in [IR_CHAR_UUID, RED_CHAR_UUID, TEMP_CHAR_UUID, IMU_CHAR_UUID, BATT_CHAR_UUID]:
                    try:
                        await asyncio.wait_for(self.client.stop_notify(uuid), timeout=0.4)
                    except Exception:
                        pass
                try:
                    await asyncio.wait_for(self.client.disconnect(), timeout=1.0)
                except Exception:
                    pass
        finally:
            await self._finalize_disconnect(reason)

    async def _finalize_disconnect(self, reason: str):
        self.connected = False
        self.client = None
        self.logger.stop()
        self.status_update.emit(reason)
        self._log(f"Disconnected: {reason}")
        if not self.disconnect_notified:
            self.disconnect_notified = True
            self.disconnect_done.emit(reason)

    def _on_disconnected(self, _client):
        if self.connected and not self.manual_disconnect:
            self._log("BLE disconnected callback triggered")
            self._submit(self._finalize_disconnect("Connection lost / device asleep"))

    async def _notification_handler(self, sender, data):
        try:
            if not self.connected or self.manual_disconnect:
                return

            uuid = sender.uuid
            if uuid == IR_CHAR_UUID:
                val = struct.unpack("<L", data)[0]
                self.sensor_values["IR"] = val
                self.ir_buffer.append(val)
            elif uuid == RED_CHAR_UUID:
                val = struct.unpack("<L", data)[0]
                self.sensor_values["Red"] = val
                self.red_buffer.append(val)
            elif uuid == TEMP_CHAR_UUID:
                self.sensor_values["Temp_C"] = struct.unpack("<L", data)[0] / 100.0
            elif uuid == IMU_CHAR_UUID:
                ax, ay, az, gx, gy, gz = struct.unpack("<ffffff", data)
                self.sensor_values["Accel_X"] = ax
                self.sensor_values["Accel_Y"] = ay
                self.sensor_values["Accel_Z"] = az
                self.sensor_values["Gyro_X"] = gx
                self.sensor_values["Gyro_Y"] = gy
                self.sensor_values["Gyro_Z"] = gz
                self.accel_x_series.append(ax)
                self.accel_y_series.append(ay)
                self.accel_z_series.append(az)
                self.gyro_x_series.append(gx)
                self.gyro_y_series.append(gy)
                self.gyro_z_series.append(gz)
            elif uuid == BATT_CHAR_UUID:
                batt_raw = struct.unpack("<L", data)[0]
                if batt_raw == DEVICE_SLEEP_MARKER:
                    self._log("Received sleep marker from device")
                    await self._disconnect("Device entered deep sleep")
                    return
                vbat = batt_raw / 1000.0
                self.sensor_values["Battery_V"] = vbat
                self.sensor_values["Battery_%"] = max(0.0, min(100.0, ((vbat - 3.0) / 1.2) * 100.0))

            now_mono = time.monotonic()
            if now_mono - self.last_hr_compute >= 1.0:
                self.last_hr_compute = now_mono
                hr, spo2 = compute_hr_spo2(self.ir_buffer, self.red_buffer)
                if hr > 0 and spo2 > 0:
                    self.sensor_values["Signal_Valid"] = True
                    self.sensor_values["HR"] = float(hr)
                    self.sensor_values["SpO2"] = float(spo2)
                else:
                    self.sensor_values["Signal_Valid"] = False
                    self.sensor_values["HR"] = 0.0
                    self.sensor_values["SpO2"] = 0.0

            self.logger.write(self.sensor_values)

            payload = {
                "values": dict(self.sensor_values),
                "ir": np.array(self.ir_buffer, dtype=np.float64),
                "red": np.array(self.red_buffer, dtype=np.float64),
                "accel_x": np.array(self.accel_x_series, dtype=np.float64),
                "accel_y": np.array(self.accel_y_series, dtype=np.float64),
                "accel_z": np.array(self.accel_z_series, dtype=np.float64),
                "gyro_x": np.array(self.gyro_x_series, dtype=np.float64),
                "gyro_y": np.array(self.gyro_y_series, dtype=np.float64),
                "gyro_z": np.array(self.gyro_z_series, dtype=np.float64),
            }
            self.data_update.emit(payload)
        except Exception as exc:
            self.error.emit(f"Decode error: {exc}")


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("WirstWatch_BCS Monitor (PyQt6)")
        self.resize(1250, 920)

        logs_dir = Path(__file__).resolve().parent / "logs"
        logs_dir.mkdir(parents=True, exist_ok=True)
        log_stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = logs_dir / f"gui_{log_stamp}.log"
        self.log_file = self.log_path.open("a", encoding="utf-8")

        self.worker = BleWorker()
        self.devices = []
        self.selected_address = None
        self.latest_payload = None
        self.is_closing = False

        self._build_ui()
        self._wire_signals()
        self.plot_timer = QTimer(self)
        self.plot_timer.setInterval(120)
        self.plot_timer.timeout.connect(self._render_latest_payload)
        self.plot_timer.start()

    def _build_ui(self):
        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        top = QHBoxLayout()
        self.filter_input = QLineEdit("BCS")
        self.filter_input.setPlaceholderText("Device name contains...")
        self.scan_btn = QPushButton("Scan")
        self.device_combo = QComboBox()
        self.connect_btn = QPushButton("Connect")
        self.disconnect_btn = QPushButton("Disconnect")
        self.disconnect_btn.setEnabled(False)

        top.addWidget(QLabel("Name filter:"))
        top.addWidget(self.filter_input, 2)
        top.addWidget(self.scan_btn)
        top.addWidget(self.device_combo, 3)
        top.addWidget(self.connect_btn)
        top.addWidget(self.disconnect_btn)
        layout.addLayout(top)

        info = QFormLayout()
        self.status_label = QLabel("Idle")
        self.device_state_label = QLabel("UNKNOWN")
        self.device_state_label.setStyleSheet("font-weight: 700;")
        self.session_label = QLabel("No active session")
        info.addRow("Status:", self.status_label)
        info.addRow("Device state:", self.device_state_label)
        info.addRow("Session file:", self.session_label)
        layout.addLayout(info)

        stats_row = QHBoxLayout()

        vitals_box = QGroupBox("Vitals")
        vitals_layout = QGridLayout(vitals_box)
        self.temp_label = QLabel("0.00 °F")
        self.hr_label = QLabel("-- bpm")
        self.spo2_label = QLabel("-- %")
        self.signal_label = QLabel("No signal")
        self.batt_label = QLabel("0.00 V (0%)")

        vitals_fields = [
            ("Temperature", self.temp_label),
            ("Heart Rate", self.hr_label),
            ("SpO2", self.spo2_label),
            ("Signal", self.signal_label),
            ("Battery", self.batt_label),
        ]
        for i, (name, value) in enumerate(vitals_fields):
            label = QLabel(name)
            value.setStyleSheet("font-size: 18px; font-weight: 700;")
            vitals_layout.addWidget(label, i, 0)
            vitals_layout.addWidget(value, i, 1)

        imu_box = QGroupBox("IMU Live")
        imu_layout = QGridLayout(imu_box)
        self.accel_x_label = QLabel("0.000 g")
        self.accel_y_label = QLabel("0.000 g")
        self.accel_z_label = QLabel("0.000 g")
        self.gyro_x_label = QLabel("0.00 °/s")
        self.gyro_y_label = QLabel("0.00 °/s")
        self.gyro_z_label = QLabel("0.00 °/s")

        imu_fields = [
            ("Accel X", self.accel_x_label),
            ("Accel Y", self.accel_y_label),
            ("Accel Z", self.accel_z_label),
            ("Gyro X", self.gyro_x_label),
            ("Gyro Y", self.gyro_y_label),
            ("Gyro Z", self.gyro_z_label),
        ]
        for i, (name, value) in enumerate(imu_fields):
            label = QLabel(name)
            value.setStyleSheet("font-size: 16px; font-weight: 700;")
            imu_layout.addWidget(label, i, 0)
            imu_layout.addWidget(value, i, 1)

        stats_row.addWidget(vitals_box, 1)
        stats_row.addWidget(imu_box, 1)
        layout.addLayout(stats_row)

        fig = Figure(figsize=(11, 6), dpi=100)
        self.ax_ir = fig.add_subplot(211)
        self.ax_red = fig.add_subplot(212)

        self.ir_line, = self.ax_ir.plot([], [], label="IR")
        self.ax_ir.set_title("IR Signal")
        self.ax_ir.legend(loc="upper right")

        self.red_line, = self.ax_red.plot([], [], label="Red", color="tab:red")
        self.ax_red.set_title("Red Signal")
        self.ax_red.legend(loc="upper right")

        self.canvas = FigureCanvas(fig)
        layout.addWidget(self.canvas)

    def _wire_signals(self):
        self.scan_btn.clicked.connect(self.on_scan)
        self.connect_btn.clicked.connect(self.on_connect)
        self.disconnect_btn.clicked.connect(self.on_disconnect)
        self.device_combo.currentIndexChanged.connect(self.on_device_selected)

        self.worker.scan_done.connect(self.on_scan_done)
        self.worker.connect_done.connect(self.on_connect_done)
        self.worker.disconnect_done.connect(self.on_disconnect_done)
        self.worker.data_update.connect(self.on_data_update)
        self.worker.status_update.connect(self.status_label.setText)
        self.worker.log.connect(self.append_log)
        self.worker.error.connect(self.on_error)

    def on_scan(self):
        self.device_combo.clear()
        self.devices = []
        self.selected_address = None
        self.device_state_label.setText("OFF")
        self.append_log("Scan requested")
        self.worker.scan(self.filter_input.text())

    def on_scan_done(self, devices):
        self.devices = devices
        self.device_combo.clear()

        if not devices:
            self.status_label.setText("No matching devices found (likely OFF / out of range)")
            self.device_state_label.setText("OFF")
            return

        for d in devices:
            self.device_combo.addItem(f"{d['name']} | {d['address']}", d["address"])

        self.status_label.setText(f"Found {len(devices)} matching device(s)")
        self.device_state_label.setText("ON")
        self.selected_address = devices[0]["address"]

    def on_device_selected(self, index):
        if index < 0:
            self.selected_address = None
            self.device_state_label.setText("OFF")
            return
        self.selected_address = self.device_combo.itemData(index)
        self.device_state_label.setText("ON")

    def on_connect(self):
        if not self.selected_address:
            QMessageBox.warning(self, "No device", "Scan and select a device first.")
            return
        self.latest_payload = None
        self.connect_btn.setEnabled(False)
        self.status_label.setText("Connecting...")
        self.append_log(f"Connect requested: {self.selected_address}")
        self.worker.connect(self.selected_address)

    def on_connect_done(self, ok, message):
        self.connect_btn.setEnabled(True)
        if ok:
            self.disconnect_btn.setEnabled(True)
            self.device_state_label.setText("ON")
            self.status_label.setText("Connected")
            self.session_label.setText(message)
        else:
            self.disconnect_btn.setEnabled(False)
            self.status_label.setText("Connect failed")
            self.device_state_label.setText("OFF")
            detail = (
                f"{message}\n\n"
                "If your board switch is OFF or sleeping, turn it ON and scan again."
            )
            QMessageBox.critical(self, "Connection failed", detail)

    def on_disconnect(self):
        self.append_log("Manual disconnect requested")
        self.worker.disconnect("Disconnected")

    def on_disconnect_done(self, reason):
        self.disconnect_btn.setEnabled(False)
        self.session_label.setText("No active session")
        self.device_state_label.setText("OFF")
        self.status_label.setText(reason)
        self.latest_payload = None

    def on_error(self, msg):
        self.status_label.setText(msg)
        self.append_log(f"Error: {msg}")

    def append_log(self, message):
        print(message, flush=True)
        self.log_file.write(message + "\n")
        self.log_file.flush()

    def on_data_update(self, payload):
        self.latest_payload = payload

    def _render_latest_payload(self):
        if self.is_closing:
            return

        payload = self.latest_payload
        if payload is None:
            return

        values = payload["values"]
        temp_f = (values["Temp_C"] * 9.0 / 5.0) + 32.0
        self.temp_label.setText(f"{temp_f:.2f} °F")
        self.batt_label.setText(f"{values['Battery_V']:.2f} V ({values['Battery_%']:.0f}%)")
        self.hr_label.setText("-- bpm" if values["HR"] <= 0 else f"{values['HR']:.1f} bpm")
        self.spo2_label.setText("-- %" if values["SpO2"] <= 0 else f"{values['SpO2']:.1f} %")
        self.signal_label.setText("Good" if values["Signal_Valid"] else "Poor/No contact")
        self.accel_x_label.setText(f"{values['Accel_X']:+.3f} g")
        self.accel_y_label.setText(f"{values['Accel_Y']:+.3f} g")
        self.accel_z_label.setText(f"{values['Accel_Z']:+.3f} g")
        self.gyro_x_label.setText(f"{values['Gyro_X']:+.2f} °/s")
        self.gyro_y_label.setText(f"{values['Gyro_Y']:+.2f} °/s")
        self.gyro_z_label.setText(f"{values['Gyro_Z']:+.2f} °/s")

        ir = payload["ir"]
        red = payload["red"]
        x_ir = np.arange(len(ir))
        x_red = np.arange(len(red))
        self.ir_line.set_data(x_ir, ir)
        self.ax_ir.set_xlim(0, max(120, len(ir)))
        if len(ir) > 3:
            ir_low = ir.min()
            ir_high = ir.max()
            ir_margin = max(10.0, (ir_high - ir_low) * 0.05)
            self.ax_ir.set_ylim(ir_low - ir_margin, ir_high + ir_margin)

        self.red_line.set_data(x_red, red)
        self.ax_red.set_xlim(0, max(120, len(red)))
        if len(red) > 3:
            red_low = red.min()
            red_high = red.max()
            red_margin = max(10.0, (red_high - red_low) * 0.05)
            self.ax_red.set_ylim(red_low - red_margin, red_high + red_margin)

        self.canvas.draw_idle()

    def closeEvent(self, event):
        self.is_closing = True
        self.plot_timer.stop()
        self.append_log("[GUI] App closing")
        self.worker.disconnect("App closing")
        if self.log_file:
            self.log_file.close()
        event.accept()


def main():
    app = QApplication([])
    window = MainWindow()
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
