import asyncio
import csv
import json
import math
import os
import re
import sys
import time
from collections import deque
from datetime import datetime

import numpy as np

# Prefer bundled pyqtgraph source (./pyqtgraph/pyqtgraph) over namespace stub.
APP_DIR = os.path.dirname(os.path.abspath(__file__))
VENDORED_PG_PARENT = os.path.join(APP_DIR, "pyqtgraph")
if os.path.isdir(os.path.join(VENDORED_PG_PARENT, "pyqtgraph")) and VENDORED_PG_PARENT not in sys.path:
    sys.path.insert(0, VENDORED_PG_PARENT)

import pyqtgraph as pg
import qasync
from bleak import BleakClient, BleakScanner
from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMenuBar,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSplitter,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)
from bluetooth_dialog import BluetoothDialog

MAX_PLOTS_PER_WINDOW = 10_000
TARGET_PROCESSING_HZ = 500
BLE_BASE_SUFFIX = "-0000-1000-8000-00805f9b34fb"
BLE_PARAM_UUID_CANONICAL = f"0000bfee{BLE_BASE_SUFFIX}"
EEG_CH_SHORT_RE = re.compile(r"^(eef[1-8]|eff[1-8])$")
EEG_CFG_SHORT_RE = re.compile(r"^(eeff|efff)$")
IMU_STREAM_SHORT_RE = re.compile(r"^1cf[1-4]$")
IMU_CFG_SHORT_RE = re.compile(r"^1cff$")
PPG_STREAM_SHORT_RE = re.compile(r"^aec[1-2]$")
BLE_PARAM_SHORT_RE = re.compile(r"^bfee$")
AS7341_DATA_SHORT_RE = re.compile(r"^(1526|1527)$")
AS7341_CFG_SHORT_RE = re.compile(r"^1525$")

ICM_ACCEL_SAMPLE_BYTES = 6
ICM_GYRO_SAMPLE_BYTES = 6
ICM_TEMP_SAMPLE_BYTES = 2
ICM_FULL_SAMPLE_BYTES = 14  # accel(6) + gyro(6) + temp(2), int16 big-endian per axis
PPG_SAMPLE_BYTES = 3
PPG_FIXED_SAMPLE_RATE_HZ = 100.0
PPG_FIXED_DT_S = 1.0 / PPG_FIXED_SAMPLE_RATE_HZ
PPG_FILTER_HP_HZ = 0.5
PPG_FILTER_LP_HZ = 8.0
PPG_FILTER_NOTCH_HZ = 0.0
PPG_FILTER_Q = 30.0
PPG_FILTER_SMOOTH_LEN = 3
PPG_FILTER_SPIKE_K = 6.0
PPG_FILTER_SPIKE_FLOOR = 120.0
PPG_HR_MIN_BPM = 30.0
PPG_HR_MAX_BPM = 220.0
IMU_SCALE_PROFILES = {
    # UUID 0x1CF1 (IMU1): +/-4 g, +/-500 dps
    1: {
        "accel_lsb_per_g": 8192.0,
        "gyro_lsb_per_dps": 65.5,
        "accel_range_g": 4.0,
        "gyro_range_dps": 500.0,
    },
    # UUID 0x1CF2 (IMU2): +/-2 g, +/-250 dps
    2: {
        "accel_lsb_per_g": 16384.0,
        "gyro_lsb_per_dps": 131.0,
        "accel_range_g": 2.0,
        "gyro_range_dps": 250.0,
    },
}
IMU_DEFAULT_PROFILE_INDEX = 1
AUTO_RECONNECT_INTERVAL_S = 6.0
METRICS_SCAN_INTERVAL_S = 3.0
SAMPLE_RATE_AVG_WINDOW_S = 5
IMU_RECEIVER_RATE_AVG_WINDOW = 30
LAST_DEVICE_STATE_PATH = os.path.join(APP_DIR, "data", "last_ble_device.json")
PERIPHERAL_TX_POWER_CFG_DBM = 4
AS7341_CFG_DEFAULT_RED_PERCENT = 20
AS7341_CFG_DEFAULT_INTEGRATION_20MS = 1
AS7341_CFG_DEFAULT_IR_PERCENT = 20
AS7341_CFG_DEFAULT_LED_LOCATION_S = 1


def is_ignored_uuid(uuid: str) -> bool:
    return False


def _extract_ble_short_uuid(uuid: str) -> str:
    u = uuid.lower()
    if u.startswith("0000") and u.endswith(BLE_BASE_SUFFIX) and len(u) >= 8:
        return u[4:8]
    m = re.fullmatch(
        r"([0-9a-f]{8})-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}",
        u,
    )
    if m:
        # Accept vendor-specific 128-bit UUIDs by matching on the low 16 bits
        # of the first 32-bit field, e.g. 228baec1 -> aec1.
        return m.group(1)[-4:]
    if len(u) == 4 and re.fullmatch(r"[0-9a-f]{4}", u):
        return u
    return ""


def _extract_eeg_channel_short_uuid(uuid: str) -> str:
    short = _extract_ble_short_uuid(uuid)
    if EEG_CH_SHORT_RE.fullmatch(short):
        return short
    return ""


def is_eeg_config_uuid(uuid: str) -> bool:
    short = _extract_ble_short_uuid(uuid)
    return bool(EEG_CFG_SHORT_RE.fullmatch(short))


def is_eefx_uuid(uuid: str) -> bool:
    return bool(_extract_eeg_channel_short_uuid(uuid))


def is_imu_stream_uuid(uuid: str) -> bool:
    short = _extract_ble_short_uuid(uuid)
    return bool(IMU_STREAM_SHORT_RE.fullmatch(short))


def is_imu_config_uuid(uuid: str) -> bool:
    short = _extract_ble_short_uuid(uuid)
    return bool(IMU_CFG_SHORT_RE.fullmatch(short))

def is_ble_param_uuid(uuid: str) -> bool:
    u = str(uuid or "").strip().lower()
    if u == BLE_PARAM_UUID_CANONICAL:
        return True
    if re.fullmatch(r"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}", u):
        if u.split("-", 1)[0].endswith("bfee"):
            return True
    short = _extract_ble_short_uuid(uuid)
    return bool(BLE_PARAM_SHORT_RE.fullmatch(short))


def is_ppg_stream_uuid(uuid: str) -> bool:
    short = _extract_ble_short_uuid(uuid)
    return bool(PPG_STREAM_SHORT_RE.fullmatch(short))


def is_as7341_data_uuid(uuid: str) -> bool:
    short = _extract_ble_short_uuid(uuid)
    return bool(AS7341_DATA_SHORT_RE.fullmatch(short))


def is_as7341_config_uuid(uuid: str) -> bool:
    short = _extract_ble_short_uuid(uuid)
    return bool(AS7341_CFG_SHORT_RE.fullmatch(short))


def imu_short_uuid(uuid: str) -> str:
    return _extract_ble_short_uuid(uuid)


def imu_device_index_from_uuid(uuid: str) -> int:
    short = imu_short_uuid(uuid)
    if not short:
        return 0
    try:
        v = int(short, 16)
    except Exception:
        return 0
    base = int("1cf1", 16)
    if v < base:
        return 0
    return (v - base) + 1


def imu_scale_profile_from_uuid(uuid: str):
    idx = imu_device_index_from_uuid(uuid)
    return IMU_SCALE_PROFILES.get(idx, IMU_SCALE_PROFILES[IMU_DEFAULT_PROFILE_INDEX])


def short_uuid_id(uuid: str) -> str:
    short = _extract_ble_short_uuid(uuid)
    if short:
        return short.upper()
    return uuid


def decode_payload_values(data: bytes, uuid: str, vref: float = 4.5, gain: float = 24.0):
    """Decode BLE payload into a numeric sequence for plotting.

    Priority:
    - 24-bit signed samples (common for ADS/biopotential) -> microvolts
    - 16-bit signed samples
    - byte values fallback
    """
    if not data:
        return []

    if is_eefx_uuid(uuid):
        # EEFx characteristics are 24-bit signed samples.
        lsb = vref / (gain * (2**23))
        out = []
        usable = len(data) - (len(data) % 3)
        for i in range(0, usable, 3):
            val = int.from_bytes(data[i:i + 3], byteorder="big", signed=False)
            if val & 0x800000:
                val -= 1 << 24
            out.append(val * lsb * 1e6)
        if len(data) % 3 != 0:
            print(f"[DEBUG] Non-3-byte tail ignored for {uuid}: {len(data) % 3} byte(s)")
        return out

    if len(data) % 2 == 0:
        return [
            float(int.from_bytes(data[i:i + 2], byteorder="little", signed=True))
            for i in range(0, len(data), 2)
        ]

    return [float(b) for b in data]


def estimate_ppg_bpm(ir_values, sample_rate_hz: float):
    if sample_rate_hz <= 0 or len(ir_values) < int(6.0 * sample_rate_hz):
        return None

    arr = np.fromiter(ir_values, dtype=np.float64)
    recent_len = min(arr.size, int(10.0 * sample_rate_hz))
    arr = arr[-recent_len:]

    baseline_win = max(3, int(sample_rate_hz * 1.0))
    baseline = np.convolve(
        arr,
        np.ones(baseline_win, dtype=np.float64) / float(baseline_win),
        mode="same",
    )
    ac = arr - baseline

    smooth_win = max(1, int(0.08 * sample_rate_hz))
    if smooth_win > 1:
        ac = np.convolve(
            ac,
            np.ones(smooth_win, dtype=np.float64) / float(smooth_win),
            mode="same",
        )

    ac -= float(np.mean(ac))
    if abs(float(np.percentile(ac, 5))) > abs(float(np.percentile(ac, 95))):
        ac = -ac
    signal_std = float(np.std(ac))
    if signal_std < 1e-6:
        return None

    autocorr = np.correlate(ac, ac, mode="full")[ac.size - 1:]
    energy = float(autocorr[0]) if autocorr.size else 0.0
    if energy <= 0.0:
        return None

    min_lag = max(1, int(sample_rate_hz * 60.0 / PPG_HR_MAX_BPM))
    max_lag = min(autocorr.size - 1, int(sample_rate_hz * 60.0 / PPG_HR_MIN_BPM))
    if max_lag <= min_lag:
        return None

    search = autocorr[min_lag:max_lag + 1]
    if search.size < 3:
        return None
    lag = int(np.argmax(search)) + min_lag
    peak_strength = float(autocorr[lag]) / energy
    if peak_strength < 0.20:
        return None

    bpm = 60.0 * float(sample_rate_hz) / float(lag)
    if bpm < PPG_HR_MIN_BPM or bpm > PPG_HR_MAX_BPM:
        return None
    return bpm


class PlotPageWindow(QMainWindow):
    def __init__(self, page_number: int):
        super().__init__()
        self.page_number = page_number
        self.setWindowTitle(f"BLE UUID Visualizer - Window {page_number}")
        self.setGeometry(120 + 80 * page_number, 120 + 40 * page_number, 1200, 820)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.plot_host = QWidget()
        self.grid = QGridLayout(self.plot_host)
        self.grid.setContentsMargins(4, 4, 4, 4)
        self.grid.setHorizontalSpacing(6)
        self.grid.setVerticalSpacing(6)
        self.scroll.setWidget(self.plot_host)
        layout.addWidget(self.scroll)


class ClickablePlotWidget(pg.PlotWidget):
    clicked = pyqtSignal()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(event)


class PoppedOutPlotWindow(QMainWindow):
    def __init__(self, uuid: str):
        super().__init__()
        self.uuid = uuid
        self.setWindowTitle(f"Popped Out Plot - {uuid}")
        self.setGeometry(180, 160, 1050, 680)
        self.plot = pg.PlotWidget()
        self.plot.setBackground("white")
        self.plot.setStyleSheet("border: 1px solid #000000;")
        self.plot.showGrid(x=True, y=True)
        self.curve = self.plot.plot(pen=pg.mkPen("#0044AA", width=2.0))
        self.setCentralWidget(self.plot)


class IMUStatusWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Live View (ICM20649)")
        self.setGeometry(180, 120, 520, 320)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.config_label = QLabel("Config: waiting for IMU stream")
        self.config_label.setStyleSheet("color: #111111; font-weight: bold;")
        layout.addWidget(self.config_label)

        self.data_line_1 = QLabel(
            "Accel [g]    X: --   Y: --   Z: --    |    Gyro [deg/s] X: --   Y: --   Z: --"
        )
        self.data_line_2 = QLabel("Angles [deg] Roll: --   Pitch: --    |    Temp [raw]: --")

        for w in (self.data_line_1, self.data_line_2):
            w.setStyleSheet("color: #222222; font-family: Menlo, Consolas, monospace;")
            layout.addWidget(w)

    def update_values(self, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_raw, device_label="IMU"):
        roll = math.degrees(math.atan2(ay_g, az_g))
        pitch = math.degrees(math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g)))
        self.config_label.setText("Config: accel ±4 g, gyro ±1000 deg/s")
        self.accel_label.setText(f"Accel [g]    X: {ax_g:+.3f}   Y: {ay_g:+.3f}   Z: {az_g:+.3f}")
        self.gyro_label.setText(f"Gyro [deg/s] X: {gx_dps:+.2f}   Y: {gy_dps:+.2f}   Z: {gz_dps:+.2f}")
        self.angles_label.setText(f"Angles [deg] Roll: {roll:+.2f}   Pitch: {pitch:+.2f}")
        self.temp_label.setText(f"Temp [raw]: {temp_raw}")


class BLEDiagnosticsWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BLE Diagnostics")
        self.setGeometry(220, 160, 760, 360)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        info = QLabel("RSSI Real-Time Dot Plot (dBm)")
        info.setStyleSheet("color: #111111; font-weight: bold;")
        layout.addWidget(info)
        self.signal_label = QLabel("Signal Strength: --")
        self.signal_label.setStyleSheet("color: #0A5F38; font-weight: bold;")
        layout.addWidget(self.signal_label)
        self.bw_label = QLabel("BLE BW: -- kbps | Mode: --")
        self.bw_label.setStyleSheet("color: #1E3A8A; font-weight: bold;")
        layout.addWidget(self.bw_label)

        controls = QHBoxLayout()
        self.x_zoom_in_btn = QPushButton("X Zoom In")
        self.x_zoom_out_btn = QPushButton("X Zoom Out")
        self.x_zoom_reset_btn = QPushButton("X Reset")
        self.x_zoom_in_btn.clicked.connect(self.zoom_x_in)
        self.x_zoom_out_btn.clicked.connect(self.zoom_x_out)
        self.x_zoom_reset_btn.clicked.connect(self.zoom_x_reset)
        controls.addWidget(self.x_zoom_in_btn)
        controls.addWidget(self.x_zoom_out_btn)
        controls.addWidget(self.x_zoom_reset_btn)
        controls.addStretch()
        layout.addLayout(controls)

        self.plot = pg.PlotWidget()
        self.plot.setBackground("white")
        self.plot.setStyleSheet("border: 1px solid #000000;")
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel("left", "RSSI (dBm)")
        self.plot.setLabel("bottom", "Time (s)")
        self.plot.setYRange(-110, -30)
        self.curve = self.plot.plot(
            [],
            [],
            pen=pg.mkPen("#0078D7", width=1.6),
            symbol="o",
            symbolSize=6,
            symbolBrush=pg.mkBrush("#0078D7"),
            symbolPen=pg.mkPen("#0078D7"),
        )
        layout.addWidget(self.plot)

        tx_info = QLabel("Antenna/Tx Power (dBm)")
        tx_info.setStyleSheet("color: #111111; font-weight: bold;")
        layout.addWidget(tx_info)
        self.tx_plot = pg.PlotWidget()
        self.tx_plot.setBackground("white")
        self.tx_plot.setStyleSheet("border: 1px solid #000000;")
        self.tx_plot.showGrid(x=True, y=True)
        self.tx_plot.setLabel("left", "Tx Power (dBm)")
        self.tx_plot.setLabel("bottom", "Time (s)")
        self.tx_plot.setYRange(-40, 20)
        self.tx_curve = self.tx_plot.plot(
            [],
            [],
            pen=pg.mkPen("#AA5500", width=1.6),
            symbol="o",
            symbolSize=5,
            symbolBrush=pg.mkBrush("#CC6600"),
            symbolPen=pg.mkPen("#CC6600"),
        )
        layout.addWidget(self.tx_plot)

        self._t0 = None
        self._times = deque(maxlen=1200)
        self._values = deque(maxlen=1200)
        self._tx_times = deque(maxlen=1200)
        self._tx_values = deque(maxlen=1200)
        self._x_window_s = 60.0
        self._x_window_min_s = 5.0
        self._x_window_max_s = 600.0

    @staticmethod
    def _quality_from_rssi(rssi_dbm):
        if rssi_dbm is None:
            return "--"
        if rssi_dbm >= -60:
            return "Excellent"
        if rssi_dbm >= -70:
            return "Good"
        if rssi_dbm >= -80:
            return "Fair"
        return "Weak"

    def add_ble_sample(self, rssi_dbm: int = None, tx_power_dbm: int = None, t_mono: float = None):
        if rssi_dbm is None and tx_power_dbm is None:
            return
        if t_mono is None:
            t_mono = time.monotonic()
        if self._t0 is None:
            self._t0 = t_mono
        x = float(t_mono - self._t0)
        if rssi_dbm is not None:
            self._times.append(x)
            self._values.append(float(rssi_dbm))
            self.curve.setData(list(self._times), list(self._values))
            self.signal_label.setText(
                f"Signal Strength: {self._quality_from_rssi(int(rssi_dbm))} ({int(rssi_dbm)} dBm)"
            )
        if tx_power_dbm is not None:
            self._tx_times.append(x)
            self._tx_values.append(float(tx_power_dbm))
            self.tx_curve.setData(list(self._tx_times), list(self._tx_values))

        x_end = 0.0
        if self._times:
            x_end = max(x_end, self._times[-1])
        if self._tx_times:
            x_end = max(x_end, self._tx_times[-1])
        if x_end > 0.0:
            x_start = max(0.0, x_end - self._x_window_s)
            self.plot.setXRange(x_start, max(self._x_window_s, x_end), padding=0.0)
            self.tx_plot.setXRange(x_start, max(self._x_window_s, x_end), padding=0.0)

    def set_ble_bandwidth(self, ble_bps: float = None, mode_kbps: int = None):
        if ble_bps is None or ble_bps < 0:
            self.bw_label.setText("BLE BW: -- kbps | Mode: --")
            return
        kbps = ble_bps / 1000.0
        if mode_kbps in (1000, 2000):
            self.bw_label.setText(f"BLE BW: {kbps:.1f} kbps | Mode: {mode_kbps}")
        else:
            self.bw_label.setText(f"BLE BW: {kbps:.1f} kbps | Mode: --")

    def zoom_x_in(self):
        self._x_window_s = max(self._x_window_min_s, self._x_window_s * 0.7)

    def zoom_x_out(self):
        self._x_window_s = min(self._x_window_max_s, self._x_window_s * 1.4)

    def zoom_x_reset(self):
        self._x_window_s = 60.0


class IMUStatusPanel(QWidget):
    def __init__(self):
        super().__init__()
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(6, 4, 6, 4)
        self._layout.setSpacing(4)

        self.config_label = QLabel("IMU Config: waiting for IMU stream")
        self.config_label.setStyleSheet("color: #111111; font-weight: bold;")
        self._layout.addWidget(self.config_label)
        self._ensure_compact_lines()

    def _ensure_compact_lines(self):
        if hasattr(self, "data_line_1") and hasattr(self, "data_line_2"):
            return
        self.data_line_1 = QLabel(
            "Accel [g]    X: --   Y: --   Z: --    |    Gyro [deg/s] X: --   Y: --   Z: --"
        )
        self.data_line_2 = QLabel("Angles [deg] Roll: --   Pitch: --    |    Temp [raw]: --")
        for w in (self.data_line_1, self.data_line_2):
            w.setStyleSheet("color: #222222; font-family: Menlo, Consolas, monospace;")
            self._layout.addWidget(w)

    def update_values(
        self,
        ax_g,
        ay_g,
        az_g,
        gx_dps,
        gy_dps,
        gz_dps,
        temp_raw,
        device_label="IMU",
        accel_range_g=None,
        gyro_range_dps=None,
    ):
        self._ensure_compact_lines()
        roll = math.degrees(math.atan2(ay_g, az_g))
        pitch = math.degrees(math.atan2(-ax_g, math.sqrt(ay_g * ay_g + az_g * az_g)))
        if accel_range_g is not None and gyro_range_dps is not None:
            self.config_label.setText(
                f"{device_label}: accel +/-{accel_range_g:g} g, gyro +/-{gyro_range_dps:g} deg/s"
            )
        else:
            self.config_label.setText(f"{device_label}: accel/gyro config unknown")
        self.data_line_1.setText(
            f"Accel [g]    X: {ax_g:+.3f}   Y: {ay_g:+.3f}   Z: {az_g:+.3f}    |    "
            f"Gyro [deg/s] X: {gx_dps:+.2f}   Y: {gy_dps:+.2f}   Z: {gz_dps:+.2f}"
        )
        self.data_line_2.setText(
            f"Angles [deg] Roll: {roll:+.2f}   Pitch: {pitch:+.2f}    |    Temp [raw]: {temp_raw}"
        )

    def reset(self):
        self._ensure_compact_lines()
        self.config_label.setText("IMU Config: waiting for IMU stream")
        self.data_line_1.setText(
            "Accel [g]    X: --   Y: --   Z: --    |    Gyro [deg/s] X: --   Y: --   Z: --"
        )
        self.data_line_2.setText("Angles [deg] Roll: --   Pitch: --    |    Temp [raw]: --")


class AS7341StatusPanel(QWidget):
    def __init__(self):
        super().__init__()
        self._layout = QVBoxLayout(self)
        self._layout.setContentsMargins(6, 4, 6, 4)
        self._layout.setSpacing(4)

        self.config_label = QLabel("AS7341 Spectral: waiting for stream (UUID 0x1526/0x1527)")
        self.config_label.setStyleSheet("color: #111111; font-weight: bold;")
        self._layout.addWidget(self.config_label)

        self.latest_label = QLabel("Latest: RED630 -- | RED680 -- | NIR -- | LED --")
        self.latest_label.setStyleSheet("color: #222222; font-family: Menlo, Consolas, monospace;")
        self._layout.addWidget(self.latest_label)

        self.raw_plot = pg.PlotWidget()
        self.raw_plot.setBackground("white")
        self.raw_plot.showGrid(x=True, y=True)
        self.raw_plot.setLabel("left", "Raw Counts")
        self.raw_plot.setLabel("bottom", "Time (s)")
        self.raw_plot.addLegend(offset=(8, 8))
        self.raw_curve_630 = self.raw_plot.plot(name="RED630", pen=pg.mkPen("#C81D25", width=2.0))
        self.raw_curve_680 = self.raw_plot.plot(name="RED680", pen=pg.mkPen("#F18F01", width=2.0))
        self.raw_curve_nir = self.raw_plot.plot(name="NIR", pen=pg.mkPen("#1F6FEB", width=2.0))
        self._layout.addWidget(self.raw_plot)

        self.spec_plot = pg.PlotWidget()
        self.spec_plot.setBackground("white")
        self.spec_plot.showGrid(x=True, y=True)
        self.spec_plot.setLabel("left", "Spectral Index")
        self.spec_plot.setLabel("bottom", "Time (s)")
        self.spec_plot.addLegend(offset=(8, 8))
        self.spec_curve_nir_630 = self.spec_plot.plot(name="NIR/RED630", pen=pg.mkPen("#8A2BE2", width=2.0))
        self.spec_curve_nir_680 = self.spec_plot.plot(name="NIR/RED680", pen=pg.mkPen("#2A9D8F", width=2.0))
        self.spec_curve_ndi_680 = self.spec_plot.plot(name="NDI(680)", pen=pg.mkPen("#495057", width=2.0))
        self._layout.addWidget(self.spec_plot)

        self._window_s = 30.0
        self._t0 = None
        self._times = deque(maxlen=2400)
        self._red630 = deque(maxlen=2400)
        self._red680 = deque(maxlen=2400)
        self._nir = deque(maxlen=2400)
        self._nir_red630 = deque(maxlen=2400)
        self._nir_red680 = deque(maxlen=2400)
        self._ndi_680 = deque(maxlen=2400)

    def update_values(self, red_630, red_680, nir, led_idx, nir_red630, nir_red680, ndi_680):
        self.config_label.setText("AS7341 Spectral: receiving data")
        self.latest_label.setText(
            f"Latest: RED630 {int(red_630)} | RED680 {int(red_680)} | NIR {int(nir)} | LED {int(led_idx)}"
        )

        t_mono = time.monotonic()
        if self._t0 is None:
            self._t0 = t_mono
        t = float(t_mono - self._t0)

        self._times.append(t)
        self._red630.append(float(red_630))
        self._red680.append(float(red_680))
        self._nir.append(float(nir))
        self._nir_red630.append(float(nir_red630))
        self._nir_red680.append(float(nir_red680))
        self._ndi_680.append(float(ndi_680))

        x = list(self._times)
        self.raw_curve_630.setData(x, list(self._red630))
        self.raw_curve_680.setData(x, list(self._red680))
        self.raw_curve_nir.setData(x, list(self._nir))
        self.spec_curve_nir_630.setData(x, list(self._nir_red630))
        self.spec_curve_nir_680.setData(x, list(self._nir_red680))
        self.spec_curve_ndi_680.setData(x, list(self._ndi_680))

        if x:
            x_end = x[-1]
            x_start = max(0.0, x_end - self._window_s)
            self.raw_plot.setXRange(x_start, max(self._window_s, x_end), padding=0.0)
            self.spec_plot.setXRange(x_start, max(self._window_s, x_end), padding=0.0)

    def reset(self):
        self.config_label.setText("AS7341 Spectral: waiting for stream (UUID 0x1526/0x1527)")
        self.latest_label.setText("Latest: RED630 -- | RED680 -- | NIR -- | LED --")
        self._t0 = None
        self._times.clear()
        self._red630.clear()
        self._red680.clear()
        self._nir.clear()
        self._nir_red630.clear()
        self._nir_red680.clear()
        self._ndi_680.clear()
        self.raw_curve_630.setData([], [])
        self.raw_curve_680.setData([], [])
        self.raw_curve_nir.setData([], [])
        self.spec_curve_nir_630.setData([], [])
        self.spec_curve_nir_680.setData([], [])
        self.spec_curve_ndi_680.setData([], [])


class AS7341SpectralWindow(QMainWindow):
    def __init__(self, stream_uuid: str, on_close=None):
        super().__init__()
        self.stream_uuid = str(stream_uuid)
        self._on_close = on_close
        self.setWindowTitle(f"AS7341 Spectral Graphs - {self.stream_uuid}")
        self.setGeometry(220, 160, 1100, 760)
        self.panel = AS7341StatusPanel()
        self.setCentralWidget(self.panel)

    def closeEvent(self, event):
        if callable(self._on_close):
            try:
                self._on_close(self.stream_uuid)
            except Exception:
                pass
        super().closeEvent(event)


class IMUDevicePlotWindow(QMainWindow):
    def __init__(self, device_label: str, uuid_key: str):
        super().__init__()
        self.device_label = device_label
        self.uuid_key = uuid_key
        self.short_uuid = short_uuid_id(uuid_key)
        self._freq_text = "Freq: -- Hz"
        self._t0_monotonic = None
        self._last_rx_monotonic = None
        self._last_rate_rx_monotonic = None
        self._avg_group_size_high_rate = 1
        self._avg_enable_hz = 100.0
        self._averaging_enabled = True
        self._rate_hz_ema = 0.0
        self._packet_rate_hz_history = deque(maxlen=IMU_RECEIVER_RATE_AVG_WINDOW)
        self._x_window_s = 12.0
        self._render_dirty = False

        self._t = deque(maxlen=2400)
        self._ax = deque(maxlen=2400)
        self._ay = deque(maxlen=2400)
        self._az = deque(maxlen=2400)
        self._gx = deque(maxlen=2400)
        self._gy = deque(maxlen=2400)
        self._gz = deque(maxlen=2400)
        self._pending_t = []
        self._pending_ax = []
        self._pending_ay = []
        self._pending_az = []
        self._pending_gx = []
        self._pending_gy = []
        self._pending_gz = []

        self.setWindowTitle(f"{device_label} Live Graph ({self.short_uuid}) | {self._freq_text}")
        self.setGeometry(260, 140, 980, 640)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.header_label = QLabel("")
        self.header_label.setStyleSheet("color: #111111; font-weight: bold;")
        layout.addWidget(self.header_label)
        self._update_header_label()

        self.avg_checkbox = QCheckBox("Enable IMU Averaging")
        self.avg_checkbox.setChecked(self._averaging_enabled)
        self.avg_checkbox.setToolTip("Average grouped IMU samples before plotting.")
        self.avg_checkbox.toggled.connect(self._on_averaging_toggled)
        layout.addWidget(self.avg_checkbox)

        self.accel_plot = pg.PlotWidget()
        self.accel_plot.setBackground("white")
        self.accel_plot.setStyleSheet("border: 1px solid #000000;")
        self.accel_plot.showGrid(x=True, y=True)
        self.accel_plot.setLabel("left", "Accel (g)")
        self.accel_plot.setLabel("bottom", "Time (s)")
        self.accel_plot.getAxis("left").enableAutoSIPrefix(False)
        self.accel_plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.accel_plot.addLegend(offset=(8, 8))
        self.accel_curve_x = self.accel_plot.plot(
            [], [], pen=None, symbol="o", symbolSize=4,
            symbolBrush=pg.mkBrush("#D11A2A"), symbolPen=pg.mkPen("#D11A2A"), name="Ax"
        )
        self.accel_curve_y = self.accel_plot.plot(
            [], [], pen=None, symbol="o", symbolSize=4,
            symbolBrush=pg.mkBrush("#0A8A2A"), symbolPen=pg.mkPen("#0A8A2A"), name="Ay"
        )
        self.accel_curve_z = self.accel_plot.plot(
            [], [], pen=None, symbol="o", symbolSize=4,
            symbolBrush=pg.mkBrush("#1E4DD8"), symbolPen=pg.mkPen("#1E4DD8"), name="Az"
        )
        layout.addWidget(self.accel_plot)

        self.gyro_plot = pg.PlotWidget()
        self.gyro_plot.setBackground("white")
        self.gyro_plot.setStyleSheet("border: 1px solid #000000;")
        self.gyro_plot.showGrid(x=True, y=True)
        self.gyro_plot.setLabel("left", "Gyro (deg/s)")
        self.gyro_plot.setLabel("bottom", "Time (s)")
        self.gyro_plot.getAxis("left").enableAutoSIPrefix(False)
        self.gyro_plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.gyro_plot.addLegend(offset=(8, 8))
        self.gyro_curve_x = self.gyro_plot.plot(
            [], [], pen=None, symbol="o", symbolSize=4,
            symbolBrush=pg.mkBrush("#D11A2A"), symbolPen=pg.mkPen("#D11A2A"), name="Gx"
        )
        self.gyro_curve_y = self.gyro_plot.plot(
            [], [], pen=None, symbol="o", symbolSize=4,
            symbolBrush=pg.mkBrush("#0A8A2A"), symbolPen=pg.mkPen("#0A8A2A"), name="Gy"
        )
        self.gyro_curve_z = self.gyro_plot.plot(
            [], [], pen=None, symbol="o", symbolSize=4,
            symbolBrush=pg.mkBrush("#1E4DD8"), symbolPen=pg.mkPen("#1E4DD8"), name="Gz"
        )
        layout.addWidget(self.gyro_plot)

        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._render_curves)
        self._render_timer.start(50)

    def _update_header_label(self):
        imu_profile = imu_scale_profile_from_uuid(self.uuid_key)
        self.header_label.setText(
            f"{self.device_label}  |  UUID {self.short_uuid}  |  "
            f"Accel +/-{imu_profile['accel_range_g']:g} g, "
            f"Gyro +/-{imu_profile['gyro_range_dps']:g} deg/s  |  "
            f"{self._freq_text}"
        )
        self.setWindowTitle(f"{self.device_label} Live Graph ({self.short_uuid}) | {self._freq_text}")

    def set_receiver_frequency_hz(self, avg_frequency_hz):
        if avg_frequency_hz is None:
            self._freq_text = "Freq: -- Hz"
        else:
            self._freq_text = f"Freq: {avg_frequency_hz:.1f} Hz"
        self._update_header_label()

    def _drain_pending_samples(self, group_size: int):
        k = max(1, int(group_size))
        while len(self._pending_t) >= k:
            self._t.append(float(sum(self._pending_t[:k]) / k))
            self._ax.append(float(sum(self._pending_ax[:k]) / k))
            self._ay.append(float(sum(self._pending_ay[:k]) / k))
            self._az.append(float(sum(self._pending_az[:k]) / k))
            self._gx.append(float(sum(self._pending_gx[:k]) / k))
            self._gy.append(float(sum(self._pending_gy[:k]) / k))
            self._gz.append(float(sum(self._pending_gz[:k]) / k))
            del self._pending_t[:k]
            del self._pending_ax[:k]
            del self._pending_ay[:k]
            del self._pending_az[:k]
            del self._pending_gx[:k]
            del self._pending_gy[:k]
            del self._pending_gz[:k]

    def _on_averaging_toggled(self, checked: bool):
        self._averaging_enabled = bool(checked)
        if not self._averaging_enabled:
            self._drain_pending_samples(1)
            self._render_dirty = True
        print(f"[DEBUG] IMU averaging for {self.short_uuid}: {'ON' if self._averaging_enabled else 'OFF'}")

    def update_receiver_rate(self, decoded_sample_count: int, rx_monotonic_s: float):
        rx_t = float(rx_monotonic_s)
        prev_rx = self._last_rate_rx_monotonic
        self._last_rate_rx_monotonic = rx_t
        if prev_rx is None or decoded_sample_count <= 0 or rx_t <= prev_rx:
            return

        dt = rx_t - prev_rx
        inst_rate_hz = float(decoded_sample_count) / dt
        self._packet_rate_hz_history.append(inst_rate_hz)
        avg_rate_hz = float(sum(self._packet_rate_hz_history) / len(self._packet_rate_hz_history))
        self._rate_hz_ema = avg_rate_hz
        self.set_receiver_frequency_hz(avg_rate_hz)

    def add_samples(self, samples, rx_monotonic_s: float):
        if not samples:
            return

        rx_t = float(rx_monotonic_s)
        if self._t0_monotonic is None:
            self._t0_monotonic = rx_t

        n = len(samples)
        sample_times = [rx_t] * n
        prev_rx = self._last_rx_monotonic
        if prev_rx is not None and rx_t > prev_rx and n > 0:
            dt = rx_t - prev_rx
            step = dt / float(n)
            sample_times = [prev_rx + (step * (i + 1)) for i in range(n)]
        self._last_rx_monotonic = rx_t

        for (ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps), sample_t in zip(samples, sample_times):
            t = max(0.0, sample_t - self._t0_monotonic)
            self._pending_t.append(t)
            self._pending_ax.append(float(ax_g))
            self._pending_ay.append(float(ay_g))
            self._pending_az.append(float(az_g))
            self._pending_gx.append(float(gx_dps))
            self._pending_gy.append(float(gy_dps))
            self._pending_gz.append(float(gz_dps))

        if self._averaging_enabled and self._rate_hz_ema > self._avg_enable_hz:
            k = self._avg_group_size_high_rate
        else:
            k = 1
        self._drain_pending_samples(k)
        self._render_dirty = True

    def _render_curves(self):
        if not self._render_dirty or not self.isVisible():
            return
        t = list(self._t)
        if not t:
            return
        self.accel_curve_x.setData(t, list(self._ax))
        self.accel_curve_y.setData(t, list(self._ay))
        self.accel_curve_z.setData(t, list(self._az))
        self.gyro_curve_x.setData(t, list(self._gx))
        self.gyro_curve_y.setData(t, list(self._gy))
        self.gyro_curve_z.setData(t, list(self._gz))

        x_end = t[-1]
        x_start = max(0.0, x_end - self._x_window_s)
        self.accel_plot.setXRange(x_start, max(self._x_window_s, x_end), padding=0.0)
        self.gyro_plot.setXRange(x_start, max(self._x_window_s, x_end), padding=0.0)
        self._render_dirty = False

    def reset(self):
        self._t0_monotonic = None
        self._last_rx_monotonic = None
        self._last_rate_rx_monotonic = None
        self._rate_hz_ema = 0.0
        self._packet_rate_hz_history.clear()
        self.set_receiver_frequency_hz(None)
        for dq in (self._t, self._ax, self._ay, self._az, self._gx, self._gy, self._gz):
            dq.clear()
        self._pending_t.clear()
        self._pending_ax.clear()
        self._pending_ay.clear()
        self._pending_az.clear()
        self._pending_gx.clear()
        self._pending_gy.clear()
        self._pending_gz.clear()
        self._render_dirty = False
        self.accel_curve_x.setData([], [])
        self.accel_curve_y.setData([], [])
        self.accel_curve_z.setData([], [])
        self.gyro_curve_x.setData([], [])
        self.gyro_curve_y.setData([], [])
        self.gyro_curve_z.setData([], [])


class PPGPlotWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self._freq_text = f"Freq: {PPG_FIXED_SAMPLE_RATE_HZ:.0f} Hz/ch"
        self._last_rate_rx_monotonic = {"red": None, "ir": None}
        self._sample_rate_history = {
            "red": deque(maxlen=IMU_RECEIVER_RATE_AVG_WINDOW),
            "ir": deque(maxlen=IMU_RECEIVER_RATE_AVG_WINDOW),
        }
        self._estimated_sample_rate_hz = PPG_FIXED_SAMPLE_RATE_HZ
        self._x_window_s = 12.0
        self._render_dirty = False
        self._bpm = None
        self._last_bpm_update_t = 0.0
        self._sample_counts = {"red": 0, "ir": 0}
        self.filter_enabled = False
        self._filter_states = {}
        self._filter_coeff_cache = None

        self._t_red = deque(maxlen=2400)
        self._red = deque(maxlen=2400)
        self._red_filtered = deque(maxlen=2400)
        self._t_ir = deque(maxlen=2400)
        self._ir = deque(maxlen=2400)
        self._ir_filtered = deque(maxlen=2400)

        self.setWindowTitle(f"PPG Live View (MAX30102) | {self._freq_text}")
        self.setGeometry(300, 180, 980, 520)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        self.header_label = QLabel("")
        self.header_label.setStyleSheet("color: #111111; font-weight: bold;")
        layout.addWidget(self.header_label)
        self._update_header_label()

        self.bpm_label = QLabel("PPG BPM: --")
        self.bpm_label.setStyleSheet("color: #1565C0; font-weight: bold;")
        layout.addWidget(self.bpm_label)

        control_row = QHBoxLayout()
        self.filter_toggle = QCheckBox("Enable Filter")
        self.filter_toggle.setText("Enable PPG Filter")
        self.filter_toggle.setChecked(self.filter_enabled)
        self.filter_toggle.setToolTip("Check to display filtered PPG. Uncheck to display raw PPG.")
        self.filter_toggle.toggled.connect(self._on_filter_toggled)
        control_row.addWidget(self.filter_toggle)
        control_row.addStretch()
        layout.addLayout(control_row)

        self.splitter = QSplitter(Qt.Vertical)
        self.splitter.setChildrenCollapsible(False)

        self.red_plot = pg.PlotWidget()
        self.red_plot.setBackground("white")
        self.red_plot.setStyleSheet("border: 1px solid #000000;")
        self.red_plot.showGrid(x=True, y=True)
        self.red_plot.setLabel("left", "Red Counts")
        self.red_plot.setLabel("bottom", "Time (s)")
        self.red_plot.getAxis("left").enableAutoSIPrefix(False)
        self.red_plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.red_plot.setTitle("Red PPG")
        self.red_plot.enableAutoRange(axis=pg.ViewBox.XAxis, enable=False)
        self.red_plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)
        self.red_curve = self.red_plot.plot([], [], pen=pg.mkPen("#C62828", width=2.0))
        self.splitter.addWidget(self.red_plot)

        self.ir_plot = pg.PlotWidget()
        self.ir_plot.setBackground("white")
        self.ir_plot.setStyleSheet("border: 1px solid #000000;")
        self.ir_plot.showGrid(x=True, y=True)
        self.ir_plot.setLabel("left", "IR Counts")
        self.ir_plot.setLabel("bottom", "Time (s)")
        self.ir_plot.getAxis("left").enableAutoSIPrefix(False)
        self.ir_plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.ir_plot.setTitle("IR PPG")
        self.ir_plot.setXLink(self.red_plot)
        self.ir_plot.enableAutoRange(axis=pg.ViewBox.XAxis, enable=False)
        self.ir_plot.enableAutoRange(axis=pg.ViewBox.YAxis, enable=False)
        self.ir_curve = self.ir_plot.plot([], [], pen=pg.mkPen("#1565C0", width=2.0))
        self.splitter.addWidget(self.ir_plot)
        self.splitter.setStretchFactor(0, 1)
        self.splitter.setStretchFactor(1, 1)
        layout.addWidget(self.splitter, 1)

        self.latest_label = QLabel("Red: --    IR: --")
        self.latest_label.setStyleSheet("color: #222222; font-family: Menlo, Consolas, monospace;")
        layout.addWidget(self.latest_label)

        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._render_curves)
        self._render_timer.start(20)

    def _update_header_label(self):
        self.header_label.setText(f"MAX30102 PPG  |  Service UUID AEC0  |  {self._freq_text}")
        self.setWindowTitle(f"PPG Live View (MAX30102) | {self._freq_text}")

    def set_receiver_frequency_hz(self, avg_frequency_hz):
        if avg_frequency_hz is None:
            self._freq_text = "Freq: -- Hz"
        else:
            self._freq_text = f"Freq: {avg_frequency_hz:.1f} Hz/ch"
        self._update_header_label()

    def update_receiver_rate(self, channel_name: str, decoded_sample_count: int, rx_monotonic_s: float):
        if channel_name not in self._last_rate_rx_monotonic:
            return
        self._last_rate_rx_monotonic[channel_name] = float(rx_monotonic_s)
        self._estimated_sample_rate_hz = PPG_FIXED_SAMPLE_RATE_HZ
        self.set_receiver_frequency_hz(PPG_FIXED_SAMPLE_RATE_HZ)

    def add_samples(self, channel_name: str, values, rx_monotonic_s: float):
        if not values:
            return
        if channel_name == "red":
            target_t = self._t_red
            raw_target_v = self._red
            filtered_target_v = self._red_filtered
        else:
            target_t = self._t_ir
            raw_target_v = self._ir
            filtered_target_v = self._ir_filtered
        filtered_values = self._apply_ppg_realtime_filter(channel_name, values)
        sample_count = self._sample_counts[channel_name]
        for raw_value, filtered_value in zip(values, filtered_values):
            target_t.append(sample_count * PPG_FIXED_DT_S)
            raw_target_v.append(float(raw_value))
            filtered_target_v.append(float(filtered_value))
            sample_count += 1
        self._sample_counts[channel_name] = sample_count
        latest_red = self._red[-1] if self._red else None
        latest_ir = self._ir[-1] if self._ir else None
        red_text = "--" if latest_red is None else f"{int(latest_red)}"
        ir_text = "--" if latest_ir is None else f"{int(latest_ir)}"
        self.latest_label.setText(f"Red: {red_text}    IR: {ir_text}")
        self._render_dirty = True

    def _render_curves(self):
        if not self._render_dirty or not self.isVisible():
            return
        red_values = self._red_filtered if self.filter_enabled else self._red
        ir_values = self._ir_filtered if self.filter_enabled else self._ir
        if self._t_red:
            self.red_curve.setData(list(self._t_red), list(red_values))
        if self._t_ir:
            self.ir_curve.setData(list(self._t_ir), list(ir_values))
        all_t = []
        if self._t_red:
            all_t.append(self._t_red[-1])
        if self._t_ir:
            all_t.append(self._t_ir[-1])
        if not all_t:
            return
        x_end = max(all_t)
        x_start = max(0.0, x_end - self._x_window_s)
        x_stop = max(self._x_window_s, x_end)
        self.red_plot.setXRange(x_start, x_stop, padding=0.0)
        self.ir_plot.setXRange(x_start, x_stop, padding=0.0)
        if red_values:
            red_tail = np.fromiter(red_values, dtype=float)[-500:]
            red_lo = float(np.min(red_tail))
            red_hi = float(np.max(red_tail))
            red_margin = max((red_hi - red_lo) * 0.08, 500.0)
            self.red_plot.setYRange(red_lo - red_margin, red_hi + red_margin, padding=0.0)
        if ir_values:
            ir_tail = np.fromiter(ir_values, dtype=float)[-500:]
            ir_lo = float(np.min(ir_tail))
            ir_hi = float(np.max(ir_tail))
            ir_margin = max((ir_hi - ir_lo) * 0.08, 500.0)
            self.ir_plot.setYRange(ir_lo - ir_margin, ir_hi + ir_margin, padding=0.0)
        now = time.monotonic()
        if now - self._last_bpm_update_t >= 1.0:
            bpm_source = self._ir_filtered if self._ir_filtered else self._ir
            bpm = estimate_ppg_bpm(bpm_source, self._estimated_sample_rate_hz)
            self._bpm = bpm
            if bpm is None:
                self.bpm_label.setText("PPG BPM: --")
            else:
                self.bpm_label.setText(f"PPG BPM: {bpm:.1f}")
            self._last_bpm_update_t = now
        self._render_dirty = False

    def reset(self):
        self._last_rate_rx_monotonic = {"red": None, "ir": None}
        for history in self._sample_rate_history.values():
            history.clear()
        self._estimated_sample_rate_hz = PPG_FIXED_SAMPLE_RATE_HZ
        self._sample_counts = {"red": 0, "ir": 0}
        self._filter_states.clear()
        self._filter_coeff_cache = None
        self.set_receiver_frequency_hz(PPG_FIXED_SAMPLE_RATE_HZ)
        self._t_red.clear()
        self._red.clear()
        self._red_filtered.clear()
        self._t_ir.clear()
        self._ir.clear()
        self._ir_filtered.clear()
        self.red_curve.setData([], [])
        self.ir_curve.setData([], [])
        self.red_plot.setXRange(0.0, self._x_window_s, padding=0.0)
        self.ir_plot.setXRange(0.0, self._x_window_s, padding=0.0)
        self._bpm = None
        self._last_bpm_update_t = 0.0
        self.bpm_label.setText("PPG BPM: --")
        self.latest_label.setText("Red: --    IR: --")
        self._render_dirty = False

    def _on_filter_toggled(self, checked: bool):
        self.filter_enabled = bool(checked)
        self._render_dirty = True
        self._render_curves()

    def _get_ppg_filter_state(self, channel_name: str):
        state = self._filter_states.get(channel_name)
        if state is not None:
            return state
        state = {
            "hp_x_prev": 0.0,
            "hp_y_prev": 0.0,
            "lp_y_prev": 0.0,
            "notch_x1": 0.0,
            "notch_x2": 0.0,
            "notch_y1": 0.0,
            "notch_y2": 0.0,
            "smooth_buf": deque(maxlen=PPG_FILTER_SMOOTH_LEN),
            "smooth_sum": 0.0,
            "spike_prev": 0.0,
            "spike_dev": 0.0,
            "spike_init": False,
        }
        self._filter_states[channel_name] = state
        return state

    def _apply_ppg_realtime_filter(self, channel_name: str, values):
        if not values:
            return []
        fs = float(PPG_FIXED_SAMPLE_RATE_HZ)
        hp_hz = max(0.01, min(PPG_FILTER_HP_HZ, 0.45 * fs))
        lp_hz = max(hp_hz + 0.25, min(PPG_FILTER_LP_HZ, 0.49 * fs))
        notch_hz = float(PPG_FILTER_NOTCH_HZ)
        state = self._get_ppg_filter_state(channel_name)
        cache_key = (fs, hp_hz, lp_hz, notch_hz, float(PPG_FILTER_Q))
        if self._filter_coeff_cache and self._filter_coeff_cache.get("key") == cache_key:
            coeff = self._filter_coeff_cache
        else:
            dt = 1.0 / fs
            rc_hp = 1.0 / (2.0 * np.pi * hp_hz)
            alpha_hp = rc_hp / (rc_hp + dt)
            rc_lp = 1.0 / (2.0 * np.pi * lp_hz)
            alpha_lp = dt / (rc_lp + dt)
            use_notch = 1.0 <= notch_hz < (0.5 * fs - 1.0)
            coeff = {
                "key": cache_key,
                "alpha_hp": alpha_hp,
                "alpha_lp": alpha_lp,
                "use_notch": use_notch,
                "b0": 0.0,
                "b1": 0.0,
                "b2": 0.0,
                "a1": 0.0,
                "a2": 0.0,
            }
            if use_notch:
                w0 = 2.0 * np.pi * notch_hz / fs
                c = np.cos(w0)
                s = np.sin(w0)
                alpha = s / (2.0 * PPG_FILTER_Q)
                a0 = 1.0 + alpha
                coeff["b0"] = 1.0 / a0
                coeff["b1"] = (-2.0 * c) / a0
                coeff["b2"] = 1.0 / a0
                coeff["a1"] = (-2.0 * c) / a0
                coeff["a2"] = (1.0 - alpha) / a0
            self._filter_coeff_cache = coeff

        alpha_hp = coeff["alpha_hp"]
        alpha_lp = coeff["alpha_lp"]
        use_notch = coeff["use_notch"]
        b0 = coeff["b0"]
        b1 = coeff["b1"]
        b2 = coeff["b2"]
        a1 = coeff["a1"]
        a2 = coeff["a2"]

        out = []
        smooth_buf = state["smooth_buf"]
        smooth_sum = float(state.get("smooth_sum", 0.0))
        spike_prev = float(state["spike_prev"])
        spike_dev = float(state["spike_dev"])
        spike_init = bool(state["spike_init"])
        hp_x_prev = float(state["hp_x_prev"])
        hp_y_prev = float(state["hp_y_prev"])
        lp_y_prev = float(state["lp_y_prev"])
        notch_x1 = float(state["notch_x1"])
        notch_x2 = float(state["notch_x2"])
        notch_y1 = float(state["notch_y1"])
        notch_y2 = float(state["notch_y2"])
        for x in values:
            hp = alpha_hp * (hp_y_prev + float(x) - hp_x_prev)
            hp_x_prev = float(x)
            hp_y_prev = hp

            lp = lp_y_prev + alpha_lp * (hp - lp_y_prev)
            lp_y_prev = lp

            y = lp
            if use_notch:
                y = (
                    b0 * lp
                    + b1 * notch_x1
                    + b2 * notch_x2
                    - a1 * notch_y1
                    - a2 * notch_y2
                )
                notch_x2 = notch_x1
                notch_x1 = lp
                notch_y2 = notch_y1
                notch_y1 = y

            if len(smooth_buf) == smooth_buf.maxlen:
                smooth_sum -= smooth_buf[0]
            smooth_buf.append(y)
            smooth_sum += y
            y = float(smooth_sum / len(smooth_buf))

            if not spike_init:
                spike_prev = y
                spike_dev = 0.0
                spike_init = True
            delta = y - spike_prev
            abs_delta = abs(delta)
            spike_dev = 0.995 * spike_dev + 0.005 * abs_delta
            step_limit = max(PPG_FILTER_SPIKE_FLOOR, PPG_FILTER_SPIKE_K * spike_dev)
            if abs_delta > step_limit:
                y = spike_prev + np.sign(delta) * step_limit
            spike_prev = y

            out.append(float(y))

        state["hp_x_prev"] = hp_x_prev
        state["hp_y_prev"] = hp_y_prev
        state["lp_y_prev"] = lp_y_prev
        state["notch_x1"] = notch_x1
        state["notch_x2"] = notch_x2
        state["notch_y1"] = notch_y1
        state["notch_y2"] = notch_y2
        state["smooth_sum"] = smooth_sum
        state["spike_prev"] = spike_prev
        state["spike_dev"] = spike_dev
        state["spike_init"] = spike_init
        return out


class UUIDVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("BLE UUID Visualizer (Dynamic per UUID)")
        self.setGeometry(80, 80, 900, 900)
        self._async_loop = asyncio.get_event_loop()

        self.sample_rate = TARGET_PROCESSING_HZ
        self.display_seconds = 8
        self.display_size = self.sample_rate * self.display_seconds

        self.ble_client = None
        self.connected_device_name = ""
        self.connected_device_address = ""
        self.notify_uuids = []
        self._notify_started = False
        self._samples_in_last_second = 0
        self._bytes_in_last_second = 0
        self._packets_in_last_second = 0
        self.uuid_samples_in_last_second = {}
        self.plotting_enabled = False
        self.y_zoom_factor = 1.0
        self.x_zoom_factor = 1.0
        self._x_range_smoothing = 0.08
        self._y_range_smoothing = 0.08
        self.x_range_locked = False
        self.y_range_locked = False
        self.filter_enabled = True
        self.filter_hp_hz = 0.8
        self.filter_lp_hz = 22.0
        self.filter_notch_hz = 60.0
        self.filter_q = 30.0
        self.filter_smooth_len = 7
        self.filter_spike_k = 6.0
        self.filter_spike_floor = 600.0
        self.filter_states = {}
        self._filter_config_version = 0
        # Fixed half-range for ECG display (microvolts). Disables dynamic auto-amplitude scaling.
        self.ecg_display_half_range_uv = 5000.0
        self.dynamic_y_autoscale_enabled = False
        self.bpm_calculation_enabled = False

        self.plot_queues = {}
        self.buffers = {}
        self.current_y_range = {}
        self.current_x_view = {}
        self.plots = {}
        self.curves = {}
        self.plot_titles = {}
        self.total_samples = {}
        self.uuid_sample_rate_history = {}
        self.uuid_avg_samples_per_second = {}
        self.popout_windows = {}
        self.extra_plot_windows = []
        self.skipped_notify_uuids = {}
        self.imu_panel = IMUStatusPanel()
        self.imu_device_windows = {}
        self._imu_device_state = {}
        self.as7341_windows = {}
        self._as7341_sensor_uuids = set()
        self._as7341_data_seen_by_uuid = set()
        self._as7341_window_closed_by_user = set()
        self._as7341_window_programmatic_close = False
        self._as7341_last_sample = {}
        self._as7341_config_uuid = ""
        self._as7341_config = bytearray(
            (
                AS7341_CFG_DEFAULT_RED_PERCENT,
                AS7341_CFG_DEFAULT_INTEGRATION_20MS,
                AS7341_CFG_DEFAULT_IR_PERCENT,
                AS7341_CFG_DEFAULT_LED_LOCATION_S,
            )
        )
        self._as7341_led_dirty = False
        self._as7341_led_write_pending = False
        self.available_imu_uuids = []
        self.available_ppg_uuids = []
        self.available_eeg_uuids = []
        self.available_as7341_uuids = []
        self.ppg_window = None
        self._ppg_state = {"red": 0, "ir": 0}
        self.ble_diag_window = None

        self.csv_streams = {}
        self._csv_flush_interval_s = 1.0
        self._csv_writer_started = False
        self._csv_time_origin_monotonic = None
        self._ppg_csv_sample_counts = {"red": 0, "ir": 0}
        self._imu_csv_last_rx_monotonic = {}
        self.terminal_log_queue = deque()
        self.terminal_log_enabled = False
        self._terminal_logger_started = False
        self._shutting_down = False
        self.partial_payload_by_uuid = {}
        self._filter_coeff_cache = None
        self._ble_session_id = 0
        self._connect_in_progress = False
        self._manual_disconnect_requested = False
        self._auto_reconnect_enabled = True
        self._auto_reconnect_interval_s = AUTO_RECONNECT_INTERVAL_S
        self._auto_reconnect_task = None
        self._metrics_task = None
        self._last_device_name = ""
        self._last_device_address = ""
        self._last_rssi_dbm = None
        self._last_tx_power_dbm = None
        self._last_ble_bps = 0.0
        self._last_ble_mode_kbps = None
        self._rssi_from_nrf_uuid = False
        self._ble_param_rssi_uuid = ""
        self._last_notify_rx_monotonic = 0.0
        self._disconnect_miss_streak = 0
        self._bfee_read_debug_once = False

        self._time_axis = np.linspace(-self.display_seconds, 0, self.display_size)
        self._load_last_device_state()

        self._init_ui()
        if self._last_device_address:
            self.status_label.setText(
                f"Last device: {self._last_device_name or self._last_device_address}"
            )
        self._init_timers()
        pg.setConfigOptions(antialias=False, useOpenGL=False)

    def _init_ui(self):
        self.setStyleSheet("background-color: white;")
        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)

        menubar = QMenuBar()
        menubar.setStyleSheet(
            "QMenuBar { background: white; color: #111111; }"
            "QMenuBar::item { color: #111111; background: transparent; padding: 4px 8px; }"
            "QMenuBar::item:selected { background: #E8F0FE; }"
        )
        layout.setMenuBar(menubar)
        connect_menu = menubar.addMenu("Connect")
        connect_menu.addAction("Bluetooth").triggered.connect(self.connect_bluetooth)
        connect_menu.addAction("Show BLE Diagnostics").triggered.connect(self.show_ble_diagnostics)

        self.freq_label = QLabel("Sampling: -- samples/s")
        self.freq_label.setStyleSheet("color: #0044AA; font-weight: bold;")
        self.ble_speed_label = QLabel("BLE: -- B/s | -- pkt/s")
        self.ble_speed_label.setStyleSheet("color: #0A5F38; font-weight: bold;")
        self.rssi_label = QLabel("RSSI: -- dBm")
        self.tx_power_label = QLabel(f"Peripheral Tx (cfg): {PERIPHERAL_TX_POWER_CFG_DBM:+d} dBm")
        self.signal_quality_label = QLabel("Signal: --")
        self.conn_state_label = QLabel("BLE: Disconnected")
        self.conn_state_label.setStyleSheet("color: #9B1C1C; font-weight: bold;")
        self.status_label = QLabel("Not Connected")
        self.uuid_count_label = QLabel("UUIDs exposed: 0")
        self.uuid_list_label = QLabel("UUID list: -")
        self.uuid_list_label.setWordWrap(True)
        self.uuid_list_label.setStyleSheet("color: #333333;")
        self.connect_button = QPushButton("Connect Bluetooth")
        self.connect_button.setStyleSheet("color: #111111; background: #E8F0FE; padding: 6px 10px;")
        self.connect_button.clicked.connect(self.connect_bluetooth)
        self.plot_toggle_button = QPushButton("Start Plotting")
        self.plot_toggle_button.setEnabled(False)
        self.plot_toggle_button.setStyleSheet("color: #111111; background: #D8F3DC; padding: 6px 10px;")
        self.plot_toggle_button.clicked.connect(self.toggle_plotting)
        self.zoom_in_button = QPushButton("Y Zoom In")
        self.zoom_in_button.setStyleSheet("color: #111111; background: #FFF4E6; padding: 6px 10px;")
        self.zoom_in_button.clicked.connect(self.zoom_y_in)
        self.zoom_out_button = QPushButton("Y Zoom Out")
        self.zoom_out_button.setStyleSheet("color: #111111; background: #FFF4E6; padding: 6px 10px;")
        self.zoom_out_button.clicked.connect(self.zoom_y_out)
        self.zoom_reset_button = QPushButton("Y Reset")
        self.zoom_reset_button.setStyleSheet("color: #111111; background: #FFF4E6; padding: 6px 10px;")
        self.zoom_reset_button.clicked.connect(self.zoom_y_reset)
        self.dynamic_y_toggle = QCheckBox("Auto Y")
        self.dynamic_y_toggle.setChecked(self.dynamic_y_autoscale_enabled)
        self.dynamic_y_toggle.setToolTip("Enable dynamic Y-range readjust based on signal amplitude.")
        self.dynamic_y_toggle.toggled.connect(self._on_dynamic_y_toggled)
        self.bpm_toggle = QCheckBox("Calculate BPM")
        self.bpm_toggle.setChecked(self.bpm_calculation_enabled)
        self.bpm_toggle.setToolTip("Enable BPM estimation from recent signal peaks.")
        self.bpm_toggle.toggled.connect(self._on_bpm_toggled)
        self.x_zoom_in_button = QPushButton("X Zoom In")
        self.x_zoom_in_button.setStyleSheet("color: #111111; background: #EAF4FF; padding: 6px 10px;")
        self.x_zoom_in_button.clicked.connect(self.zoom_x_in)
        self.x_zoom_out_button = QPushButton("X Zoom Out")
        self.x_zoom_out_button.setStyleSheet("color: #111111; background: #EAF4FF; padding: 6px 10px;")
        self.x_zoom_out_button.clicked.connect(self.zoom_x_out)
        self.x_zoom_reset_button = QPushButton("X Reset")
        self.x_zoom_reset_button.setStyleSheet("color: #111111; background: #EAF4FF; padding: 6px 10px;")
        self.x_zoom_reset_button.clicked.connect(self.zoom_x_reset)
        self.filter_toggle = QCheckBox("Enable Filter")
        self.filter_toggle.setText("Enable ADS1299 Filter")
        self.filter_toggle.setChecked(self.filter_enabled)
        self.filter_toggle.setToolTip("Uncheck to display raw (unfiltered) readings.")
        self.filter_toggle.toggled.connect(self._on_filter_controls_changed)

        self.sensor_selector = QComboBox()
        self.sensor_selector.setEnabled(False)
        self.sensor_selector.setMinimumWidth(220)
        self.sensor_selector.currentIndexChanged.connect(self._open_selected_sensor_visualizer)
        self._set_sensor_selector_items([])

        self.hp_label = QLabel("HP")
        self.hp_spin = QDoubleSpinBox()
        self.hp_spin.setDecimals(2)
        self.hp_spin.setRange(0.05, 10.0)
        self.hp_spin.setSingleStep(0.05)
        self.hp_spin.setValue(self.filter_hp_hz)
        self.hp_spin.setSuffix(" Hz")
        self.hp_spin.valueChanged.connect(self._on_filter_controls_changed)

        self.lp_label = QLabel("LP")
        self.lp_spin = QDoubleSpinBox()
        self.lp_spin.setDecimals(1)
        self.lp_spin.setRange(5.0, 120.0)
        self.lp_spin.setSingleStep(1.0)
        self.lp_spin.setValue(self.filter_lp_hz)
        self.lp_spin.setSuffix(" Hz")
        self.lp_spin.valueChanged.connect(self._on_filter_controls_changed)

        self.notch_label = QLabel("Notch")
        self.notch_combo = QComboBox()
        self.notch_combo.addItem("Off", 0.0)
        self.notch_combo.addItem("50 Hz", 50.0)
        self.notch_combo.addItem("60 Hz", 60.0)
        self.notch_combo.setCurrentIndex(2)
        self.notch_combo.currentIndexChanged.connect(self._on_filter_controls_changed)

        self.disconnect_button = QPushButton("Disconnect")
        self.disconnect_button.hide()
        self.disconnect_button.clicked.connect(self.disconnect_bluetooth)

        self.as7341_led_status_label = QLabel("AS7341 LED intensity/PWM: unavailable")
        self.as7341_led_status_label.setStyleSheet("color: #555555;")
        self.as7341_red_spin = QSpinBox()
        self.as7341_red_spin.setRange(0, 100)
        self.as7341_red_spin.setSuffix(" %")
        self.as7341_red_spin.valueChanged.connect(self._on_as7341_led_control_changed)
        self.as7341_ir_spin = QSpinBox()
        self.as7341_ir_spin.setRange(0, 100)
        self.as7341_ir_spin.setSuffix(" %")
        self.as7341_ir_spin.valueChanged.connect(self._on_as7341_led_control_changed)
        self.as7341_led_apply_button = QPushButton("Apply LED")
        self.as7341_led_apply_button.clicked.connect(self._write_as7341_led_config_now)
        self.as7341_led_debounce_timer = QTimer(self)
        self.as7341_led_debounce_timer.setSingleShot(True)
        self.as7341_led_debounce_timer.setInterval(150)
        self.as7341_led_debounce_timer.timeout.connect(self._write_as7341_led_config_now)
        self._set_as7341_led_controls_enabled(False)

        control_bar = QHBoxLayout()
        control_bar.addWidget(self.connect_button)
        control_bar.addWidget(self.plot_toggle_button)
        control_bar.addWidget(self.zoom_in_button)
        control_bar.addWidget(self.zoom_out_button)
        control_bar.addWidget(self.zoom_reset_button)
        control_bar.addWidget(self.dynamic_y_toggle)
        control_bar.addWidget(self.bpm_toggle)
        control_bar.addWidget(self.x_zoom_in_button)
        control_bar.addWidget(self.x_zoom_out_button)
        control_bar.addWidget(self.x_zoom_reset_button)
        control_bar.addWidget(self.filter_toggle)
        control_bar.addWidget(self.hp_label)
        control_bar.addWidget(self.hp_spin)
        control_bar.addWidget(self.lp_label)
        control_bar.addWidget(self.lp_spin)
        control_bar.addWidget(self.notch_label)
        control_bar.addWidget(self.notch_combo)
        control_bar.addWidget(self.sensor_selector)
        control_bar.addStretch()
        layout.addLayout(control_bar)

        as7341_control_bar = QHBoxLayout()
        as7341_control_bar.addWidget(self.as7341_led_status_label)
        as7341_control_bar.addSpacing(10)
        as7341_control_bar.addWidget(QLabel("RED Intensity"))
        as7341_control_bar.addWidget(self.as7341_red_spin)
        as7341_control_bar.addWidget(QLabel("IR Intensity"))
        as7341_control_bar.addWidget(self.as7341_ir_spin)
        as7341_control_bar.addWidget(self.as7341_led_apply_button)
        as7341_control_bar.addStretch()
        layout.addLayout(as7341_control_bar)

        status_bar = QHBoxLayout()
        status_bar.addWidget(self.freq_label)
        status_bar.addWidget(self.ble_speed_label)
        status_bar.addWidget(self.rssi_label)
        status_bar.addWidget(self.tx_power_label)
        status_bar.addWidget(self.signal_quality_label)
        status_bar.addSpacing(18)
        status_bar.addWidget(self.uuid_count_label)
        status_bar.addStretch()
        status_bar.addWidget(self.conn_state_label)
        status_bar.addWidget(self.status_label)
        status_bar.addWidget(self.disconnect_button)
        layout.addLayout(status_bar)
        layout.addWidget(self.uuid_list_label)

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.plot_host = QWidget()
        self.grid = QGridLayout(self.plot_host)
        self.grid.setContentsMargins(4, 4, 4, 4)
        self.grid.setHorizontalSpacing(6)
        self.grid.setVerticalSpacing(6)
        self.scroll.setWidget(self.plot_host)
        layout.addWidget(self.scroll)

    def _init_timers(self):
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(33)

        self.freq_timer = QTimer()
        self.freq_timer.timeout.connect(self._flush_sampling_rate)
        self.freq_timer.start(1000)

        self.conn_watchdog_timer = QTimer()
        self.conn_watchdog_timer.timeout.connect(self._connection_watchdog_tick)
        self.conn_watchdog_timer.start(2000)

        QTimer.singleShot(0, self._start_background_tasks)

    def _clear_existing_plots(self):
        while self.grid.count():
            item = self.grid.takeAt(0)
            widget = item.widget()
            if widget is not None:
                widget.setParent(None)
                widget.deleteLater()
        for window in self.extra_plot_windows:
            window.close()
        self.extra_plot_windows.clear()
        for window in self.popout_windows.values():
            window.close()
        self.popout_windows.clear()

        self.plot_queues.clear()
        self.buffers.clear()
        self.current_y_range.clear()
        self.current_x_view.clear()
        self.filter_states.clear()
        self.plots.clear()
        self.curves.clear()
        self.plot_titles.clear()
        self.total_samples.clear()
        self.uuid_sample_rate_history.clear()
        self.uuid_avg_samples_per_second.clear()
        self.uuid_samples_in_last_second.clear()
        self.partial_payload_by_uuid.clear()

    def _set_as7341_led_controls_enabled(self, enabled: bool):
        enabled = bool(enabled)
        self.as7341_red_spin.setEnabled(enabled)
        self.as7341_ir_spin.setEnabled(enabled)
        self.as7341_led_apply_button.setEnabled(enabled)

    def _sync_as7341_led_controls_from_state(self):
        self.as7341_red_spin.blockSignals(True)
        self.as7341_ir_spin.blockSignals(True)
        try:
            self.as7341_red_spin.setValue(int(self._as7341_config[0]))
            self.as7341_ir_spin.setValue(int(self._as7341_config[2]))
        finally:
            self.as7341_red_spin.blockSignals(False)
            self.as7341_ir_spin.blockSignals(False)

    def _set_as7341_led_status(self, text: str, color: str = "#555555"):
        self.as7341_led_status_label.setText(text)
        self.as7341_led_status_label.setStyleSheet(f"color: {color};")

    def _on_as7341_led_control_changed(self, _value: int):
        self._as7341_config[0] = int(self.as7341_red_spin.value()) & 0xFF
        self._as7341_config[2] = int(self.as7341_ir_spin.value()) & 0xFF
        self._as7341_led_dirty = True
        if not self._as7341_config_uuid:
            self._set_as7341_led_status("AS7341 LED intensity/PWM: config UUID not available", "#9B1C1C")
            return
        if not (self.ble_client and getattr(self.ble_client, "is_connected", False)):
            self._set_as7341_led_status("AS7341 LED intensity/PWM: waiting for BLE connection", "#9B1C1C")
            return
        self._set_as7341_led_status(
            f"AS7341 LED intensity/PWM: queued RED {self._as7341_config[0]}% | IR {self._as7341_config[2]}%",
            "#8A6D1D",
        )
        self.as7341_led_debounce_timer.start()

    def _write_as7341_led_config_now(self):
        if self.as7341_led_debounce_timer.isActive():
            self.as7341_led_debounce_timer.stop()
        if self._shutting_down:
            return
        if not self._as7341_config_uuid:
            self._set_as7341_led_status("AS7341 LED intensity/PWM: config UUID not available", "#9B1C1C")
            return
        if not (self.ble_client and getattr(self.ble_client, "is_connected", False)):
            self._set_as7341_led_status("AS7341 LED intensity/PWM: not connected", "#9B1C1C")
            return
        asyncio.create_task(self._write_as7341_led_config())

    async def _write_as7341_led_config(self):
        if self._as7341_led_write_pending:
            return
        if not self._as7341_config_uuid or not self.ble_client:
            return
        self._as7341_led_write_pending = True
        try:
            while True:
                self._as7341_led_dirty = False
                payload = bytes(self._as7341_config[:4])
                try:
                    await self.ble_client.write_gatt_char(self._as7341_config_uuid, payload, response=True)
                    self._set_as7341_led_status(
                        f"AS7341 LED intensity/PWM: applied RED {payload[0]}% | IR {payload[2]}%",
                        "#0A5F38",
                    )
                except Exception as exc:
                    self._set_as7341_led_status(f"AS7341 LED intensity/PWM write failed: {exc}", "#9B1C1C")
                    break
                if not self._as7341_led_dirty:
                    break
        finally:
            self._as7341_led_write_pending = False

    async def _read_as7341_config(self):
        if not self._as7341_config_uuid or not self.ble_client:
            return
        try:
            payload = await self.ble_client.read_gatt_char(self._as7341_config_uuid)
        except Exception as exc:
            self._set_as7341_led_status(
                f"AS7341 LED intensity/PWM: using local defaults ({exc})",
                "#8A6D1D",
            )
            self._sync_as7341_led_controls_from_state()
            return

        if len(payload) >= 4:
            self._as7341_config = bytearray(payload[:4])
        self._sync_as7341_led_controls_from_state()
        self._set_as7341_led_status(
            f"AS7341 LED intensity/PWM: ready RED {self._as7341_config[0]}% | IR {self._as7341_config[2]}%",
            "#0A5F38",
        )

    def _build_plot_card(self, grid: QGridLayout, index_in_window: int, uuid: str):
        title = QLabel(f"Signal ({uuid})")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Segoe UI", 10, QFont.Bold))

        plot = ClickablePlotWidget()
        plot.setBackground("white")
        plot.setStyleSheet("border: 1px solid #000000;")
        plot.showGrid(x=True, y=True)
        curve = plot.plot(pen=pg.mkPen("#0044AA", width=2.0))
        plot.clicked.connect(lambda uid=uuid: self._open_plot_popout(uid))

        uuid_under = QLabel(uuid)
        uuid_under.setAlignment(Qt.AlignCenter)
        uuid_under.setFont(QFont("Segoe UI", 9))
        uuid_under.setStyleSheet("color: #333333;")

        box = QVBoxLayout()
        box.setContentsMargins(2, 2, 2, 2)
        box.setSpacing(2)
        box.addWidget(title)
        box.addWidget(plot)
        box.addWidget(uuid_under)
        w = QWidget()
        w.setLayout(box)

        columns = 4
        row = index_in_window // columns
        col = index_in_window % columns
        grid.addWidget(w, row, col)
        return plot, curve, title

    def _set_plot_title_text(self, uuid: str, bpm=None):
        avg_samples_per_second = self.uuid_avg_samples_per_second.get(uuid)
        if avg_samples_per_second is None:
            rate_text = "Rate: -- samples/s"
        else:
            rate_text = f"Rate: {avg_samples_per_second:.1f} samples/s"

        base_title = f"Signal ({uuid}) | {rate_text}"
        title = self.plot_titles.get(uuid)
        if title is not None:
            if self.bpm_calculation_enabled:
                if bpm is None:
                    title.setText(f"{base_title} | BPM: --")
                else:
                    title.setText(f"{base_title} | BPM: {bpm:.1f}")
            else:
                title.setText(base_title)

        popout = self.popout_windows.get(uuid)
        if popout is not None:
            if self.bpm_calculation_enabled:
                if bpm is None:
                    popout.setWindowTitle(f"Popped Out Plot - {uuid} | {rate_text} | BPM: --")
                else:
                    popout.setWindowTitle(f"Popped Out Plot - {uuid} | {rate_text} | BPM: {bpm:.1f}")
            else:
                popout.setWindowTitle(f"Popped Out Plot - {uuid} | {rate_text}")

    def _refresh_plot_titles(self):
        for uuid in self.plot_titles:
            self._set_plot_title_text(uuid)

    def _get_grid_for_page(self, page_idx: int):
        return self.grid

    def _open_plot_popout(self, uuid: str):
        if uuid not in self.plots:
            return
        window = self.popout_windows.get(uuid)
        if window is None:
            window = PoppedOutPlotWindow(uuid=uuid)
            self.popout_windows[uuid] = window
        self._set_plot_title_text(uuid)
        window.show()
        window.raise_()
        window.activateWindow()

    def _create_uuid_plots(self, uuids):
        self._clear_existing_plots()
        if not uuids:
            self.uuid_count_label.setText("UUIDs exposed: 0")
            self.uuid_list_label.setText("UUID list: -")
            return

        self.uuid_count_label.setText(f"UUIDs exposed: {len(uuids)}")
        self.uuid_list_label.setText("UUID list: " + ", ".join(uuids))

        for i, uuid in enumerate(uuids):
            page_idx = i // MAX_PLOTS_PER_WINDOW
            idx_in_window = i % MAX_PLOTS_PER_WINDOW
            grid = self._get_grid_for_page(page_idx)
            plot, curve, title = self._build_plot_card(grid, idx_in_window, uuid)

            self.plot_queues[uuid] = deque()
            self.buffers[uuid] = deque(maxlen=self.display_size)
            self.current_y_range[uuid] = self.ecg_display_half_range_uv
            self.current_x_view[uuid] = (0.0, float(self.display_seconds))
            self.plots[uuid] = plot
            self.curves[uuid] = curve
            self.plot_titles[uuid] = title
            self.total_samples[uuid] = 0
            self.uuid_sample_rate_history[uuid] = deque(maxlen=SAMPLE_RATE_AVG_WINDOW_S)
            self.uuid_avg_samples_per_second[uuid] = None
            self.uuid_samples_in_last_second[uuid] = 0

    def _estimate_bpm_from_recent_peaks(self, buf: np.ndarray):
        if buf.size < int(2.0 * self.sample_rate):
            return None

        centered = buf - float(np.median(buf))
        envelope = np.abs(centered)
        if envelope.size < 3:
            return None

        # Adaptive threshold from upper-tail energy with robust fallback.
        p92 = float(np.percentile(envelope, 92))
        mean = float(np.mean(envelope))
        std = float(np.std(envelope))
        threshold = max(p92, mean + 1.5 * std, 50.0)

        # Enforce physiologic refractory period (<= 200 BPM).
        min_peak_distance = max(1, int(0.30 * self.sample_rate))
        peaks = []
        last_kept = -min_peak_distance
        for i in range(1, envelope.size - 1):
            v = envelope[i]
            if v < threshold:
                continue
            if v < envelope[i - 1] or v < envelope[i + 1]:
                continue
            if i - last_kept < min_peak_distance:
                if peaks and v > envelope[peaks[-1]]:
                    peaks[-1] = i
                    last_kept = i
                continue
            peaks.append(i)
            last_kept = i

        if len(peaks) < 3:
            return None

        rr = np.diff(np.asarray(peaks, dtype=float)) / float(self.sample_rate)
        rr = rr[(rr >= 0.27) & (rr <= 2.0)]
        if rr.size < 2:
            return None

        bpm = 60.0 / float(np.median(rr))
        if bpm < 30.0 or bpm > 220.0:
            return None
        return bpm

    async def _discover_notify_uuids(self):
        if not self.ble_client:
            return []

        services = self.ble_client.services
        if not services:
            services = await self.ble_client.get_services()

        uuids = []
        discovered_chars = []
        self._ble_param_rssi_uuid = ""
        self._as7341_config_uuid = ""
        self.skipped_notify_uuids = {}
        for service in services:
            for char in service.characteristics:
                props = {str(p).lower() for p in char.properties}
                discovered_chars.append((service.uuid, char.uuid, sorted(props)))
                if (
                    not self._as7341_config_uuid
                    and is_as7341_config_uuid(char.uuid)
                    and ("read" in props or "write" in props or "write-without-response" in props)
                ):
                    self._as7341_config_uuid = char.uuid
                if (
                    not self._ble_param_rssi_uuid
                    and is_ble_param_uuid(char.uuid)
                    and ("read" in props or "notify" in props)
                ):
                    self._ble_param_rssi_uuid = char.uuid
                if "notify" not in props:
                    continue
                if is_ignored_uuid(char.uuid):
                    self.skipped_notify_uuids[char.uuid] = "ignored (contains 1CF4)"
                    print(f"[DEBUG] Ignoring UUID (requested): {char.uuid}")
                    continue
                if is_eeg_config_uuid(char.uuid):
                    self.skipped_notify_uuids[char.uuid] = "EEG config UUID"
                    print(f"[DEBUG] Skipping EEG config notify UUID: {char.uuid}")
                    continue
                if is_imu_config_uuid(char.uuid):
                    self.skipped_notify_uuids[char.uuid] = "IMU config UUID"
                    print(f"[DEBUG] Skipping IMU config notify UUID: {char.uuid}")
                    continue
                if is_as7341_config_uuid(char.uuid):
                    self.skipped_notify_uuids[char.uuid] = "AS7341 config UUID"
                    print(f"[DEBUG] Skipping AS7341 config notify UUID: {char.uuid}")
                    continue
                if not (
                    is_eefx_uuid(char.uuid)
                    or is_imu_stream_uuid(char.uuid)
                    or is_ppg_stream_uuid(char.uuid)
                    or is_ble_param_uuid(char.uuid)
                    or is_as7341_data_uuid(char.uuid)
                ):
                    self.skipped_notify_uuids[char.uuid] = "unsupported notify UUID"
                    print(f"[DEBUG] Skipping unsupported notify UUID: {char.uuid}")
                    continue
                if is_ble_param_uuid(char.uuid):
                    self._ble_param_rssi_uuid = char.uuid
                uuids.append(char.uuid)

        ordered = sorted(set(uuids), key=lambda x: x.lower())
        self._as7341_sensor_uuids = {u for u in ordered if is_as7341_data_uuid(u)}
        self._as7341_data_seen_by_uuid.clear()
        for stale_uuid in list(self.as7341_windows.keys()):
            if stale_uuid not in self._as7341_sensor_uuids:
                self._close_as7341_window(stale_uuid)
        if self._as7341_sensor_uuids:
            print("[DEBUG] AS7341 spectral UUIDs detected: " + ", ".join(sorted(self._as7341_sensor_uuids)))
        if self._as7341_config_uuid:
            print(f"[DEBUG] AS7341 config UUID detected: {self._as7341_config_uuid}")
        print(f"[DEBUG] Notify UUID count: {len(ordered)}")
        for uid in ordered:
            print(f"[DEBUG] Notify UUID: {uid}")
        if self._ble_param_rssi_uuid:
            print(f"[DEBUG] BLE RSSI UUID detected: {self._ble_param_rssi_uuid}")
        else:
            print("[DEBUG] BLE RSSI UUID (BFEE) not detected.")
            print("[DEBUG] Full discovered GATT characteristics:")
            for service_uuid, char_uuid, props in discovered_chars:
                print(f"  - svc={service_uuid} char={char_uuid} props={','.join(props)}")
            print("[DEBUG] If BFEE is missing above, central may be using cached GATT. Re-pair/clear cache and reconnect.")
        if self.skipped_notify_uuids:
            print("[DEBUG] Skipped notify UUIDs:")
            for uid, reason in self.skipped_notify_uuids.items():
                print(f"  - {uid}: {reason}")
        return ordered

    @staticmethod
    def _normalize_address(address):
        if not address:
            return ""
        return str(address).strip().lower()

    def _load_last_device_state(self):
        self._last_device_name = ""
        self._last_device_address = ""
        try:
            with open(LAST_DEVICE_STATE_PATH, "r", encoding="utf-8") as f:
                state = json.load(f)
            self._last_device_name = str(state.get("name", "") or "")
            self._last_device_address = str(state.get("address", "") or "")
        except Exception:
            pass

    def _save_last_device_state(self, name, address):
        if not address:
            return
        self._last_device_name = str(name or "")
        self._last_device_address = str(address or "")
        try:
            os.makedirs(os.path.dirname(LAST_DEVICE_STATE_PATH), exist_ok=True)
            with open(LAST_DEVICE_STATE_PATH, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "name": self._last_device_name,
                        "address": self._last_device_address,
                        "updated_unix_s": time.time(),
                    },
                    f,
                    indent=2,
                )
        except Exception as e:
            print(f"[DEBUG] Failed to save last BLE device state: {e}")

    def _set_connection_state(self, state, detail=None):
        palette = {
            "Disconnected": "#9B1C1C",
            "Connecting": "#A05A00",
            "Connected": "#0A5F38",
            "Reconnecting": "#A05A00",
        }
        color = palette.get(state, "#1E3A8A")
        self.conn_state_label.setText(f"BLE: {state}")
        self.conn_state_label.setStyleSheet(f"color: {color}; font-weight: bold;")
        if detail:
            self.status_label.setText(detail)

    def show_ble_diagnostics(self):
        if self.ble_diag_window is None:
            self.ble_diag_window = BLEDiagnosticsWindow()
            if self._bytes_in_last_second >= 0:
                self._update_ble_diag_bandwidth(self._bytes_in_last_second)
        self.ble_diag_window.show()
        self.ble_diag_window.raise_()
        self.ble_diag_window.activateWindow()

    def _imu_device_display_label(self, uuid_key: str) -> str:
        idx = imu_device_index_from_uuid(uuid_key)
        if idx > 0:
            return f"ICM Device {idx}"
        return f"IMU {short_uuid_id(uuid_key)}"

    def _ensure_imu_window(self, uuid_key: str):
        window = self.imu_device_windows.get(uuid_key)
        if window is None:
            label = self._imu_device_display_label(uuid_key)
            window = IMUDevicePlotWindow(device_label=label, uuid_key=uuid_key)
            self.imu_device_windows[uuid_key] = window
        return window

    def _register_imu_windows(self, imu_uuids):
        for uuid_key in sorted(set(imu_uuids), key=lambda x: x.lower()):
            self._ensure_imu_window(uuid_key)

    def _close_imu_windows(self):
        for window in self.imu_device_windows.values():
            try:
                window.close()
            except Exception:
                pass
        self.imu_device_windows.clear()

    def _ensure_ppg_window(self):
        if self.ppg_window is None:
            self.ppg_window = PPGPlotWindow()
        return self.ppg_window

    def _set_sensor_selector_items(self, items):
        self.sensor_selector.blockSignals(True)
        self.sensor_selector.clear()
        self.sensor_selector.addItem("Open Sensor Visualizer...", None)
        for label, payload in items:
            self.sensor_selector.addItem(label, payload)
        self.sensor_selector.setCurrentIndex(0)
        self.sensor_selector.setEnabled(bool(items))
        self.sensor_selector.blockSignals(False)

    def _refresh_sensor_selector(self):
        items = []
        if self.available_eeg_uuids:
            items.append(("ADS1299 / EEG Signals", ("eeg", "")))
        if self.available_ppg_uuids:
            items.append(("PPG (MAX30102)", ("ppg", "")))
        for uuid_key in sorted(set(self.available_as7341_uuids), key=lambda x: x.lower()):
            items.append((f"AS7341 Spectral ({short_uuid_id(uuid_key)})", ("as7341", uuid_key)))
        for uuid_key in sorted(set(self.available_imu_uuids), key=lambda x: x.lower()):
            items.append((f"{self._imu_device_display_label(uuid_key)} ({short_uuid_id(uuid_key)})", ("imu", uuid_key)))
        self._set_sensor_selector_items(items)

    def _open_selected_sensor_visualizer(self, index: int):
        if index <= 0:
            return
        payload = self.sensor_selector.itemData(index)
        if not payload:
            return
        kind, value = payload
        if kind == "eeg":
            self.show()
            self.raise_()
            self.activateWindow()
        elif kind == "ppg":
            window = self._ensure_ppg_window()
            window.show()
            window.raise_()
            window.activateWindow()
        elif kind == "as7341":
            self._as7341_window_closed_by_user.discard(value)
            window = self._ensure_as7341_window(value)
            if window is not None:
                window.show()
                window.raise_()
                window.activateWindow()
        elif kind == "imu":
            window = self._ensure_imu_window(value)
            window.show()
            window.raise_()
            window.activateWindow()
        self.sensor_selector.blockSignals(True)
        self.sensor_selector.setCurrentIndex(0)
        self.sensor_selector.blockSignals(False)

    def _close_ppg_window(self):
        if self.ppg_window is not None:
            try:
                self.ppg_window.close()
            except Exception:
                pass
            self.ppg_window = None

    def _record_ble_diag_point(self, rssi_dbm=None, tx_power_dbm=None):
        if rssi_dbm is None and tx_power_dbm is None:
            return
        if self.ble_diag_window is None:
            return
        rssi = None if rssi_dbm is None else int(rssi_dbm)
        txp = None if tx_power_dbm is None else int(tx_power_dbm)
        self.ble_diag_window.add_ble_sample(rssi, txp, time.monotonic())
        mode_kbps = self._ble_mode_kbps_from_tx_power(self._last_tx_power_dbm)
        self._last_ble_mode_kbps = mode_kbps
        self.ble_diag_window.set_ble_bandwidth(self._last_ble_bps, mode_kbps)

    @staticmethod
    def _ble_mode_kbps_from_tx_power(tx_power_dbm):
        if tx_power_dbm is None:
            return None
        return 2000 if int(tx_power_dbm) <= -8 else 1000

    def _update_ble_diag_bandwidth(self, bytes_per_second: int):
        if self.ble_diag_window is None:
            return
        ble_bps = max(0.0, float(bytes_per_second) * 8.0)
        self._last_ble_bps = ble_bps
        mode_kbps = self._ble_mode_kbps_from_tx_power(self._last_tx_power_dbm)
        self._last_ble_mode_kbps = mode_kbps
        self.ble_diag_window.set_ble_bandwidth(ble_bps, mode_kbps)

    def _set_client_disconnect_callback(self, client):
        if client is None:
            return
        try:
            if hasattr(client, "set_disconnected_callback"):
                client.set_disconnected_callback(self._on_ble_disconnected)
        except Exception as e:
            print(f"[DEBUG] Could not attach disconnect callback: {e}")

    def _start_background_tasks(self):
        if self._shutting_down:
            return
        if self._metrics_task is None or self._metrics_task.done():
            self._metrics_task = asyncio.create_task(self._metrics_poll_task())
        if self._last_device_address and self._auto_reconnect_enabled:
            self._start_auto_reconnect("Trying saved BLE device...")

    def _start_auto_reconnect(self, reason=""):
        if self._shutting_down or not self._auto_reconnect_enabled:
            return
        if self._manual_disconnect_requested:
            return
        if not self._last_device_address:
            return
        if self.ble_client and getattr(self.ble_client, "is_connected", False):
            return
        if self._auto_reconnect_task and not self._auto_reconnect_task.done():
            return
        self._set_connection_state("Reconnecting", reason or "Reconnecting to last device...")
        self._auto_reconnect_task = asyncio.create_task(self._auto_reconnect_loop())

    def _stop_auto_reconnect(self):
        current_task = asyncio.current_task()
        if (
            self._auto_reconnect_task
            and self._auto_reconnect_task is not current_task
            and not self._auto_reconnect_task.done()
        ):
            self._auto_reconnect_task.cancel()
        if self._auto_reconnect_task is not current_task:
            self._auto_reconnect_task = None

    def _connection_watchdog_tick(self):
        if self._shutting_down or self._connect_in_progress:
            return
        if self.ble_client and getattr(self.ble_client, "is_connected", False):
            self._disconnect_miss_streak = 0
            if self.conn_state_label.text() != "BLE: Connected":
                self._set_connection_state("Connected")
            return
        if self.ble_client and not getattr(self.ble_client, "is_connected", False):
            self._disconnect_miss_streak += 1
            # Require many repeated misses and a long idle gap to avoid false positives from transient backend states.
            if self._disconnect_miss_streak < 8:
                return
            if (time.monotonic() - self._last_notify_rx_monotonic) < 20.0:
                return
            asyncio.create_task(self._handle_unexpected_disconnect("Link monitor detected disconnect"))

    def _on_ble_disconnected(self, _client):
        if self._shutting_down:
            return
        try:
            self._async_loop.call_soon_threadsafe(
                lambda: asyncio.create_task(self._handle_unexpected_disconnect("Peripheral disconnected"))
            )
        except Exception as e:
            print(f"[DEBUG] Disconnected callback scheduling failed: {e}")

    async def _handle_unexpected_disconnect(self, reason):
        if self._shutting_down or self._manual_disconnect_requested:
            return
        if self._connect_in_progress:
            return
        if self.ble_client and getattr(self.ble_client, "is_connected", False):
            return

        self._invalidate_ble_session()
        old_client = self.ble_client
        old_uuids = list(self.notify_uuids)
        self._apply_disconnected_ui_state(reason or "Disconnected")
        self._reset_ble_runtime_state()
        await self._disconnect_client(old_client, old_uuids)
        self._start_auto_reconnect("Reconnecting to last paired device...")

    async def _auto_reconnect_loop(self):
        try:
            while not self._shutting_down:
                if self._manual_disconnect_requested or not self._auto_reconnect_enabled:
                    return
                if self._connect_in_progress:
                    await asyncio.sleep(1.0)
                    continue
                if self.ble_client and getattr(self.ble_client, "is_connected", False):
                    return
                if not self._last_device_address:
                    return

                target_name = self._last_device_name or self._last_device_address
                self._set_connection_state("Reconnecting", f"Reconnecting to {target_name} ...")
                client = None
                try:
                    client = BleakClient(
                        self._last_device_address,
                        disconnected_callback=self._on_ble_disconnected,
                    )
                    await client.connect(timeout=8.0)
                    if getattr(client, "is_connected", False):
                        await self._finish_connect(
                            target_name,
                            client,
                            self._last_device_address,
                            from_auto_reconnect=True,
                        )
                        return
                except Exception as e:
                    print(f"[DEBUG] Auto-reconnect attempt failed: {e}")
                finally:
                    if client is not None and not getattr(client, "is_connected", False):
                        try:
                            await client.disconnect()
                        except Exception:
                            pass

                await asyncio.sleep(self._auto_reconnect_interval_s)
        finally:
            self._auto_reconnect_task = None

    @staticmethod
    def _rssi_quality(rssi_dbm):
        if rssi_dbm is None:
            return "--"
        if rssi_dbm >= -60:
            return "Excellent"
        if rssi_dbm >= -70:
            return "Good"
        if rssi_dbm >= -80:
            return "Fair"
        return "Weak"

    async def _metrics_poll_task(self):
        while not self._shutting_down:
            try:
                await self._refresh_link_metrics()
            except Exception as e:
                print(f"[DEBUG] Metrics polling error: {e}")
            await asyncio.sleep(METRICS_SCAN_INTERVAL_S)

    async def _refresh_link_metrics(self):
        if self._connect_in_progress:
            return

        addr_target = self._normalize_address(self._last_device_address)
        rssi = None
        tx_power = None
        is_connected = bool(self.ble_client and getattr(self.ble_client, "is_connected", False))
        if (not is_connected) and (not addr_target):
            return

        # Primary source when firmware EXPOSE_BLE_PARAM is enabled.
        if is_connected and not self._ble_param_rssi_uuid:
            for candidate in (BLE_PARAM_UUID_CANONICAL, "bfee"):
                try:
                    raw = await self.ble_client.read_gatt_char(candidate)
                    if raw:
                        self._ble_param_rssi_uuid = candidate
                        self._rssi_from_nrf_uuid = True
                        rssi = int.from_bytes(raw[:1], byteorder="little", signed=True)
                        if len(raw) >= 2:
                            tx_power = int.from_bytes(raw[1:2], byteorder="little", signed=True)
                        break
                except Exception as e:
                    if not self._bfee_read_debug_once:
                        print(f"[DEBUG] BFEE read failed for candidate '{candidate}': {e}")

        if is_connected and self._ble_param_rssi_uuid:
            try:
                raw = await self.ble_client.read_gatt_char(self._ble_param_rssi_uuid)
                if raw:
                    self._rssi_from_nrf_uuid = True
                    rssi = int.from_bytes(raw[:1], byteorder="little", signed=True)
                    if len(raw) >= 2:
                        tx_power = int.from_bytes(raw[1:2], byteorder="little", signed=True)
            except Exception as e:
                if not self._bfee_read_debug_once:
                    print(f"[DEBUG] BFEE read failed for detected UUID '{self._ble_param_rssi_uuid}': {e}")

        if is_connected:
            self._bfee_read_debug_once = True

        # Avoid active scanning while connected; some adapters/OS stacks become unstable.
        if not is_connected:
            try:
                adv_map = await BleakScanner.discover(timeout=1.5, return_adv=True)
                for key, value in adv_map.items():
                    if isinstance(value, tuple) and len(value) >= 2:
                        device, adv = value[0], value[1]
                    else:
                        continue
                    device_addr = self._normalize_address(getattr(device, "address", key))
                    if device_addr != addr_target:
                        continue
                    rssi = getattr(device, "rssi", None)
                    tx_power = getattr(adv, "tx_power", None)
                    break
            except TypeError:
                # Older bleak versions may not support return_adv.
                try:
                    for device in await BleakScanner.discover(timeout=1.5):
                        if self._normalize_address(getattr(device, "address", "")) != addr_target:
                            continue
                        rssi = getattr(device, "rssi", None)
                        break
                except Exception:
                    pass
            except Exception:
                pass

        if rssi is None and is_connected:
            backend = getattr(self.ble_client, "_backend", None)
            for attr in ("rssi", "_rssi"):
                backend_rssi = getattr(backend, attr, None) if backend is not None else None
                if isinstance(backend_rssi, (int, float)):
                    rssi = int(backend_rssi)
                    break

        if (not self._rssi_from_nrf_uuid) and (rssi is not None):
            self._last_rssi_dbm = int(rssi)
        if tx_power is not None:
            self._last_tx_power_dbm = int(tx_power)

        if self._last_rssi_dbm is None:
            self.rssi_label.setText("RSSI: -- dBm")
            self.signal_quality_label.setText("Signal: --")
        else:
            src = " (nRF)" if self._rssi_from_nrf_uuid else ""
            self.rssi_label.setText(f"RSSI: {self._last_rssi_dbm} dBm{src}")
            self.signal_quality_label.setText(f"Signal: {self._rssi_quality(self._last_rssi_dbm)}")
            self._record_ble_diag_point(self._last_rssi_dbm, self._last_tx_power_dbm)

        if self._last_tx_power_dbm is None:
            self.tx_power_label.setText(f"Peripheral Tx (cfg): {PERIPHERAL_TX_POWER_CFG_DBM:+d} dBm")
        else:
            self.tx_power_label.setText(f"Peripheral Tx: {self._last_tx_power_dbm} dBm")

    def _invalidate_ble_session(self) -> int:
        self._ble_session_id += 1
        return self._ble_session_id

    def _apply_disconnected_ui_state(self, detail="Not Connected"):
        self._set_connection_state("Disconnected", detail)
        self.disconnect_button.hide()
        self.connect_button.setEnabled(True)
        self.plot_toggle_button.setEnabled(False)
        self.plot_toggle_button.setText("Start Plotting")
        self.plotting_enabled = False
        self.freq_label.setText("Sampling: -- samples/s")
        self.ble_speed_label.setText("BLE: -- B/s | -- pkt/s")
        self._last_ble_bps = 0.0
        self._last_ble_mode_kbps = None
        if self.ble_diag_window is not None:
            self.ble_diag_window.set_ble_bandwidth(None, None)

    async def _disconnect_client(self, client, uuids):
        if not client:
            return
        try:
            for uuid in uuids:
                try:
                    await client.stop_notify(uuid)
                except Exception:
                    pass
            await client.disconnect()
        except Exception as e:
            print(f"disconnect error: {e}")

    def _reset_ble_runtime_state(self):
        self._notify_started = False
        self.notify_uuids = []
        self.uuid_samples_in_last_second.clear()
        self.partial_payload_by_uuid.clear()
        self.filter_states.clear()
        self.available_imu_uuids = []
        self.available_ppg_uuids = []
        self.available_eeg_uuids = []
        self.available_as7341_uuids = []
        self._set_sensor_selector_items([])
        self._samples_in_last_second = 0
        self._bytes_in_last_second = 0
        self._packets_in_last_second = 0
        self._rssi_from_nrf_uuid = False
        self._disconnect_miss_streak = 0
        self._last_notify_rx_monotonic = 0.0
        self.ble_client = None
        self.connected_device_name = ""
        self.connected_device_address = ""
        self._imu_last_accel = (0.0, 0.0, 0.0)
        self._imu_last_gyro = (0.0, 0.0, 0.0)
        self._imu_last_temp = 0
        self._imu_device_state = {}
        self._ppg_state = {"red": 0, "ir": 0}
        self._as7341_last_sample = {}
        self._as7341_sensor_uuids.clear()
        self._as7341_data_seen_by_uuid.clear()
        self._as7341_window_closed_by_user.clear()
        self._as7341_config_uuid = ""
        self._as7341_config = bytearray(
            (
                AS7341_CFG_DEFAULT_RED_PERCENT,
                AS7341_CFG_DEFAULT_INTEGRATION_20MS,
                AS7341_CFG_DEFAULT_IR_PERCENT,
                AS7341_CFG_DEFAULT_LED_LOCATION_S,
            )
        )
        self._as7341_led_dirty = False
        self._as7341_led_write_pending = False
        if hasattr(self, "as7341_led_debounce_timer"):
            self.as7341_led_debounce_timer.stop()
        self._set_as7341_led_controls_enabled(False)
        self._sync_as7341_led_controls_from_state()
        self._set_as7341_led_status("AS7341 LED intensity/PWM: unavailable", "#555555")
        self.imu_panel.reset()
        self._close_imu_windows()
        self._close_ppg_window()
        self._close_all_as7341_windows()
        self._clear_existing_plots()
        self._stop_csv_logging()

    def connect_bluetooth(self, *_args):
        if self._connect_in_progress or self._shutting_down:
            return
        self._manual_disconnect_requested = False
        self._stop_auto_reconnect()
        self._set_connection_state("Connecting", "Open scan dialog and select a BLE device.")
        dialog = BluetoothDialog()
        result = dialog.exec_()
        if result == dialog.Accepted and dialog.connected_device_name:
            asyncio.create_task(
                self._finish_connect(
                    dialog.connected_device_name,
                    dialog.get_client(),
                    dialog.get_selected_address() or "",
                )
            )
            return
        if not (self.ble_client and getattr(self.ble_client, "is_connected", False)):
            self._set_connection_state("Disconnected", "Connection failed")
        self.connect_button.setEnabled(True)

    async def _finish_connect(
        self,
        device_name: str,
        new_client,
        device_address: str = "",
        from_auto_reconnect: bool = False,
    ):
        if self._connect_in_progress or self._shutting_down:
            return
        self._connect_in_progress = True
        self.connect_button.setEnabled(False)
        self._manual_disconnect_requested = False
        if not from_auto_reconnect:
            self._stop_auto_reconnect()
        try:
            # Ensure any previous BLE session is fully torn down before creating a new one.
            self._invalidate_ble_session()
            old_client = self.ble_client
            old_uuids = list(self.notify_uuids)
            self._reset_ble_runtime_state()
            await self._disconnect_client(old_client, old_uuids)

            self._set_client_disconnect_callback(new_client)
            self.connected_device_name = str(device_name or "")
            self.connected_device_address = str(
                device_address
                or getattr(new_client, "address", "")
                or self._last_device_address
            )
            if not self.connected_device_name:
                self.connected_device_name = self.connected_device_address or "BLE Device"
            if self.connected_device_address:
                self._save_last_device_state(
                    self.connected_device_name or self.connected_device_address,
                    self.connected_device_address,
                )
            self._set_connection_state("Connected", f"Connected: {self.connected_device_name}")
            self.disconnect_button.show()
            self.connect_button.setEnabled(False)
            self.plot_toggle_button.setEnabled(True)
            self.plotting_enabled = False
            self.plot_toggle_button.setText("Start Plotting")
            self.ble_client = new_client
            self._bytes_in_last_second = 0
            self._packets_in_last_second = 0
            self._disconnect_miss_streak = 0
            self._last_notify_rx_monotonic = time.monotonic()
            session_id = self._invalidate_ble_session()
            if not self._terminal_logger_started:
                self._terminal_logger_started = True
                asyncio.create_task(self._terminal_logger_task())
            if not self._csv_writer_started:
                self._csv_writer_started = True
                asyncio.create_task(self._csv_writer_task())
            asyncio.create_task(self.start_notify_for_all_uuids(session_id))
        finally:
            self._connect_in_progress = False

    def toggle_plotting(self):
        self.plotting_enabled = not self.plotting_enabled
        if self.plotting_enabled:
            self._start_csv_logging()
            self.plot_toggle_button.setText("Stop Plotting")
            self.filter_states.clear()
            for uuid in self.plot_queues:
                self.plot_queues[uuid].clear()
                self.buffers[uuid] = deque(maxlen=self.display_size)
                self.total_samples[uuid] = 0
                self.current_x_view[uuid] = (0.0, float(self.display_seconds))
            print("[DEBUG] Plotting enabled.")
        else:
            self._stop_csv_logging()
            self.plot_toggle_button.setText("Start Plotting")
            print("[DEBUG] Plotting disabled (stream still active).")

    def zoom_y_in(self):
        self.y_range_locked = True
        for plot in self.plots.values():
            y0, y1 = plot.viewRange()[1]
            center = 0.5 * (y0 + y1)
            half = max(1e-6, 0.5 * (y1 - y0) * 0.8)
            plot.setYRange(center - half, center + half, padding=0.0)
        print("[DEBUG] Y range locked (zoom in).")

    def zoom_y_out(self):
        self.y_range_locked = True
        for plot in self.plots.values():
            y0, y1 = plot.viewRange()[1]
            center = 0.5 * (y0 + y1)
            half = max(1e-6, 0.5 * (y1 - y0) * 1.25)
            plot.setYRange(center - half, center + half, padding=0.0)
        print("[DEBUG] Y range locked (zoom out).")

    def zoom_y_reset(self):
        self.y_zoom_factor = 1.0
        self.y_range_locked = False
        for uuid in self.current_y_range:
            self.current_y_range[uuid] = self.ecg_display_half_range_uv
        print("[DEBUG] Y range reset (auto).")

    def _on_dynamic_y_toggled(self, checked: bool):
        self.dynamic_y_autoscale_enabled = bool(checked)
        if not self.dynamic_y_autoscale_enabled:
            for uuid in self.current_y_range:
                self.current_y_range[uuid] = self.ecg_display_half_range_uv
        print(f"[DEBUG] Dynamic Y autoscale: {'ON' if self.dynamic_y_autoscale_enabled else 'OFF'}")

    def _on_bpm_toggled(self, checked: bool):
        self.bpm_calculation_enabled = bool(checked)
        self._refresh_plot_titles()
        print(f"[DEBUG] BPM calculation: {'ON' if self.bpm_calculation_enabled else 'OFF'}")

    def zoom_x_in(self):
        self.x_range_locked = True
        for uuid, plot in self.plots.items():
            x0, x1 = plot.viewRange()[0]
            span = max(0.1, (x1 - x0) * 0.7)
            end_t = max(
                self.total_samples.get(uuid, 0) / float(self.sample_rate),
                1.0 / float(self.sample_rate),
            )
            start_t = max(0.0, end_t - span)
            plot.setXRange(start_t, end_t, padding=0.0)
        print("[DEBUG] X range locked (zoom in).")

    def zoom_x_out(self):
        self.x_range_locked = True
        for uuid, plot in self.plots.items():
            x0, x1 = plot.viewRange()[0]
            span = min(float(self.display_seconds), max(0.1, (x1 - x0) * 1.25))
            end_t = max(
                self.total_samples.get(uuid, 0) / float(self.sample_rate),
                1.0 / float(self.sample_rate),
            )
            start_t = max(0.0, end_t - span)
            plot.setXRange(start_t, end_t, padding=0.0)
        print("[DEBUG] X range locked (zoom out).")

    def zoom_x_reset(self):
        self.x_zoom_factor = 1.0
        self.x_range_locked = False
        print("[DEBUG] X range reset (auto).")

    def _on_filter_controls_changed(self):
        self.filter_enabled = bool(self.filter_toggle.isChecked())
        self.filter_hp_hz = float(self.hp_spin.value())
        self.filter_lp_hz = float(self.lp_spin.value())
        self.filter_notch_hz = float(self.notch_combo.currentData() or 0.0)
        self._filter_config_version += 1
        self.filter_states.clear()
        self._filter_coeff_cache = None
        print(
            "[DEBUG] Filter config:"
            f" enabled={self.filter_enabled}, HP={self.filter_hp_hz:.2f}Hz,"
            f" LP={self.filter_lp_hz:.1f}Hz, Notch={self.filter_notch_hz:.0f}Hz"
        )

    def _decode_i16_be(self, b: bytes, idx: int) -> int:
        return int.from_bytes(b[idx:idx + 2], byteorder="big", signed=True)

    def _decode_imu_sample_packet(self, uuid_key: str, data: bytes):
        if not data:
            return []
        short = imu_short_uuid(uuid_key)
        if short == "1cf1":
            sample_bytes = ICM_ACCEL_SAMPLE_BYTES
        elif short == "1cf2":
            sample_bytes = ICM_GYRO_SAMPLE_BYTES
        elif short == "1cf3":
            sample_bytes = ICM_TEMP_SAMPLE_BYTES
        else:
            sample_bytes = ICM_FULL_SAMPLE_BYTES
        if len(data) < sample_bytes:
            return []
        if (len(data) % sample_bytes) != 0:
            print(
                f"[DEBUG] IMU payload length {len(data)} not aligned to {sample_bytes} bytes/sample for {uuid_key}; dropping packet."
            )
            return []
        out = []
        for i in range(0, len(data), sample_bytes):
            s = data[i:i + sample_bytes]
            ax = ay = az = 0
            gx = gy = gz = 0
            temp = None
            if short == "1cf1":
                ax = self._decode_i16_be(s, 0)
                ay = self._decode_i16_be(s, 2)
                az = self._decode_i16_be(s, 4)
            elif short == "1cf2":
                gx = self._decode_i16_be(s, 0)
                gy = self._decode_i16_be(s, 2)
                gz = self._decode_i16_be(s, 4)
            elif short == "1cf3":
                temp = self._decode_i16_be(s, 0)
            else:
                ax = self._decode_i16_be(s, 0)
                ay = self._decode_i16_be(s, 2)
                az = self._decode_i16_be(s, 4)
                gx = self._decode_i16_be(s, 6)
                gy = self._decode_i16_be(s, 8)
                gz = self._decode_i16_be(s, 10)
                temp = self._decode_i16_be(s, 12)
            out.append((ax, ay, az, gx, gy, gz, temp))
        return out

    @staticmethod
    def _decode_ppg_value_18bit(sample_bytes: bytes) -> int:
        return int.from_bytes(sample_bytes, byteorder="big", signed=False) & 0x3FFFF

    def _decode_ppg_packet(self, data: bytes):
        if not data or (len(data) % PPG_SAMPLE_BYTES) != 0:
            return []
        return [
            self._decode_ppg_value_18bit(data[i:i + PPG_SAMPLE_BYTES])
            for i in range(0, len(data), PPG_SAMPLE_BYTES)
        ]

    def _update_imu_from_uuid(self, uuid_key: str, data: bytes, rx_monotonic_s: float):
        decoded = self._decode_imu_sample_packet(uuid_key, data)
        if not decoded:
            return 0

        imu_profile = imu_scale_profile_from_uuid(uuid_key)
        accel_lsb_per_g = float(imu_profile["accel_lsb_per_g"])
        gyro_lsb_per_dps = float(imu_profile["gyro_lsb_per_dps"])

        state = self._imu_device_state.setdefault(
            uuid_key,
            {
                "accel": (0.0, 0.0, 0.0),
                "gyro": (0.0, 0.0, 0.0),
                "temp": 0,
            },
        )
        plot_samples = []
        for ax, ay, az, gx, gy, gz, temp in decoded:
            state["accel"] = (
                float(ax) / accel_lsb_per_g,
                float(ay) / accel_lsb_per_g,
                float(az) / accel_lsb_per_g,
            )
            state["gyro"] = (
                float(gx) / gyro_lsb_per_dps,
                float(gy) / gyro_lsb_per_dps,
                float(gz) / gyro_lsb_per_dps,
            )
            if temp is not None:
                state["temp"] = int(temp)
            plot_samples.append((
                state["accel"][0], state["accel"][1], state["accel"][2],
                state["gyro"][0], state["gyro"][1], state["gyro"][2],
            ))
        if self.plotting_enabled:
            self._queue_imu_csv_rows(uuid_key, decoded, rx_monotonic_s, accel_lsb_per_g, gyro_lsb_per_dps)

        self._imu_last_accel = state["accel"]
        self._imu_last_gyro = state["gyro"]
        self._imu_last_temp = state["temp"]
        self.imu_panel.update_values(
            state["accel"][0], state["accel"][1], state["accel"][2],
            state["gyro"][0], state["gyro"][1], state["gyro"][2],
            state["temp"],
            device_label=self._imu_device_display_label(uuid_key),
            accel_range_g=float(imu_profile["accel_range_g"]),
            gyro_range_dps=float(imu_profile["gyro_range_dps"]),
        )

        imu_window = self._ensure_imu_window(uuid_key)
        imu_window.update_receiver_rate(len(decoded), rx_monotonic_s)
        imu_window.add_samples(plot_samples, rx_monotonic_s)
        return len(decoded)

    def _update_ppg_from_uuid(self, uuid_key: str, data: bytes, rx_monotonic_s: float):
        values = self._decode_ppg_packet(data)
        if not values:
            return 0
        short = short_uuid_id(uuid_key)
        channel_name = "red" if short == "AEC1" else "ir"
        if channel_name == "red":
            self._ppg_state["red"] = int(values[-1])
        else:
            self._ppg_state["ir"] = int(values[-1])
        ppg_window = self._ensure_ppg_window()
        ppg_window.update_receiver_rate(channel_name, len(values), rx_monotonic_s)
        ppg_window.add_samples(channel_name, values, rx_monotonic_s)
        if self.plotting_enabled:
            self._queue_ppg_csv_rows(channel_name, values)
        return len(values)

    def _update_ble_param_from_uuid(self, uuid_key: str, data: bytes):
        if not data:
            return
        short = imu_short_uuid(uuid_key)
        if short != "bfee":
            return
        # Firmware exposes RSSI as int8 dBm on UUID 0xBFEE when EXPOSE_BLE_PARAM=1.
        rssi_dbm = int.from_bytes(data[:1], byteorder="little", signed=True)
        tx_power_dbm = None
        if len(data) >= 2:
            tx_power_dbm = int.from_bytes(data[1:2], byteorder="little", signed=True)
        self._rssi_from_nrf_uuid = True
        self._last_rssi_dbm = int(rssi_dbm)
        if tx_power_dbm is not None:
            self._last_tx_power_dbm = int(tx_power_dbm)
        self.rssi_label.setText(f"RSSI: {self._last_rssi_dbm} dBm (nRF)")
        self.signal_quality_label.setText(f"Signal: {self._rssi_quality(self._last_rssi_dbm)}")
        self._record_ble_diag_point(self._last_rssi_dbm, self._last_tx_power_dbm)

    @staticmethod
    def _decode_u16_le(buf: bytes, idx: int) -> int:
        return int.from_bytes(buf[idx:idx + 2], byteorder="little", signed=False)

    def _decode_as7341_samples(self, data: bytes):
        # Firmware packs each AS7341 sample as 4 x uint16 LE: RED630, RED680, NIR, LED_IDX.
        usable = len(data) - (len(data) % 8)
        if usable <= 0:
            return []
        out = []
        for i in range(0, usable, 8):
            red_630 = self._decode_u16_le(data, i)
            red_680 = self._decode_u16_le(data, i + 2)
            nir = self._decode_u16_le(data, i + 4)
            led_idx = self._decode_u16_le(data, i + 6)
            out.append((red_630, red_680, nir, led_idx))
        return out

    def _on_as7341_window_closed(self, stream_uuid: str):
        stream_uuid = str(stream_uuid)
        if self._as7341_window_programmatic_close:
            self.as7341_windows.pop(stream_uuid, None)
            return
        self._as7341_window_closed_by_user.add(stream_uuid)
        self.as7341_windows.pop(stream_uuid, None)

    def _close_as7341_window(self, stream_uuid: str):
        stream_uuid = str(stream_uuid)
        window = self.as7341_windows.get(stream_uuid)
        if window is None:
            return
        self._as7341_window_programmatic_close = True
        try:
            window.close()
        finally:
            self.as7341_windows.pop(stream_uuid, None)
            self._as7341_window_programmatic_close = False

    def _close_all_as7341_windows(self):
        for stream_uuid in list(self.as7341_windows.keys()):
            self._close_as7341_window(stream_uuid)

    def _ensure_as7341_window(self, stream_uuid: str):
        stream_uuid = str(stream_uuid)
        if stream_uuid not in self._as7341_sensor_uuids:
            return None
        if stream_uuid in self._as7341_window_closed_by_user:
            return None
        window = self.as7341_windows.get(stream_uuid)
        if window is None:
            window = AS7341SpectralWindow(stream_uuid=stream_uuid, on_close=self._on_as7341_window_closed)
            self.as7341_windows[stream_uuid] = window
        if not window.isVisible():
            window.show()
            window.raise_()
            window.activateWindow()
        return window

    def _update_as7341_from_uuid(self, uuid_key: str, data: bytes) -> int:
        if not is_as7341_data_uuid(uuid_key):
            return 0
        self._as7341_sensor_uuids.add(uuid_key)
        samples = self._decode_as7341_samples(data)
        if not samples:
            return 0
        self._as7341_data_seen_by_uuid.add(uuid_key)
        red_630, red_680, nir, led_idx = samples[-1]
        self._as7341_last_sample[uuid_key] = (red_630, red_680, nir, led_idx)

        nir_red630 = float(nir) / max(1.0, float(red_630))
        nir_red680 = float(nir) / max(1.0, float(red_680))
        denom = float(nir) + float(red_680)
        ndi_680 = 0.0 if denom <= 0.0 else ((float(nir) - float(red_680)) / denom)
        spectral_window = self._ensure_as7341_window(uuid_key)
        if spectral_window is None:
            return len(samples)
        spectral_window.panel.update_values(
            red_630, red_680, nir, led_idx, nir_red630, nir_red680, ndi_680
        )
        return len(samples)

    def _decode_values_for_uuid(self, uuid_key: str, data: bytes):
        if not data:
            return []
        if is_as7341_data_uuid(uuid_key):
            return []
        if is_imu_stream_uuid(uuid_key):
            imu_profile = imu_scale_profile_from_uuid(uuid_key)
            accel_lsb_per_g = float(imu_profile["accel_lsb_per_g"])
            samples = self._decode_imu_sample_packet(uuid_key, data)
            if not samples:
                return []
            return [
                float(
                    math.sqrt(
                        (ax / accel_lsb_per_g) ** 2
                        + (ay / accel_lsb_per_g) ** 2
                        + (az / accel_lsb_per_g) ** 2
                    )
                )
                for ax, ay, az, _gx, _gy, _gz, _temp in samples
            ]
        if not is_eefx_uuid(uuid_key):
            return decode_payload_values(data, uuid_key)

        merged = self.partial_payload_by_uuid.get(uuid_key, b"") + data
        usable = len(merged) - (len(merged) % 3)
        self.partial_payload_by_uuid[uuid_key] = merged[usable:]
        if usable == 0:
            return []
        return decode_payload_values(merged[:usable], uuid_key)

    def _get_filter_state(self, uuid_key: str):
        state = self.filter_states.get(uuid_key)
        if state and state.get("version") == self._filter_config_version:
            return state
        state = {
            "version": self._filter_config_version,
            "hp_x_prev": 0.0,
            "hp_y_prev": 0.0,
            "lp_y_prev": 0.0,
            "notch_x1": 0.0,
            "notch_x2": 0.0,
            "notch_y1": 0.0,
            "notch_y2": 0.0,
            "smooth_buf": deque(maxlen=max(1, int(self.filter_smooth_len))),
            "smooth_sum": 0.0,
            "spike_prev": 0.0,
            "spike_dev": 0.0,
            "spike_init": False,
        }
        self.filter_states[uuid_key] = state
        return state

    def _apply_realtime_filter(self, uuid_key: str, values):
        if not values:
            return values
        if not self.filter_enabled:
            return values

        fs = float(self.sample_rate)
        hp_hz = max(0.01, min(self.filter_hp_hz, 0.45 * fs))
        lp_hz = max(hp_hz + 0.5, min(self.filter_lp_hz, 0.49 * fs))
        notch_hz = float(self.filter_notch_hz)
        state = self._get_filter_state(uuid_key)
        cache_key = (fs, hp_hz, lp_hz, notch_hz, float(self.filter_q))
        if self._filter_coeff_cache and self._filter_coeff_cache.get("key") == cache_key:
            coeff = self._filter_coeff_cache
        else:
            dt = 1.0 / fs
            rc_hp = 1.0 / (2.0 * np.pi * hp_hz)
            alpha_hp = rc_hp / (rc_hp + dt)
            rc_lp = 1.0 / (2.0 * np.pi * lp_hz)
            alpha_lp = dt / (rc_lp + dt)
            use_notch = 1.0 <= notch_hz < (0.5 * fs - 1.0)
            coeff = {
                "key": cache_key,
                "alpha_hp": alpha_hp,
                "alpha_lp": alpha_lp,
                "use_notch": use_notch,
                "b0": 0.0,
                "b1": 0.0,
                "b2": 0.0,
                "a1": 0.0,
                "a2": 0.0,
            }
            if use_notch:
                w0 = 2.0 * np.pi * notch_hz / fs
                c = np.cos(w0)
                s = np.sin(w0)
                alpha = s / (2.0 * self.filter_q)
                a0 = 1.0 + alpha
                coeff["b0"] = 1.0 / a0
                coeff["b1"] = (-2.0 * c) / a0
                coeff["b2"] = 1.0 / a0
                coeff["a1"] = (-2.0 * c) / a0
                coeff["a2"] = (1.0 - alpha) / a0
            self._filter_coeff_cache = coeff

        alpha_hp = coeff["alpha_hp"]
        alpha_lp = coeff["alpha_lp"]
        use_notch = coeff["use_notch"]
        b0 = coeff["b0"]
        b1 = coeff["b1"]
        b2 = coeff["b2"]
        a1 = coeff["a1"]
        a2 = coeff["a2"]

        out = []
        smooth_buf = state["smooth_buf"]
        smooth_sum = state.get("smooth_sum")
        if smooth_sum is None:
            smooth_sum = float(sum(smooth_buf))
        spike_prev = state["spike_prev"]
        spike_dev = state["spike_dev"]
        spike_init = state["spike_init"]
        hp_x_prev = state["hp_x_prev"]
        hp_y_prev = state["hp_y_prev"]
        lp_y_prev = state["lp_y_prev"]
        notch_x1 = state["notch_x1"]
        notch_x2 = state["notch_x2"]
        notch_y1 = state["notch_y1"]
        notch_y2 = state["notch_y2"]
        spike_floor = self.filter_spike_floor
        spike_k = self.filter_spike_k
        for x in values:
            hp = alpha_hp * (hp_y_prev + x - hp_x_prev)
            hp_x_prev = x
            hp_y_prev = hp

            lp = lp_y_prev + alpha_lp * (hp - lp_y_prev)
            lp_y_prev = lp

            y = lp
            if use_notch:
                y = (
                    b0 * lp
                    + b1 * notch_x1
                    + b2 * notch_x2
                    - a1 * notch_y1
                    - a2 * notch_y2
                )
                notch_x2 = notch_x1
                notch_x1 = lp
                notch_y2 = notch_y1
                notch_y1 = y

            # Short moving-average smoothing to reduce high-frequency roughness.
            if len(smooth_buf) == smooth_buf.maxlen:
                smooth_sum -= smooth_buf[0]
            smooth_buf.append(y)
            smooth_sum += y
            y = float(smooth_sum / len(smooth_buf))

            # Adaptive spike limiter to suppress brief large transients.
            if not spike_init:
                spike_prev = y
                spike_dev = 0.0
                spike_init = True
            delta = y - spike_prev
            abs_delta = abs(delta)
            spike_dev = 0.995 * spike_dev + 0.005 * abs_delta
            step_limit = max(spike_floor, spike_k * spike_dev)
            if abs_delta > step_limit:
                y = spike_prev + np.sign(delta) * step_limit
            spike_prev = y

            out.append(float(y))

        state["hp_x_prev"] = hp_x_prev
        state["hp_y_prev"] = hp_y_prev
        state["lp_y_prev"] = lp_y_prev
        state["notch_x1"] = notch_x1
        state["notch_x2"] = notch_x2
        state["notch_y1"] = notch_y1
        state["notch_y2"] = notch_y2
        state["smooth_sum"] = smooth_sum
        state["spike_prev"] = spike_prev
        state["spike_dev"] = spike_dev
        state["spike_init"] = spike_init
        return out

    async def start_notify_for_all_uuids(self, session_id: int):
        if (
            not self.ble_client
            or self._notify_started
            or session_id != self._ble_session_id
            or self._shutting_down
        ):
            return

        uuids = await self._discover_notify_uuids()
        if session_id != self._ble_session_id or self._shutting_down:
            return
        self.notify_uuids = uuids
        eeg_uuids = [u for u in uuids if is_eefx_uuid(u)]
        imu_uuids = [u for u in uuids if is_imu_stream_uuid(u)]
        ppg_uuids = [u for u in uuids if is_ppg_stream_uuid(u)]
        as7341_uuids = [u for u in uuids if is_as7341_data_uuid(u)]
        self.available_eeg_uuids = list(eeg_uuids)
        self.available_imu_uuids = list(imu_uuids)
        self.available_ppg_uuids = list(ppg_uuids)
        self.available_as7341_uuids = list(as7341_uuids)
        self._refresh_sensor_selector()
        self._create_uuid_plots(eeg_uuids)
        self._set_as7341_led_controls_enabled(bool(self._as7341_config_uuid))
        if self._as7341_config_uuid:
            await self._read_as7341_config()
        else:
            self._sync_as7341_led_controls_from_state()
            self._set_as7341_led_status("AS7341 LED intensity/PWM: config UUID not exposed", "#9B1C1C")
        if not self._ble_param_rssi_uuid:
            self.rssi_label.setText("RSSI: BFEE not exposed")

        if not uuids:
            self._set_connection_state("Connected", "No notify UUID found")
            QMessageBox.warning(self, "BLE", "No notify characteristic found on this device.")
            return

        failed = []
        for uuid in uuids:
            async def handler(_, data: bytes, uuid_key=uuid):
                if self._shutting_down or session_id != self._ble_session_id:
                    return
                rx_monotonic = time.monotonic()
                self._last_notify_rx_monotonic = rx_monotonic
                self._disconnect_miss_streak = 0
                self._bytes_in_last_second += len(data)
                self._packets_in_last_second += 1
                if is_ble_param_uuid(uuid_key):
                    self._update_ble_param_from_uuid(uuid_key, data)
                    # Count BLE param packets so diagnostics UUID does not show as "quiet".
                    self.uuid_samples_in_last_second[uuid_key] = (
                        self.uuid_samples_in_last_second.get(uuid_key, 0) + 1
                    )
                    return
                if is_imu_stream_uuid(uuid_key):
                    sample_count = self._update_imu_from_uuid(uuid_key, data, rx_monotonic)
                    if sample_count > 0:
                        self._samples_in_last_second += sample_count
                        self.uuid_samples_in_last_second[uuid_key] = (
                            self.uuid_samples_in_last_second.get(uuid_key, 0) + sample_count
                        )
                    return
                if is_ppg_stream_uuid(uuid_key):
                    sample_count = self._update_ppg_from_uuid(uuid_key, data, rx_monotonic)
                    if sample_count > 0:
                        self._samples_in_last_second += sample_count
                        self.uuid_samples_in_last_second[uuid_key] = (
                            self.uuid_samples_in_last_second.get(uuid_key, 0) + sample_count
                        )
                    return
                if is_as7341_data_uuid(uuid_key):
                    sample_count = self._update_as7341_from_uuid(uuid_key, data)
                    if sample_count > 0:
                        self._samples_in_last_second += sample_count
                        self.uuid_samples_in_last_second[uuid_key] = (
                            self.uuid_samples_in_last_second.get(uuid_key, 0) + sample_count
                        )
                    return
                values = self._decode_values_for_uuid(uuid_key, data)
                if not values:
                    return
                if is_eefx_uuid(uuid_key):
                    values = self._apply_realtime_filter(uuid_key, values)

                queue = self.plot_queues.get(uuid_key)
                if queue is None:
                    return

                now = time.time()
                if self.plotting_enabled:
                    queue.extend(values)
                    self._queue_eeg_csv_values(uuid_key, values)
                    if self.terminal_log_enabled:
                        self.terminal_log_queue.extend((now, uuid_key, value) for value in values)
                self._samples_in_last_second += len(values)
                self.uuid_samples_in_last_second[uuid_key] = (
                    self.uuid_samples_in_last_second.get(uuid_key, 0) + len(values)
                )

            try:
                # Bleak start_notify writes CCCD for this characteristic.
                await self.ble_client.start_notify(uuid, handler)
                print(f"[DEBUG] start_notify OK (CCCD enabled): {uuid}")
            except Exception as e:
                print(f"[DEBUG] start_notify failed for {uuid}: {e}")
                failed.append(uuid)

        active_count = len(uuids) - len(failed)
        if session_id != self._ble_session_id or self._shutting_down:
            return
        self._notify_started = active_count > 0

        if active_count == 0:
            self._set_connection_state("Connected", "Notify failed")
            QMessageBox.warning(self, "BLE", "Could not start notification on any UUID.")
            return

        status_msg = f"Connected: {self.connected_device_name} | Notify: {active_count}/{len(uuids)}"
        self._set_connection_state("Connected", status_msg)

        if failed:
            print("[DEBUG] Failed UUIDs:")
            for uid in failed:
                print(f"  - {uid}")

    def _start_csv_logging(self):
        if self.csv_streams:
            return
        os.makedirs("data", exist_ok=True)
        self._csv_time_origin_monotonic = None
        self._ppg_csv_sample_counts = {"red": 0, "ir": 0}
        self._imu_csv_last_rx_monotonic = {}

        eeg_columns = [short_uuid_id(u) for u in self.plot_queues.keys()] if self.plot_queues else []
        if not eeg_columns:
            eeg_columns = ["Value"]
        now = time.monotonic()

        eeg_path = os.path.join("data", "ble_uuid_data_latest.csv")
        eeg_file = open(eeg_path, "w", newline="")
        eeg_writer = csv.writer(eeg_file)
        eeg_writer.writerow(eeg_columns)
        eeg_file.flush()

        ppg_path = os.path.join("data", "ppg_data_latest.csv")
        ppg_file = open(ppg_path, "w", newline="")
        ppg_writer = csv.writer(ppg_file)
        ppg_writer.writerow(["time_s", "red", "ir"])
        ppg_file.flush()

        imu_path = os.path.join("data", "imu_data_latest.csv")
        imu_file = open(imu_path, "w", newline="")
        imu_writer = csv.writer(imu_file)
        imu_writer.writerow(["time_s", "uuid", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps", "temp_raw"])
        imu_file.flush()

        self.csv_streams = {
            "eeg": {
                "kind": "columns",
                "file": eeg_file,
                "writer": eeg_writer,
                "columns": eeg_columns,
                "buffers": {col: deque() for col in eeg_columns},
                "last_flush_time": now,
            },
            "ppg": {
                "kind": "columns",
                "file": ppg_file,
                "writer": ppg_writer,
                "columns": ["time_s", "red", "ir"],
                "buffers": {
                    "time_s": deque(),
                    "red": deque(),
                    "ir": deque(),
                },
                "last_flush_time": now,
            },
            "imu": {
                "kind": "rows",
                "file": imu_file,
                "writer": imu_writer,
                "rows": deque(),
                "last_flush_time": now,
            },
        }

    def _stop_csv_logging(self):
        if not self.csv_streams:
            return
        for stream in self.csv_streams.values():
            if stream["kind"] == "columns":
                buffers = stream["buffers"]
                columns = stream["columns"]
                while any(buffers.get(col) for col in columns):
                    row = []
                    for col in columns:
                        buf = buffers.get(col)
                        row.append(buf.popleft() if buf else "")
                    stream["writer"].writerow(row)
            else:
                rows = stream["rows"]
                while rows:
                    stream["writer"].writerow(rows.popleft())
            stream["file"].flush()
            stream["file"].close()
        self.csv_streams = {}
        self._csv_time_origin_monotonic = None
        self._ppg_csv_sample_counts = {"red": 0, "ir": 0}
        self._imu_csv_last_rx_monotonic = {}

    async def _csv_writer_task(self):
        while True:
            if self.csv_streams:
                max_rows = 1000
                now = time.monotonic()
                for stream in self.csv_streams.values():
                    if stream["kind"] == "columns":
                        out_rows = []
                        buffers = stream["buffers"]
                        columns = stream["columns"]
                        while len(out_rows) < max_rows and any(buffers.get(col) for col in columns):
                            row = []
                            for col in columns:
                                buf = buffers.get(col)
                                row.append(buf.popleft() if buf else "")
                            out_rows.append(row)
                        if out_rows:
                            stream["writer"].writerows(out_rows)
                    else:
                        rows = stream["rows"]
                        out_rows = []
                        while rows and len(out_rows) < max_rows:
                            out_rows.append(rows.popleft())
                        if out_rows:
                            stream["writer"].writerows(out_rows)
                    if now - stream["last_flush_time"] >= self._csv_flush_interval_s:
                        stream["file"].flush()
                        stream["last_flush_time"] = now
            await asyncio.sleep(0.1)

    def _queue_eeg_csv_values(self, uuid_key: str, values):
        stream = self.csv_streams.get("eeg")
        if not stream or not values:
            return
        col = short_uuid_id(uuid_key)
        buffers = stream["buffers"]
        if col in buffers:
            buffers[col].extend(values)

    def _queue_ppg_csv_rows(self, channel_name: str, values):
        stream = self.csv_streams.get("ppg")
        if not stream or not values:
            return
        sample_count = self._ppg_csv_sample_counts.get(channel_name, 0)
        buffers = stream["buffers"]
        for value in values:
            buffers["time_s"].append(f"{sample_count * PPG_FIXED_DT_S:.6f}")
            if channel_name == "red":
                buffers["red"].append(float(value))
                buffers["ir"].append("")
            else:
                buffers["red"].append("")
                buffers["ir"].append(float(value))
            sample_count += 1
        self._ppg_csv_sample_counts[channel_name] = sample_count

    def _queue_imu_csv_rows(
        self,
        uuid_key: str,
        decoded_samples,
        rx_monotonic_s: float,
        accel_lsb_per_g: float,
        gyro_lsb_per_dps: float,
    ):
        stream = self.csv_streams.get("imu")
        if not stream or not decoded_samples:
            return
        rx_t = float(rx_monotonic_s)
        if self._csv_time_origin_monotonic is None:
            self._csv_time_origin_monotonic = rx_t
        prev_rx = self._imu_csv_last_rx_monotonic.get(uuid_key)
        sample_times = [rx_t] * len(decoded_samples)
        if prev_rx is not None and rx_t > prev_rx:
            step = (rx_t - prev_rx) / float(len(decoded_samples))
            sample_times = [prev_rx + (step * (i + 1)) for i in range(len(decoded_samples))]
        self._imu_csv_last_rx_monotonic[uuid_key] = rx_t

        short = short_uuid_id(uuid_key)
        rows = stream["rows"]
        for (ax, ay, az, gx, gy, gz, temp), sample_t in zip(decoded_samples, sample_times):
            rows.append((
                f"{max(0.0, sample_t - self._csv_time_origin_monotonic):.6f}",
                short,
                float(ax) / accel_lsb_per_g,
                float(ay) / accel_lsb_per_g,
                float(az) / accel_lsb_per_g,
                float(gx) / gyro_lsb_per_dps,
                float(gy) / gyro_lsb_per_dps,
                float(gz) / gyro_lsb_per_dps,
                "" if temp is None else int(temp),
            ))

    async def _terminal_logger_task(self):
        while True:
            if self.terminal_log_enabled and self.terminal_log_queue:
                batch_size = min(len(self.terminal_log_queue), 400)
                lines = []
                for _ in range(batch_size):
                    ts, uuid, value = self.terminal_log_queue.popleft()
                    t = datetime.fromtimestamp(ts).strftime("%H:%M:%S.%f")[:-3]
                    lines.append(f"{t} | {uuid} | {value:.3f}")
                sys.stdout.write("\n".join(lines) + "\n")
                sys.stdout.flush()
            await asyncio.sleep(0.02)

    def update_plot(self):
        if self._shutting_down:
            return
        max_per_tick = 1200
        dirty_uuids = set()

        for uuid, queue in self.plot_queues.items():
            processed = 0
            while queue and processed < max_per_tick:
                self.buffers[uuid].append(queue.popleft())
                self.total_samples[uuid] = self.total_samples.get(uuid, 0) + 1
                processed += 1
            if processed > 0:
                dirty_uuids.add(uuid)

        for uuid in dirty_uuids:
            buf_deque = self.buffers[uuid]
            buf = np.fromiter(buf_deque, dtype=float)
            if buf.size < 2:
                continue

            if self.dynamic_y_autoscale_enabled:
                tail = buf[-min(200, buf.size):]
                center = float(tail.mean())
                peak = float(np.max(np.abs(tail - center))) if tail.size else 100.0
                target_range = max(peak * 1.2, 100.0)
                self.current_y_range[uuid] = (
                    (1.0 - self._y_range_smoothing) * self.current_y_range[uuid]
                    + self._y_range_smoothing * target_range
                )
            else:
                center = 0.0
                self.current_y_range[uuid] = self.ecg_display_half_range_uv
            yr = self.current_y_range[uuid] * self.y_zoom_factor

            total = self.total_samples.get(uuid, buf.size)
            total_duration = max(total / float(self.sample_rate), 1.0 / float(self.sample_rate))
            available_duration = min(total_duration, float(self.display_seconds))
            end_t = total_duration
            start_t = max(0.0, end_t - available_duration)
            t = (np.arange(buf.size, dtype=float) / float(self.sample_rate)) + start_t

            visible_window = min(
                float(self.display_seconds),
                max(0.1, float(self.display_seconds) * self.x_zoom_factor),
            )
            if total_duration <= visible_window:
                target_x = (0.0, visible_window)
            else:
                target_x = (end_t - visible_window, end_t)

            prev_start, prev_end = self.current_x_view.get(uuid, target_x)
            new_start = prev_start + self._x_range_smoothing * (target_x[0] - prev_start)
            new_end = prev_end + self._x_range_smoothing * (target_x[1] - prev_end)
            self.current_x_view[uuid] = (new_start, new_end)
            if self.x_range_locked:
                x0, x1 = self.plots[uuid].viewRange()[0]
                locked_span = max(0.1, x1 - x0)
                follow_start = max(0.0, end_t - locked_span)
                self.plots[uuid].setXRange(follow_start, end_t, padding=0.0)
            else:
                self.plots[uuid].setXRange(new_start, new_end, padding=0.0)

            self.curves[uuid].setData(t, buf, clear=True)
            if not self.y_range_locked:
                self.plots[uuid].setYRange(center - yr, center + yr)

            bpm = None
            if self.bpm_calculation_enabled:
                bpm = self._estimate_bpm_from_recent_peaks(buf)
            self._set_plot_title_text(uuid, bpm)
            popout = self.popout_windows.get(uuid)
            if popout and popout.isVisible():
                popout.curve.setData(t, buf, clear=True)
                if not self.y_range_locked:
                    popout.plot.setYRange(center - yr, center + yr)
                if self.x_range_locked:
                    x0, x1 = self.plots[uuid].viewRange()[0]
                    popout.plot.setXRange(x0, x1, padding=0.0)
                else:
                    popout.plot.setXRange(new_start, new_end, padding=0.0)
                self._set_plot_title_text(uuid, bpm)

    def _flush_sampling_rate(self):
        self.freq_label.setText(f"Sampling: {self._samples_in_last_second} samples/s")
        self.ble_speed_label.setText(
            f"BLE: {self._bytes_in_last_second} B/s | {self._packets_in_last_second} pkt/s"
        )
        self._update_ble_diag_bandwidth(self._bytes_in_last_second)
        if self.notify_uuids:
            quiet = [u for u in self.notify_uuids if self.uuid_samples_in_last_second.get(u, 0) == 0]
            if quiet:
                print(f"[DEBUG] UUIDs with no data in last 1s: {', '.join(quiet)}")
            for u in self.notify_uuids:
                if u in self.uuid_sample_rate_history:
                    count = float(self.uuid_samples_in_last_second.get(u, 0))
                    history = self.uuid_sample_rate_history[u]
                    history.append(count)
                    self.uuid_avg_samples_per_second[u] = float(sum(history) / len(history)) if history else None
                self.uuid_samples_in_last_second[u] = 0
            self._refresh_plot_titles()
        self._samples_in_last_second = 0
        self._bytes_in_last_second = 0
        self._packets_in_last_second = 0

    def disconnect_bluetooth(self, *_args):
        if self._shutting_down:
            return
        self._manual_disconnect_requested = True
        self._stop_auto_reconnect()
        self._invalidate_ble_session()
        old_client = self.ble_client
        old_uuids = list(self.notify_uuids)
        self._apply_disconnected_ui_state("Disconnected by user")
        self._reset_ble_runtime_state()
        asyncio.create_task(self._disconnect_client(old_client, old_uuids))

    def keyPressEvent(self, event):
        # macOS Backspace/Delete should return app to initial screen state.
        if event.key() in (Qt.Key_Backspace, Qt.Key_Delete):
            self.disconnect_bluetooth()
            event.accept()
            return
        super().keyPressEvent(event)

    def closeEvent(self, event):
        self._shutting_down = True
        self._manual_disconnect_requested = True
        self.plotting_enabled = False
        self.timer.stop()
        self.freq_timer.stop()
        if hasattr(self, "conn_watchdog_timer"):
            self.conn_watchdog_timer.stop()
        self._stop_auto_reconnect()
        if self._metrics_task and not self._metrics_task.done():
            self._metrics_task.cancel()
        if self.ble_diag_window is not None:
            self.ble_diag_window.close()
            self.ble_diag_window = None
        try:
            self._invalidate_ble_session()
            old_client = self.ble_client
            old_uuids = list(self.notify_uuids)
            self._reset_ble_runtime_state()
            asyncio.get_event_loop().create_task(self._disconnect_client(old_client, old_uuids))
        except Exception:
            self._stop_csv_logging()
        super().closeEvent(event)


if __name__ == "__main__":
    import sys

    app = QApplication(sys.argv)
    loop = qasync.QEventLoop(app)
    asyncio.set_event_loop(loop)

    window = UUIDVisualizer()
    window.show()
    app.aboutToQuit.connect(loop.stop)

    with loop:
        loop.run_forever()
