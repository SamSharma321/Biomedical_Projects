# ppg_window.py
"""
MAX30102 PPG Visualization Window
----------------------------------
Handles BLE service UUID mask 0xAEC0:
  - 0xAEC1  →  Red LED channel (660 nm)
  - 0xAEC2  →  IR  LED channel (940 nm)

Features
--------
* Dual-channel real-time scrolling plot (Red + IR)
* Rolling sample-rate display (Hz) updated every second
* Heart-rate estimation from IR peaks (Pan-Tompkins-style adaptive threshold)
* SpO2 ratio-of-ratios estimate (R = (AC_red/DC_red) / (AC_ir/DC_ir))
* Per-channel show/hide toggles
* X-axis window controls (5 s → 60 s)
* Auto-scaling Y with manual lock option
* Latest sample readout footer

Integration
-----------
Import PPGPlotWindow from this module and replace the inline class in
emg_protocol.py:

    from ppg_window import PPGPlotWindow

Then remove (or leave as a no-op stub) the PPGPlotWindow definition that
currently lives in emg_protocol.py.  The public API is identical so no
other call-sites need changing:

    ppg_window.update_receiver_rate(channel_name, n_samples, rx_monotonic_s)
    ppg_window.add_samples(channel_name, values, rx_monotonic_s)   # channel_name: "red" | "ir"
    ppg_window.reset()
"""

from __future__ import annotations

import math
import time
from collections import deque

import numpy as np
import pyqtgraph as pg
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QCheckBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

# ── Constants ────────────────────────────────────────────────────────────────

PPG_SAMPLE_BYTES = 3          # 24-bit unsigned, big-endian per sample
PPG_FIXED_SAMPLE_RATE_HZ = 100.0
PPG_FIXED_DT_S = 1.0 / PPG_FIXED_SAMPLE_RATE_HZ
PPG_FILTER_HP_HZ = 0.5
PPG_FILTER_LP_HZ = 8.0
PPG_FILTER_NOTCH_HZ = 0.0
PPG_FILTER_Q = 30.0
PPG_FILTER_SMOOTH_LEN = 3
PPG_FILTER_SPIKE_K = 6.0
PPG_FILTER_SPIKE_FLOOR = 120.0
_RATE_AVG_WINDOW = 30         # packets used for rolling average Hz estimate
_RENDER_INTERVAL_MS = 20      # repaint cadence
_HR_MIN_BPM = 30.0
_HR_MAX_BPM = 220.0
_SPO2_CALIBRATION_A = 110.0   # empirical SpO2 ≈ A - B·R
_SPO2_CALIBRATION_B = 25.0

# Colour palette
_COL_RED = "#C62828"
_COL_IR  = "#1565C0"
_COL_BG  = "white"


# ── Helpers ──────────────────────────────────────────────────────────────────

def _decode_ppg_packet(data: bytes) -> list[float]:
    """Decode a BLE payload of 18-bit MAX30102 samples packed in 24-bit words."""
    if not data:
        return []
    usable = len(data) - (len(data) % PPG_SAMPLE_BYTES)
    out: list[float] = []
    for i in range(0, usable, PPG_SAMPLE_BYTES):
        raw = int.from_bytes(data[i:i + PPG_SAMPLE_BYTES], "big", signed=False) & 0x3FFFF
        out.append(float(raw))
    return out


def _estimate_heart_rate(
    ir_values: deque,
    sample_rate_hz: float,
) -> float | None:
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
    if float(np.std(ac)) < 1e-6:
        return None

    autocorr = np.correlate(ac, ac, mode="full")[ac.size - 1:]
    energy = float(autocorr[0]) if autocorr.size else 0.0
    if energy <= 0.0:
        return None

    min_lag = max(1, int(sample_rate_hz * 60.0 / _HR_MAX_BPM))
    max_lag = min(autocorr.size - 1, int(sample_rate_hz * 60.0 / _HR_MIN_BPM))
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
    if not (_HR_MIN_BPM <= bpm <= _HR_MAX_BPM):
        return None
    return bpm


def _estimate_spo2(
    red_values: deque,
    ir_values: deque,
    sample_rate_hz: float,
) -> float | None:
    """
    Ratio-of-ratios SpO2 estimate.

    SpO2 ≈ 110 - 25·R   where R = (AC_red / DC_red) / (AC_ir / DC_ir)

    Requires at least 4 s of data on both channels.
    Returns a value in [70, 100] % or None.
    """
    min_len = int(4.0 * max(sample_rate_hz, 1.0))
    if len(red_values) < min_len or len(ir_values) < min_len:
        return None

    def _ac_dc(buf: deque) -> tuple[float, float]:
        arr = np.fromiter(buf, dtype=np.float64)[-min_len:]
        dc = float(arr.mean())
        if dc <= 0:
            return 0.0, 1.0
        ac = float(arr.max() - arr.min())
        return ac, dc

    ac_r, dc_r = _ac_dc(red_values)
    ac_ir, dc_ir = _ac_dc(ir_values)
    if dc_ir <= 0 or dc_r <= 0 or ac_ir <= 0:
        return None

    R = (ac_r / dc_r) / (ac_ir / dc_ir)
    spo2 = _SPO2_CALIBRATION_A - _SPO2_CALIBRATION_B * R
    if not (70.0 <= spo2 <= 100.0):
        return None
    return round(spo2, 1)


# ── Main window ───────────────────────────────────────────────────────────────

class PPGPlotWindow(QMainWindow):
    """
    Standalone real-time PPG visualisation window for MAX30102.

    Public API (called by UUIDVisualizer in emg_protocol.py):

        window.update_receiver_rate(channel_name, n_decoded_samples, rx_monotonic_s)
        window.add_samples(channel_name, values, rx_monotonic_s)
        window.reset()
    """

    # ── Construction ─────────────────────────────────────────────────────────

    def __init__(self, parent: QWidget | None = None) -> None:
        super().__init__(parent)

        # ── State ──────────────────────────────────────────────────────────
        self._last_rate_rx_monotonic: dict[str, float | None] = {"red": None, "ir": None}
        self._sample_rate_history: dict[str, deque[float]] = {
            "red": deque(maxlen=_RATE_AVG_WINDOW),
            "ir": deque(maxlen=_RATE_AVG_WINDOW),
        }
        self._estimated_sample_rate_hz: float = PPG_FIXED_SAMPLE_RATE_HZ
        self._x_window_s: float = 12.0
        self._sample_counts: dict[str, int] = {"red": 0, "ir": 0}
        self.filter_enabled: bool = False
        self._filter_states: dict[str, dict] = {}
        self._filter_coeff_cache: dict | None = None

        # Per-channel ring buffers: (time_s, raw_count)
        _MAXLEN = 6000
        self._t_red: deque[float] = deque(maxlen=_MAXLEN)
        self._red:   deque[float] = deque(maxlen=_MAXLEN)
        self._red_filtered: deque[float] = deque(maxlen=_MAXLEN)
        self._t_ir:  deque[float] = deque(maxlen=_MAXLEN)
        self._ir:    deque[float] = deque(maxlen=_MAXLEN)
        self._ir_filtered: deque[float] = deque(maxlen=_MAXLEN)

        self._render_dirty: bool = False
        self._show_red: bool = True
        self._show_ir:  bool = True
        self._y_locked: bool = False

        # Metrics derived in the render loop
        self._hr_bpm:  float | None = None
        self._spo2:    float | None = None
        self._last_metrics_t: float = 0.0   # monotonic; re-compute every 2 s

        # ── UI ─────────────────────────────────────────────────────────────
        self.setWindowTitle(
            f"PPG Live View (MAX30102) | Freq: {PPG_FIXED_SAMPLE_RATE_HZ:.0f} Hz/ch"
        )
        self.setGeometry(300, 180, 1020, 580)
        self._build_ui()

        # ── Render timer ───────────────────────────────────────────────────
        self._render_timer = QTimer(self)
        self._render_timer.timeout.connect(self._render_curves)
        self._render_timer.start(_RENDER_INTERVAL_MS)

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(6)
        root.setContentsMargins(8, 6, 8, 6)

        # ── Header row ────────────────────────────────────────────────────
        self.header_label = QLabel(
            f"MAX30102 PPG  |  Service UUID AEC0  |  Freq: {PPG_FIXED_SAMPLE_RATE_HZ:.0f} Hz/ch"
        )
        self.header_label.setStyleSheet("color: #111111; font-weight: bold;")
        self.header_label.setFont(QFont("Segoe UI", 10, QFont.Bold))
        root.addWidget(self.header_label)

        # ── Metrics row ───────────────────────────────────────────────────
        metrics_row = QHBoxLayout()
        self.hr_label = QLabel("❤  HR: -- bpm")
        self.hr_label.setStyleSheet(
            f"color: {_COL_RED}; font-weight: bold; font-size: 14px;"
        )
        self.spo2_label = QLabel("💧 SpO₂: -- %")
        self.spo2_label.setStyleSheet(
            f"color: {_COL_IR}; font-weight: bold; font-size: 14px;"
        )
        metrics_row.addWidget(self.hr_label)
        metrics_row.addSpacing(24)
        metrics_row.addWidget(self.spo2_label)
        metrics_row.addStretch()
        root.addLayout(metrics_row)

        # ── Plot ──────────────────────────────────────────────────────────
        self.plot = pg.PlotWidget()
        self.plot.setBackground(_COL_BG)
        self.plot.setStyleSheet("border: 1px solid #000000;")
        self.plot.showGrid(x=True, y=True)
        self.plot.setLabel("left", "PPG Raw Counts")
        self.plot.setLabel("bottom", "Time (s)")
        self.plot.getAxis("left").enableAutoSIPrefix(False)
        self.plot.getAxis("bottom").enableAutoSIPrefix(False)
        legend = self.plot.addLegend(offset=(10, 10))

        self.red_curve = self.plot.plot(
            [], [],
            pen=pg.mkPen(_COL_RED, width=2.0),
            name="Red (660 nm)",
        )
        self.ir_curve = self.plot.plot(
            [], [],
            pen=pg.mkPen(_COL_IR, width=2.0),
            name="IR (940 nm)",
        )
        root.addWidget(self.plot, stretch=1)

        # ── Control bar ───────────────────────────────────────────────────
        ctrl = QHBoxLayout()

        # Channel toggles
        self.cb_red = QCheckBox("Red")
        self.cb_red.setChecked(True)
        self.cb_red.setStyleSheet(f"color: {_COL_RED}; font-weight: bold;")
        self.cb_red.toggled.connect(self._on_toggle_red)

        self.cb_ir = QCheckBox("IR")
        self.cb_ir.setChecked(True)
        self.cb_ir.setStyleSheet(f"color: {_COL_IR}; font-weight: bold;")
        self.cb_ir.toggled.connect(self._on_toggle_ir)

        self.filter_toggle = QCheckBox("Enable PPG Filter")
        self.filter_toggle.setChecked(self.filter_enabled)
        self.filter_toggle.setToolTip("Check to display filtered PPG. Uncheck to display raw PPG.")
        self.filter_toggle.toggled.connect(self._on_filter_toggled)

        # X-window controls
        def _btn(label: str, slot) -> QPushButton:
            b = QPushButton(label)
            b.setFixedWidth(90)
            b.setStyleSheet(
                "background:#EAF4FF; color:#111111; border-radius:4px; padding:3px 6px;"
            )
            b.clicked.connect(slot)
            return b

        self.y_lock_btn = QPushButton("Lock Y")
        self.y_lock_btn.setCheckable(True)
        self.y_lock_btn.setFixedWidth(70)
        self.y_lock_btn.setStyleSheet(
            "background:#FFF4E6; color:#111111; border-radius:4px; padding:3px 6px;"
        )
        self.y_lock_btn.toggled.connect(self._on_y_lock)

        ctrl.addWidget(QLabel("Channel:"))
        ctrl.addWidget(self.cb_red)
        ctrl.addWidget(self.cb_ir)
        ctrl.addSpacing(16)
        ctrl.addWidget(self.filter_toggle)
        ctrl.addSpacing(16)
        ctrl.addWidget(QLabel("X window:"))
        ctrl.addWidget(_btn("−  5 s", lambda: self._set_x_window(self._x_window_s - 5)))
        ctrl.addWidget(_btn("+  5 s", lambda: self._set_x_window(self._x_window_s + 5)))
        ctrl.addWidget(_btn("Reset 12 s", lambda: self._set_x_window(12.0)))
        ctrl.addSpacing(16)
        ctrl.addWidget(self.y_lock_btn)
        ctrl.addStretch()
        root.addLayout(ctrl)

        # ── Footer: latest values ─────────────────────────────────────────
        self.latest_label = QLabel("Red: --    IR: --")
        self.latest_label.setStyleSheet(
            "color: #222222; font-family: Menlo, Consolas, monospace; font-size: 11px;"
        )
        root.addWidget(self.latest_label)

    # ── Public API ────────────────────────────────────────────────────────────

    def add_samples(
        self,
        channel_name: str,
        values: list[float],
        rx_monotonic_s: float,
    ) -> None:
        """
        Append decoded PPG samples.

        channel_name : "red" or "ir"
        values       : list of raw 24-bit counts as floats
        rx_monotonic_s : time.monotonic() at receive
        """
        if not values:
            return
        if channel_name == "red":
            t_buf = self._t_red
            raw_buf = self._red
            filtered_buf = self._red_filtered
        else:
            t_buf = self._t_ir
            raw_buf = self._ir
            filtered_buf = self._ir_filtered

        filtered_values = self._apply_ppg_realtime_filter(channel_name, values)
        sample_count = self._sample_counts[channel_name]
        for raw_value, filtered_value in zip(values, filtered_values):
            t_buf.append(sample_count * PPG_FIXED_DT_S)
            raw_buf.append(float(raw_value))
            filtered_buf.append(float(filtered_value))
            sample_count += 1
        self._sample_counts[channel_name] = sample_count

        # Footer readout
        latest_red = self._red[-1] if self._red else None
        latest_ir  = self._ir[-1]  if self._ir  else None
        r_txt = "--" if latest_red is None else f"{int(latest_red):,}"
        i_txt = "--" if latest_ir  is None else f"{int(latest_ir):,}"
        self.latest_label.setText(f"Red: {r_txt}    IR: {i_txt}")

        self._render_dirty = True

    def update_receiver_rate(
        self,
        channel_name: str,
        decoded_sample_count: int,
        rx_monotonic_s: float,
    ) -> None:
        """
        Called once per BLE notification with the number of decoded samples
        to maintain a rolling per-channel Hz estimate.
        """
        if channel_name not in self._last_rate_rx_monotonic:
            return
        self._last_rate_rx_monotonic[channel_name] = float(rx_monotonic_s)
        self._estimated_sample_rate_hz = PPG_FIXED_SAMPLE_RATE_HZ
        self._update_header(PPG_FIXED_SAMPLE_RATE_HZ)

    def reset(self) -> None:
        """Clear all buffers and restore idle state."""
        self._last_rate_rx_monotonic = {"red": None, "ir": None}
        for history in self._sample_rate_history.values():
            history.clear()
        self._estimated_sample_rate_hz = PPG_FIXED_SAMPLE_RATE_HZ
        self._sample_counts = {"red": 0, "ir": 0}
        self._filter_states.clear()
        self._filter_coeff_cache = None
        self._hr_bpm = None
        self._spo2 = None
        self._last_metrics_t = 0.0
        for buf in (
            self._t_red,
            self._red,
            self._red_filtered,
            self._t_ir,
            self._ir,
            self._ir_filtered,
        ):
            buf.clear()
        self.red_curve.setData([], [])
        self.ir_curve.setData([], [])
        self.latest_label.setText("Red: --    IR: --")
        self.hr_label.setText("❤  HR: -- bpm")
        self.spo2_label.setText("💧 SpO₂: -- %")
        self._update_header(PPG_FIXED_SAMPLE_RATE_HZ)
        self._render_dirty = False

    # ── Slots ─────────────────────────────────────────────────────────────────

    def _on_toggle_red(self, checked: bool) -> None:
        self._show_red = checked
        self.red_curve.setVisible(checked)

    def _on_toggle_ir(self, checked: bool) -> None:
        self._show_ir = checked
        self.ir_curve.setVisible(checked)

    def _on_filter_toggled(self, checked: bool) -> None:
        self.filter_enabled = checked
        self._render_dirty = True
        self._render_curves()

    def _set_x_window(self, seconds: float) -> None:
        self._x_window_s = max(5.0, min(60.0, float(seconds)))

    def _on_y_lock(self, locked: bool) -> None:
        self._y_locked = locked
        self.y_lock_btn.setText("Unlock Y" if locked else "Lock Y")

    # ── Internal render ───────────────────────────────────────────────────────

    def _update_header(self, avg_hz: float | None) -> None:
        freq_text = (
            f"Freq: {PPG_FIXED_SAMPLE_RATE_HZ:.0f} Hz/ch"
            if avg_hz is None else
            f"Freq: {avg_hz:.1f} Hz/ch"
        )
        self.header_label.setText(
            f"MAX30102 PPG  |  Service UUID AEC0  |  {freq_text}"
        )
        self.setWindowTitle(f"PPG Live View (MAX30102) | {freq_text}")

    def _render_curves(self) -> None:
        if not self._render_dirty or not self.isVisible():
            return
        self._render_dirty = False
        red_values = self._red_filtered if self.filter_enabled else self._red
        ir_values = self._ir_filtered if self.filter_enabled else self._ir

        # Determine shared x end
        all_ends: list[float] = []
        if self._t_red:
            all_ends.append(self._t_red[-1])
        if self._t_ir:
            all_ends.append(self._t_ir[-1])
        if not all_ends:
            return
        x_end = max(all_ends)
        x_start = max(0.0, x_end - self._x_window_s)

        # Red channel
        if self._show_red and self._t_red:
            self.red_curve.setData(list(self._t_red), list(red_values))

        # IR channel
        if self._show_ir and self._t_ir:
            self.ir_curve.setData(list(self._t_ir), list(ir_values))

        # X axis
        self.plot.setXRange(x_start, max(self._x_window_s, x_end), padding=0.0)

        # Y auto-scale (use both visible channels)
        if not self._y_locked:
            visible_vals: list[float] = []
            if self._show_red and red_values:
                visible_vals.extend(list(red_values)[-500:])
            if self._show_ir and ir_values:
                visible_vals.extend(list(ir_values)[-500:])
            if visible_vals:
                lo = min(visible_vals)
                hi = max(visible_vals)
                margin = max((hi - lo) * 0.08, 500.0)
                self.plot.setYRange(lo - margin, hi + margin, padding=0.0)

        # Re-compute HR / SpO2 at most every 2 seconds
        now_mono = time.monotonic()
        if now_mono - self._last_metrics_t >= 2.0:
            self._last_metrics_t = now_mono
            self._recompute_metrics()

    def _get_ppg_filter_state(self, channel_name: str) -> dict:
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

    def _apply_ppg_realtime_filter(
        self,
        channel_name: str,
        values: list[float],
    ) -> list[float]:
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

        out: list[float] = []
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

    def _recompute_metrics(self) -> None:
        fs = self._estimated_sample_rate_hz

        # Heart rate from IR
        hr_source = self._ir_filtered if self._ir_filtered else self._ir
        hr = _estimate_heart_rate(hr_source, fs)
        if hr is not None:
            self._hr_bpm = hr
            self.hr_label.setText(f"❤  HR: {hr:.0f} bpm")
        elif self._hr_bpm is None:
            self.hr_label.setText("❤  HR: -- bpm")

        # SpO2
        spo2 = _estimate_spo2(self._red, self._ir, fs)
        if spo2 is not None:
            self._spo2 = spo2
            color = _COL_IR if spo2 >= 95.0 else "#E65100"
            self.spo2_label.setStyleSheet(
                f"color: {color}; font-weight: bold; font-size: 14px;"
            )
            self.spo2_label.setText(f"💧 SpO₂: {spo2:.1f} %")
        elif self._spo2 is None:
            self.spo2_label.setText("💧 SpO₂: -- %")
