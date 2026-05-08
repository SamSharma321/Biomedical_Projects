# protocol_display.py
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QPushButton, QProgressBar
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont

class ProtocolDisplay(QWidget):
    """Right-side panel showing gesture, countdown, progress, and start button."""
    def __init__(self, parent=None):
        super().__init__(parent)

        # ---- Start button ----
        self.start_button = QPushButton("▶ Start Protocol")
        self.start_button.setFont(QFont("Segoe UI", 18, QFont.Bold))
        self.start_button.setStyleSheet(
            "background-color: #0078D7; color: white; border-radius: 8px; padding: 15px;"
        )
        self.start_button.setFixedHeight(80)

        # ---- Gesture display ----
        self.state_label = QLabel("Idle")
        self.state_label.setAlignment(Qt.AlignCenter)
        self.state_label.setFont(QFont("Segoe UI", 28, QFont.Bold))
        self.state_label.setStyleSheet(
            "background-color: #F5F5F5; border: 2px solid #CCCCCC; "
            "padding: 20px; border-radius: 10px;"
        )

        # ---- Countdown ----
        self.timer_label = QLabel("--")
        self.timer_label.setAlignment(Qt.AlignCenter)
        self.timer_label.setFont(QFont("Segoe UI", 48, QFont.Bold))
        self.timer_label.setStyleSheet(
            "background-color: #E8F0FE; border: 2px solid #A0B9F5; "
            "padding: 20px; border-radius: 10px; color: #0044AA;"
        )

        # ---- Progress bar ----
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFormat("0%")
        self.progress_bar.setStyleSheet(
            "QProgressBar {border: 2px solid #CCCCCC; border-radius: 10px; text-align: center;}"
            "QProgressBar::chunk {background-color: #0078D7; border-radius: 8px;}"
        )
        self.progress_bar.setFixedHeight(40)

        # ---- Layout ----
        layout = QVBoxLayout()
        layout.setSpacing(25)
        layout.setAlignment(Qt.AlignTop)
        layout.addWidget(self.start_button)        # (1) 맨 위 Start 버튼
        layout.addWidget(self.state_label)
        layout.addWidget(self.timer_label)
        layout.addWidget(self.progress_bar)        # (2) 맨 아래 Progress Bar
        self.setLayout(layout)

        # ---- Countdown ----
        self._timer = QTimer()
        self._timer.timeout.connect(self._tick)
        self.remaining_time = 0

        # Progress tracking
        self.total_time = 1
        self.elapsed_time = 0
        self.progress_timer = QTimer()
        self.progress_timer.timeout.connect(self._update_progress)

    def start_protocol(self, label: str, duration: int):
        """Display gesture and countdown."""
        self.state_label.setText(label)
        self.remaining_time = duration
        self.timer_label.setText(str(duration))
        self._timer.start(1000)
        self.progress_timer.start(1000)
        self.elapsed_time = 0

    def _tick(self):
        self.remaining_time -= 1
        if self.remaining_time > 0:
            self.timer_label.setText(str(self.remaining_time))
        else:
            self.timer_label.setText("✅ Done!")
            self._timer.stop()

    def _update_progress(self):
        self.elapsed_time += 1
        progress = min(100, int(self.elapsed_time / max(1, self.total_time) * 100))
        self.progress_bar.setValue(progress)
        self.progress_bar.setFormat(f"{progress}%")
        if progress >= 100:
            self.progress_timer.stop()

    def reset(self):
        """Reset all visual states."""
        self.state_label.setText("Idle")
        self.timer_label.setText("--")
        self.progress_bar.setValue(0)
        self.progress_bar.setFormat("0%")
        self._timer.stop()
        self.progress_timer.stop()
