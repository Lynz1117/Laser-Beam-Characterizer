from collections import deque

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QDialog, QLabel, QVBoxLayout
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class PowerMeterDialog(QDialog):
    """Standalone live power readout with a short rolling history."""

    def __init__(self, parent=None, history_size: int = 240):
        super().__init__(parent)
        self.setWindowTitle("Live Power Meter")
        self.resize(720, 420)

        self._history = deque(maxlen=max(32, int(history_size)))
        self._sample_idx = 0

        layout = QVBoxLayout(self)

        self.lbl_value = QLabel("-- mW")
        self.lbl_value.setAlignment(Qt.AlignCenter)
        self.lbl_value.setStyleSheet("font-size: 28px; font-weight: 700;")

        self.lbl_status = QLabel("Waiting for live power data...")
        self.lbl_status.setAlignment(Qt.AlignCenter)
        self.lbl_status.setStyleSheet("color: gray;")

        self.fig = Figure(figsize=(6.2, 3.0), tight_layout=True)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_title("Recent Power History")
        self.ax.set_xlabel("Sample")
        self.ax.set_ylabel("Power (mW)")
        self.ax.grid(True, alpha=0.3)
        (self._line,) = self.ax.plot([], [], color="#d97706", linewidth=2)

        self.lbl_meta = QLabel("Raw: --    ADC: -- V")
        self.lbl_meta.setAlignment(Qt.AlignCenter)
        self.lbl_meta.setWordWrap(True)

        layout.addWidget(self.lbl_value)
        layout.addWidget(self.lbl_status)
        layout.addWidget(self.canvas, 1)
        layout.addWidget(self.lbl_meta)

    def reset_display(self):
        self._history.clear()
        self._sample_idx = 0
        self.lbl_value.setText("-- mW")
        self.lbl_status.setText("Waiting for live power data...")
        self.lbl_meta.setText("Raw: --    ADC: -- V")
        self._line.set_data([], [])
        self.ax.relim()
        self.ax.autoscale_view()
        self.canvas.draw_idle()

    def update_reading(self, raw=None, vadc=None, power_mw=None):
        self._sample_idx += 1

        if power_mw is not None:
            power_mw = float(power_mw)
            self._history.append((self._sample_idx, power_mw))
            self.lbl_value.setText(f"{power_mw:.3f} mW")
            self.lbl_status.setText("Streaming live power from the meter")
        else:
            self.lbl_status.setText("POWER packet received without a calibrated mW value")

        meta_parts = []
        meta_parts.append(f"Raw: {raw}" if raw is not None else "Raw: --")
        if vadc is not None:
            meta_parts.append(f"ADC: {float(vadc):.3f} V")
        else:
            meta_parts.append("ADC: -- V")

        if self._history:
            xs, ys = zip(*self._history)
            self._line.set_data(xs, ys)
            self.ax.relim()
            self.ax.autoscale_view()
            y_min = min(ys)
            y_max = max(ys)
            meta_parts.append(f"Window min/max: {y_min:.3f} / {y_max:.3f} mW")
        else:
            self._line.set_data([], [])

        self.lbl_meta.setText("    ".join(meta_parts))
        self.canvas.draw_idle()
