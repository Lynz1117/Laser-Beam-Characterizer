from PySide6.QtWidgets import QWidget, QVBoxLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class PlotCanvas(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.fig = Figure(constrained_layout=True)
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)
        self.ax = self.fig.add_subplot(111)
        self._line = None
        self.set_mode("polarization")

    def set_mode(self, mode: str):
        self.ax.clear()
        if mode == "polarization":
            self.ax.set_title("Polarization: DoP vs sample")
            self.ax.set_xlabel("Sample #")
            self.ax.set_ylabel("DoP")
        elif mode == "wavelength":
            self.ax.set_title("Spectrum")
            self.ax.set_xlabel("Index")
            self.ax.set_ylabel("Signal")
        elif mode == "divergence":
            self.ax.set_title("Divergence profile")
            self.ax.set_xlabel("Position index")
            self.ax.set_ylabel("Intensity (a.u.)")
        else:
            self.ax.set_title("M² / z-scan")
            self.ax.set_xlabel("Index")
            self.ax.set_ylabel("Beam Radius (mm)")
        (self._line,) = self.ax.plot([], [], marker="o", lw=1)
        self.ax.relim(); self.ax.autoscale_view(); self.canvas.draw_idle()

    def clear(self):
        self._line.set_data([], [])
        self.ax.relim(); self.ax.autoscale_view(); self.canvas.draw_idle()

    def extend(self, xs, ys):
        xdata = list(self._line.get_xdata()) + list(xs)
        ydata = list(self._line.get_ydata()) + list(ys)
        self._line.set_data(xdata, ydata)
        self.ax.relim(); self.ax.autoscale_view(); self.canvas.draw_idle()
