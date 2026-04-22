# lbc_gui/beam_width_dialog.py
from PySide6.QtWidgets import QDialog, QVBoxLayout, QLabel
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.patches as patches

class BeamWidthDialog(QDialog):
    """
    Displays a simple 2D spot view of the beam radius (1/e^2 radius).
    Works for Divergence (w0') and M2 (w0).
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Beam Width")
        self.fig = Figure(figsize=(4.2, 4.2), tight_layout=True)
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("x [mm]")
        self.ax.set_ylabel("y [mm]")

        self.lbl = QLabel("")
        self.lbl.setWordWrap(True)

        lay = QVBoxLayout(self)
        lay.addWidget(self.canvas)
        lay.addWidget(self.lbl)

        self._circle = None

    def _draw_spot(self, radius_mm: float, title: str):
        self.ax.clear()
        self.ax.set_aspect("equal", adjustable="box")
        R = max(1e-6, float(radius_mm))
        lim = 1.25 * R
        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.add_patch(
            patches.Circle((0, 0), R, fill=False, linewidth=2)
        )
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title(title)
        self.ax.set_xlabel("x [mm]"); self.ax.set_ylabel("y [mm]")
        self.canvas.draw()
        # auto-size dialog so labels don't get clipped
        self.adjustSize()
        self.resize(self.sizeHint())

    def show_divergence(self, w0p_mm: float, theta_mrad: float | None = None, s_mm: float | None = None):
        self._draw_spot(w0p_mm, "Focus spot (1/e² radius, w₀′)")
        extras = []
        if theta_mrad is not None:
            extras.append(f"θ≈{theta_mrad:.2f} mrad")
        if s_mm is not None:
            extras.append(f"s'≈{s_mm:.1f} mm")
        self.lbl.setText(f"w₀′≈{w0p_mm:.3f} mm" + (("   " + "   ".join(extras)) if extras else ""))

    def show_m2(self, w0_mm: float, M2: float | None = None, z0_mm: float | None = None, zR_mm: float | None = None):
        self._draw_spot(w0_mm, "Waist (1/e² radius, w₀)")
        extras = []
        if M2 is not None:
            extras.append(f"M²≈{M2:.2f}")
        if z0_mm is not None:
            extras.append(f"z₀≈{z0_mm:.1f} mm")
        if zR_mm is not None:
            extras.append(f"zR′≈{zR_mm:.1f} mm")
        self.lbl.setText(f"w₀≈{w0_mm:.3f} mm" + (("   " + "   ".join(extras)) if extras else ""))
