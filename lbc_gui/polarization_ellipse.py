from PySide6.QtWidgets import QDialog, QVBoxLayout, QLayout
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import numpy as np

class PolarizationEllipseDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Polarization Ellipse")

        lay = QVBoxLayout(self)
        # KEEP this if you want adjustSize() to follow minimums
        lay.setSizeConstraint(QLayout.SetMinimumSize)

        self.fig = Figure(constrained_layout=True, dpi=110)
        self.canvas = FigureCanvas(self.fig)
        lay.addWidget(self.canvas)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_aspect("equal")

        self.resize(480, 420)

        # NEW: autosize control
        self._autosized_once = False
        self._frozen_min = None   # (w,h) after autosize

    def draw_from_stokes(self, S0, S1, S2, S3, convention="ieee"):
        S0 = float(S0) if abs(S0) > 1e-12 else 1.0
        s1, s2, s3 = S1/S0, S2/S0, S3/S0

        psi = 0.5 * np.arctan2(s2, s1)
        chi = 0.5 * np.arcsin(np.clip(s3, -1.0, 1.0))
        DoLP = float(np.hypot(s1, s2))
        DoCP = float(abs(s3))
        DoP  = float(np.sqrt(s1*s1 + s2*s2 + s3*s3))

        if abs(s3) < 1e-6:
            hand = "Linear"
        else:
            if convention.lower() == "ieee":
                hand = "Right-Handed" if s3 > 0 else "Left-Handed"
            else:
                hand = "Left-Handed" if s3 > 0 else "Right-Handed"

        a = 1.0
        b = abs(np.tan(chi))
        t = np.linspace(0, 2*np.pi, 400)
        x = a*np.cos(t); y = b*np.sin(t)

        c, s = np.cos(psi), np.sin(psi)
        xr =  x*c - y*s
        yr =  x*s + y*c

        maj = np.array([[-a, a], [0, 0]])
        minr = np.array([[0, 0], [-b, b]])
        R = np.array([[c, -s],[s,  c]])
        maj = R @ maj; minr = R @ minr

        self.ax.clear()
        self.ax.plot(xr, yr, lw=1.6)
        self.ax.plot(maj[0],  maj[1],  'k-', lw=1.0)
        self.ax.plot(minr[0], minr[1], 'k-', lw=1.0)
        self.ax.axhline(0, lw=0.6); self.ax.axvline(0, lw=0.6)
        self.ax.set_aspect("equal")
        lim = 1.2
        self.ax.set_xlim(-lim, lim); self.ax.set_ylim(-lim, lim)
        self.ax.set_xlabel("Eₓ"); self.ax.set_ylabel("Eᵧ")
        self.ax.set_title(
            f"ψ={np.degrees(psi):.1f}°, χ={np.degrees(chi):.1f}° • "
            f"DoLP={DoLP:.3f}  DoCP={DoCP:.3f}  DoP={DoP:.3f} • {hand}",
            wrap=True
        )

        self.canvas.draw()

        # Autosize ONLY ONCE
        if not self._autosized_once:
            self._autosize_to_fit(grow_only=True)
            self._autosized_once = True
            # Freeze future minimums at the autosized size (prevents “bounce”)
            self._frozen_min = (self.width(), self.height())
            self.setMinimumSize(*self._frozen_min)

    def _autosize_to_fit(self, grow_only: bool = True, pad_px: int = 28):
        try:
            renderer = self.canvas.get_renderer()
        except Exception:
            self.canvas.draw()
            renderer = self.canvas.get_renderer()

        bbox = self.fig.get_tightbbox(renderer)
        dpi = float(self.fig.get_dpi())
        w_px = int(bbox.width  * dpi) + pad_px
        h_px = int(bbox.height * dpi) + pad_px

        cur_cw, cur_ch = self.canvas.width(), self.canvas.height()
        if grow_only:
            w_px = max(w_px, cur_cw)
            h_px = max(h_px, cur_ch)

        self.canvas.setMinimumSize(w_px, h_px)
        self.adjustSize()
