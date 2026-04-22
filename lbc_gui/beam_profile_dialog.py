# beam_profile_dialog.py
from typing import Optional, Tuple
import numpy as np
from PySide6.QtWidgets import QDialog, QVBoxLayout, QLabel
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure

class BeamProfileDialog(QDialog):
    """
    Shows 1D X/Y intensity profiles (with optional Gaussian fits) and an optional 2D heatmap.
    Expect normalized intensities (0..1), positions in mm.
    """
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Beam Profile")
        self.resize(980, 760)
        lay = QVBoxLayout(self)

        # X axis profile
        self.fig_x = Figure(figsize=(5, 3))
        self.ax_x = self.fig_x.add_subplot(111)
        self.can_x = FigureCanvasQTAgg(self.fig_x)
        lay.addWidget(QLabel("X-axis Intensity Profile"))
        lay.addWidget(self.can_x)

        # Y axis profile
        self.fig_y = Figure(figsize=(5, 3))
        self.ax_y = self.fig_y.add_subplot(111)
        self.can_y = FigureCanvasQTAgg(self.fig_y)
        lay.addWidget(QLabel("Y-axis Intensity Profile"))
        lay.addWidget(self.can_y)

        # 2D preview
        self.fig_2d = Figure(figsize=(5, 3))
        self.ax_2d = self.fig_2d.add_subplot(111)
        self.can_2d = FigureCanvasQTAgg(self.fig_2d)
        # Track the colorbar so we can remove it between updates
        self._cbar = None
        lay.addWidget(QLabel("2D Pseudo-Image (Elliptical Gaussian)"))
        lay.addWidget(self.can_2d)

    def show_profiles(
        self,
        xX, IX, fitX: Optional[Tuple[float,float,float,float,float]],   # (x0, w, A, C, R2) or None
        xY, IY, fitY: Optional[Tuple[float,float,float,float,float]],
        heatmap=None                                                   # (Xvec, Yvec, Z) or None
    ):
        # X
        self.ax_x.clear()
        self.ax_x.plot(xX, IX, label="Intensity (X)")
        if fitX is not None:
            x0, w, A, C, R2 = fitX
            model = A*np.exp(-2.0*((np.asarray(xX)-x0)**2)/(w*w)) + C
            self.ax_x.plot(xX, model, linestyle="--", label=f"Fit: w1/e2={w:.3f} mm, R²={R2:.3f}")
        self.ax_x.set_xlabel("Position X (mm)"); self.ax_x.set_ylabel("Norm. Intensity")
        self.ax_x.legend(); self.can_x.draw()

        # Y
        self.ax_y.clear()
        self.ax_y.plot(xY, IY, label="Intensity (Y)")
        if fitY is not None:
            x0, w, A, C, R2 = fitY
            model = A*np.exp(-2.0*((np.asarray(xY)-x0)**2)/(w*w)) + C
            self.ax_y.plot(xY, model, linestyle="--", label=f"Fit: w1/e2={w:.3f} mm, R²={R2:.3f}")
        self.ax_y.set_xlabel("Position Y (mm)"); self.ax_y.set_ylabel("Norm. Intensity")
        self.ax_y.legend(); self.can_y.draw()

        # 2D
        self.ax_2d.clear()

        # Remove previous colorbar if present so we don't stack them
        if self._cbar is not None:
            self._cbar.remove()
            self._cbar = None

        if heatmap is not None:
            Xvec, Yvec, Z = heatmap
            im = self.ax_2d.imshow(
                Z,
                extent=[Xvec[0], Xvec[-1], Yvec[0], Yvec[-1]],
                origin="lower",
                aspect="equal",
            )
            self.ax_2d.set_xlabel("X (mm)")
            self.ax_2d.set_ylabel("Y (mm)")
            self._cbar = self.fig_2d.colorbar(im, ax=self.ax_2d,
                                              fraction=0.046, pad=0.04)

        self.can_2d.draw()

