# poincare_sphere.py
import math
from PySide6.QtWidgets import QDialog, QVBoxLayout
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class PoincareSphereDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Poincaré Sphere")
        self.resize(480, 480)

        layout = QVBoxLayout(self)

        self.fig = Figure(figsize=(4, 4))
        self.canvas = FigureCanvas(self.fig)
        layout.addWidget(self.canvas)

        self.ax = self.fig.add_subplot(111, projection="3d")

        self._init_sphere()

    def _init_sphere(self):
        ax = self.ax
        ax.clear()

        # draw unit sphere (wireframe)
        u = [i * math.pi / 30 for i in range(61)]
        v = [i * 2 * math.pi / 30 for i in range(61)]
        xs, ys, zs = [], [], []
        for ui in u:
            for vi in v:
                xs.append(math.sin(ui) * math.cos(vi))
                ys.append(math.sin(ui) * math.sin(vi))
                zs.append(math.cos(ui))
        # make it look nicer: we just plot a few meridians/parallels
        # meridians
        for vi in v[::6]:
            xm, ym, zm = [], [], []
            for ui in u:
                xm.append(math.sin(ui) * math.cos(vi))
                ym.append(math.sin(ui) * math.sin(vi))
                zm.append(math.cos(ui))
            ax.plot(xm, ym, zm, linewidth=0.3, color="0.8")
        # parallels
        for ui in u[::6]:
            xp, yp, zp = [], [], []
            for vi in v:
                xp.append(math.sin(ui) * math.cos(vi))
                yp.append(math.sin(ui) * math.sin(vi))
                zp.append(math.cos(ui))
            ax.plot(xp, yp, zp, linewidth=0.3, color="0.8")

        # axes
        ax.plot([-1, 1], [0, 0], [0, 0], color="r", linewidth=1.0)  # S1
        ax.plot([0, 0], [-1, 1], [0, 0], color="g", linewidth=1.0)  # S2
        ax.plot([0, 0], [0, 0], [-1, 1], color="b", linewidth=1.0)  # S3

        ax.set_xlim([-1, 1]); ax.set_ylim([-1, 1]); ax.set_zlim([-1, 1])
        ax.set_box_aspect((1, 1, 1))
        ax.set_xticks([-1, 0, 1]); ax.set_yticks([-1, 0, 1]); ax.set_zticks([-1, 0, 1])
        ax.set_xlabel("S1")
        ax.set_ylabel("S2")
        ax.set_zlabel("S3")
        ax.set_title("Poincaré Sphere")

        self.canvas.draw()

    def draw_stokes(self, S0, S1, S2, S3):
        # normalize
        S0 = max(abs(S0), 1e-12)
        s1, s2, s3 = S1 / S0, S2 / S0, S3 / S0

        self._init_sphere()
        # plot point
        self.ax.scatter([s1], [s2], [s3], s=40, color="k")
        # optional: line from origin
        self.ax.plot([0, s1], [0, s2], [0, s3], color="k", linewidth=1.0)

        self.canvas.draw()
