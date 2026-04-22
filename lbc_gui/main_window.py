from PySide6.QtCore import Qt, QTimer, Slot
from .beam_width_dialog import BeamWidthDialog
from PySide6.QtWidgets import (
    QWidget, QMainWindow, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QComboBox, QGroupBox, QRadioButton,
    QSpinBox, QDoubleSpinBox, QTextEdit, QFileDialog, QMessageBox, QCheckBox,
    QLineEdit
)
import time, numpy as np

from .plotter import PlotCanvas
from .serial_link import SerialLink, list_ports
from .simulator import Simulator
from .run_plans import PolarizationPlan, WavelengthPlan, DivergencePlan, M2Plan
from .polarization_ellipse import PolarizationEllipseDialog
from .poincare_sphere import PoincareSphereDialog
from .beam_profile_dialog import BeamProfileDialog
from .PowerMeter import PowerMeterDialog
from .analysis_utils import knife_edge_to_profile, fit_gaussian_1d, d4sigma_radius, gaussian_2d_preview



class MainWindow(QMainWindow):
    """
    - Polarization: requires STM32 polarimeter (STOKES). Live DoP + ellipse.
    - Wavelength, Divergence, M2: simulator or serial SAMPLE stream.
    - Parameters panel visible only in Simulator; MCU owns params otherwise.
    """
    def __init__(self, simulate: bool = True, port=None, baud=115200):
        super().__init__()
        self.setWindowTitle("Automated Laser Beam Characterizer")
        self.resize(1200, 750)

        # backend
        self.simulate = simulate
        self.serial = None
        if not self.simulate and port:
            try: self.serial = SerialLink(port, baud)
            except Exception as e:
                self.simulate = True
                print(f"[MainWindow] Serial failed, using simulator. {e}")

        # central
        root = QWidget(); self.setCentralWidget(root)
        main = QHBoxLayout(root)

        # ---------- LEFT: sidebar ----------
        sidebar = QVBoxLayout(); main.addLayout(sidebar, 0)
        self._sidebar_layout = sidebar  # keep reference for helper

        # Connection group
        self.grp_conn = QGroupBox("Connection")
        conn = QGridLayout(self.grp_conn)
        self.cmb_ports = QComboBox()
        self.btn_refresh = QPushButton("Refresh")
        self.btn_connect = QPushButton("Connect")
        self.lbl_status = QLabel("Simulator active" if self.simulate else "Disconnected")
        self.chk_sim = QCheckBox("Use Simulator"); self.chk_sim.setChecked(self.simulate)
        conn.addWidget(QLabel("Port:"), 0, 0)
        conn.addWidget(self.cmb_ports, 0, 1)
        conn.addWidget(self.btn_refresh, 0, 2)
        conn.addWidget(self.btn_connect, 1, 1)
        conn.addWidget(self.chk_sim, 1, 2)
        conn.addWidget(self.lbl_status, 2, 0, 1, 3)
        sidebar.addWidget(self.grp_conn)

        # Measurement group (order: Polarization, Wavelength, Divergence, M2)
        self.grp_meas = QGroupBox("Measurement")
        meas = QVBoxLayout(self.grp_meas)
        self.rb_pol = QRadioButton("Polarization")
        self.rb_wav = QRadioButton("Wavelength")
        self.rb_div = QRadioButton("Divergence")
        self.rb_m2  = QRadioButton("M² / z-scan")
        self.rb_pol.setChecked(True)
        for rb in (self.rb_pol, self.rb_wav, self.rb_div, self.rb_m2): meas.addWidget(rb)
        sidebar.addWidget(self.grp_meas)

        # Parameters group (hidden when hardware)
        self.grp_params = QGroupBox("Run Parameters (Simulator)")
        prm = QGridLayout(self.grp_params)
        self.spin_points = QSpinBox(); self.spin_points.setRange(3, 4096); self.spin_points.setValue(512)
        self.spin_settle = QDoubleSpinBox(); self.spin_settle.setRange(0.0, 5.0); self.spin_settle.setDecimals(3); self.spin_settle.setSingleStep(0.05); self.spin_settle.setValue(0.15)
        self.spin_rate   = QSpinBox(); self.spin_rate.setRange(10, 100000); self.spin_rate.setValue(2000)
        prm.addWidget(QLabel("Points/steps"), 0, 0); prm.addWidget(self.spin_points, 0, 1)
        prm.addWidget(QLabel("Settle (s)"),   1, 0); prm.addWidget(self.spin_settle, 1, 1)
        prm.addWidget(QLabel("Sample rate"),  2, 0); prm.addWidget(self.spin_rate,   2, 1)
        sidebar.addWidget(self.grp_params)

        self.lbl_param_src = QLabel("")
        self.lbl_param_src.setStyleSheet("color: gray;")
        sidebar.addWidget(self.lbl_param_src)
        
        # --- Polarization (Simulator-only) custom inputs ---
        self.grp_pol_sim = QGroupBox("Polarization Inputs (Simulator)")
        pol = QGridLayout(self.grp_pol_sim)

        self.edit_pol_angles = QLineEdit()
        self.edit_pol_angles.setPlaceholderText("Angles (deg), e.g. 0,22.5,45,67.5,90,112.5,135,157.5")
        pol.addWidget(QLabel("Angles (°)"), 0, 0)
        pol.addWidget(self.edit_pol_angles, 0, 1)

        self.edit_pol_powers = QLineEdit()
        self.edit_pol_powers.setPlaceholderText("Powers (same length), e.g. 1.2,1.8,2.0,1.6,1.1,1.5,1.9,1.3")
        pol.addWidget(QLabel("Powers"), 1, 0)
        pol.addWidget(self.edit_pol_powers, 1, 1)

        self.spin_pol_noise = QDoubleSpinBox()
        self.spin_pol_noise.setRange(0.0, 1.0); self.spin_pol_noise.setDecimals(4)
        self.spin_pol_noise.setSingleStep(0.005); self.spin_pol_noise.setValue(0.01)
        pol.addWidget(QLabel("Noise σ"), 2, 0)
        pol.addWidget(self.spin_pol_noise, 2, 1)

        self.chk_pol_norm = QCheckBox("Normalize powers to S0=1 (divide by max)")
        pol.addWidget(self.chk_pol_norm, 3, 0, 1, 2)

        row = QHBoxLayout()
        self.btn_pol_apply = QPushButton("Apply Custom Profile")
        self.btn_pol_reset = QPushButton("Use Harmonic Model")
        row.addStretch(1); row.addWidget(self.btn_pol_apply); row.addWidget(self.btn_pol_reset)
        pol.addLayout(row, 4, 0, 1, 2)

        sidebar.addWidget(self.grp_pol_sim)
        
        # --- Divergence base params (needed by plan & UI) ---
        self.lbl_div_f  = QLabel("Focal length f (mm)")
        self.spin_div_f = QDoubleSpinBox()
        self.spin_div_f.setRange(1.0, 5000.0)
        self.spin_div_f.setDecimals(3)
        self.spin_div_f.setValue(100.0)

        self.lbl_div_dz  = QLabel("Position scale (mm/index)")
        self.spin_div_dz = QDoubleSpinBox()
        self.spin_div_dz.setRange(0.001, 10.0)
        self.spin_div_dz.setDecimals(4)
        self.spin_div_dz.setValue(0.10)

        prm.addWidget(self.lbl_div_f,  3, 0); prm.addWidget(self.spin_div_f,  3, 1)
        prm.addWidget(self.lbl_div_dz, 4, 0); prm.addWidget(self.spin_div_dz, 4, 1)
                
        # --- Divergence-specific extras (for waist calc) ---
        self.lbl_div_lambda = QLabel("λ (nm)")
        self.spin_div_lambda = QDoubleSpinBox()
        self.spin_div_lambda.setRange(200.0, 2000.0)
        self.spin_div_lambda.setDecimals(3)
        self.spin_div_lambda.setValue(632.8)

        self.lbl_div_M2 = QLabel("M² (known)")
        self.spin_div_M2 = QDoubleSpinBox()
        self.spin_div_M2.setRange(1.0, 50.0)
        self.spin_div_M2.setDecimals(3)
        self.spin_div_M2.setValue(1.30)

        prm.addWidget(self.lbl_div_lambda, 7, 0); prm.addWidget(self.spin_div_lambda, 7, 1)
        prm.addWidget(self.lbl_div_M2,     8, 0); prm.addWidget(self.spin_div_M2,     8, 1)

        # optional: keep a list so we can show/hide as a group
        self.div_fields = (
            self.lbl_div_f, self.spin_div_f,
            self.lbl_div_dz, self.spin_div_dz,
            self.lbl_div_lambda, self.spin_div_lambda,
            self.lbl_div_M2, self.spin_div_M2,
        )
        
        # --- M²-specific params (Simulator) ---
        self.lbl_m2_lambda = QLabel("λ (nm)")
        self.spin_m2_lambda = QDoubleSpinBox()
        self.spin_m2_lambda.setRange(200.0, 2000.0)
        self.spin_m2_lambda.setDecimals(3)
        self.spin_m2_lambda.setValue(632.8)

        self.lbl_m2_dz  = QLabel("Position scale (mm/index)")
        self.spin_m2_dz = QDoubleSpinBox()
        self.spin_m2_dz.setRange(0.001, 10.0)
        self.spin_m2_dz.setDecimals(4)
        self.spin_m2_dz.setValue(0.10)

        prm.addWidget(self.lbl_m2_lambda, 9, 0); prm.addWidget(self.spin_m2_lambda, 9, 1)
        prm.addWidget(self.lbl_m2_dz,    10, 0); prm.addWidget(self.spin_m2_dz,    10, 1)
        
        # --- Wavelength (Michelson) scale (Simulator only) ---
        self.lbl_wav_dz  = QLabel("WL position scale (mm/index)")
        self.spin_wav_dz = QDoubleSpinBox()
        self.spin_wav_dz.setRange(0.00005, 10.0)
        self.spin_wav_dz.setDecimals(4)
        self.spin_wav_dz.setValue(0.020)   # 20 µm per index is a nice default

        prm.addWidget(self.lbl_wav_dz, 11, 0); prm.addWidget(self.spin_wav_dz, 11, 1)
        self.wav_fields = (self.lbl_wav_dz, self.spin_wav_dz)

        # group for show/hide
        self.m2_fields = (
            self.lbl_m2_lambda, self.spin_m2_lambda,
            self.lbl_m2_dz,     self.spin_m2_dz,
        )

        # Buttons
        self.btn_start   = QPushButton("Start")
        self.btn_abort   = QPushButton("Abort")
        self.btn_save    = QPushButton("Save CSV")
        self.btn_power_window = QPushButton("Power Meter Window")
        self.btn_ellipse = QPushButton("Show Ellipse")
        self.btn_ellipse.setEnabled(False)
        self.btn_poincare = QPushButton("Show Poincaré")
        self.btn_poincare.setEnabled(False)
        self.btn_width   = QPushButton("Show Beam Width")
        self.btn_width.setEnabled(False)
        self.btn_power_window.setEnabled(False)
        for b in (
            self.btn_start,
            self.btn_abort,
            self.btn_save,
            self.btn_power_window,
            self.btn_ellipse,
            self.btn_width,
            self.btn_poincare,
        ):
            sidebar.addWidget(b)
        sidebar.addStretch(1)
        self.btn_pol_apply.clicked.connect(self.on_apply_pol_profile)
        self.btn_pol_reset.clicked.connect(self.on_reset_pol_profile)
        self.btn_profile = QPushButton("Show Beam Profile")
        self.btn_profile.setEnabled(False)
        sidebar.addWidget(self.btn_profile)
        self.btn_profile.clicked.connect(self.on_show_profile)

        self._profile_dlg = None

        # ---------- CENTER: plot ----------
        self.canvas = PlotCanvas(self); main.addWidget(self.canvas, 1)

        # ---------- RIGHT: metrics & log ----------
        right = QVBoxLayout(); main.addLayout(right, 0)
        self.lbl_mode = QLabel("Mode: Polarization")
        self.lbl_metrics = QLabel("Metrics: —")
        self.lbl_power = QLabel("Power Meter: -")
        self.txt_log = QTextEdit(); self.txt_log.setReadOnly(True)
        right.addWidget(self.lbl_mode)
        right.addWidget(self.lbl_metrics)
        right.addWidget(self.lbl_power)
        right.addWidget(QLabel("Log:"))
        right.addWidget(self.txt_log, 1)

        # state
        self.sim = Simulator(seed=1234)
        self.plan = None
        self._ellipse_dlg = None
        self._last_stokes = None
        self._last_divergence = None   # {"w0p_mm":..., "theta_mrad":..., "sprime_mm":...}
        self._last_m2 = None           # {"M2":..., "w0_mm":..., "z0_mm":..., "zR_mm":..., "R2":...}
        self._width_dlg = None
        self._poincare_dlg = None
        self._power_dlg = None
        self._run_active = False

        # timer
        self.timer = QTimer(self); self.timer.setInterval(50)
        self.timer.timeout.connect(self.on_timer)

        # wiring
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect.clicked.connect(self.on_connect_clicked)
        self.chk_sim.toggled.connect(self.on_sim_toggled)
        self.btn_start.clicked.connect(self.on_start_clicked)
        self.btn_abort.clicked.connect(self.on_abort_clicked)
        self.btn_save.clicked.connect(self.on_save_clicked)
        self.btn_power_window.clicked.connect(self.on_show_power_window)
        self.btn_ellipse.clicked.connect(self.on_show_ellipse)
        self.btn_poincare.clicked.connect(self.on_show_poincare)
        self.rb_pol.toggled.connect(self.on_mode_changed)
        self.rb_wav.toggled.connect(self.on_mode_changed)
        self.rb_div.toggled.connect(self.on_mode_changed)
        self.rb_m2.toggled.connect(self.on_mode_changed)
        self.btn_width.clicked.connect(self.on_show_width)

        # init
        self.refresh_ports()
        self.on_mode_changed()
        self._update_param_ui()
        self._update_power_window_button()
        self._ensure_timer_state()
        self._current_mode = None

    # -------- helpers --------
    def log(self, msg: str):
        self.txt_log.append(f"[{time.strftime('%H:%M:%S')}] {msg}")
        
    def _parse_floats(self, s: str):
        if not s.strip(): return []
        vals = []
        for tok in s.replace(";", ",").split(","):
            tok = tok.strip()
            if tok:
                vals.append(float(tok))
        return vals

    def _update_power_window_button(self):
        self.btn_power_window.setEnabled((not self.simulate) and (self.serial is not None))

    def _ensure_timer_state(self):
        need_timer = self._run_active or ((not self.simulate) and (self.serial is not None))
        if need_timer:
            if not self.timer.isActive():
                self.timer.start()
        elif self.timer.isActive():
            self.timer.stop()

    def _handle_power_packet(self, pkt: dict):
        raw = pkt.get("raw")
        vadc = pkt.get("vadc")
        power_mw = pkt.get("power_mW")

        parts = []
        parts.append(f"raw={raw}" if raw is not None else "raw=-")
        if vadc is not None:
            parts.append(f"Vadc={float(vadc):.3f} V")
        if power_mw is not None:
            parts.append(f"Power={float(power_mw):.3f} mW")
        self.lbl_power.setText("Power Meter: " + ", ".join(parts))

        if self._power_dlg is not None:
            self._power_dlg.update_reading(raw=raw, vadc=vadc, power_mw=power_mw)

    def _poll_power_stream(self):
        if self.simulate or self.serial is None:
            return

        while True:
            pkt = self.serial.recv_nowait()
            if pkt is None:
                break

            if pkt.get("ev") == "POWER":
                self._handle_power_packet(pkt)
                continue

            # Keep non-power packets available for the active measurement plan.
            if self.plan is not None and not self.plan.done:
                self.serial.q.put(pkt)
                break

    def _update_param_ui(self):
        sim = self.chk_sim.isChecked()
        self.grp_params.setVisible(sim)
        self.lbl_param_src.setText("Parameters: editable (Simulator)" if sim
                                else "Parameters: controlled by MCU (hidden)")

        # Divergence fields
        show_div = sim and self.rb_div.isChecked()
        for w in getattr(self, "div_fields", ()):
            w.setVisible(show_div)

        # M² fields
        show_m2 = sim and self.rb_m2.isChecked()
        for w in getattr(self, "m2_fields", ()):
            w.setVisible(show_m2)

        # Wavelength (Michelson) fields
        show_wav = sim and self.rb_wav.isChecked()
        for w in getattr(self, "wav_fields", ()):
            w.setVisible(show_wav)
            
        # NEW: Polarization custom inputs
        show_pol_sim = sim and self.rb_pol.isChecked()
        self.grp_pol_sim.setVisible(show_pol_sim)
        
    def _update_metrics_generic(self, mode: str):
        """
        Line 1: samples, mean, std, min, max
        Line 2:
        - wavelength: λ via Michelson 2d/N
        - divergence: w0' (mm), θ (mrad), s' (mm) if scales known
        - m2: ISO 11146 Method 2 -> M², w0, z0, zR', R²
        """
        if not self.plan or not getattr(self.plan, "buffer", None):
            return
        import numpy as np, math

        xs = np.array([x for x, _ in self.plan.buffer], dtype=float)
        ys = np.array([y for _, y in self.plan.buffer], dtype=float)
        if ys.size < 2:
            return

        # Line 1 — unchanged
        line1 = (f"samples={ys.size}  mean={ys.mean():.3f}  std={ys.std():.3f}  "
                f"min={ys.min():.3f}  max={ys.max():.3f}")

        line2 = ""

        if mode == "wavelength":
            # FFT Michelson: λ = 2 / f_peak   (f in cycles/mm)
            est_fft = getattr(self.plan, "estimate_lambda_fft", None)
            if callable(est_fft):
                r = self.plan.estimate_lambda_fft()
                if r:
                    lam_nm, f_cyc_mm, dz_mm = r
                    if dz_mm:
                        line2 = (f"λ≈{lam_nm:.1f} nm   f≈{f_cyc_mm:.2f} cyc/mm   "
                                f"dz≈{dz_mm*1e3:.1f} µm/step")
                    else:
                        line2 = f"λ≈{lam_nm:.1f} nm   f≈{f_cyc_mm:.2f} cyc/mm"
                else:
                    line2 = "estimating λ (FFT)…"
            else:
                line2 = "FFT estimator unavailable"

        elif mode == "divergence":
            # log-Gaussian width w0' in index units → mm via dz_mm
            try:
                y_clip = np.clip(ys, ys.max()*0.05, None)
                L = np.log(y_clip / y_clip.max())
                A, B, _ = np.polyfit(xs, L, 2)
                if A < 0:
                    wprime_idx = float(np.sqrt(-2.0 / A))
                    x0_idx = float(-B / (2.0 * A))

                    dz_mm  = getattr(self.plan, "dz_mm", None)
                    f_mm   = getattr(self.plan, "f_mm", None)
                    lam_nm = getattr(self.plan, "lambda_nm", None)
                    M2     = getattr(self.plan, "m2_factor", None)

                    w0p_mm = wprime_idx * dz_mm if dz_mm else wprime_idx
                    line2  = f"w0'≈{w0p_mm:.3f} mm"

                    theta_mrad = None
                    if f_mm:
                        theta_rad  = w0p_mm / float(f_mm)
                        theta_mrad = theta_rad * 1e3
                        line2 += f"   θ≈{theta_mrad:.2f} mrad"

                        if M2 and lam_nm:
                            lam_mm = float(lam_nm) * 1e-6
                            if theta_rad > 0:
                                w0_mm = (float(M2) * lam_mm) / (np.pi * theta_rad)
                                alpha = w0p_mm / w0_mm if w0_mm > 0 else np.nan
                                line2 += f"   w0≈{w0_mm:.3f} mm   α≈{alpha:.2f}"

                    sprime_mm = None
                    if dz_mm is not None:
                        sprime_mm = x0_idx * dz_mm
                        line2 += f"   s'≈{sprime_mm:.1f} mm"

                    # Cache & enable width dialog safely (only on valid fit)
                    self._last_divergence = {
                        "w0p_mm": w0p_mm,
                        "theta_mrad": theta_mrad,
                        "sprime_mm": sprime_mm,
                    }
                    if self.rb_div.isChecked():
                        self.btn_width.setEnabled(True)
                    if getattr(self, "_width_dlg", None) and self._width_dlg.isVisible():
                        self._width_dlg.show_divergence(w0p_mm, theta_mrad, sprime_mm)
                else:
                    # Unstable fit → clear cache & disable
                    self._last_divergence = None
                    if self.rb_div.isChecked():
                        self.btn_width.setEnabled(False)
                    line2 = "fit unstable"
            except Exception:
                self._last_divergence = None
                if self.rb_div.isChecked():
                    self.btn_width.setEnabled(False)

        else:  # m2
            dz_mm = getattr(self.plan, "dz_mm", None)
            z_mm = xs * dz_mm if dz_mm else xs
            w_mm = ys

            if z_mm.size >= 5:
                W2 = w_mm**2
                c, b, a = np.polyfit(z_mm, W2, 2)  # [c2, c1, c0]
                W2_fit = a + b*z_mm + c*z_mm**2

                # Fit quality
                ss_res = float(np.sum((W2 - W2_fit)**2))
                ss_tot = float(np.sum((W2 - W2.mean())**2)) if W2.size > 1 else 0.0
                R2 = 1.0 - ss_res/ss_tot if ss_tot > 0 else 1.0

                lam_mm = ((getattr(self.plan, "lambda_nm", None) or 632.8) * 1e-6)

                w0_sq = a - (b*b)/(4.0*c)
                if c > 0 and w0_sq > 0 and lam_mm > 0:
                    w0 = math.sqrt(w0_sq)
                    M2 = (math.pi/lam_mm) * w0 * math.sqrt(c)  # corrected formula
                    z0 = -b/(2.0*c)
                    zR = math.pi * w0*w0 / (M2 * lam_mm)

                    # Cache for beam-width dialog
                    self._last_m2 = {"M2": M2, "w0_mm": w0, "z0_mm": z0, "zR_mm": zR, "R2": R2}
                    if self.rb_m2.isChecked():
                        self.btn_width.setEnabled(True)
                    if getattr(self, "_width_dlg", None) and self._width_dlg.isVisible():
                        self._width_dlg.show_m2(w0, M2=M2, z0_mm=z0, zR_mm=zR)

                    line2 = f"M²≈{M2:.2f}   w0≈{w0:.3f} mm   z0≈{z0:.1f} mm   zR'≈{zR:.1f} mm   R²={R2:.3f}"
                else:
                    self._last_m2 = None
                    if self.rb_m2.isChecked():
                        self.btn_width.setEnabled(False)
                    line2 = "fit unstable"
            else:
                self._last_m2 = None
                if self.rb_m2.isChecked():
                    self.btn_width.setEnabled(False)

        # ← IMPORTANT: update label for ALL modes
        self.lbl_metrics.setText(line1 + ("\n" + line2 if line2 else ""))
        
        

    # -------- slots --------
    @Slot()
    def refresh_ports(self):
        self.cmb_ports.clear()
        ports = list_ports()
        if not ports:
            self.cmb_ports.addItem("(no ports)")
        else:
            for p in ports: self.cmb_ports.addItem(p)
        self.log("Ports refreshed.")

    @Slot()
    def on_connect_clicked(self):
        if self.chk_sim.isChecked():
            self.lbl_status.setText("Simulator active")
            self.log("Using simulator backend; no serial connection.")
            self._update_power_window_button()
            self._ensure_timer_state()
            return
        port = self.cmb_ports.currentText()
        if not port or port.startswith("("):
            QMessageBox.warning(self, "No port", "Select a serial port or enable Simulator.")
            return
        try:
            self.serial = SerialLink(port, 115200)
            self.lbl_status.setText(f"Connected: {port}")
            self.log(f"Connected to {port}")
            self.lbl_power.setText("Power Meter: waiting for stream...")
            if self._power_dlg is not None:
                self._power_dlg.reset_display()
            self._update_power_window_button()
            self._ensure_timer_state()
        except Exception as e:
            QMessageBox.critical(self, "Serial error", str(e))
            self.log(f"Serial error: {e}")

    @Slot(bool)
    def on_sim_toggled(self, checked: bool):
        self.simulate = checked
        self._update_param_ui()
        if checked:
            self.lbl_status.setText("Simulator active")
            self.log("Simulator enabled.")
            self.lbl_power.setText("Power Meter: -")
            if self._power_dlg is not None:
                self._power_dlg.reset_display()
        else:
            self.lbl_status.setText("Disconnected")
            self.log("Simulator disabled; connect to a serial port.")
        self._update_power_window_button()
        self._ensure_timer_state()

    @Slot()
    def on_mode_changed(self):
        # Determine the selected mode
        if self.rb_pol.isChecked():
            new_mode, plot_mode = "Polarization", "polarization"
        elif self.rb_wav.isChecked():
            new_mode, plot_mode = "Wavelength", "wavelength"
        elif self.rb_div.isChecked():
            new_mode, plot_mode = "Divergence", "divergence"
        else:
            new_mode, plot_mode = "M2", "m2"
            
        # Enable width button in Divergence/M2 when we have a fit
        if self.rb_div.isChecked():
            self.btn_width.setEnabled(bool(self._last_divergence))
        elif self.rb_m2.isChecked():
            self.btn_width.setEnabled(bool(self._last_m2))
        else:
            self.btn_width.setEnabled(False)
            
        if not (self.rb_div.isChecked() or self.rb_m2.isChecked()):
            if getattr(self, "_width_dlg", None):
                self._width_dlg.hide()

        # If nothing actually changed, still refresh ellipse button state and return
        if new_mode == getattr(self, "_current_mode", None):
            self.btn_ellipse.setEnabled(
                self.rb_pol.isChecked() and bool(getattr(self.plan, "last_stokes", None))
            )
            return
        self._current_mode = new_mode

        # Apply UI updates once
        self.canvas.set_mode(plot_mode)
        self.lbl_mode.setText(f"Mode: {new_mode}")
        self.lbl_metrics.setText("Metrics: —")
        self.log(f"Mode switched to {new_mode}.")
        self._update_param_ui()

        # Enable/disable the ellipse button based on mode + Stokes availability
        self.btn_ellipse.setEnabled(
            self.rb_pol.isChecked() and bool(getattr(self.plan, "last_stokes", None))
        )

        # Optional: hide the ellipse window when leaving Polarization
        if not self.rb_pol.isChecked():
            if getattr(self, "_ellipse_dlg", None):
                self._ellipse_dlg.hide()
            if getattr(self, "_poincare_dlg", None):
                self._poincare_dlg.hide()

    @Slot()
    def on_start_clicked(self):
        points = int(self.spin_points.value())
        settle = float(self.spin_settle.value())
        rate   = int(self.spin_rate.value())
        backend = self.sim if self.simulate else self.serial
        use_mcu_params = not self.simulate

        if self.rb_pol.isChecked():
            if (not self.simulate) and (backend is None or not hasattr(backend, "send")):
                QMessageBox.warning(self, "No serial connection",
                                    "Polarization (hardware) needs the STM32 polarimeter.\n"
                                    "Connect a serial port or enable Simulator.")
                return
            self.btn_ellipse.setEnabled(False)
            self.plan = PolarizationPlan(points=points, settle_s=settle,
                                        sample_rate=rate, backend=backend,
                                        use_mcu_params=use_mcu_params)

        elif self.rb_wav.isChecked():
            self.plan = WavelengthPlan(points=points, settle_s=settle,
                                    sample_rate=rate, backend=backend,
                                    use_mcu_params=use_mcu_params,
                                    dz_mm=(self.spin_wav_dz.value() if self.simulate else None))

        elif self.rb_div.isChecked():
            self.plan = DivergencePlan(points=points, settle_s=settle,
                                    sample_rate=rate, backend=backend,
                                    use_mcu_params=use_mcu_params,
                                    f_mm=(self.spin_div_f.value() if self.simulate else None),
                                    dz_mm=(self.spin_div_dz.value() if self.simulate else None),
                                    lambda_nm=(self.spin_div_lambda.value() if self.simulate else None),
                                    m2_factor=(self.spin_div_M2.value() if self.simulate else None))
        else:
            self.plan = M2Plan(points=points, settle_s=settle,
                       sample_rate=rate, backend=backend,
                       use_mcu_params=use_mcu_params,
                       lambda_nm=(self.spin_m2_lambda.value() if self.simulate else None),
                       dz_mm=(self.spin_m2_dz.value() if self.simulate else None))

        # start run
        self.canvas.clear()
        try:
            self.plan.start()
        except Exception as e:
            QMessageBox.critical(self, "Start error", str(e))
            return
        self._run_active = True
        self._ensure_timer_state()
        self._last_stokes = None
        self.log("Run started.")

    @Slot()
    def on_abort_clicked(self):
        if self.plan:
            self.plan.abort()
            self._run_active = False
            self._ensure_timer_state()
            self.log("Run aborted.")

    @Slot()
    def on_save_clicked(self):
        if not self.plan:
            QMessageBox.information(self, "Nothing to save", "Run something first.")
            return

        # Polarization: save DoP series + last STOKES
        if self.rb_pol.isChecked():
            path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "polarization.csv", "CSV Files (*.csv)")
            if not path: return
            try:
                import csv
                with open(path, "w", newline="") as f:
                    w = csv.writer(f)
                    w.writerow(["sample_idx","DoP"])
                    for idx, dop in self.plan.buffer:
                        w.writerow([idx, f"{dop:.6f}"])
                    w.writerow([])
                    st = self.plan.last_stokes or {}
                    w.writerow(["last_S0","last_S1","last_S2","last_S3"])
                    w.writerow([st.get("S0",""), st.get("S1",""), st.get("S2",""), st.get("S3","")])
                self.log(f"Saved: {path}")
            except Exception as e:
                QMessageBox.critical(self, "Save error", str(e))
            return

        # Other modes: save x,y
        if not self.plan.buffer:
            QMessageBox.information(self, "Nothing to save", "No data yet.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save CSV", "lba_run.csv", "CSV Files (*.csv)")
        if not path: return
        try:
            import csv
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["idx","x","y"])
                for i,(x,y) in enumerate(self.plan.buffer):
                    w.writerow([i,x,y])
            self.log(f"Saved: {path}")
        except Exception as e:
            QMessageBox.critical(self, "Save error", str(e))

    @Slot()
    def on_show_ellipse(self):
        if not self.plan or not hasattr(self.plan, "last_stokes") or not self.plan.last_stokes:
            QMessageBox.information(self, "No Stokes yet", "Run Polarization (hardware) first.")
            return
        if self._ellipse_dlg is None:
            self._ellipse_dlg = PolarizationEllipseDialog(self)
        self._ellipse_dlg.show()
        d = self.plan.last_stokes
        self._ellipse_dlg.draw_from_stokes(d["S0"], d["S1"], d["S2"], d["S3"], convention="ieee")
        
    @Slot()
    def on_show_width(self):
        if self._width_dlg is None:
            self._width_dlg = BeamWidthDialog(self)

        if self.rb_div.isChecked():
            if not self._last_divergence:
                QMessageBox.information(self, "No width yet", "Run Divergence to estimate w₀′ first.")
                return
            d = self._last_divergence
            self._width_dlg.show_divergence(d["w0p_mm"], d.get("theta_mrad"), d.get("sprime_mm"))
            self._width_dlg.show()
            self._width_dlg.raise_(); self._width_dlg.activateWindow()

        elif self.rb_m2.isChecked():
            if not self._last_m2:
                QMessageBox.information(self, "No waist yet", "Run M² to estimate w₀ first.")
                return
            m = self._last_m2
            if m["w0_mm"] != m["w0_mm"]:  # NaN check
                QMessageBox.information(self, "Fit not ready", "Waist fit is unstable; collect more data.")
                return
            self._width_dlg.show_m2(m["w0_mm"], M2=m.get("M2"), z0_mm=m.get("z0_mm"), zR_mm=m.get("zR_mm"))
            self._width_dlg.show()
            self._width_dlg.raise_(); self._width_dlg.activateWindow()
        else:
            QMessageBox.information(self, "Not available", "Beam width view is available in Divergence or M² modes.")
    
    @Slot()
    def on_show_poincare(self):
        # need current polarization Stokes
        if not self.plan or not hasattr(self.plan, "last_stokes") or not self.plan.last_stokes:
            QMessageBox.information(self, "No Stokes yet", "Run Polarization first.")
            return
        if self._poincare_dlg is None:
            self._poincare_dlg = PoincareSphereDialog(self)
        d = self.plan.last_stokes
        self._poincare_dlg.show()
        self._poincare_dlg.raise_(); self._poincare_dlg.activateWindow()
        self._poincare_dlg.draw_stokes(d["S0"], d["S1"], d["S2"], d["S3"])

    @Slot()
    def on_show_power_window(self):
        if self._power_dlg is None:
            self._power_dlg = PowerMeterDialog(self)

        self._power_dlg.show()
        self._power_dlg.raise_()
        self._power_dlg.activateWindow()

    @Slot()
    def on_timer(self):
        self._poll_power_stream()
        
        if not self.plan: return

        new_pts = self.plan.poll()
        if new_pts:
            xs, ys = zip(*new_pts)
            self.canvas.extend(xs, ys)

        # Live Stokes/ellipse for Polarization (hardware)
        if self.rb_pol.isChecked() and hasattr(self.plan, "last_stokes") and self.plan.last_stokes:
            d = self.plan.last_stokes
            S0,S1,S2,S3 = d["S0"], d["S1"], d["S2"], d["S3"]
            s0 = max(abs(S0), 1e-12); s1,s2,s3 = S1/s0, S2/s0, S3/s0
            DoLP = float(np.hypot(s1, s2))
            DoCP = float(abs(s3))
            DoP  = float(np.sqrt(s1*s1 + s2*s2 + s3*s3))
            import math
            psi_deg = 0.5 * math.degrees(math.atan2(s2, s1))
            chi_deg = 0.5 * math.degrees(math.asin(max(-1.0, min(1.0, s3))))
            self.lbl_metrics.setText(
                f"S0={S0:.3f} S1={S1:.3f} S2={S2:.3f} S3={S3:.3f}\n"
                f"DoLP={DoLP:.3f} DoCP={DoCP:.3f} DoP={DoP:.3f}  "
                f"ψ={psi_deg:.1f}° χ={chi_deg:.1f}°"
            )
            if not self.btn_ellipse.isEnabled():
                self.btn_ellipse.setEnabled(True) 
            if not self.btn_poincare.isEnabled():
                self.btn_poincare.setEnabled(True)
                
            if self._ellipse_dlg is not None and self._ellipse_dlg.isVisible():
                self._ellipse_dlg.draw_from_stokes(S0,S1,S2,S3, convention="ieee")
                
            if self._poincare_dlg is not None and self._poincare_dlg.isVisible():
                self._poincare_dlg.draw_stokes(S0,S1,S2,S3)
        else:
            # Non-polarization modes: show generic metrics
            mode = "wavelength" if self.rb_wav.isChecked() else ("divergence" if self.rb_div.isChecked() else "m2")
            self._update_metrics_generic(mode)
            
            # Decide when the Beam Profile button can be used.
            # We require BOTH a valid Divergence fit and a valid M² fit.
            if (self.rb_div.isChecked() or self.rb_m2.isChecked()) and \
            self._last_divergence and self._last_m2:
                self.btn_profile.setEnabled(True)
            else:
                self.btn_profile.setEnabled(False)

        if self.plan.done and self._run_active:
            self._run_active = False
            self._ensure_timer_state()
            self.log("Run complete.")

            
    @Slot()
    def on_apply_pol_profile(self):
        if not self.chk_sim.isChecked() or not self.rb_pol.isChecked():
            QMessageBox.information(self, "Not available",
                                    "Custom polarization inputs are only available in Simulator mode (Polarization).")
            return
        try:
            angles = self._parse_floats(self.edit_pol_angles.text())
            powers = self._parse_floats(self.edit_pol_powers.text())
            if len(angles) != len(powers) or len(angles) == 0:
                raise ValueError("Angles and powers must be the same non-zero length.")

            # optional normalization
            if self.chk_pol_norm.isChecked():
                m = max(powers)
                if m > 0:
                    powers = [p / m for p in powers]

            # set simulator noise
            self.sim.set_noise(float(self.spin_pol_noise.value()))

            # load and activate
            self.sim.load_polarization_profile("UserProfile", angles, powers, use_as_active=True)
            self.log("Applied custom polarization profile (Simulator).")
            QMessageBox.information(self, "Applied", "Custom profile applied.\nStart a run to see it plotted.")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))

    @Slot()
    def on_reset_pol_profile(self):
        try:
            # back to harmonic model
            self.sim.set_active_profile(None)
            self.log("Switched to harmonic polarization model (Simulator).")
            QMessageBox.information(self, "Harmonic Model", "Using harmonic model again.")
        except Exception as e:
            QMessageBox.critical(self, "Error", str(e))
            
    @Slot()
    def on_show_profile(self):
        # Prefer real knife-edge traces if the plan provides them:
        # Expect the plan to optionally set:
        #   self.plan.knife_x = (x_positions_mm, cumulative_power)
        #   self.plan.knife_y = (y_positions_mm, cumulative_power)
        have_knife = hasattr(self.plan, "knife_x") and hasattr(self.plan, "knife_y")

        # ----- 1) TRUE MEASURED PROFILE FROM KNIFE-EDGE -----
        if have_knife:
            xX, PX = self.plan.knife_x
            xY, PY = self.plan.knife_y

            xX, IX = knife_edge_to_profile(xX, PX)
            xY, IY = knife_edge_to_profile(xY, PY)

            fitX = fit_gaussian_1d(xX, IX)
            fitY = fit_gaussian_1d(xY, IY)

            # D4σ (log to metrics panel for traceability)
            cx, rx = d4sigma_radius(xX, IX)
            cy, ry = d4sigma_radius(xY, IY)
            self.log(
                f"Beam Profile (knife-edge): "
                f"X w1/e2={fitX[1]:.3f}mm R²={fitX[4]:.3f}, D4σ={rx:.3f}mm; "
                f"Y w1/e2={fitY[1]:.3f}mm R²={fitY[4]:.3f}, D4σ={ry:.3f}mm"
            )

            # 2D preview using fit radii (fallback to D4σ if needed)
            wx = fitX[1] if fitX and fitX[4] > 0.9 else (rx if np.isfinite(rx) else 0.5)
            wy = fitY[1] if fitY and fitY[4] > 0.9 else (ry if np.isfinite(ry) else 0.5)
            Xvec, Yvec, Z = gaussian_2d_preview(
                cx if np.isfinite(cx) else fitX[0],
                wx,
                cy if np.isfinite(cy) else fitY[0],
                wy,
                amp=1.0,
            )

            if self._profile_dlg is None:
                self._profile_dlg = BeamProfileDialog(self)
            self._profile_dlg.show_profiles(xX, IX, fitX, xY, IY, fitY,
                                            heatmap=(Xvec, Yvec, Z))
            self._profile_dlg.show()
            self._profile_dlg.raise_()
            self._profile_dlg.activateWindow()
            return

        # ----- 2) SYNTHETIC PROFILE FROM BOTH DIVERGENCE + M² -----
        # Require both cached fits for a "complete" Gaussian beam description.
        if not (self._last_divergence and self._last_m2):
            QMessageBox.information(
                self,
                "Beam Profile Unavailable",
                "Beam profile synthesis requires both a Divergence run and an M² / z-scan.\n\n"
                "Run Divergence first, then run M², then click 'Show Beam Profile'.",
            )
            return

        w0_mm   = float(self._last_m2.get("w0_mm", 0.0) or 0.0)
        M2_val  = self._last_m2.get("M2", None)
        w0p_mm  = self._last_divergence.get("w0p_mm", None)
        theta_m = self._last_divergence.get("theta_mrad", None)

        if w0_mm <= 0:
            QMessageBox.information(
                self,
                "Beam Profile Unavailable",
                "M² fit did not yield a valid waist radius w₀. "
                "Re-run the M² / z-scan with good data.",
            )
            return

        # Use M² waist for X; divergence-derived w0' for Y to reflect any ellipticity.
        if w0p_mm is not None and w0p_mm > 0:
            wx = w0_mm
            wy = w0p_mm
        else:
            wx = wy = w0_mm

        # Build synthetic 1D profiles (normalized) across ±3·w in each axis
        xX = np.linspace(-3.0 * wx, 3.0 * wx, 401)
        xY = np.linspace(-3.0 * wy, 3.0 * wy, 401)
        IX = np.exp(-2.0 * (xX ** 2) / (wx * wx))
        IY = np.exp(-2.0 * (xY ** 2) / (wy * wy))

        fitX = fit_gaussian_1d(xX, IX)
        fitY = fit_gaussian_1d(xY, IY)
        cx, rx = d4sigma_radius(xX, IX)
        cy, ry = d4sigma_radius(xY, IY)

        # 2D pseudo-image at the waist plane
        Xvec, Yvec, Z = gaussian_2d_preview(0.0, wx, 0.0, wy, amp=1.0)

        # Log how we combined Divergence + M²
        msg = f"Beam Profile (Divergence+M²): w0={w0_mm:.3f} mm"
        if theta_m is not None:
            msg += f", θ≈{theta_m:.2f} mrad"
        if M2_val is not None:
            msg += f", M²≈{M2_val:.2f}"
        if w0p_mm is not None:
            msg += f", w0'≈{w0p_mm:.3f} mm"
        self.log(msg)

        if self._profile_dlg is None:
            self._profile_dlg = BeamProfileDialog(self)
        self._profile_dlg.show_profiles(xX, IX, fitX, xY, IY, fitY,
                                        heatmap=(Xvec, Yvec, Z))
        self._profile_dlg.show()
        self._profile_dlg.raise_()
        self._profile_dlg.activateWindow()


            
            


