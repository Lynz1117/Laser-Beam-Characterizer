import time, math, random
import numpy as np

def _is_serial_backend(backend):
    return hasattr(backend, "send") and hasattr(backend, "recv_nowait")

class BasePlan:
    def __init__(self, points, settle_s, sample_rate, backend, use_mcu_params=False):
        self.points = int(points)
        self.settle_s = float(settle_s)
        self.sample_rate = int(sample_rate)
        self.backend = backend
        self.use_mcu_params = bool(use_mcu_params)
        self.done = False
        self.buffer = []
        self._idx = 0
        self._t_last = 0.0
        self._serial = _is_serial_backend(backend)

    def start(self):
        self.done = False
        self.buffer.clear()
        self._idx = 0
        self._t_last = time.time()
        if self._serial:
            self._start_serial()

    def abort(self):
        self.done = True
        if self._serial:
            try: self.backend.send({"op":"ABORT"})
            except Exception: pass

    def _start_serial(self): pass
    def _poll_sim(self): return []

    def _poll_serial(self):
        out = []
        while True:
            pkt = self.backend.recv_nowait()
            if pkt is None: break
            ev = pkt.get("ev")
            if ev == "SAMPLE":
                x, y = pkt.get("x"), pkt.get("y")
                if x is not None and y is not None:
                    self.buffer.append((x, y))
                    out.append((x, y))
                    self._idx += 1
                    if self.points > 0 and self._idx >= self.points:
                        self.done = True
            elif ev == "DONE":
                self.done = True
            elif ev == "ERROR":
                self.done = True
        return out

    def poll(self):
        if self.done: return []
        return self._poll_serial() if self._serial else self._poll_sim()


# ---------- UPDATED: Polarization works in both simulator & hardware ----------
class PolarizationPlan(BasePlan):
    """
    Hardware: consume SAMPLE packets with (theta_deg, I), estimate Stokes and DoP.
    Simulator: generate I(θ) samples, estimate Stokes with discrete sums,
               and plot DoP as it converges.
    """
    def __init__(self, points, settle_s, sample_rate, backend, use_mcu_params=False):
        super().__init__(points, settle_s, sample_rate, backend, use_mcu_params)
        self.last_stokes = None
        self._thetaI = []  # list of (theta_deg, I) for both sim & hardware

    # ---- tell MCU to start polarization scan ----
    def _start_serial(self):
        # ask MCU to run N points (if points <= 0, let MCU use its default)
        msg = {"op": "RUN_POLARIMETER"}
        if self.points > 0:
            msg["N"] = int(self.points)
        self.backend.send(msg)
        
    @staticmethod
    def _stokes_discrete(thetaI):
        if len(thetaI) < 3:
            # very rough initial estimate
            Ivals = np.array([v for _, v in thetaI], float)
            S0 = 2*np.mean(Ivals)
            return {"S0": S0, "S1": 0.0, "S2": 0.0, "S3": 0.0}

        th = np.radians([t for t, _ in thetaI])
        I  = np.array([v for _, v in thetaI], float)

        c2 = np.cos(2*th)
        s2 = np.sin(2*th)
        c4 = np.cos(4*th)
        s4 = np.sin(4*th)

        A = np.column_stack([np.ones_like(th), c2, s2, c4, s4])
        coeffs, *_ = np.linalg.lstsq(A, I, rcond=None)
        a0, a2c, a2s, a4c, a4s = coeffs

        S0 = 2*(a0 - a4c)
        if abs(S0) < 1e-9:
            return None

        S1 = 4*a4c
        S2 = 4*a4s
        S3 = -2*a2s

        return {"S0": float(S0), "S1": float(S1),
                "S2": float(S2), "S3": float(S3)}

        
    def poll(self):
        if self.done:
            return []

        if self._serial:
            # ---------- HARDWARE PATH ----------
            out = []
            while True:
                pkt = self.backend.recv_nowait()
                if pkt is None:
                    break

                ev = pkt.get("ev")
                if ev == "SAMPLE":
                    # expect: {"ev":"SAMPLE","theta":..., "I":...}
                    theta = pkt.get("theta")
                    I = pkt.get("I", pkt.get("y"))
                    if theta is None or I is None:
                        continue

                    theta = float(theta)
                    I = float(I)

                    self._thetaI.append((theta, I))

                    st = self._stokes_discrete(self._thetaI)
                    if st is not None:
                        self.last_stokes = {k: float(v) for k, v in st.items()}
                        S0, S1, S2, S3 = st["S0"], st["S1"], st["S2"], st["S3"]
                        s0 = abs(S0) if abs(S0) > 1e-12 else 1.0
                        dop = math.sqrt((S1 / s0) ** 2 +
                                        (S2 / s0) ** 2 +
                                        (S3 / s0) ** 2)

                        self.buffer.append((self._idx, dop))
                        out.append((self._idx, dop))
                        self._idx += 1

                        if self.points > 0 and self._idx >= self.points:
                            self.done = True

                elif ev in ("DONE", "ERROR"):
                    self.done = True

            return out

        else:
            # ---------- SIMULATOR PATH (UNCHANGED) ----------
            now = time.time()
            if now - self._t_last < max(0.005, self.settle_s / max(1, self.points)):
                return []
            self._t_last = now

            out = []
            for _ in range(3):  # a few samples per tick
                if self._idx >= self.points:
                    self.done = True
                    break
                theta_deg, I = self.backend.polarization_point(self._idx, self.points)
                self._thetaI.append((theta_deg, I))

                st = self._stokes_discrete(self._thetaI)
                if st is not None:
                    self.last_stokes = {k: float(v) for k, v in st.items()}
                    S0, S1, S2, S3 = st["S0"], st["S1"], st["S2"], st["S3"]
                    s0 = abs(S0) if abs(S0) > 1e-12 else 1.0
                    dop = math.sqrt((S1 / s0) ** 2 +
                                    (S2 / s0) ** 2 +
                                    (S3 / s0) ** 2)
                    self.buffer.append((self._idx, dop))
                    out.append((self._idx, dop))
                self._idx += 1
            return out


# -------- WAVELENGTH (FFT Michelson) --------
class WavelengthPlan(BasePlan):
    """
    Michelson wavelength via FFT of I(z):
      I(z) ~ 1 + V cos(4π z / λ)
    Dominant spatial frequency f (cycles/mm) -> λ = 2 / f
    Requires a spatial sampling scale:
      - dz_mm (mm/step) OR
      - vel_mm_s and fs (mm/s and samples/s) so dz_mm = vel/fs
    """
    def __init__(self, points, settle_s, sample_rate, backend,
                 use_mcu_params=False, dz_mm=None):
        super().__init__(points, settle_s, sample_rate, backend, use_mcu_params)
        self.dz_mm = dz_mm          # preferred (mm per x-index)
        self.vel_mm_s = None        # optional (from MCU META)
        self.fs = None              # optional (from MCU META)
        # simulator params
        self._sim_lambda_nm = 632.8
        self._sim_contrast = 0.85
        self._sim_noise = 0.02

    def _start_serial(self):
        # MCU should stream SAMPLE (x,y) and META with either dz_mm or (vel_mm_s, fs)
        if self.use_mcu_params:
            self.backend.send({"op":"RUN_WAVELENGTH"})
        else:
            self.backend.send({"op":"RUN_WAVELENGTH",
                               "points": self.points, "rate": self.sample_rate})

    def _poll_serial(self):
        out = []
        while True:
            pkt = self.backend.recv_nowait()
            if pkt is None: break
            ev = pkt.get("ev")
            if ev == "SAMPLE":
                x = pkt.get("x")
                y = pkt.get("y")
                if x is None or y is None: continue
                self.buffer.append((x, float(y)))
                out.append((x, float(y)))
                self._idx += 1
                if self.points > 0 and self._idx >= self.points:
                    self.done = True
            elif ev == "META":
                if "dz_mm" in pkt: self.dz_mm = float(pkt["dz_mm"])
                if "vel_mm_s" in pkt: self.vel_mm_s = float(pkt["vel_mm_s"])
                if "fs" in pkt: self.fs = float(pkt["fs"])
            elif ev in ("DONE","ERROR"):
                self.done = True
        return out

    def _poll_sim(self):
        """
        Generate clean Michelson fringes vs z with slight noise:
          z = idx * dz_mm
          I = 0.5 + 0.5*V*cos(4π z / λ) + noise
        """
        out = []
        dz = self.dz_mm if self.dz_mm else 0.02  # 20 µm per step default
        lam_mm = self._sim_lambda_nm * 1e-6
        V = self._sim_contrast
        for _ in range(12):  # stream several samples per tick
            if self._idx >= self.points:
                self.done = True
                break
            z_mm = self._idx * dz
            phase = 4.0 * math.pi * z_mm / lam_mm
            y = 0.5 + 0.5 * V * math.cos(phase) + random.gauss(0.0, self._sim_noise)
            y = max(0.0, min(1.0, y))
            self.buffer.append((self._idx, y)); out.append((self._idx, y))
            self._idx += 1
        return out

    def _effective_dz_mm(self):
        """Choose sampling step in mm: dz_mm OR vel/fs if available."""
        if self.dz_mm and self.dz_mm > 0:
            return float(self.dz_mm)
        if self.vel_mm_s and self.fs and self.vel_mm_s > 0 and self.fs > 0:
            return float(self.vel_mm_s / self.fs)
        return None

    """//Change wavelength lambda to be f = inverse meters so if x axis is time need to multiple
        2/f(v) to get inverse meters where v is the velocity of the mirror movement 
    """
    
    def estimate_lambda_fft(self):
        """
        Returns (lambda_nm, f_cyc_per_mm, dz_mm) or None while insufficient.
        """
        # need enough samples
        if not self.buffer or len(self.buffer) < 16:
            return None

        dz = self._effective_dz_mm()
        if not dz or dz <= 0:
            return None

        import numpy as np

        xs = np.array([x for x, _ in self.buffer], dtype=float)
        y  = np.array([y for _, y in self.buffer], dtype=float)

        # detrend
        y = y - np.mean(y)
        if y.std() < 1e-6:
            return None

        # window
        w = np.hanning(y.size)
        yw = y * w

        # FFT: spatial domain, so d = dz
        freqs = np.fft.rfftfreq(yw.size, d=dz)   # cycles / mm
        mag   = np.abs(np.fft.rfft(yw))

        if freqs.size < 3:
            return None

        # ignore DC
        kmin = 1
        kmax = len(freqs) - 1
        if kmax - kmin < 2:
            return None

        # base peak
        k0 = np.argmax(mag[kmin:kmax]) + kmin

        # --- robust sub-bin parabolic interpolation ---
        delta = 0.0
        if 1 <= k0 < len(mag) - 1:
            a = float(mag[k0 - 1])
            b = float(mag[k0])
            c = float(mag[k0 + 1])

            denom = (a - 2.0 * b + c)
            if abs(denom) > 1e-6:
                delta = 0.5 * (a - c) / denom
                # clamp to at most half-bin; otherwise we jump to nonsense
                if delta > 0.5:
                    delta = 0.5
                elif delta < -0.5:
                    delta = -0.5
            else:
                delta = 0.0  # flat peak → don't refine

        df = freqs[1] - freqs[0]
        f_peak = freqs[k0] + delta * df   # cycles / mm

        # sanity: must be finite and positive
        if not np.isfinite(f_peak) or f_peak <= 0:
            return None

        # sanity: don’t go past Nyquist
        f_nyq = 0.5 / dz
        if f_peak > f_nyq:
            # if it happens, just say "can't estimate" — better than bad λ
            return None

        lam_mm = 2.0 / f_peak
        lam_nm = lam_mm * 1e6
        return (lam_nm, f_peak, dz)


# -------- DIVERGENCE --------
class DivergencePlan(BasePlan):
    def __init__(self, points, settle_s, sample_rate, backend,
                 use_mcu_params=False, f_mm=None, dz_mm=None,
                 lambda_nm=None, m2_factor=None):
        super().__init__(points, settle_s, sample_rate, backend, use_mcu_params)
        # meta used by GUI metrics
        self.f_mm = f_mm
        self.dz_mm = dz_mm
        self.lambda_nm = lambda_nm
        self.m2_factor = m2_factor

        # ----- simulator “truth” -----
        self._sim_w0p_mm = 0.20      # beam radius at focus (mm)
        self._sim_I0     = 1.0       # peak intensity (a.u.)
        self._sim_noise  = 0.02      # additive noise (a.u.)
        self._sim_x0_idx = None      # focus center in index units

    # center the synthetic focus near the middle of the scan
    def start(self):
        super().start()
        self._sim_x0_idx = (self.points - 1) / 2.0

    # ----- serial (hardware) -----
    def _start_serial(self):
        if self.use_mcu_params:
            self.backend.send({"op": "RUN_DIVERGENCE"})
        else:
            self.backend.send({"op": "RUN_DIVERGENCE",
                               "points": self.points, "rate": self.sample_rate})

    def _poll_serial(self):
        out = []
        while True:
            pkt = self.backend.recv_nowait()
            if pkt is None:
                break
            ev = pkt.get("ev")
            if ev == "SAMPLE":
                x, y = pkt.get("x"), pkt.get("y")
                if x is not None and y is not None:
                    x = float(x); y = float(y)
                    self.buffer.append((x, y)); out.append((x, y)); self._idx += 1
                    if self.points > 0 and self._idx >= self.points:
                        self.done = True
            elif ev == "META":
                if "f_mm"      in pkt: self.f_mm      = float(pkt["f_mm"])
                if "dz_mm"     in pkt: self.dz_mm     = float(pkt["dz_mm"])
                if "lambda_nm" in pkt: self.lambda_nm = float(pkt["lambda_nm"])
                if "M2" in pkt or "m2_factor" in pkt:
                    self.m2_factor = float(pkt.get("M2", pkt.get("m2_factor")))
            elif ev in ("DONE", "ERROR"):
                self.done = True
        return out

    # ----- simulator (software) -----
    def _poll_sim(self):
        """Gaussian around focus:
           I(x) = I0 * exp(-2 * x_mm^2 / w0p^2) + noise, x_mm = (i - i0)*dz_mm
        """
        out = []
        if self.points <= 0:
            self.done = True
            return out

        # default scale so metrics (θ, s') can be computed
        dz_mm = self.dz_mm if (self.dz_mm and self.dz_mm > 0) else 0.10

        # stream a few samples per GUI tick
        for _ in range(8):
            if self._idx >= self.points:
                self.done = True
                break

            i = float(self._idx)
            dx_idx = i - (self._sim_x0_idx if self._sim_x0_idx is not None else 0.0)
            x_mm = dx_idx * dz_mm

            w0p = self._sim_w0p_mm
            y = self._sim_I0 * math.exp(-2.0 * (x_mm * x_mm) / (w0p * w0p))
            y += random.gauss(0.0, self._sim_noise)

            self.buffer.append((i, y)); out.append((i, y))
            self._idx += 1

        return out

# -------- M2 --------
class M2Plan(BasePlan):
    def __init__(self, points, settle_s, sample_rate, backend,
                 use_mcu_params=False, lambda_nm=None, dz_mm=None):
        super().__init__(points, settle_s, sample_rate, backend, use_mcu_params)
        # metadata (from simulator UI or MCU META)
        self.lambda_nm = lambda_nm   # wavelength in nm
        self.dz_mm     = dz_mm       # mm per x-index

        # simulator-only "true" values to synthesize a clean hyperbola
        self._sim_M2 = 1.40
        self._sim_w0 = 0.20   # mm (1/e^2 radius at waist)
        self._sim_z0 = None   # waist location (mm), set at start

    def _start_serial(self):
        if self.use_mcu_params:
            self.backend.send({"op": "RUN_M2"})
        else:
            self.backend.send({"op": "RUN_M2",
                               "points": self.points, "rate": self.sample_rate})

    def _poll_serial(self):
        out = []
        while True:
            pkt = self.backend.recv_nowait()
            if pkt is None:
                break
            ev = pkt.get("ev")
            if ev == "SAMPLE":
                x, y = pkt.get("x"), pkt.get("y")  # x: position index or mm, y: radius (mm)
                if x is not None and y is not None:
                    self.buffer.append((x, y)); out.append((x, y)); self._idx += 1
                    if self.points > 0 and self._idx >= self.points:
                        self.done = True
            elif ev == "META":
                if "lambda_nm" in pkt: self.lambda_nm = float(pkt["lambda_nm"])
                if "dz_mm"     in pkt: self.dz_mm     = float(pkt["dz_mm"])
            elif ev in ("DONE", "ERROR"):
                self.done = True
        return out

    def _poll_sim(self):
        """Generate w(z) = sqrt(w0^2 + K (z - z0)^2) with light noise."""
        out = []
        lam_mm = (self.lambda_nm if self.lambda_nm else 632.8) * 1e-6  # nm->mm
        dz_mm  = self.dz_mm if self.dz_mm else 0.10
        if self._sim_z0 is None:
            self._sim_z0 = (self.points/2) * dz_mm

        K = (self._sim_M2**2 * lam_mm**2) / (math.pi**2 * self._sim_w0**2)

        for _ in range(5):  # stream a few points per poll for UI smoothness
            if self._idx >= self.points:
                self.done = True
                break
            z_mm = self._idx * dz_mm
            w = math.sqrt(self._sim_w0**2 + K * (z_mm - self._sim_z0)**2)
            w += random.gauss(0, 0.003)  # small noise
            # store x as index (consistent with other modes); metrics convert using dz_mm
            self.buffer.append((self._idx, w)); out.append((self._idx, w))
            self._idx += 1
        return out
