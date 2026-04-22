import math, random, time

class Simulator:
    """Simulator for all non-hardware modes, incl. Polarization."""
    def __init__(self, seed=1234):
        # use our own RNG so UI calls like set_noise are deterministic
        self.rng = random.Random(seed)
        self._t0 = time.time()

        # default noise (std dev)
        self._noise_sigma = 0.01

        # preset spreadsheet-ish data (playback profiles)
        self._polar_profiles = {
            "Vertical": [
                (0.0, 0.00165), (22.5, 2.15), (45.0, 4.23), (67.5, 2.07),
                (90.0, 0.00175), (112.5, 2.26), (135.0, 4.46), (157.5, 2.20)
            ],
            "Horizontal": [
                (0.0, 13.35), (22.5, 10.02), (45.0, 6.82), (67.5, 10.18),
                (90.0, 13.40), (112.5, 9.98), (135.0, 6.56), (157.5, 9.95)
            ],
            "45 Degree": [
                (0.0, 1.5), (22.5, 0.8), (45.0, 1.63), (67.5, 2.36),
                (90.0, 1.52), (112.5, 0.75), (135.0, 1.54), (157.5, 2.27)
            ],
            "RHCP": [
                (0.0, 13.35), (22.5, 10.02), (45.0, 6.82), (67.5, 10.18),
                (90.0, 13.40), (112.5, 9.98), (135.0, 6.56), (157.5, 9.95)
            ]
        }

        # active dataset name; None → use harmonic model
        self._active_profile = None

        # default harmonic stokes (S0=1)
        self._stokes = (1.0, 0.55, 0.25, 0.20)

        # optional: angle override for harmonic model
        self._angles_override = None

    # ------------------------------------------------------------
    # PUBLIC HELPERS (called from GUI)

    def set_noise(self, sigma: float):
        """Set Gaussian noise used by all simulated outputs."""
        self._noise_sigma = max(0.0, float(sigma))

    def set_stokes(self, S0=1.0, s1=0.55, s2=0.25, s3=0.20):
        """Change the synthetic Stokes used by the harmonic model."""
        self._stokes = (S0, s1, s2, s3)

    def set_angles(self, angles_deg):
        """
        Force polarization to use these exact angles (harmonic model only).
        Pass [] or None to go back to 0..180° sweep.
        """
        if not angles_deg:
            self._angles_override = None
        else:
            self._angles_override = [float(a) for a in angles_deg]

    def load_polarization_profile(self, name, angles_deg, powers, use_as_active=True):
        """
        Register a user dataset (angle, power). Used by “Apply Custom Profile”.
        """
        if len(angles_deg) != len(powers) or len(angles_deg) == 0:
            raise ValueError("angles_deg and powers must be same nonzero length")
        data = [(float(a), float(p)) for a, p in zip(angles_deg, powers)]
        self._polar_profiles[name] = data
        if use_as_active:
            self._active_profile = name

    def set_active_profile(self, name=None):
        """Switch between user/built-in datasets and the harmonic model."""
        if name is None:
            self._active_profile = None
        else:
            if name not in self._polar_profiles:
                raise KeyError(f"Profile {name!r} not found")
            self._active_profile = name

    # ------------------------------------------------------------
    # CORE SIM METHODS

    def polarization_point(self, idx, total):
        """
        Simulate I(θ) for a rotating QWP + fixed polarizer.
        If a profile is active, we just play that back.
        """
        # 1) Active profile: map idx → that list
        if self._active_profile is not None:
            data = self._polar_profiles[self._active_profile]
            n = len(data)
            if n == 1 or total <= 1:
                j = 0
            else:
                j = round((idx / (total - 1)) * (n - 1))
            j = max(0, min(n - 1, j))
            theta_deg, base = data[j]
            y = base
            if self._noise_sigma > 0:
                y += self.rng.gauss(0, self._noise_sigma)
            return float(theta_deg), max(1e-6, float(y))

        # 2) Harmonic model
        # angle selection
        if self._angles_override:
            m = len(self._angles_override)
            if m == 1 or total <= 1:
                theta_deg = self._angles_override[0]
            else:
                k = round((idx / (total - 1)) * (m - 1))
                k = max(0, min(m - 1, k))
                theta_deg = self._angles_override[k]
        else:
            theta_deg = 0.0 if total <= 1 else 180.0 * idx / (total - 1)

        th = math.radians(theta_deg)

        S0_true, s1_true, s2_true, s3_true = self._stokes

        # mapping (rotating QWP)
        a4c = 0.25 * s1_true * S0_true
        a4s = 0.25 * s2_true * S0_true
        a2s = -0.5 * s3_true * S0_true
        a0  = 0.5 * S0_true + a4c
        a2c = 0.0

        y = (a0
             + a2c * math.cos(2 * th)
             + a2s * math.sin(2 * th)
             + a4c * math.cos(4 * th)
             + a4s * math.sin(4 * th))

        if self._noise_sigma > 0:
            y += self.rng.gauss(0, self._noise_sigma)

        return theta_deg, max(1e-6, y)

    def wavelength_spectrum(self, idx, total):
        x = idx
        def peak(center, width): return math.exp(-0.5*((x-center)/width)**2)
        y = 0.65*peak(300,18) + 1.0*peak(600,22) + 0.03*self.rng.random()
        if self._noise_sigma > 0:
            y += self.rng.gauss(0, self._noise_sigma * 0.2)
        return x, y

    def divergence_point(self, idx, total):
        x = idx
        x0 = total/2
        w  = max(4.0, total/10)
        I0 = 1.0
        y  = I0 * math.exp(-2*((x-x0)/w)**2)
        if self._noise_sigma > 0:
            y += self.rng.gauss(0, self._noise_sigma)
        return x, max(1e-6, y)

    def m2_point(self, idx, total):
        x = idx
        t = time.time() - self._t0
        y = 0.5 + 0.3*math.sin(0.3*x) + 0.02*math.sin(3*x + 0.2*t)
        if self._noise_sigma > 0:
            y += self.rng.gauss(0, self._noise_sigma)
        return x, y
