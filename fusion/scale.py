# =====================================================================================
# fusion/scale.py  -  height above the floor from motion (bench, no rangefinder)   BETAFLIGHT CHANGE
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Turning pixels into metres needs the height above the floor. On the bench the judge
#   holds the aircraft "about 1 m" up (Stage 2 section 05) and we cannot measure exactly
#   how high: the barometer only knows CHANGES (and is noisy: +-0.3-0.6 m measured).
#   But while the judge carries the aircraft it sways a little with every step. The
#   accelerometer feels that sway in real metres; the camera sees the same sway as a
#   fraction of the height (5 cm at 1 m looks like 5% of the height; at 2 m like 2.5%).
#   Compare the two and you get the height. This is the "inertial" half of visual-inertial
#   odometry, and it is what turns the bench guess (1.0 m) into a measurement.
#   OFF by default (config.VIO_HEIGHT): the status line prints its estimate ("vio h=") so
#   the team can check it against a tape measure in a rehearsal before trusting it.
#
# HOW IT WORKS (technical)
#   ImuIntegrator: raw accelerometer (Betaflight MSP_RAW_IMU, m/s^2, sensor axes) minus a slow
#     gravity estimate (low-pass, tau 1.5 s), integrated twice with a slow leak -> "IMU
#     position" (drifts; only its wiggles are used).
#   ScaleEstimator: the SAME band-pass filter (3 x high-pass 0.35 Hz + low-pass 4 Hz) on the
#     IMU position and on the vision position in "heights" (metres / assumed height), so
#     bias, drift and gravity leak cancel and both have the same delay. Then least squares
#     over the last ~8 s: imu = h * vision. The axes are matched automatically: the pairing
#     (and sign) of IMU and camera axes with the strongest agreement is used, so the
#     sensor's mounting and sign conventions do not matter. A sigma comes from the fit.
#   Simulation (prototype, 300 carries, height 0.6-1.4 m, 1 deg tilt-error bias, 3 deg axis
#   misalignment, 10% focal error): B1 within 0.25 m in 100% vs 64% with the 1.0 m guess.
#   Real hardware: NOT checked. Proof of the maths: tests/test_scale.py.
#
# REFERENCES
#   - Kelly & Sukhatme, "Visual-Inertial Sensor Fusion: Localization, Mapping and Sensor-to-
#     Sensor Self-calibration", IJRR 2011 (metric scale is observable from accelerations).
#   - Nutzi, Weiss, Scaramuzza, Siegwart, "Fusion of IMU and Vision for Absolute Scale
#     Estimation in Monocular SLAM", J. Intell. Robot. Syst. 2011.
# =====================================================================================
import itertools
import math
from collections import deque


class BandPass:
    """3 x first-order high-pass + 1 x low-pass. Identical instances on both signals = identical delay."""

    def __init__(self, f_hp=0.35, f_lp=4.0):
        self.f_hp, self.f_lp = f_hp, f_lp
        self.prev = None
        self.y = [0.0, 0.0, 0.0]
        self.lp = 0.0

    def __call__(self, x, dt):
        if self.prev is None:
            self.prev = [x, 0.0, 0.0]
            return 0.0
        a = 1.0 / (1.0 + 2 * math.pi * self.f_hp * dt)
        v = x
        for i in range(3):
            y = a * (self.y[i] + v - self.prev[i])
            self.prev[i], self.y[i] = v, y
            v = y
        b = dt / (dt + 1.0 / (2 * math.pi * self.f_lp))
        self.lp += b * (v - self.lp)
        return self.lp


class HighPass:
    """First-order high-pass with time constant tau (the same maths as 'x minus its slow average')."""

    def __init__(self, tau):
        self.tau, self.lp = tau, None

    def __call__(self, x, dt):
        if self.lp is None:
            self.lp = x
        self.lp += min(1.0, dt / self.tau) * (x - self.lp)
        return x - self.lp


class ImuIntegrator:
    """Raw accelerometer -> gravity removed -> velocity -> position (leaky, sensor axes). History kept."""

    def __init__(self, tau_gravity=1.5, tau_leak=20.0):
        self.g = None
        self.v = [0.0, 0.0, 0.0]
        self.p = [0.0, 0.0, 0.0]
        self.t = None
        self.tau_g, self.tau_leak = tau_gravity, tau_leak
        self.hist = deque(maxlen=400)

    def add(self, t, acc):
        if self.t is None:
            self.t, self.g = t, list(acc)
            self.hist.append((t, tuple(self.p)))
            return
        dt = max(1e-3, min(0.2, t - self.t))
        self.t = t
        kg, leak = dt / self.tau_g, math.exp(-dt / self.tau_leak)
        for i in range(3):
            self.g[i] += kg * (acc[i] - self.g[i])
            a = acc[i] - self.g[i]
            self.v[i] = leak * self.v[i] + a * dt
            self.p[i] = leak * self.p[i] + self.v[i] * dt
        self.hist.append((t, tuple(self.p)))

    def at(self, t):
        """IMU position closest in time to t, or None."""
        hist = list(self.hist)                            # one atomic copy: the link thread keeps appending
        if not hist:
            return None
        s = min(hist, key=lambda h: abs(h[0] - t))
        return s[1] if abs(s[0] - t) < 0.1 else None


class ScaleEstimator:
    def __init__(self, tau_s=8.0, min_motion_m=0.006, mirror=(1.5,)):
        """mirror: the IMU path removes gravity with a slow average (tau 1.5 s, ImuIntegrator). The vision path
        gets the SAME high-pass first, or the slow part of a long carry would not match (measured: 0.50 vs 0.70).
        (The 20 s leak is slow enough to ignore after the band-pass.)"""
        self.tau, self.min_motion = tau_s, min_motion_m
        self.mirror = [[HighPass(m) for m in mirror] for _ in range(3)]
        self.bp_i = [BandPass() for _ in range(3)]
        self.bp_v = [BandPass() for _ in range(3)]
        self.C = [[0.0] * 3 for _ in range(3)]            # sum imu_i * vis_j
        self.I2 = [0.0] * 3
        self.V2 = [0.0] * 3
        self.n = 0.0
        self.t = None

    def update(self, t, imu_pos, vis_norm):
        """imu_pos: 3 numbers (metres, any sensor axes, drifting is fine). vis_norm: (north/h, east/h, ln h)
        from vision with the ASSUMED height h. Call once per frame."""
        if imu_pos is None or vis_norm is None or not all(map(math.isfinite, vis_norm)):
            return
        dt = 1 / 15 if self.t is None else max(1e-3, t - self.t)
        self.t = t
        lam = math.exp(-dt / self.tau)
        a = [self.bp_i[k](imu_pos[k], dt) for k in range(3)]
        v = []
        for k in range(3):
            x = vis_norm[k]
            for f in self.mirror[k]:
                x = f(x, dt)
            v.append(self.bp_v[k](x, dt))
        for i in range(3):
            self.I2[i] = lam * self.I2[i] + a[i] * a[i]
            self.V2[i] = lam * self.V2[i] + v[i] * v[i]
            for j in range(3):
                self.C[i][j] = lam * self.C[i][j] + a[i] * v[j]
        self.n = lam * self.n + 1

    def estimate(self):
        """-> (height_m, sigma_m) or (None, inf) when the aircraft has not moved enough to tell."""
        if self.n < 30 or sum(self.V2) <= 0:
            return None, math.inf
        best = None
        for perm in itertools.permutations(range(3)):     # which IMU axis goes with which vision axis
            num = sum(abs(self.C[perm[j]][j]) for j in range(3))
            if best is None or num > best[0]:
                best = (num, perm)
        num, perm = best
        den = sum(self.V2)
        h = num / den
        imu_rms = math.sqrt(sum(self.I2[perm[j]] for j in range(3)) / self.n)
        if h <= 0.05 or imu_rms < self.min_motion:
            return None, math.inf
        res = max(sum(self.I2[perm[j]] for j in range(3)) - h * num, 1e-12) / self.n
        sigma = math.sqrt(res / den * self.n / 15.0)       # ~15 independent samples per window
        return h, max(sigma, 0.03 * h)


def fuse(prior_h, prior_sigma, vio_h, vio_sigma):
    """Inverse-variance blend of the bench guess and the motion estimate."""
    if vio_h is None or not math.isfinite(vio_sigma):
        return prior_h
    w1, w2 = 1 / prior_sigma ** 2, 1 / vio_sigma ** 2
    return (prior_h * w1 + vio_h * w2) / (w1 + w2)
