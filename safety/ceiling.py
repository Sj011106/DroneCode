# =====================================================================================
# safety/ceiling.py  -  Altitude Ceiling Advisor: "the system knows its own limit
#                       before it reaches it"                           (OWNER: Ayush, task A4)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The higher we fly, the more ground each camera pixel covers, so the same small
#   tracking error becomes more metres. Our tilt correction also has a small leftover
#   error (the IMU's roll/pitch is never perfect), and that too grows with height:
#     pixel part : pixel error x height / focal length      (0.6 px x 40 m / 208 px = 12 cm)
#     tilt part  : height x tan(leftover tilt error)         (40 m x tan(0.5 deg)      = 35 cm)
#   Every frame this file predicts our position error in metres from what is happening
#   NOW (height, focal length, tracking quality), compares it with the hold limits
#   (config.py HOLD_*, placeholders until Stage 2), and logs "predicted max safe height
#   here". Before the next climb step (10 -> 20 -> 40 -> 60 m, docs/WORKLIST.md 3.8) it
#   warns if that step is predicted to fail. It ONLY advises: it never changes what is
#   sent to ArduPilot (tests/test_ceiling.py proves the sent messages are identical).
#
# HOW IT WORKS (technical)
#   sigma_px = link.drone_link.odometry_sigma_px(quality): the SAME pixel-noise model we
#   already use for the ODOMETRY covariance (0.5 px at quality 100, 3 px at quality 1).
#   Horizontal RMS error (2-D):  e(h) = h * k,  k = sqrt((sigma_px / f)^2 + 2 tan^2(sigma_tilt))
#   (the factor 2: roll and pitch errors each move one axis). Because e grows linearly
#   with h, the ceiling has a closed form. Two limits, both from config:
#     RMS rule : e(h)            <= HOLD_RMS_DRIFT_M / CEILING_MARGIN
#     max rule : 3 * e(h)        <= HOLD_MAX_DRIFT_M / CEILING_MARGIN
#   3x: for random 2-D noise with RMS e, a distance beyond 3e has probability exp(-9) =
#   0.012% per frame (~0.1 times in a 30 s hold at 30 fps). CEILING_MARGIN = 1.5 leaves a
#   third of each limit for what vision can't see (wind, the controller).
#   sigma_tilt = CEILING_TILT_SIGMA_DEG: 0.5 deg per axis, SIMULATED (ArduCopter SITL, EKF
#   attitude vs truth: roll 0.62-0.69 deg, pitch 0.25 deg, 16 Nov). Measure it in flight.
#   Inputs are smoothed with a median over the last CEILING_WINDOW trusted frames.
#
# WHERE IT PLUGS IN
#   safety/side_monitors.py calls update() once per frame from vision_hold.py, after the
#   ODOMETRY message is already sent. Switch: config.CEILING_ADVISOR.
#
# REFERENCES
#   - Pinhole model, metres = pixels x height / focal: Hartley & Zisserman, "Multiple
#     View Geometry", 2nd ed., 2004, ch. 6.
#   - Rayleigh distribution of 2-D Gaussian error radius: P(r > c * r_rms) = exp(-c^2).
#   - tests/test_altitude.py (pixel error ~0.6 px at every height).
# =====================================================================================

import math
from collections import deque

import numpy as np

import config

MAX_OVER_RMS = 3.0          # max distance in a hold ~ 3 x RMS for random 2-D noise (see header)


def sigma_px(quality):
    """The team's pixel-noise model: the SAME function that sets our ODOMETRY covariance."""
    from link.drone_link import odometry_sigma_px
    return odometry_sigma_px(min(max(quality, 1), 100))


def k_factor(sig_px, focal_px, tilt_deg):
    """Metres of RMS error per metre of height."""
    t = math.tan(math.radians(tilt_deg))
    return math.sqrt((sig_px / focal_px) ** 2 + 2 * t * t)


def predicted_rms_m(height_m, sig_px, focal_px, tilt_deg):
    return height_m * k_factor(sig_px, focal_px, tilt_deg)


def max_safe_height(sig_px, focal_px, tilt_deg, rms_limit=None, max_limit=None, margin=None):
    """Highest height where the predicted error still fits the limits (inf if none set)."""
    margin = config.CEILING_MARGIN if margin is None else margin
    rms_limit = config.HOLD_RMS_DRIFT_M if rms_limit is None else rms_limit
    max_limit = config.HOLD_MAX_DRIFT_M if max_limit is None else max_limit
    k = k_factor(sig_px, focal_px, tilt_deg)
    caps = []
    if rms_limit:
        caps.append(rms_limit / (margin * k))
    if max_limit:
        caps.append(max_limit / (margin * MAX_OVER_RMS * k))
    return min(caps) if caps else math.inf


class CeilingAdvisor:
    COLUMNS = ["ceil_height_m", "ceil_sigma_px", "ceil_pred_rms_m", "ceil_pred_max_m",
               "ceil_max_safe_m", "ceil_next_step_m", "ceil_next_pred_max_m", "ceil_advice"]

    def __init__(self, tilt_deg=None, steps=None, window=None):
        self.tilt_deg = config.CEILING_TILT_SIGMA_DEG if tilt_deg is None else tilt_deg
        self.steps = sorted(config.CEILING_CLIMB_STEPS_M if steps is None else steps)
        self.q = deque(maxlen=window or config.CEILING_WINDOW)
        self.h = deque(maxlen=window or config.CEILING_WINDOW)
        self.advice = "WAIT"
        self.last = {}

    def next_step(self, h):
        for s in self.steps:
            if s > h * 1.1:                       # a step must be >10% above where we are
                return s
        return None

    def update(self, height_m, focal_px, quality):
        """One frame. Returns a dict (the CSV columns) with the prediction and the advice."""
        if height_m is None or not np.isfinite(height_m) or height_m <= 0 or not focal_px or focal_px <= 0:
            return self._out(None, None, "NO HEIGHT")
        if quality > 0:
            self.q.append(quality)
            self.h.append(height_m)
        if not self.q:
            return self._out(height_m, None, "NO TRUSTED FRAMES")
        q = float(np.median(self.q))
        h = float(np.median(self.h))
        sp = sigma_px(q)
        rms = predicted_rms_m(h, sp, focal_px, self.tilt_deg)
        safe = max_safe_height(sp, focal_px, self.tilt_deg)
        nxt = self.next_step(h)
        nxt_max = MAX_OVER_RMS * predicted_rms_m(nxt, sp, focal_px, self.tilt_deg) if nxt else None
        if h > safe:
            advice = "ABOVE CEILING - DESCEND"
        elif nxt is not None and nxt > safe:
            advice = f"STAY - next step {nxt:g} m predicted to fail"
        else:
            advice = "CLIMB OK" if nxt is not None else "OK (top step)"
        return self._out(h, dict(sigma=sp, rms=rms, maxd=MAX_OVER_RMS * rms, safe=safe, nxt=nxt, nxt_max=nxt_max),
                         advice)

    def _out(self, h, p, advice):
        self.advice = advice
        f = lambda x, d=2: "" if x is None or (isinstance(x, float) and not math.isfinite(x)) else f"{x:.{d}f}"
        self.last = dict(zip(self.COLUMNS, [
            f(h, 1), f(p and p["sigma"]), f(p and p["rms"]), f(p and p["maxd"]), f(p and p["safe"], 1),
            "" if not p or p["nxt"] is None else f"{p['nxt']:g}", f(p and p["nxt_max"]), advice]))
        self.last["_safe"] = p["safe"] if p else None
        self.last["_rms"] = p["rms"] if p else None
        return self.last

    def message(self):
        p = self.last
        if not p.get("ceil_pred_rms_m"):
            return f"CEILING: {self.advice}"
        tag = "" if config.HOLD_LIMITS_OFFICIAL else " (PLACEHOLDER limits)"
        return (f"CEILING: at {p['ceil_height_m']} m predicted error RMS {p['ceil_pred_rms_m']} m / "
                f"max {p['ceil_pred_max_m']} m; predicted max safe height here {p['ceil_max_safe_m']} m"
                f"{tag} -> {self.advice}")
