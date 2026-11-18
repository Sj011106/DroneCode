# =====================================================================================
# tests/test_truth_drift.py  -  the real-drift calculator gives the right numbers
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We invent a practice flight with KNOWN answers: the drone climbs 5 -> 30 m and
#   wanders 0.5 m around the hold point (GPS truth), our vision position is off by a
#   known 0.2 m, and the autopilot's estimate is off by a known 0.1 m, each with its own
#   origin and timing. tools/truth_drift.py must report those numbers per altitude band.
#
# Run (from the repo root):  python -m tests.test_truth_drift
# =====================================================================================

import math

import numpy as np

from tools.truth_drift import drift_report, latlon_to_ne, print_report

# ---- 1. lat/lon -> metres ----
n, e = latlon_to_ne(45.0 + 1e-5, -75.0 + 1e-5, 45.0, -75.0)
print(f"Test 1: 0.00001 deg north/east at 45 deg latitude -> N {n:.3f} m, E {e:.3f} m "
      f"(expect 1.113, {1.113 * math.cos(math.radians(45)):.3f})")
assert abs(n - 1.1132) < 1e-3 and abs(e - 1.1132 * math.cos(math.radians(45))) < 1e-3

# ---- 2. synthetic practice flight ----
t_gps = np.arange(0, 100, 0.2)                                  # GPS 5 Hz
wander_n, wander_e = 0.5 * np.cos(0.3 * t_gps), 0.5 * np.sin(0.3 * t_gps)   # 0.5 m circle
truth = (t_gps, 1000 + wander_n, -300 + wander_e)               # arbitrary origin
alt = (t_gps, 5 + 25 * t_gps / 100)                             # 5 -> 30 m

t_vis = np.arange(10, 100, 1 / 30)                              # vision from t = 10 s, 30 Hz
vn = np.interp(t_vis, t_gps, wander_n) + 0.2                    # constant 0.2 m north error...
vis = (t_vis, vn - vn[0] + 0.2 * (t_vis > 50), np.interp(t_vis, t_gps, wander_e))   # ...from t = 50 s

t_ekf = np.arange(0, 100, 0.1)
ekf = (t_ekf, 7 + np.interp(t_ekf, t_gps, wander_n), np.interp(t_ekf, t_gps, wander_e) + 0.1 * (t_ekf > 50))

rows, _ = drift_report(truth, alt, vis, ekf)                     # hold starts at first vision sample
print("\nTest 2: synthetic flight (known: drift = distance from the point at t = 10 s,")
print("        vision error 0.2 m and EKF error 0.1 m after t = 50 s, i.e. above 17.5 m)")
print_report(rows)

# expected drift: distance from the position at t0 = 10 s
t = t_gps[t_gps >= 10]
d = np.hypot(0.5 * np.cos(0.3 * t) - 0.5 * np.cos(3), 0.5 * np.sin(0.3 * t) - 0.5 * np.sin(3))
h = 5 + 25 * t / 100
by_band = {r["band"]: r for r in rows}
exp = d[(h >= 20) & (h < 40)]
assert abs(by_band["20-40 m"]["drift_rms"] - math.sqrt(np.mean(exp ** 2))) < 0.01
assert abs(by_band["20-40 m"]["vision_err_rms"] - 0.2) < 0.01
assert abs(by_band["20-40 m"]["ekf_err_rms"] - 0.1) < 0.01
assert by_band["5-10 m"]["vision_err_rms"] < 0.01 and by_band["5-10 m"]["ekf_err_rms"] < 0.01

# ---- 3. the .bin reader: field names, filters, hold start, vision resets ----
from tools.truth_drift import load_bin


class Msg:
    """Stands in for a pymavlink DFReader message."""
    def __init__(self, kind, **fields):
        self.kind, self.fields = kind, fields

    def get_type(self):
        return self.kind

    def to_dict(self):
        return self.fields


LAT0, LON0 = 45.0, -75.0
east_at = lambda t: 0.01 * t                                    # truth: 1 cm/s eastwards
deg_e = 1 / (math.radians(1) * 6378137.0 * math.cos(math.radians(LAT0)))   # degrees per metre east
msgs = [Msg("MODE", TimeUS=5e6, Mode=2, ModeNum=2), Msg("MODE", TimeUS=20e6, Mode=5, ModeNum=5)]
for t in np.arange(0, 60, 0.2):
    msgs.append(Msg("GPS", TimeUS=t * 1e6, I=0, Status=3, Lat=LAT0, Lng=LON0 + east_at(t) * deg_e))
    msgs.append(Msg("GPS", TimeUS=t * 1e6, I=1, Status=3, Lat=LAT0 + 0.001, Lng=LON0))   # 2nd receiver: ignore
    msgs.append(Msg("BARO", TimeUS=t * 1e6, I=0, Alt=12.0))
for t in np.arange(10, 60, 1 / 30):
    rst = 1 if t < 40 else 2                                    # home re-saved at t = 40 s
    origin = 10 if rst == 1 else 40
    msgs.append(Msg("VISP", TimeUS=t * 1e6, PX=0.0, PY=east_at(t) - east_at(origin), Rst=rst, Ign=0))
msgs.sort(key=lambda m: m.fields["TimeUS"])
print("\nTest 3: reading a (pretend) ArduPilot log")
tr = load_bin(None, messages=msgs)
rows, _ = drift_report(tr["truth"], tr["alt"], tr.get("vision"), tr.get("ekf"), tr["hold_start"])
print(f"  hold start {tr['hold_start']:.1f} s (pilot switched to LOITER at 20 s), "
      f"GPS samples {len(tr['truth'][0])} (receiver 1 ignored), vision error {rows[0]['vision_err_rms'] * 100:.2f} cm "
      f"across a vision reset, drift {rows[0]['drift_max']:.2f} m (expect 0.40)")
assert tr["hold_start"] == 20.0
assert len(tr["truth"][0]) == 300
assert rows[0]["vision_err_rms"] < 0.005, "vision reset (Rst) must not look like an error"
assert abs(rows[0]["drift_max"] - 0.398) < 0.01
print("\nALL TRUTH-DRIFT TESTS PASSED")
