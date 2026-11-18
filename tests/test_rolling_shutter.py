# =====================================================================================
# tests/test_rolling_shutter.py  -  proof the rolling-shutter fix works   BRIEF ITEM 1
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We simulate the Justice D on a vibrating, slowly turning drone that is hovering
#   perfectly still (true position = home, always). Every simulated picture is scanned
#   row by row over 30 ms, exactly like a rolling shutter, and arrives at our program
#   20 ms late. Then:
#     Test 1: can calibrate_timing.py measure that 20 ms delay and 30 ms scan time?
#     Test 2: how far off is the position WITHOUT vs WITH the rolling-shutter fix?
#   Both runs use the same tilt fix, so the difference is the rolling shutter alone.
#
# Run (from the repo root):  python -m tests.test_rolling_shutter
#
# REFERENCES: see vision/rolling_shutter.py and calibration/calibrate_timing.py.
# =====================================================================================

import math

import numpy as np

from calibration.calibrate_timing import estimate_offset, estimate_readout, track_pair
from fusion.tilt import cancel_tilt
from tools.degrade_video import make_ground, render
from vision.rolling_shutter import RollingShutterTracker, attitude_at
from vision.timing import CameraTiming
from vision.tracker import to_north_east

W, H = 640, 480
HEIGHT = 4.16
READOUT = 0.030               # true scan time
DELAY = 0.020                 # frames reach us 20 ms after their middle row was captured
FPS = 30
from tests.common import K                  # fixed test camera (not the repo's calib.npz)
ground = make_ground()
rng = np.random.default_rng(2)


def pose(t):
    """Hovering at home, vibrating (roll 3 deg @ 4 Hz, pitch 2 deg @ 3 Hz), turning 20 deg/s."""
    return (0.0, 0.0, HEIGHT, math.radians(3) * math.sin(2 * math.pi * 4 * t),
            math.radians(2) * math.sin(2 * math.pi * 3 * t + 1), math.radians(20) * t)


# IMU at 50 Hz, stamped with true time (like DroneLink: arrival time, small delay)
history = [(t, *pose(t)[3:]) for t in np.arange(-0.5, 4.0, 0.02)]

# Frames: middle row at T, our timestamp t_capture = T + DELAY
frames = []
for i in range(int(3 * FPS) + 1):
    T = i / FPS
    frames.append((T + DELAY, render(ground, K, (W, H), pose, T, readout_s=READOUT, noise=2, rng=rng)))

# ---- Test 1: calibration recovers the timing ----
print("Test 1: timing calibration from the simulated video + IMU")
pairs = []
for (t0, g0), (t1, g1) in zip(frames, frames[1:]):
    pr = track_pair(g0, g1)
    if pr:
        pairs.append((t0, pr[0], t1, pr[1]))
offset, mismatch = estimate_offset(pairs, history, K)
print(f"  offset  : measured {offset * 1000:+.1f} ms   (true {-DELAY * 1000:+.1f} ms)")
readout, res, res0 = estimate_readout(pairs, history, K, None, H, offset)
print(f"  readout : measured {readout * 1000:.1f} ms   (true {READOUT * 1000:.1f} ms), "
      f"fit residual {res0:.3f} -> {res:.3f} px")
assert abs(offset + DELAY) <= 0.004, "offset estimate off"
assert abs(readout - READOUT) <= 0.004, "readout estimate off"

# ---- Test 2: position error without vs with the fix ----
print("\nTest 2: hovering still (truth = 0 cm), position error")
timing = CameraTiming(readout, offset, source="test calibration")
errors = {}
for enabled in (False, True):
    tr = RollingShutterTracker(K, None, attitude_history=history, timing=timing, enabled=enabled)
    t_home, home_img = frames[0]
    tr.set_reference(home_img, t_home)
    att_home = attitude_at(history, timing.mid_time(t_home))[0]
    errs = []
    for t_capture, img in frames[1:]:
        res = tr.track(img, t_capture)
        assert res.ok, res.reason
        att = attitude_at(history, timing.mid_time(t_capture))[0]
        d = cancel_tilt(res.shift_px, att, att_home, tr.focal_px, res.rotation_deg)
        n, e = to_north_east(d, HEIGHT, tr.focal_px, att[2])
        errs.append(math.hypot(n, e) * 100)
    errors[enabled] = errs
    print(f"  rolling-shutter fix {'ON ' if enabled else 'OFF'}: worst {max(errs):5.1f} cm, "
          f"average {np.mean(errs):4.1f} cm")

# Floor: the same flight with a GLOBAL shutter camera (no rolling shutter at all). What's
# left there is the tilt fix's 2nd-order residual, which no rolling-shutter fix can remove.
tr = RollingShutterTracker(K, None, attitude_history=history, timing=CameraTiming(0, 0), enabled=False)
tr.set_reference(render(ground, K, (W, H), pose, 0.0, noise=2, rng=rng), 0.0)
att_home = attitude_at(history, 0.0)[0]
floor = []
for i in range(1, int(3 * FPS) + 1):
    T = i / FPS
    res = tr.track(render(ground, K, (W, H), pose, T, noise=2, rng=rng), T)
    att = attitude_at(history, T)[0]
    n, e = to_north_east(cancel_tilt(res.shift_px, att, att_home, tr.focal_px, res.rotation_deg),
                         HEIGHT, tr.focal_px, att[2])
    floor.append(math.hypot(n, e) * 100)
print(f"  global shutter camera (floor) : worst {max(floor):5.1f} cm, average {np.mean(floor):4.1f} cm")
rs_off = np.mean(errors[False]) - np.mean(floor)      # error CAUSED by rolling shutter
rs_on = np.mean(errors[True]) - np.mean(floor)
print(f"  -> rolling-shutter error {rs_off:.1f} cm -> {max(rs_on, 0):.1f} cm: "
      f"the fix removes {100 * (1 - max(rs_on, 0) / rs_off):.0f}% of it")
assert rs_on < 0.2 * rs_off, "fix should remove most of the rolling-shutter error"
assert max(errors[True]) < 3.0
print("\nALL ROLLING SHUTTER TESTS PASSED")
