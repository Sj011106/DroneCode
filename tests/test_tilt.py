# =====================================================================================
# tests/test_tilt.py  -  proof that tilt cancellation works        BRIEF ITEM 1
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We simulate a flight: the drone tilts forward 10 degrees, rolls the other way, moves
#   30 cm north while turning 20 degrees, then levels out. At every frame we compare
#   the position our pipeline reports with the true position, once WITHOUT and once
#   WITH fusion/tilt.py. The "without" error is the fake drift a tilting drone creates.
#
# Run (from the repo root):  python -m tests.test_tilt
#
# REFERENCES: see fusion/tilt.py.
# =====================================================================================

import math

import numpy as np

from fusion.tilt import cancel_tilt, nadir_offset_px
from tools.degrade_video import make_ground, render
from vision.tracker import GroundTracker, to_north_east

W, H = 640, 480
HEIGHT = 4.16                 # metres -> about 1 cm per pixel with this calibration
from tests.common import K                  # fixed test camera (not the repo's calib.npz)
F = (K[0, 0] + K[1, 1]) / 2
ground = make_ground()
rng = np.random.default_rng(1)

# ---- 1. sign checks on the maths alone ----
print("Test 1: sign of the nadir offset")
o = nadir_offset_px(0.0, math.radians(10), F)
print(f"  nose UP 10 deg   -> ground below appears at y = {o[1]:+.1f} px (should be +f*tan10 = {F * math.tan(math.radians(10)):+.1f})")
assert abs(o[1] - F * math.tan(math.radians(10))) < 1e-6 and abs(o[0]) < 1e-9
o = nadir_offset_px(math.radians(10), 0.0, F)
print(f"  right wing DOWN  -> ground below appears at x = {o[0]:+.1f} px (should be +)")
assert o[0] > 0 and abs(o[1]) < 1e-9


# ---- 2. rendered flight ----
def pose(i):
    """(north, east, height, roll, pitch, yaw) at frame i. 120 frames = 4 s at 30 fps."""
    s = np.clip((i - 10) / 40, 0, 1)            # 10..50: tilt in
    m = np.clip((i - 30) / 50, 0, 1)            # 30..80: move + turn
    back = np.clip((i - 90) / 20, 0, 1)         # 90..110: level out
    tilt = s * (1 - back)
    return (0.30 * m, 0.0, HEIGHT, math.radians(-6) * tilt, math.radians(-10) * tilt,
            math.radians(20) * m)


print("\nTest 2: tilting flight, position error per frame")
tracker = GroundTracker(K, None)
home = pose(0)
tracker.set_reference(render(ground, K, (W, H), lambda t: home, 0.0, noise=2, rng=rng))
err_off, err_on = [], []
for i in range(1, 121):
    p = pose(i)
    res = tracker.track(render(ground, K, (W, H), lambda t: p, 0.0, noise=2, rng=rng), t_capture=i / 30)
    assert res.ok, f"tracker failed at frame {i}: {res.reason}"
    att, att_home = p[3:], home[3:]
    for shift, errs in ((res.shift_px, err_off),
                        (cancel_tilt(res.shift_px, att, att_home, tracker.focal_px, res.rotation_deg), err_on)):
        n, e = to_north_east(shift, HEIGHT, tracker.focal_px, att[2])
        errs.append(math.hypot(n - p[0], e - p[1]) * 100)
    if i % 20 == 0:
        print(f"  frame {i:3d}: pitch {math.degrees(p[4]):+5.1f}  roll {math.degrees(p[3]):+5.1f}  "
              f"error without {err_off[-1]:5.1f} cm   with {err_on[-1]:4.1f} cm")

print(f"  WORST error without tilt fix: {max(err_off):5.1f} cm")
print(f"  WORST error with tilt fix   : {max(err_on):5.1f} cm")
print(f"  -> the fix removes {100 * (1 - max(err_on) / max(err_off)):.0f}% of the fake drift "
      f"(what's left is the 2nd-order perspective effect, see fusion/tilt.py)")
assert max(err_on) < 0.15 * max(err_off), "tilt fix should remove most of the fake drift"
assert np.mean(err_on) < 3.0, "tilt fix not accurate enough on average"
print("\nALL TILT TESTS PASSED")
