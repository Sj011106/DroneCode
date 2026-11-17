# =====================================================================================
# tests/test_health.py  -  the extra sanity checks around health.py   BRIEF ITEMS 1 + 3
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Saumya's health.py is already tested in test_tracker.py (Test 4). Here we test the
#   two new sanity checks that feed into trust:
#     1. fusion/imu_check.py: vision says "turned 20 degrees", the IMU agrees -> fine.
#        The IMU says "didn't turn" -> vision is lying, reject.
#     2. fusion/height.py: a one-off barometer spike is ignored, a real climb is followed.
#
# Run (from the repo root):  python -m tests.test_health
# =====================================================================================

import math

import numpy as np

from fusion.height import HeightFilter
from fusion.imu_check import ImuCheck
from tools.degrade_video import make_ground, render
from vision.tracker import GroundTracker, load_calibration

K, _ = load_calibration()
ground = make_ground()
rng = np.random.default_rng(4)

# ---- 1. IMU cross-check on a real tracker result ----
print("Test 1: vision rotation vs IMU yaw")
tracker = GroundTracker(K, None)
tracker.set_reference(render(ground, K, (640, 480), lambda _: (0, 0, 4.16, 0, 0, 0), 0, noise=2, rng=rng))
check = ImuCheck(max_deg=5.0)
check.set_home(0.0)
for i in range(1, 21):                                       # turn to 20 deg over 20 frames
    yaw = math.radians(i)
    res = tracker.track(render(ground, K, (640, 480), lambda _: (0, 0, 4.16, 0, 0, yaw), 0,
                               noise=2, rng=rng), t_capture=i / 30)
ok, err = check.check(res.rotation_deg, yaw)
print(f"  drone turned 20 deg, IMU says 20 deg -> agree={ok}, error {err:+.2f} deg")
assert ok and abs(err) < 1.0
ok, err = check.check(res.rotation_deg, 0.0)
print(f"  drone turned 20 deg, IMU says  0 deg -> agree={ok}, error {err:+.2f} deg (vision must be rejected)")
assert not ok
ok, err = check.check(-179.0, math.radians(179.0))
print(f"  wrap-around at 180 deg               -> agree={ok}, error {err:+.2f} deg")
assert ok
assert ImuCheck().check(30.0, None) == (True, 0.0)            # no IMU data: don't block

# ---- 2. Height filter ----
print("\nTest 2: height filter")
hf = HeightFilter(max_rate=3.0, alpha=0.3, max_rejects=5)
t, out = 0.0, None
for _ in range(30):
    t += 1 / 30
    out = hf.update(t, 10.0)
t += 1 / 30
spike = hf.update(t, 25.0)                                   # baro spike: +15 m in 33 ms
print(f"  steady 10 m, one reading of 25 m -> filter gives {spike:.2f} m (should stay ~10)")
assert abs(spike - 10.0) < 0.01
true_h = 10.0
for _ in range(60):                                          # real climb at 1 m/s for 2 s
    t += 1 / 30
    true_h += 1.0 / 30
    out = hf.update(t, true_h)
print(f"  real climb 1 m/s for 2 s           -> filter at {out:.2f} m (should be ~12)")
assert 11.5 < out < 12.5
for _ in range(6):                                           # rangefinder takes over: real step
    t += 1 / 30
    out = hf.update(t, 15.0)
print(f"  lasting step to 15 m               -> filter at {out:.2f} m after 6 frames (accepted)")
assert out > 12.5
assert hf.update(t, None) is None                            # no reading -> no height
print("\nALL HEALTH TESTS PASSED")
