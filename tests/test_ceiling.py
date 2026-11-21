# =====================================================================================
# tests/test_ceiling.py  -  does the Ceiling Advisor predict our error correctly?
#                                                                     (OWNER: Ayush, task A4)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Test 1: the formula on small numbers you can check by hand.
#   Test 2: the SAME simulated flights as tests/test_altitude.py (same terrain, camera,
#           blur, wobble, contrast boost) at 5, 10, 20, 40, 80 m. We measure the real
#           position error and compare it with what the advisor predicted from the
#           tracking quality alone. STATED TOLERANCE: within +-25% at every height.
#           (That test feeds the exact attitude, so the tilt part is 0 here.)
#   Test 3: the tilt part. We give the team's real tilt correction (fusion/tilt.py) a
#           known attitude error and check the metres it produces = height x tan(error),
#           per axis, and sqrt(2) x that for random roll AND pitch errors (within 5%).
#   Test 4: the advice: climb OK / stay / descend, and the closed-form ceiling.
#   Test 5: it only WATCHES: the real vision_hold.py replays a recorded flight with the
#           advisor ON and OFF; every ODOMETRY message sent to ArduPilot must be identical.
#           Its cost per frame is printed (profiler stage "safety").
#   Test 6: garbage in (no height, focal 0, quality -1) -> no crash.
#
# Run (from the repo root):  python -m tests.test_ceiling      (~30 s)
# REFERENCES: safety/ceiling.py, tests/test_altitude.py
# =====================================================================================

import contextlib
import csv
import io
import math
import os
import tempfile

import cv2
import numpy as np

import config
from fusion.tilt import cancel_tilt
from health.health import HealthChecker
from link.drone_link import odometry_sigma_px
from safety import ceiling as ce
from tests.common import K, use_test_camera
from tools.degrade_video import make_ground, make_pyramid, make_terrain, render, render_mip
from vision.enhance import enhance
from vision.rolling_shutter import RollingShutterTracker
from vision.tracker import to_north_east

F = (K[0, 0] + K[1, 1]) / 2
TOL = 0.25

print("Test 1: the formula by hand")
k = ce.k_factor(0.6, 208.0, 0.5)
print(f"  0.6 px at focal 208 px + 0.5 deg tilt: k = {k:.5f} m per m of height "
      f"-> {100 * 40 * k:.0f} cm RMS at 40 m (pixel part {100 * 40 * 0.6 / 208:.0f} cm, "
      f"tilt part {100 * 40 * math.tan(math.radians(0.5)) * math.sqrt(2):.0f} cm)")
assert abs(k - math.sqrt((0.6 / 208) ** 2 + 2 * math.tan(math.radians(0.5)) ** 2)) < 1e-12
assert ce.sigma_px(100) == odometry_sigma_px(100) == config.ODOM_PX_NOISE_BEST, "same pixel model as ODOMETRY"
assert abs(ce.predicted_rms_m(80, 0.6, 416, 0) - 0.6 * 80 / 416) < 1e-12

print("\nTest 2: prediction vs the measured error on test_altitude.py's simulated flights")


def pose_at(h):                                   # identical to tests/test_altitude.py
    return lambda t: (0.25 * math.sin(0.8 * t), 0.15 * t, h,
                      math.radians(1) * math.sin(2 * math.pi * 5 * t),
                      math.radians(1) * math.sin(2 * math.pi * 4 * t), 0.0)


print(f"  {'height':>6s} {'quality':>7s} {'measured':>9s} {'predicted':>9s} {'diff':>6s}")
for h in (5, 10, 20, 40, 80):
    mpt = h / F * 0.75
    pyr = make_pyramid(make_terrain(2048, mpt, contrast=1.0, seed=h))
    pose, rng = pose_at(h), np.random.default_rng(h)
    frames = [cv2.GaussianBlur(render_mip(pyr, K, (640, 480), pose, i / 30, mpt, noise=2, rng=rng),
                               (0, 0), 0.7) for i in range(45)]
    tr, hc = RollingShutterTracker(K, None, enabled=False), HealthChecker()
    tr.set_reference(enhance(frames[0]), 0.0)
    n0, e0 = pose(0)[:2]
    errs, adv = [], ce.CeilingAdvisor(tilt_deg=0.0)
    for i in range(1, len(frames)):
        t = i / 30
        g = enhance(frames[i])
        r = tr.track(g, t)
        rep = hc.check(g, r)
        adv.update(h, tr.focal_px, rep.quality if rep.status == "OK" else -1)
        if r.ok:
            p = pose(t)
            d = cancel_tilt(r.shift_px, p[3:], pose(0)[3:], tr.focal_px, r.rotation_deg)
            n, e = to_north_east(d, h, tr.focal_px, 0.0)
            errs.append(math.hypot(n - (p[0] - n0), e - (p[1] - e0)))
    measured = math.sqrt(np.mean(np.square(errs)))
    predicted = adv.last["_rms"]
    diff = predicted / measured - 1
    print(f"  {h:5d}m {np.median(adv.q):7.0f} {100 * measured:7.1f}cm {100 * predicted:7.1f}cm {100 * diff:+5.0f}%")
    assert abs(diff) <= TOL, f"{h} m: predicted {predicted:.3f} m vs measured {measured:.3f} m (> {TOL:.0%})"
print(f"  -> within the stated +-{TOL:.0%} at every height")

print("\nTest 3: the tilt part = what fusion/tilt.py really leaves after a wrong attitude")
for h in (10, 40, 80):
    for err_deg in (0.25, 0.5, 1.0):
        e = math.radians(err_deg)
        shift = cancel_tilt((0.0, 0.0), (e, 0.0, 0.0), (0.0, 0.0, 0.0), F, 0.0)   # true motion 0, roll wrong by e
        n, east = to_north_east(shift, h, F, 0.0)
        got, want = math.hypot(n, east), h * math.tan(e)
        assert abs(got / want - 1) < 0.05, (h, err_deg, got, want)
rng = np.random.default_rng(1)
sig = 0.5
errs = []
for _ in range(2000):
    att = (math.radians(rng.normal(0, sig)), math.radians(rng.normal(0, sig)), 0.0)
    n, e = to_north_east(cancel_tilt((0.0, 0.0), att, (0.0, 0.0, 0.0), F, 0.0), 40.0, F, 0.0)
    errs.append(n * n + e * e)
got = math.sqrt(np.mean(errs))
want = ce.predicted_rms_m(40.0, 0.0, F, sig)
print(f"  one axis: metres = height x tan(error) within 5% (10-80 m, 0.25-1 deg)")
print(f"  random roll+pitch, {sig} deg each, 40 m: measured {100 * got:.1f} cm RMS, advisor {100 * want:.1f} cm")
assert abs(got / want - 1) < 0.05

print("\nTest 4: advice and the closed-form ceiling (placeholder limits RMS 0.5 m, max 1.0 m, margin 1.5)")
# STAGE 2: config.py now holds the official limits (3 m, no RMS limit). This test checks the FORMULA, so it
# puts the old placeholder limits back for its own duration and restores the official ones afterwards.
official = (config.HOLD_RMS_DRIFT_M, config.HOLD_MAX_DRIFT_M, config.HOLD_LIMITS_OFFICIAL)
config.HOLD_RMS_DRIFT_M, config.HOLD_MAX_DRIFT_M, config.HOLD_LIMITS_OFFICIAL = 0.5, 1.0, False
safe = ce.max_safe_height(0.75, 208.0, 0.5)
print(f"  quality 90 on the Justice D (focal ~208 px at 640 wide), 0.5 deg tilt: max safe height {safe:.1f} m")
assert abs(safe - 1.0 / (1.5 * 3 * ce.k_factor(0.75, 208.0, 0.5))) < 1e-9
assert ce.max_safe_height(0.75, 208.0, 0.2) > 2 * safe, "better attitude -> much higher ceiling"
assert abs(ce.predicted_rms_m(safe, 0.75, 208.0, 0.5) * 3 * 1.5 - 1.0) < 1e-9, "at the ceiling, 3 x RMS x margin = limit"
for h, want_word in ((5.0, "CLIMB OK"), (12.0, "STAY"), (30.0, "ABOVE CEILING")):
    a = ce.CeilingAdvisor(tilt_deg=0.5)
    for _ in range(10):
        a.update(h, 208.0, 90)
    print(f"  at {h:4.0f} m -> {a.advice}")
    assert a.advice.startswith(want_word), a.advice
assert "PLACEHOLDER" in a.message() and "max safe height" in a.message()
config.HOLD_RMS_DRIFT_M, config.HOLD_MAX_DRIFT_M, config.HOLD_LIMITS_OFFICIAL = official
real = ce.max_safe_height(0.75, 208.0, 0.5)
print(f"  with the official Stage 2 limit ({config.HOLD_MAX_DRIFT_M:g} m): max safe height {real:.1f} m")
assert abs(real - config.HOLD_MAX_DRIFT_M / (1.5 * 3 * ce.k_factor(0.75, 208.0, 0.5))) < 1e-9

print("\nTest 5: it only watches - ODOMETRY identical with the advisor ON and OFF")
import tools.replay as replay_mod
import vision_hold
from tools.replay import Recorder

use_test_camera()
tmp = tempfile.mkdtemp(prefix="dronecode_ceiling_")
ground = make_ground(3000)
rng = np.random.default_rng(5)
rec_dir = os.path.join(tmp, "rec")
rec = Recorder(rec_dir)
pose5 = lambda t: (0.1 * math.sin(t - 1000), 0.05 * (t - 1000), 8.0,
                   math.radians(2) * math.sin(6 * (t - 1000)), 0.0, 0.0)
imu = [(t, *pose5(t)[3:]) for t in np.arange(999.5, 1004.0, 0.02)]
for i in range(90):
    t = 1000 + i / 30
    img = render(ground, K, (640, 480), pose5, t, readout_s=config.READOUT_S, noise=2, rng=rng)
    rec.add(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), t, i == 0, 8.0, [s for s in imu if s[0] <= t])
rec.close()
sent = []
real_send = replay_mod._ReplayConn.odometry_send
replay_mod._ReplayConn.odometry_send = lambda self, *a: sent.append(a)
DEFAULTS = {k: getattr(config, k) for k in dir(config) if k.isupper()}


def run(on):
    for k_, v in DEFAULTS.items():
        setattr(config, k_, v)
    config.CEILING_ADVISOR = on
    config.GNSS_MONITOR = False              # isolate the advisor (A5's monitor has its own test)
    sent.clear()
    log = os.path.join(tmp, f"flight_{on}.csv")
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        vision_hold.main(["--replay", rec_dir, "--log", log])
    return list(sent), log, out.getvalue()


def same(a, b):
    """Message-by-message, value-by-value; NaN (ODOMETRY's 'unknown') counts as equal to NaN."""
    flat = lambda m: np.array([float(v) for x in m for v in (np.ravel(x) if np.ndim(x) else [x])], float)
    return len(a) == len(b) and all(np.array_equal(flat(x), flat(y), equal_nan=True) for x, y in zip(a, b))


on_msgs, on_log, on_text = run(True)
off_msgs, off_log, _ = run(False)
in_flight = len(on_msgs) - 5            # the last 5 are the shutdown "no data" packets (wall-clock time)
replay_mod._ReplayConn.odometry_send = real_send
safety_csv = os.path.splitext(on_log)[0] + "_safety.csv"
with open(safety_csv, newline="") as f:
    rows = list(csv.DictReader(f))
ms = [ln for ln in on_text.splitlines() if ln.startswith("| safety |")]
print(f"  ODOMETRY messages: {len(on_msgs)} with advisor, {len(off_msgs)} without; the {in_flight} in-flight "
      f"messages identical: {same(on_msgs[:in_flight], off_msgs[:in_flight])}")
print(f"  side log {os.path.basename(safety_csv)}: {len(rows)} rows; last advice: {rows[-1]['ceil_advice']}")
print(f"  cost per frame (profiler): {ms[0] if ms else 'n/a'}")
assert in_flight > 50 and same(on_msgs[:in_flight], off_msgs[:in_flight]), \
    "the advisor must never change what is sent to ArduPilot"
assert len(on_msgs) == len(off_msgs)
assert len(rows) > 50 and not os.path.exists(os.path.splitext(off_log)[0] + "_safety.csv")
assert ms and float(ms[0].split("|")[3]) < 1.0, "the advisor must cost < 1 ms per frame (95th percentile)"

print("\nTest 6: garbage in -> no crash")
a = ce.CeilingAdvisor()
for args in ((None, 208, 80), (float("nan"), 208, 80), (10.0, 0.0, 80), (-3.0, 208, 80), (10.0, 208, -1)):
    a.update(*args)
print(f"  last advice: {a.advice}")

print("\nALL CEILING TESTS PASSED")
