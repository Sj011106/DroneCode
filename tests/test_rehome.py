# =====================================================================================
# tests/test_rehome.py  -  climbing from 10 m to 40 m while holding    BRIEF ITEMS 1 + 3
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Home is saved at 10 m, then the drone climbs to 40 m while trying to stay over the
#   same spot (with a little wobble and drift). Without vision/rehome.py the ground
#   shrinks to a quarter of its size, health.py stops trusting vision and the hold is
#   lost. With it, a new reference picture is taken every ~15% of climb and the position
#   keeps counting from the original home. Prints error per height for the slides.
#
# Run (from the repo root):  python -m tests.test_rehome      (~1 min)
# REFERENCES: see vision/rehome.py.
# =====================================================================================

import math

import numpy as np

from fusion.tilt import cancel_tilt
from health.health import REGAIN_AFTER, HealthChecker
from tools.degrade_video import make_pyramid, make_terrain, render_mip
from vision.rehome import Rehome
from vision.rolling_shutter import RollingShutterTracker
from vision.tracker import to_north_east

W, H, FPS = 640, 480, 30
from tests.common import K                  # fixed test camera (not the repo's calib.npz)
M_PER_TEXEL = 0.02
pyr = make_pyramid(make_terrain(4096, M_PER_TEXEL, seed=5))      # 82 m square of ground
rng = np.random.default_rng(5)


def pose(t):
    """Hold at 10 m for 1 s, climb at 3.75 m/s to 40 m (8 s), hold 2 s. Small drift + wobble."""
    h = 10.0 + 3.75 * float(np.clip(t - 1.0, 0, 8.0))
    return (0.15 * math.sin(0.7 * t), 0.10 * math.sin(0.5 * t + 1), h,
            math.radians(1.0) * math.sin(2 * math.pi * 3 * t), math.radians(1.0) * math.sin(2 * math.pi * 2 * t),
            math.radians(10) * math.sin(0.3 * t))


frames = [(i / FPS, render_mip(pyr, K, (W, H), pose, i / FPS, M_PER_TEXEL, noise=2, rng=rng))
          for i in range(int(11 * FPS))]


def run(use_rehome):
    tracker = RollingShutterTracker(K, None, enabled=False)       # what vision_hold.py uses
    health, rh = HealthChecker(), Rehome()
    t0, img0 = frames[0]
    tracker.set_reference(img0, t0)
    home_att = pose(t0)[3:]
    n0, e0 = pose(t0)[:2]                               # truth is measured from HOME
    out = []                                            # (height, status, error_cm or None)
    for t, img in frames[1:]:
        p = pose(t)
        res = tracker.track(img, t_capture=t)
        rep = health.check(img, res)
        err = None
        if rep.quality > 0:
            d = cancel_tilt(res.shift_px, p[3:], home_att, tracker.focal_px, res.rotation_deg)
            n, e = rh.total(*to_north_east(d, p[2], tracker.focal_px, p[5]))
            err = math.hypot(n - (p[0] - n0), e - (p[1] - e0)) * 100
            if use_rehome and rh.due(rep.status, res):
                if rh.rehome(tracker, health, img, t, n, e, min_corners=50):
                    home_att = p[3:]
        out.append((p[2], rep.status, err))
    return out, rh.count


print("Climb 10 m -> 40 m over the same spot (truth = small wobble around home)")
results = {}
for use in (False, True):
    out, count = run(use)
    results[use] = out
    ok = [o for o in out if o[1] == "OK"]
    lost_at = next((h for h, s, _ in out if s != "OK" and h > 10.5), None)
    print(f"\n  re-home {'ON ' if use else 'OFF'}: {100 * len(ok) / len(out):5.1f}% of frames trusted, "
          f"{count} re-homes" + (f", first untrusted at {lost_at:.1f} m" if lost_at else ""))
    for lo, hi in ((10, 15), (15, 25), (25, 35), (35, 40.1)):
        band = [o for o in out if lo <= o[0] < hi]
        good = [e for _, s, e in band if e is not None]
        if good:
            print(f"    {lo:4.0f}-{hi:4.0f} m: trusted {100 * len(good) / len(band):5.1f}%, "
                  f"error mean {np.mean(good):5.1f} cm, max {max(good):5.1f} cm")
        else:
            print(f"    {lo:4.0f}-{hi:4.0f} m: NOT trusted")

off_top = [o for o in results[False] if o[0] >= 35]
on_top = [o for o in results[True] if o[0] >= 35]
assert sum(s == "OK" for _, s, _ in off_top) < 0.1 * len(off_top), "without re-home it should be lost at 35-40 m"
assert all(s == "OK" for _, s, _ in on_top), "with re-home it must stay trusted at 35-40 m"
assert all(s == "OK" for _, s, _ in results[True][REGAIN_AFTER - 1:]), \
    "re-home must never cause an untrusted frame (only the first frames, while trust builds)"
errs = [e for _, _, e in results[True] if e is not None]
assert max(errs) < 20.0, "position error too large after re-homes"
print("\nALL RE-HOME TESTS PASSED")
