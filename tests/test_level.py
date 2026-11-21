# =====================================================================================
# tests/test_level.py  -  the wide-lens tilt problem and its fix (config.USE_LEVEL_POINTS)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The Justice D sees 114 degrees. When the drone leans, the picture stretches on one
#   side and squeezes on the other. The tracker only allows "slide, turn, zoom" between
#   the home picture and now, so with this lens most points stop agreeing from about
#   3 degrees of lean and health.py says LOST: no position, although the answer was
#   still right. A hovering drone in wind leans more than that, and so do a judge's hands
#   on the bench. Our older tests used a narrower test lens and never showed it.
#   The fix (vision/rolling_shutter.py, LEVELLING): use the IMU to move every point to
#   where a camera looking straight down would have seen it, BEFORE matching.
#   This test leans the camera 0 to 15 degrees over the same spot, at 1 m (bench) and
#   20 m (flight), and prints OLD (match first, subtract one shift after: fusion/tilt.py)
#   against NEW (level first).
#
# HOW IT WORKS (technical)
#   Rendered nadir views (tools/degrade_video.render) with a Justice D-like camera after
#   our downscale: 640 x 360, f = 207.8 px. Home is saved at roll 3, pitch -2 deg (never
#   exactly level in reality); the drone then sits 0.2 x height North, 0.1 x height East of
#   home at increasing lean. OLD = RollingShutterTracker(level=False) + cancel_tilt.
#   NEW = RollingShutterTracker(level=True). No rolling shutter here (readout 0): that
#   part has its own test. SIMULATED: exact attitude, flat ground, perfect lens.
#
# RUN:  python -m tests.test_level
# REFERENCES: Hartley & Zisserman, "Multiple View Geometry", 2nd ed., sec. 8.4 (pure
#   rotation = homography K R K^-1); sec. 13 (plane-induced homographies).
# =====================================================================================
import math

import numpy as np

from fusion.tilt import cancel_tilt
from health.health import REGAIN_AFTER, HealthChecker
from tools.degrade_video import make_ground, render
from vision.rolling_shutter import RollingShutterTracker
from vision.timing import CameraTiming
from vision.tracker import to_north_east

W, H = 640, 360
F = 320 / math.tan(math.radians(57))                 # 114 deg across 640 px
K = np.array([[F, 0, 319.5], [0, F, 179.5], [0, 0, 1.0]])
HOME_ATT = (math.radians(3), math.radians(-2), 0.2)
TILTS = (0, 2, 3, 4, 6, 8, 10, 15)
ground = make_ground(3000)
rng = np.random.default_rng(4)


def history(att):
    return [(-10.0, *att), (1000.0, *att)]           # a steady attitude around every frame time


def run(level, height, m_per_texel):
    """Returns {tilt: (status, inliers, error_cm or None)}."""
    tr = RollingShutterTracker(K, None, attitude_history=history(HOME_ATT), timing=CameraTiming(0.0, 0.0),
                               enabled=False, level=level)
    home = render(ground, K, (W, H), lambda _: (0, 0, height, *HOME_ATT), 0, noise=2, rng=rng, m_per_texel=m_per_texel)
    n_ref = tr.set_reference(home, 0.0)
    assert tr.ref_levelled == level
    truth = (0.2 * height, 0.1 * height)
    out = {}
    for tilt in TILTS:
        att = (math.radians(tilt), math.radians(-0.6 * tilt), 0.2 + math.radians(5))
        tr.history, tr.M_last, hc = history(att), np.eye(2, 3), HealthChecker()
        for i in range(REGAIN_AFTER + 1):            # the same view a few times: health builds trust
            img = render(ground, K, (W, H), lambda _: (*truth, height, *att), 0, noise=2, rng=rng, m_per_texel=m_per_texel)
            res = tr.track(img, t_capture=1.0 + i / 30)
            rep = hc.check(img, res)
        err = None
        if res.ok:
            a_now = (0.0, 0.0, att[2]) if tr.now_levelled else att          # what vision_hold.py does
            a_home = (0.0, 0.0, HOME_ATT[2]) if tr.ref_levelled else HOME_ATT
            d = cancel_tilt(res.shift_px, a_now, a_home, tr.focal_px, res.rotation_deg)
            n, e = to_north_east(d, height, tr.focal_px, att[2])
            err = 100 * math.hypot(n - truth[0], e - truth[1])
        out[tilt] = (rep.status, res.n_inliers, n_ref, err)
    return out


print("Lean the camera over the same spot. Justice D-like lens (114 deg), SIMULATED.")
lost_old = {}
for height, mpt in ((1.0, 0.004), (20.0, 0.08)):
    old, new = run(False, height, mpt), run(True, height, mpt)
    print(f"\n  height {height:g} m, drone {0.2 * height:g} m North / {0.1 * height:g} m East of home")
    print("   lean   | OLD: match, then subtract a shift      | NEW: level the points first")
    for tilt in TILTS:
        cells = []
        for status, inl, n_ref, err in (old[tilt], new[tilt]):
            e = "   no fix" if err is None else f"{err:6.1f} cm"
            cells.append(f"{status:8s} {inl:3d}/{n_ref} agree, error {e}")
        print(f"   {tilt:2d} deg | {cells[0]} | {cells[1]}")
    assert all(new[t][1] >= old[t][1] for t in TILTS if t <= 10), "levelling must never lose agreeing points"
    lost_old[height] = [t for t in TILTS if t <= 10 and old[t][0] != "OK"]
    assert all(new[t][0] == "OK" for t in TILTS if t <= 10), "levelled points must stay trusted up to 10 deg"
    worst = max(new[t][3] for t in TILTS if t <= 10)
    assert worst < 3.0 * height + 2.0, f"levelled error too large: {worst:.1f} cm at {height} m"
assert any(lost_old.values()), "the old way should lose trust somewhere up to 10 deg with this lens"
print(f"\n  old way not trusted at (deg): {lost_old}; levelled: trusted everywhere up to 10 deg.")
print("  Known limit: at 15 deg both fail (the frame-to-frame search still runs on the un-levelled picture).")
print("\nALL LEVEL TESTS PASSED")
