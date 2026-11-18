# =====================================================================================
# tests/test_altitude.py  -  what happens as we fly higher?          BRIEF ITEM 1 (+ trial)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The brief: "VIO gets harder with altitude: features shrink". We simulate hovering
#   (with vibration and slow drift) at 5, 10, 20, 40 and 80 m over a realistic ground
#   (fine gravel on top of larger structures), in normal light and in flat/hazy light
#   (30% contrast), with and without the contrast boost (vision/enhance.py).
#   What it shows:
#     1. Tracking accuracy in PIXELS stays about the same at every height; in METRES it
#        grows in proportion to height (one pixel covers more ground). That's geometry.
#     2. In flat light, high up, the fine details fade and health.py's fixed sharpness
#        threshold rejects every frame as "blurry" although tracking is still accurate.
#        The contrast boost brings trust back without changing accuracy.
#
# Run (from the repo root):  python -m tests.test_altitude      (~1 min)
#
# REFERENCES
#   - Scaramuzza & Fraundorfer, "Visual Odometry: Part I", IEEE RAM 2011 (error scales
#     with depth for a fixed pixel error).
#   - See vision/enhance.py (CLAHE) and tools/degrade_video.py (terrain model).
# =====================================================================================

import math

import cv2
import numpy as np

import config
from fusion.tilt import cancel_tilt
from health.health import HealthChecker
from tools.degrade_video import make_pyramid, make_terrain, render_mip
from vision.enhance import enhance
from vision.rolling_shutter import RollingShutterTracker
from vision.tracker import to_north_east

from tests.common import K                  # fixed test camera (not the repo's calib.npz)
F = (K[0, 0] + K[1, 1]) / 2
HEIGHTS = (5, 10, 20, 40, 80)
N_FRAMES = 45


def pose_at(h):
    return lambda t: (0.25 * math.sin(0.8 * t), 0.15 * t, h,
                      math.radians(1) * math.sin(2 * math.pi * 5 * t),
                      math.radians(1) * math.sin(2 * math.pi * 4 * t), 0.0)


def run(frames, pose, h, use_clahe):
    config.USE_CLAHE = use_clahe
    tr, hc = RollingShutterTracker(K, None, enabled=False), HealthChecker()
    n_ref = tr.set_reference(enhance(frames[0]), 0.0)
    n0, e0 = pose(0)[:2]
    inl, q, errs, ok = [], [], [], 0
    for i in range(1, len(frames)):
        t = i / 30
        g = enhance(frames[i])
        r = tr.track(g, t)
        rep = hc.check(g, r)
        if r.ok:
            p = pose(t)
            d = cancel_tilt(r.shift_px, p[3:], pose(0)[3:], tr.focal_px, r.rotation_deg)
            n, e = to_north_east(d, h, tr.focal_px, 0.0)
            errs.append(math.hypot(n - (p[0] - n0), e - (p[1] - e0)))
            inl.append(r.n_inliers)
        if rep.status == "OK":
            ok += 1
            q.append(rep.quality)
    rms = math.sqrt(np.mean(np.square(errs))) if errs else math.nan
    return dict(corners=n_ref, inliers=np.mean(inl) if inl else 0, ok=100 * ok / (len(frames) - 1),
                quality=np.mean(q) if q else 0, cm=100 * rms, px=rms * F / h,
                sharp=cv2.Laplacian(frames[5], cv2.CV_64F).var())


results = {}
for contrast in (1.0, 0.3):
    print(f"\n{'NORMAL light' if contrast == 1 else 'FLAT / HAZY light (30% contrast)'}")
    print(f"  {'height':>6s} {'sharpness':>9s} | {'boost':>5s} {'trusted':>7s} {'quality':>7s} "
          f"{'inliers':>7s} {'error':>8s} {'error':>7s}")
    for h in HEIGHTS:
        mpt = h / F * 0.75                                      # texel slightly finer than a pixel
        pyr = make_pyramid(make_terrain(2048, mpt, contrast=contrast, seed=h))
        pose, rng = pose_at(h), np.random.default_rng(h)
        frames = [cv2.GaussianBlur(render_mip(pyr, K, (640, 480), pose, i / 30, mpt, noise=2, rng=rng),
                                   (0, 0), 0.7) for i in range(N_FRAMES)]        # lens blur 0.7 px
        for use in (False, True):
            r = run(frames, pose, h, use)
            results[(contrast, h, use)] = r
            print(f"  {h:5d}m {r['sharp']:9.0f} | {'ON' if use else 'off':>5s} {r['ok']:6.1f}% {r['quality']:7.0f} "
                  f"{r['inliers']:7.0f} {r['cm']:6.1f}cm {r['px']:5.2f}px")
config.USE_CLAHE = True

px = [results[(1.0, h, False)]["px"] for h in HEIGHTS]
cm = [results[(1.0, h, False)]["cm"] for h in HEIGHTS]
print(f"\n  pixel error {min(px):.2f}-{max(px):.2f} px at every height; "
      f"metre error {cm[0]:.1f} cm at {HEIGHTS[0]} m -> {cm[-1]:.1f} cm at {HEIGHTS[-1]} m")
assert max(px) < 2 * min(px) + 0.2, "pixel accuracy should not depend much on height"
assert cm[-1] > 8 * cm[0], "metre error should grow ~ with height"
for h in (40, 80):
    off, on = results[(0.3, h, False)], results[(0.3, h, True)]
    assert off["ok"] < 10, f"expected health to reject flat-light frames at {h} m without the boost"
    assert on["ok"] > 80, f"contrast boost should restore trust at {h} m"
    assert abs(on["cm"] - off["cm"]) < 0.25 * off["cm"] + 0.5, "boost must not change accuracy"
for h in HEIGHTS:
    assert results[(1.0, h, True)]["ok"] >= results[(1.0, h, False)]["ok"], "boost must not hurt in normal light"
print("\nALL ALTITUDE TESTS PASSED")
