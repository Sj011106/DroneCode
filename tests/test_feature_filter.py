# =====================================================================================
# tests/test_feature_filter.py  -  props / OSD text in view must not fool the tracker
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   A fake downward camera moves over textured ground, but part of the picture never
#   changes: a dark "propeller arm" in one corner and white "OSD text" at the top (the
#   Justice D can burn voltage/name/timer into the image). Corners on those parts say
#   "we haven't moved". The test checks:
#     1. build_static_mask() finds the still parts from a moving recording,
#     2. with the filter on, no home corners land on them,
#     3. with the filter on, there are fewer outliers and the shift is still right,
#     4. glare corners are dropped; health's spread check ignores masked cells,
#     5. a recording where nothing moved is refused (no mask that hides the ground).
#
# Run (from the repo root):  python -m tests.test_feature_filter
# =====================================================================================

import cv2
import numpy as np

from health.health import HealthChecker, mask_cells, spread_fraction
from vision.feature_filter import PointFilter, attach, build_static_mask, keep_mask
from vision.tracker import GroundTracker

W, H = 640, 360                       # the size vision_hold.py tracks at
rng = np.random.default_rng(1)
ground = rng.integers(0, 256, (1400, 1800)).astype(np.uint8)
ground = cv2.normalize(cv2.GaussianBlur(ground, (0, 0), 3), None, 0, 255, cv2.NORM_MINMAX)
for _ in range(200):
    x, y = rng.integers(0, 1700, 2)
    cv2.rectangle(ground, (int(x), int(y)), (int(x + rng.integers(8, 50)), int(y + rng.integers(8, 50))),
                  int(rng.integers(0, 256)), -1)

ARM = (slice(250, 360), slice(0, 170))     # bottom-left: a dark propeller arm, always in view
OSD = (slice(8, 40), slice(200, 440))      # top: white OSD text, always in view


def view(tx, ty):
    """Camera picture after the drone moved (tx, ty) pixels, with the fixed overlays."""
    img = ground[500 + ty:500 + ty + H, 600 + tx:600 + tx + W].astype(np.float32)
    img = np.clip(img + rng.normal(0, 1.5, img.shape), 0, 255).astype(np.uint8)
    arm = img[ARM]
    arm[:] = 40
    cv2.line(img, (20, 340), (160, 270), 90, 6)                 # some edges on the arm
    cv2.circle(img, (60, 300), 18, 120, 3)
    img[OSD] = 30
    for i, ch in enumerate("16.4V  TEAM  03:12"):
        cv2.putText(img, ch, (205 + 13 * i, 33), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 250, 2)
    return img


def in_static(pts):
    x, y = pts[:, 0], pts[:, 1]
    arm = (y >= ARM[0].start) & (x < ARM[1].stop)
    osd = (y < OSD[0].stop) & (x >= OSD[1].start) & (x < OSD[1].stop)
    return arm | osd


K = np.array([[208.0, 0, 319.5], [0, 208.0, 179.5], [0, 0, 1]])

print("Test 1: build the mask from a recording where the drone moves")
path = [(int(30 * np.sin(i / 9)), int(25 * np.cos(i / 7))) for i in range(120)]
mask, info = build_static_mask(view(tx, ty) for tx, ty in path)
print("  " + info)
assert mask is not None and mask.shape == (H, W)
assert (mask[ARM] == 0).mean() > 0.95, "arm not masked"
assert (mask[OSD] == 0).mean() > 0.95, "OSD text not masked"
ground_area = np.ones((H, W), bool)
ground_area[ARM] = False
ground_area[OSD] = False
assert (mask[ground_area] > 0).mean() > 0.85, "too much ground masked"

print("Test 2: home corners stay off the arm and the text")
plain = GroundTracker(K, None)
filt = GroundTracker(K, None)
hc = HealthChecker()
assert attach(filt, hc, path="__no_such_mask__.png") is False      # no file -> stays off
filt.point_filter = PointFilter(mask)
home = view(0, 0)
plain.set_reference(home)
filt.set_reference(home)
on_plain = int(in_static(plain.ref_pts.reshape(-1, 2)).sum())
on_filt = int(in_static(filt.ref_pts.reshape(-1, 2)).sum())
print(f"  corners on static parts: without filter {on_plain}, with filter {on_filt}")
assert on_plain > 10 and on_filt == 0

print("Test 3: fewer outliers, same answer")
tx, ty = 24, -17
frame = view(tx, ty)
r_plain, r_filt = plain.track(frame, 1.0), filt.track(frame, 1.0)
out_plain = r_plain.n_fb_ok - r_plain.n_inliers
out_filt = r_filt.n_fb_ok - r_filt.n_inliers
print(f"  outliers: without filter {out_plain}, with filter {out_filt}")
print(f"  shift: without {np.round(r_plain.shift_px, 2)}, with {np.round(r_filt.shift_px, 2)} (true {tx}, {ty})")
assert r_filt.ok and out_filt < out_plain
assert abs(r_filt.shift_px[0] - tx) < 0.5 and abs(r_filt.shift_px[1] - ty) < 0.5

print("Test 4: glare and the health spread check")
g = np.full((100, 100), 120, np.uint8)
g[40:60, 40:60] = 255
keep = keep_mask(g, np.array([[50, 50], [10, 10], [58, 41]]), mask=np.full((100, 100), 255, np.uint8))
assert keep.tolist() == [False, True, False], keep
cells = mask_cells(mask)
print(f"  {len(cells)}/16 grid cells usable with the mask")
assert 10 <= len(cells) < 16
pts = np.array([[(c + 0.5) * W / 4, (r + 0.5) * H / 4] for c, r in cells])
assert spread_fraction(pts, W, H, valid_cells=cells) == 1.0     # every usable cell = full spread
assert spread_fraction(pts, W, H) < 1.0                          # old way: masked cells count against it

print("Test 5: a recording where nothing moved is refused")
mask2, info2 = build_static_mask(view(0, 0) for _ in range(80))
print("  " + info2)
assert mask2 is None
mask3, _ = build_static_mask(view(0, 0) for _ in range(10))
assert mask3 is None                                              # too short

print("\nALL FEATURE FILTER TESTS PASSED")
