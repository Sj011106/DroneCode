# =====================================================================================
# tests/test_frozen.py  -  the holder's legs must not fool the tracker (BETAFLIGHT CHANGE)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Bench: a judge carries the aircraft 1 m over a plain floor. Part of the picture shows
#   the judge's shoes and legs (strong texture) that move WITH the camera, so in the picture
#   they stay put while the plain floor slides. Through Saumya's real tracker:
#     1. the old corner search put ALL its corners on the legs (none on the floor); the
#        per-cell search (config.CORNERS_PER_CELL) gives the floor its share;
#     2. legs fill most of the picture: without vision/frozen.py (+ the tracker's
#        "second-best motion" check) the legs win and the motion is wrong; with it, right;
#     3. standing still, nothing is dropped.
# RUN:  python -m tests.test_frozen   (~3 s)
# =====================================================================================
import cv2
import numpy as np

import config
from health.health import HealthChecker
from vision.enhance import enhance
from vision.frozen import FrozenFilter
from vision.rehome import Rehome
from vision.rolling_shutter import RollingShutterTracker
from vision.tracker import GroundTracker

W, H = 640, 360
rng = np.random.default_rng(1)
floor = cv2.GaussianBlur(rng.normal(128, 9, (H + 200, W + 600)).astype(np.float32), (0, 0), 2.2)
legs = cv2.GaussianBlur(rng.normal(128, 70, (H, 300)).astype(np.float32), (0, 0), 1.2)
legs = cv2.GaussianBlur(rng.normal(128, 70, (H, W)).astype(np.float32), (0, 0), 1.2)
legs_mask = np.zeros((H, W), bool)


def set_legs(width):
    legs_mask[:] = False
    legs_mask[:, :width] = True                         # the left part of the picture: shoes and legs


def frame(dx):
    img = floor[100:100 + H, 300 + dx:300 + dx + W].copy()
    img[legs_mask] = legs[legs_mask]
    return np.clip(img + rng.normal(0, 1.0, img.shape), 0, 255).astype(np.uint8)


def run(use_filter, speed_px=6, frames=40, moving=True):
    tr = GroundTracker()
    tr.set_reference(frame(0))
    ff = FrozenFilter()
    res = None
    for i in range(1, frames + 1):
        t = i / 15.0
        tr.exclude = ff.for_tracker(tr) if use_filter else None
        tr.reject_static = use_filter and moving          # what vision_hold.py sets from the accelerometer
        res = tr.track(frame(speed_px * i if moving else 0), t)
        if use_filter:
            ff.update(t, tr.last_cur, tr.last_valid, moving)
    true_dx = speed_px * frames if moving else 0
    got = res.shift_px[0] if res.ok else float("nan")
    n_att = int(ff.attached.sum()) if use_filter else 0
    on_legs = 0
    if use_filter:
        pts = tr.ref_pts.reshape(-1, 2).astype(int)
        on_legs = int(legs_mask[pts[:, 1].clip(0, H - 1), pts[:, 0].clip(0, W - 1)].sum())
    return true_dx, got, n_att, on_legs, res


def floor_corners():
    tr = GroundTracker()
    tr.set_reference(frame(0))
    pts = tr.ref_pts.reshape(-1, 2).astype(int)
    return int((~legs_mask[pts[:, 1].clip(0, H - 1), pts[:, 0].clip(0, W - 1)]).sum()), len(pts)


print("Test 1: legs in the left third, plain floor: do any home corners land on the FLOOR?")
set_legs(230)
config.CORNERS_PER_CELL = False
old_floor, old_n = floor_corners()
config.CORNERS_PER_CELL = True
new_floor, new_n = floor_corners()
print(f"  old corner search: {old_floor} of {old_n} on the floor | per-cell search (config.CORNERS_PER_CELL): "
      f"{new_floor} of {new_n} on the floor")
assert old_floor == 0 and new_floor >= 0.3 * new_n

def chain(width, seed, use_filter, frames=40, speed=6):
    """The whole bench chain as vision_hold.py runs it: contrast boost, tracker, frozen filter, health, re-homes."""
    global floor, legs, rng
    rng = np.random.default_rng(seed)
    floor = cv2.GaussianBlur(rng.normal(128, 9, (H + 200, W + 600)).astype(np.float32), (0, 0), 2.2)
    legs = cv2.GaussianBlur(rng.normal(128, 70, (H, W)).astype(np.float32), (0, 0), 1.2)
    set_legs(width)
    tr = RollingShutterTracker(attitude_history=[], enabled=False)   # the tracker vision_hold.py uses
    ff, hc, rh = FrozenFilter(), HealthChecker(), Rehome()
    tr.set_reference(enhance(frame(0)))
    off = total = 0.0
    for i in range(1, frames + 1):
        t, img = i / 15.0, enhance(frame(speed * i))
        tr.exclude = ff.for_tracker(tr) if use_filter else None
        tr.reject_static = use_filter                   # carried the whole time
        res = tr.track(img, t)
        if use_filter:
            ff.update(t, tr.last_cur, tr.last_valid, True)
        rep = hc.check(img, res)
        if res.ok and rep.status == "OK":
            total = off + res.shift_px[0]
            if rh.due(rep.status, res, (W, H), inlier_drop=use_filter):
                if rh.rehome(tr, hc, img, t, total, 0.0, 50):
                    off = total
    return speed * frames, total


print("\nTest 2: the whole bench chain, carried 240 px (about 1.2 m at 1 m up), legs in view, 4 floors each")
for width in (230, 390):                                # a third, and 60% of the picture
    res = {f: [chain(width, seed, f) for seed in range(4)] for f in (False, True)}
    right = {f: sum(abs(got - true) < 5 for true, got in res[f]) for f in res}
    print(f"  legs {width / W:4.0%} of the picture: right without the filter {right[False]}/4, with it {right[True]}/4 "
          f"({', '.join(f'{got:.0f}' for _, got in res[True])} px of 240)")
    assert right[True] == 4, "with the filter every carry must come out right"
    if width == 390:
        assert right[False] <= 1, "this scene must fool the tracker without the filter (else the test proves nothing)"

print("\nTest 3: held still -> nothing is dropped (the accelerometer says 'not moving')")
_, got, n_att, _, _ = run(True, moving=False)
print(f"  shift {got:.2f} px, {n_att} spots dropped")
assert n_att == 0 and abs(got) < 1.0
print(f"\nALL FROZEN-FILTER TESTS PASSED (window {config.FROZEN_WINDOW_S} s, still {config.FROZEN_STILL_PX} px)")
