# =====================================================================================
# tests/test_process_scale.py  -  smaller pictures, same metres              BRIEF ITEM 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The Justice D gives 1280x720 pictures at 60 fps; the APB's small ARM cores can't track
#   that at 30 fps. So vision_hold.py shrinks every picture to 640x360 first
#   (config.PROCESS_WIDTH) and scales the lens numbers to match. This test flies the same
#   simulated drift at full and at half size and checks the METRES agree, i.e. shrinking
#   saves compute without moving the answer. It also times both.
#
# Run (from the repo root):  python -m tests.test_process_scale
# =====================================================================================

import math
import time

import numpy as np

from tools.degrade_video import make_ground, render
from vision.enhance import downscale, scale_camera_matrix
from vision.tracker import GroundTracker, to_north_east

W, H, HEIGHT = 1280, 720, 4.16
K_FULL = np.array([[416.0, 0, 639.5], [0, 416.0, 359.5], [0, 0, 1.0]])   # 114 deg wide, like the Justice D
ground = make_ground(3000)
rng = np.random.default_rng(5)


def pose(i):
    return (0.004 * i, -0.003 * i, HEIGHT, 0.0, 0.0, math.radians(0.2 * i))   # drift + slow turn


frames = [render(ground, K_FULL, (W, H), lambda _, p=pose(i): p, 0, noise=2, rng=rng) for i in range(61)]
results = {}
for width in (W, 640):                              # W = no downscaling
    s = width / W
    tracker = GroundTracker(scale_camera_matrix(K_FULL, s), None)
    tracker.set_reference(downscale(frames[0], width))
    errs, t0 = [], time.perf_counter()
    for i, img in enumerate(frames[1:], 1):
        res = tracker.track(downscale(img, width), t_capture=i / 30)
        assert res.ok, res.reason
        p = pose(i)
        n, e = to_north_east(res.shift_px, HEIGHT, tracker.focal_px, p[5])
        errs.append((n, e, math.hypot(n - p[0], e - p[1]) * 100))
    ms = (time.perf_counter() - t0) * 1000 / (len(frames) - 1)
    results[width] = (errs, ms)
    label = "full 1280x720" if width == W else f"downscaled {width}x{round(H * s)}"
    print(f"  {label:22s}: error mean {np.mean([x[2] for x in errs]):.2f} cm, "
          f"max {max(x[2] for x in errs):.2f} cm, track {ms:.1f} ms/frame")

diff = max(math.hypot(a[0] - b[0], a[1] - b[1]) * 100 for a, b in zip(results[W][0], results[640][0]))
print(f"  biggest difference full vs downscaled: {diff:.2f} cm "
      f"(1 px = {HEIGHT / 416 * 100:.1f} cm at full size, {2 * HEIGHT / 416 * 100:.1f} cm downscaled)")
assert max(x[2] for x in results[640][0]) < 3.0, "downscaled tracking must stay accurate"
assert diff < 3.0, "downscaling must not move the answer"
print("\nALL PROCESS-SCALE TESTS PASSED")
