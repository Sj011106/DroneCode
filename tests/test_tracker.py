# tests/test_tracker.py
# Fake drone video with a KNOWN motion, so we can check tracker.py + health.py
# give the right answer (and the right SIGN) before anything flies.
# Run (from the repo root):  python -m tests.test_tracker

# -------------------------------------------------------------------------------------
# IN PLAIN WORDS (for non-technical readers)
#   Saumya's original proof: a fake ground picture is moved and turned by KNOWN amounts,
#   and the tracker must report exactly that (error < 1 px), with the right directions,
#   and health.py must react correctly to blur, freezing, darkness and blank floors.
# -------------------------------------------------------------------------------------

import math
import cv2
import numpy as np
from vision.tracker import GroundTracker, to_north_east, load_calibration
from health.health import HealthChecker

W, H = 640, 480
rng = np.random.default_rng(0)

# A big random "ground" texture (like gravel/grass seen from above)
ground = rng.integers(0, 256, (1600, 1600)).astype(np.uint8)
ground = cv2.GaussianBlur(ground, (0, 0), 3)
ground = cv2.normalize(ground, None, 0, 255, cv2.NORM_MINMAX)
for _ in range(150):   # add some shapes so there are real corners
    x, y = rng.integers(0, 1600, 2)
    cv2.rectangle(ground, (int(x), int(y)), (int(x + rng.integers(10, 60)), int(y + rng.integers(10, 60))),
                  int(rng.integers(0, 256)), -1)

K, _ = load_calibration()
cx, cy = K[0, 2], K[1, 2]
OFF = np.array([800 - cx, 800 - cy])   # home = middle of the ground texture


def camera_view(tx, ty, yaw_deg, noise=2.0):
    """What the camera sees after moving (tx, ty) px in HOME image axes and turning yaw_deg."""
    a = math.radians(yaw_deg)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    c = np.array([cx, cy])
    A = np.hstack([R, (c - R @ c + np.array([tx, ty]) + OFF).reshape(2, 1)])   # view pixel -> ground pixel
    img = cv2.warpAffine(ground, A, (W, H), flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    img = img.astype(np.float32) + rng.normal(0, noise, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def expected_shift(tx, ty, yaw_deg):
    """The answer the tracker SHOULD give: home->drone move, in the CURRENT image's axes."""
    a = math.radians(yaw_deg)
    Rinv = np.array([[math.cos(a), math.sin(a)], [-math.sin(a), math.cos(a)]])
    return Rinv @ np.array([tx, ty])


tracker = GroundTracker(K, None)   # synthetic images have no lens distortion
health = HealthChecker()
n = tracker.set_reference(camera_view(0, 0, 0))
print(f"Home saved with {n} Harris corners")

t = 0.0
worst = 0.0


# ---- Test 1: drift right 150 px and forward, while slowly turning to 40 degrees ----
print("\nTest 1: drift + turn (should stay OK, error < 1 px)")
for i in range(91):
    tx, ty, yaw = i * 1.6, -i * 0.8, i * 0.45
    t += 1 / 30
    img = camera_view(tx, ty, yaw)
    res = tracker.track(img, t_capture=t)
    rep = health.check(img, res)
    exp = expected_shift(tx, ty, yaw)
    if res.ok:
        err = math.hypot(res.shift_px[0] - exp[0], res.shift_px[1] - exp[1])
        if rep.status == "OK":
            worst = max(worst, err)
    if i % 15 == 0:
        got = f"({res.shift_px[0]:+7.1f},{res.shift_px[1]:+7.1f})" if res.ok else "none"
        print(f"  frame {i:2d}: truth ({exp[0]:+7.1f},{exp[1]:+7.1f})  got {got}  "
              f"rot={res.rotation_deg:+5.1f}  inl={res.n_inliers:3d}  {rep.status} q={rep.quality}")
print(f"  worst error while OK: {worst:.2f} px")
assert worst < 1.0, "tracker too inaccurate"
assert rep.status == "OK"

# ---- Test 2: sign check (the 'hand-slide test') ----
print("\nTest 2: sign check")
tr2 = GroundTracker(K, None)
tr2.set_reference(camera_view(0, 0, 0))
r = tr2.track(camera_view(20, 0, 0), t_capture=1.0)
print(f"  camera moved RIGHT 20 px -> shift_px x = {r.shift_px[0]:+.1f} (should be +20)")
assert abs(r.shift_px[0] - 20) < 0.5
r = tr2.track(camera_view(0, -20, 0), t_capture=1.1)
print(f"  camera moved FORWARD (image up) 20 px -> shift_px y = {r.shift_px[1]:+.1f} (should be -20)")
assert abs(r.shift_px[1] + 20) < 0.5
n, e = to_north_east((0, -20), height_m=416 / 100, focal_px=416, yaw_rad=0)   # 20 px = 0.2 m
print(f"  facing NORTH, moved forward 0.2 m -> N={n:+.2f} E={e:+.2f} (should be N=+0.20)")
assert abs(n - 0.2) < 1e-9 and abs(e) < 1e-9
n, e = to_north_east((0, -20), height_m=416 / 100, focal_px=416, yaw_rad=math.radians(90))
print(f"  facing EAST, moved forward 0.2 m  -> N={n:+.2f} E={e:+.2f} (should be E=+0.20)")
assert abs(e - 0.2) < 1e-9 and abs(n) < 1e-9
n, e = to_north_east((20, 0), height_m=416 / 100, focal_px=416, yaw_rad=math.radians(90))
print(f"  facing EAST, moved right 0.2 m    -> N={n:+.2f} E={e:+.2f} (should be N=-0.20)")
assert abs(n + 0.2) < 1e-9

# ---- Test 3: 5 degree turn on the spot must NOT look like drift (issue 2) ----
r = tr2.track(camera_view(0, 0, 5), t_capture=1.2)
print(f"\nTest 3: turn 5 deg on the spot -> shift = ({r.shift_px[0]:+.2f}, {r.shift_px[1]:+.2f}) px (should be ~0)")
assert math.hypot(*r.shift_px) < 0.5

# ---- Test 4: health checks ----
print("\nTest 4: health checks")
img = camera_view(tx, ty, yaw)
blurry = cv2.GaussianBlur(img, (0, 0), 6)
t += 1 / 30
rep = health.check(blurry, tracker.track(blurry, t_capture=t))
print(f"  blurry frame : {rep.status} q={rep.quality}  {rep.reasons}")
assert rep.quality == -1 and rep.status == "DEGRADED"

for _ in range(8):                       # same picture over and over
    t += 1 / 30
    rep = health.check(img, tracker.track(img, t_capture=t))
print(f"  frozen camera: {rep.status} q={rep.quality}  {rep.reasons}")
assert "camera frozen" in rep.reasons and rep.status == "LOST"

dark = (img * 0.05).astype(np.uint8)
t += 1 / 30
rep = health.check(dark, tracker.track(dark, t_capture=t))
print(f"  dark frame   : {rep.status} q={rep.quality}  {rep.reasons}")
assert "too dark" in rep.reasons

for i in range(6):                       # back to normal: needs 5 good frames to regain trust
    t += 1 / 30
    img = camera_view(tx, ty, yaw)
    rep = health.check(img, tracker.track(img, t_capture=t))
    print(f"  recovering {i+1}: {rep.status} q={rep.quality}  {rep.reasons}")
assert rep.status == "OK" and rep.quality > 50

flat = np.full((H, W), 120, np.uint8)    # featureless floor
t += 1 / 30
rep = health.check(flat, tracker.track(flat, t_capture=t))
print(f"  blank floor  : {rep.status} q={rep.quality}  {rep.reasons}")
assert rep.quality == -1

print("\nALL TESTS PASSED")
