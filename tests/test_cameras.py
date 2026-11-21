# =====================================================================================
# tests/test_cameras.py  -  front camera, blur, crop mode, smooth sticks, frame times   STAGE 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   No camera needed: pretend pictures and pretend cameras check that
#     1. the front camera's grey picture is taken from the RIGHT byte (UYVY vs YUYV),
#        and blank / half-filled / interlaced pictures are recognised,
#     2. home waits (a little) for a sharp picture, and blur-skipping stops after N,
#     3. the high-altitude crop keeps the SAME metres: a ground point lands on the same
#        ray through the crop's camera matrix as through the full picture's,
#     4. our lean requests change smoothly, but "not ready" is centred at once,
#     5. the grab thread dates a picture by the camera driver's time (minus latency),
#     6. the centre mask keeps only the middle of the lens,
#     7. a simulated 30 m hold: the full-resolution crop measures the drift at least as
#        well as the 640-wide picture (in practice several times better).
#
# Run (from the repo root):  python -m tests.test_cameras
# =====================================================================================
import time

import numpy as np

import config
from vision.front_camera import comb_score, frame_problem, gray_from_raw

rng = np.random.default_rng(1)

print("Test 1: front camera pictures")
h, w = 576, 720
uyvy = np.zeros((h, w, 2), np.uint8)
uyvy[:, 0::2, 0] = 100                          # U at even pixels, V at odd ones (byte 0 of each pair)
uyvy[:, 1::2, 0] = 160
uyvy[:, :, 1] = 77                              # Y = byte 1 of each pair
g = gray_from_raw(uyvy, w, h, y_at=1)
assert g.shape == (h, w) and int(g.min()) == int(g.max()) == 77, "UYVY: grey is the second byte"
g_wrong = gray_from_raw(uyvy, w, h, y_at=0)
assert int(g_wrong.max()) != 77, "taking the first byte gives the colour channel (the old bug)"
assert gray_from_raw(uyvy.reshape(1, -1), w, h, 1).shape == (h, w), "flat raw buffer"
assert gray_from_raw(np.zeros((10, 10), np.uint8), w, h, 1) is None, "wrong size -> None"
print("  UYVY grey from byte 1 (byte 0 = colour, the reason the down-camera code fails here)")

scene = rng.integers(40, 200, (h, w)).astype(np.uint8)
assert frame_problem(scene) is None
assert frame_problem(np.full((h, w), 30, np.uint8)) == "no_signal"
part = scene.copy()
part[400:] = 0
assert frame_problem(part) == "partial"
print("  textured -> ok, flat -> no_signal, bottom not filled -> partial")

ramp = np.tile(np.linspace(0, 255, w, dtype=np.float32), (h, 1))
ramp += 40 * np.sin(np.arange(h) / 9.0)[:, None]
ramp = np.clip(ramp, 0, 255).astype(np.uint8)
combed = ramp.copy()
combed[1::2] = np.roll(ramp, 40, axis=1)[1::2]  # odd field taken 20 ms later, scene moved
assert comb_score(ramp) < 1.1 < 1.2 < comb_score(combed), (comb_score(ramp), comb_score(combed))
print(f"  comb score: still {comb_score(ramp):.2f}, moving interlaced {comb_score(combed):.2f}")

print("\nTest 2: sharpness gate")
import cv2
from vision.sharpness import SharpnessGate, sharpness

sharp_img = rng.integers(0, 255, (360, 640)).astype(np.uint8)
blur_img = cv2.GaussianBlur(sharp_img, (0, 0), 4)
assert sharpness(sharp_img) > 10 * sharpness(blur_img)
gate = SharpnessGate()
assert gate.judge(sharpness(sharp_img)) is None and gate.home_ok(0), "no history yet: never wait"
for _ in range(10):
    gate.judge(sharpness(sharp_img))
gate.judge(sharpness(blur_img))
assert not gate.home_ok(0), "blurry picture: wait for a sharper one"
assert gate.home_ok(config.HOME_SHARP_MAX_WAIT_FRAMES), "but never longer than the limit"
gate.judge(sharpness(sharp_img))
assert gate.home_ok(1)
old = config.BLUR_SKIP_RATIO
try:
    config.BLUR_SKIP_RATIO = None
    gate.judge(sharpness(blur_img))
    assert not gate.skip(), "skipping is off by default"
    config.BLUR_SKIP_RATIO = 0.4
    skips = []
    for _ in range(5):
        gate.judge(sharpness(blur_img))
        skips.append(gate.skip())
    assert skips[:config.BLUR_SKIP_MAX_IN_A_ROW] == [True] * config.BLUR_SKIP_MAX_IN_A_ROW
    assert skips[config.BLUR_SKIP_MAX_IN_A_ROW] is False, "never more than N in a row"
finally:
    config.BLUR_SKIP_RATIO = old
print("  home waits for a sharp picture (at most N); blur-skip off by default, at most N in a row")

print("\nTest 3: high-altitude crop keeps the metres")
from vision.enhance import scale_camera_matrix
from vision.process_mode import ProcessMode

K = np.array([[874.0, 0, 951.0], [0, 874.0, 547.0], [0, 0, 1]])
full = rng.integers(0, 255, (1080, 1920)).astype(np.uint8)
old = config.HIGH_ALT_CROP_FROM_M
try:
    config.HIGH_ALT_CROP_FROM_M = None
    m = ProcessMode()
    assert not m.choose(50.0) and m.name == "small", "off by default"
    small = m.image(full)
    assert small.shape == (360, 640)
    assert np.allclose(m.camera_matrix(K, 1920, 640), scale_camera_matrix(K, 640 / 1920)), "small = as before"
    config.HIGH_ALT_CROP_FROM_M = 15.0
    m = ProcessMode()
    m.set_principal_point(K)
    assert not m.choose(5.0) and m.name == "small", "low: small picture"
    assert m.choose(30.0) and m.name == "crop", "high: crop"
    assert m.choose(None) and m.name == "small", "no height known: small picture"
    assert m.choose(30.0) and m.name == "crop"
    crop = m.image(full)
    x0, y0 = m.offset
    assert crop.shape == (config.HIGH_ALT_CROP_SIZE[1], config.HIGH_ALT_CROP_SIZE[0])
    assert np.array_equal(crop, full[y0:y0 + crop.shape[0], x0:x0 + crop.shape[1]]), "full resolution, no resize"
    Kc = m.camera_matrix(K, 1920, crop.shape[1])
    ray = np.array([0.07, -0.05, 1.0])          # a ground point near the lens centre
    p_full = K @ ray
    p_crop = Kc @ ray
    assert np.allclose(p_crop[:2] / p_crop[2] + (x0, y0), p_full[:2] / p_full[2]), "same ray -> same metres"
    assert Kc[0, 0] == K[0, 0], "focal length unchanged (874 px, not 291)"
    assert abs((x0 + crop.shape[1] / 2) - K[0, 2]) <= 1 and abs((y0 + crop.shape[0] / 2) - K[1, 2]) <= 1, \
        "centred on the lens centre"
    m2 = ProcessMode()
    assert not m2.choose(30.0, have_K=False) and m2.name == "small", "no lens file: stay small"
finally:
    config.HIGH_ALT_CROP_FROM_M = old
print("  off by default; engaged high -> full-res lens-centre crop, same ray, same focal; no lens -> small")

print("\nTest 4: smooth lean requests, centred at once when not ready")
from fusion.position_controller import SlewLimiter

lim = SlewLimiter()
from link.stick_curve import angle_from_us                       # the limit is in DEGREES of lean, through the curve
deg_s = config.BF_MAX_TILT_RATE_DEG_S
want = (1600, 1400)
a = lim(1.0, want)
assert all(abs(angle_from_us(v)) <= deg_s * 0.02 + 0.2 for v in a), a
b = lim(1.1, want)
assert all(abs(angle_from_us(y) - angle_from_us(x)) <= deg_s * 0.1 + 0.2 for x, y in zip(a, b)), (a, b)
assert b[0] >= a[0] > 1500 and b[1] <= a[1] < 1500, (a, b)
for k in range(20):
    c = lim(1.2 + 0.1 * k, (1600, 1400))
assert c == (1600, 1400), "gets there"
lim.reset()
assert lim.last == (1500, 1500)
old = config.BF_MAX_TILT_RATE_DEG_S
try:
    config.BF_MAX_TILT_RATE_DEG_S = None
    assert SlewLimiter()(0.0, (1600, 1400)) == (1600, 1400), "None = no limit"
finally:
    config.BF_MAX_TILT_RATE_DEG_S = old

from link.betaflight_link import BetaflightLink

link = BetaflightLink()
link.on_telemetry(dict(t=0.0, att=[0.0, 0.0, 0.0, 0.0], alt=0.0, rc=[1500] * 10, engaged=True))
link._wait_reset = None
link.send_odometry(2.0, 2.0, 0.0, 0.0, (0, 0, 0), (0, 0, 0), 90, reset_counter=1)
first = link.last_sticks
assert first[1] < 1500 and abs(angle_from_us(first[1])) <= deg_s * 0.02 + 0.2, f"far off: starts leaning gently {first}"
link.send_odometry(2.1, 2.0, 0.0, 0.0, (0, 0, 0), (0, 0, 0), -1, reset_counter=1)
assert link.last_sticks == (1500, 1500), "not ready: centred at once"
link._tx.close()
print(f"  first request {first} (gentle), not ready -> (1500, 1500) at once")

print("\nTest 5: grab thread dates pictures by the driver's time")
from compute.budget import LatestFrame


class _Cam:
    def __init__(self, latency):
        self.latency, self.n = latency, 0

    def read(self):
        time.sleep(0.02)
        self.n += 1
        return True, self.n

    def get(self, prop):
        return (time.monotonic() - self.latency) * 1000.0      # V4L2 buffer time, ms, monotonic

    def release(self):
        pass


lf = LatestFrame(_Cam(0.030), clock=time.monotonic)
time.sleep(0.1)
ok, _ = lf.read()
t_read = time.monotonic()
expect = 0.030 + config.READOUT_S / 2
assert ok and lf.age is not None and abs(lf.age - 0.030) < 0.01, lf.age
assert expect - 0.01 < t_read - lf.t < expect + 0.04, (t_read - lf.t, expect)
lf.release()
lf2 = LatestFrame(_Cam(1000.0), clock=time.monotonic)          # driver on another clock -> ignored
time.sleep(0.06)
lf2.read()
assert lf2.age is None and time.monotonic() - lf2.t < 0.03
lf2.release()
print(f"  latency {1000 * 0.030:.0f} ms seen, picture dated {1000 * expect:.1f} ms back; other clock -> ignored")

print("\nTest 6: centre mask")
from tools.centre_mask import centre_mask

cm = centre_mask((360, 640), 0.35)
assert cm[180, 320] == 255 and cm[0, 0] == 0 and cm[180, 0] == 0
frac = float((cm > 0).mean())
assert 0.5 < frac < 0.9, frac
print(f"  radius 0.35 x width keeps {100 * frac:.0f}% of a 640x360 picture (corners and sides dropped)")

print("\nTest 7: simulated drift at 30 m, small picture vs full-resolution crop")
import math
from tools.degrade_video import make_ground, render
from vision.tracker import GroundTracker, to_north_east

W7, H7, HEIGHT, TEX = 1920, 1080, 30.0, 0.03      # 3 cm ground texture: gravel seen from 30 m
K7 = np.array([[874.0, 0, 959.5], [0, 874.0, 539.5], [0, 0, 1.0]])
ground = make_ground(3000, seed=3)
rng7 = np.random.default_rng(7)
pose7 = lambda i: (0.01 * i, -0.008 * i, HEIGHT, 0.0, 0.0, 0.0)   # slow drift north-west
frames7 = [render(ground, K7, (W7, H7), lambda _, p=pose7(i): p, 0, noise=2, rng=rng7, m_per_texel=TEX)
           for i in range(0, 41, 4)]
old = config.HIGH_ALT_CROP_FROM_M
worst = {}
try:
    config.HIGH_ALT_CROP_FROM_M = 15.0
    for h_engage in (5.0, 30.0):                    # engaged low -> small picture; high -> crop
        m = ProcessMode()
        m.set_principal_point(K7)
        m.choose(h_engage)
        img0 = m.image(frames7[0])
        tr = GroundTracker(m.camera_matrix(K7, W7, img0.shape[1]), None)
        tr.set_reference(img0)
        errs = []
        for k, f in enumerate(frames7[1:], 1):
            res = tr.track(m.image(f), t_capture=k / 10)
            assert res.ok, res.reason
            p = pose7(4 * k)
            n, e = to_north_east(res.shift_px, HEIGHT, tr.focal_px, 0.0)
            errs.append(math.hypot(n - p[0], e - p[1]) * 100)
        worst[m.name] = max(errs)
        print(f"  {m.name:5s} {img0.shape[1]}x{img0.shape[0]}: worst error {max(errs):.2f} cm")
finally:
    config.HIGH_ALT_CROP_FROM_M = old
assert worst["crop"] <= worst["small"] and worst["crop"] < 1.0, worst

print("\nALL CAMERA TESTS PASSED")
