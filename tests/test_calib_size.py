# =====================================================================================
# tests/test_calib_size.py  -  a lens file must match the camera it is used on
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   calib.npz turns pixels into metres. The one in the repo is from a 640x480 laptop
#   webcam; the Justice D gives 1280x720 and is 114 degrees wide. Using the wrong file
#   gives wrong metres with no error. This test checks:
#     1. the datasheet focal length (416 px at 1280 wide, 208 px at 640 wide),
#     2. load_calibration(image_size=...) keeps, rescales or refuses a file correctly,
#     3. check_calib's datasheet file passes its own check, and a webcam file fails,
#     4. auto_calibrate's lens fit picks the wide-angle model for a strongly bent lens.
#
# Run (from the repo root):  python -m tests.test_calib_size
# =====================================================================================

import os
import tempfile

import cv2
import numpy as np

from calibration.auto_calibrate import BOARD, fit_lens
from calibration.check_calib import check, write_spec
from vision.tracker import (DEFAULT_FOCAL_PX, GroundTracker, calibration_size,
                            focal_from_fov, load_calibration)

tmp = tempfile.mkdtemp(prefix="dronecode_calib_")

print("Test 1: focal length from the Justice D datasheet")
print(f"  1280 wide: {focal_from_fov(1280):.1f} px, 640 wide: {focal_from_fov(640):.1f} px")
assert abs(focal_from_fov(1280) - 415.6) < 0.5 and abs(DEFAULT_FOCAL_PX - 207.8) < 0.5
tr = GroundTracker(None, None)                       # no lens file -> datasheet guess at this size
tr.set_reference(np.zeros((360, 640), np.uint8))
assert abs(tr.focal_px - 207.8) < 0.5 and tr.centre == (320, 180)

print("Test 2: load_calibration checks the saved size")
K = np.array([[416.0, 0, 639.5], [0, 416.0, 359.5], [0, 0, 1]])
full = os.path.join(tmp, "full.npz")
np.savez(full, K=K, dist=np.zeros((1, 5)), size=np.array([1280, 720]))
assert calibration_size(full) == (1280, 720)
K1, _ = load_calibration(full, image_size=(1280, 720))
assert np.allclose(K1, K)                                        # same size: as saved
K2, _ = load_calibration(full, image_size=(640, 360))
assert np.allclose(K2[0, 0], 208.0) and np.allclose(K2[:2, 2], [319.5, 179.5])   # half size
webcam = os.path.join(tmp, "webcam.npz")
np.savez(webcam, K=K, dist=np.zeros((1, 5)), size=np.array([640, 480]))
assert load_calibration(webcam, image_size=(1280, 720)) == (None, None)       # 4:3 file on 16:9 camera
old = os.path.join(tmp, "old.npz")
np.savez(old, K=K, dist=np.zeros((1, 5)))                        # no size saved (old calibrate.py)
assert calibration_size(old) is None
K3, _ = load_calibration(old, image_size=(1280, 720))           # warns, but still loads
assert np.allclose(K3, K)
K4, _ = load_calibration(full)                                   # old call style unchanged
assert np.allclose(K4, K)

print("Test 3: check_calib")
spec = os.path.join(tmp, "spec.npz")
assert write_spec(spec) and not write_spec(spec)                 # second time refuses without --force
assert check(spec)
assert not check(old)                                            # no size -> FAIL
bad = os.path.join(tmp, "bad.npz")                               # laptop webcam lens at 1280 wide
np.savez(bad, K=np.array([[1100.0, 0, 640], [0, 1100.0, 360], [0, 0, 1]]), dist=np.zeros((1, 5)),
         size=np.array([1280, 720]))
assert not check(bad)

print("Test 4: a 114-degree fisheye-style lens calibrates accurately")
import calibration.auto_calibrate as ac
w, h = 1280, 720
f_true = 640 / np.radians(57)                    # equidistant fisheye, 114 deg across 1280 px
Ktrue = np.array([[f_true, 0, 640], [0, f_true, 360], [0, 0, 1]])
obj = np.zeros((BOARD[0] * BOARD[1], 1, 3), np.float32)
obj[:, 0, :2] = np.mgrid[0:BOARD[0], 0:BOARD[1]].T.reshape(-1, 2) * 0.04
rng = np.random.default_rng(3)
objs, imgs = [], []
while len(objs) < 30:                            # board views reaching every part of the picture
    rvec = rng.normal(0, 0.4, 3).reshape(1, 1, 3)
    z = rng.uniform(0.25, 0.5)
    tvec = np.array([rng.uniform(-0.9, 0.6) * z, rng.uniform(-0.6, 0.4) * z, z]).reshape(1, 1, 3)
    p, _ = cv2.fisheye.projectPoints(obj, rvec, tvec, Ktrue, np.zeros(4))
    p = p.reshape(-1, 2)
    if (p[:, 0] > 5).all() and (p[:, 0] < w - 5).all() and (p[:, 1] > 5).all() and (p[:, 1] < h - 5).all():
        objs.append(obj.reshape(-1, 3))
        imgs.append((p + rng.normal(0, 0.05, p.shape)).astype(np.float32).reshape(-1, 1, 2))
err, Kfit, dfit, model = fit_lens(objs, imgs, (w, h))
print(f"  {model} model, error {err:.3f} px, focal {Kfit[0, 0]:.1f} (true {f_true:.1f})")
assert err < 0.3 and abs(Kfit[0, 0] - f_true) < 0.02 * f_true
fish = os.path.join(tmp, "fish.npz")                             # a correct fisheye result must PASS
np.savez(fish, K=Kfit, dist=dfit, size=np.array([w, h]))
assert check(fish)

print("Test 5: fit_lens keeps whichever lens model fits better")
saved = ac.TRY_WIDE_MODEL_ABOVE_PX
ac.TRY_WIDE_MODEL_ABOVE_PX = 0.0                 # force it to try both models
e_normal = cv2.calibrateCamera(objs, imgs, (w, h), None, None)[0]
e_rat = cv2.calibrateCamera(objs, imgs, (w, h), None, None, flags=cv2.CALIB_RATIONAL_MODEL)[0]
err2, _, d2, model2 = fit_lens(objs, imgs, (w, h))
ac.TRY_WIDE_MODEL_ABOVE_PX = saved
assert np.isclose(err2, min(e_normal, e_rat))
assert model2 == ("rational" if e_rat < e_normal else "normal")
assert (d2.size >= 8) == (model2 == "rational")

print("\nALL CALIBRATION SIZE TESTS PASSED")
