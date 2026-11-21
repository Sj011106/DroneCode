# =====================================================================================
# calibration/check_calib.py   (OWNER: Saumya)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Two jobs, both about calib.npz (the lens file that turns pixels into metres):
#   1. CHECK it: was it made at the camera's size? Does its focal length look like the
#      Justice D datasheet (114 deg wide -> ~416 px at 1280 wide)? A lens file from a
#      laptop webcam on the drone gives wrong metres without any error message.
#   2. WRITE a fallback from the datasheet (no checkerboard), so the code runs on day 1.
#      A real checkerboard calibration (calibration.auto_calibrate) replaces it later.
#
# COMMANDS (from the repo root)
#   python -m calibration.check_calib                       -> check calib.npz
#   python -m calibration.check_calib --camera /dev/video2  -> ...and that it matches the camera
#   python -m calibration.check_calib --write-spec          -> datasheet calib.npz (1920x1080)
#   python -m calibration.check_calib --write-spec --force  -> overwrite an existing one
# Exit code 0 = PASS, 1 = FAIL (so it can be used in a pre-flight script).
# =====================================================================================

import argparse
import math
import os
import sys

import cv2
import numpy as np

from vision.tracker import DEFAULT_HFOV_DEG, calibration_size, focal_from_fov

SPEC_W, SPEC_H = 1920, 1080       # Stage 2: the Justice D delivers 1920x1080 @ 60 fps (datasheet said 1280x720)
FOCAL_TOLERANCE = 0.15            # margin around the normal-lens..fisheye focal range
CENTRE_TOLERANCE = 0.10           # optical centre within 10% of the image middle


def write_spec(path, force=False):
    if os.path.exists(path) and not force:
        print(f"{path} already exists - add --force to replace it with the datasheet version")
        return False
    f = focal_from_fov(SPEC_W)
    K = np.array([[f, 0, (SPEC_W - 1) / 2], [0, f, (SPEC_H - 1) / 2], [0, 0, 1]])
    np.savez(path, K=K, dist=np.zeros((1, 5)), size=np.array([SPEC_W, SPEC_H]), source="datasheet")
    print(f"Wrote {path}: {SPEC_W}x{SPEC_H}, focal {f:.1f} px (HFOV {DEFAULT_HFOV_DEG:.0f} deg), "
          f"no lens bending. Replace it with a checkerboard calibration when you can.")
    return True


def camera_size(source):
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
    ok, frame = cap.read() if cap.isOpened() else (False, None)
    cap.release()
    return (frame.shape[1], frame.shape[0]) if ok else None


def check(path, camera=None):
    if not os.path.exists(path):
        print(f"FAIL: no {path}. Run calibration.auto_calibrate, or --write-spec for a fallback.")
        return False
    data = np.load(path)
    K, dist = data["K"], data["dist"].ravel()
    size = calibration_size(path)
    ok = True
    src = str(data["source"]) if "source" in data.files else "checkerboard"
    print(f"{path}: focal {K[0, 0]:.1f} x {K[1, 1]:.1f} px, centre ({K[0, 2]:.1f}, {K[1, 2]:.1f}), "
          f"{len(dist)} distortion numbers, from {src}")
    if size is None:
        print("FAIL: no image size saved (old calibrate.py or the laptop webcam file). Recalibrate.")
        return False
    w, h = size
    print(f"  made at {w}x{h}")

    # A 114 deg lens can be "normal" (rectilinear) or more fisheye-like. Same field of view,
    # different focal length in the image centre:
    #   rectilinear  f = (w/2) / tan(57 deg)       -> 416 px at 1280 wide (the datasheet guess)
    #   equisolid    f = (w/2) / (2 sin(28.5 deg)) -> 670 px (a strong fisheye)
    # A checkerboard result anywhere in that range (+-15%) is plausible for the Justice D.
    half = math.radians(DEFAULT_HFOV_DEG) / 2
    f_low = focal_from_fov(w)
    f_high = (w / 2) / (2 * math.sin(half / 2))
    f = (K[0, 0] + K[1, 1]) / 2
    msg = (f"  focal {f:.0f} px; a {DEFAULT_HFOV_DEG:.0f} deg lens at {w} wide is {f_low:.0f} (normal lens) "
           f"to {f_high:.0f} (fisheye)")
    if not (1 - FOCAL_TOLERANCE) * f_low <= f <= (1 + FOCAL_TOLERANCE) * f_high:
        print(msg + "  <- FAIL: not the Justice D, or a bad calibration")
        ok = False
    else:
        where = "close to the datasheet guess" if f < 1.15 * f_low else "fisheye-like lens"
        print(msg + f"  ok ({where})")

    dx, dy = K[0, 2] / w - 0.5, K[1, 2] / h - 0.5
    if max(abs(dx), abs(dy)) > CENTRE_TOLERANCE:
        print(f"  centre is {dx:+.0%} / {dy:+.0%} away from the middle  <- FAIL: bad calibration")
        ok = False

    if camera is not None:
        cam = camera_size(camera)
        if cam is None:
            print(f"FAIL: can't read a frame from {camera}")
            return False
        if cam != (w, h):
            print(f"  camera {camera} gives {cam[0]}x{cam[1]}  <- FAIL: calibrate at the camera's size")
            ok = False
        else:
            print(f"  camera {camera} gives {cam[0]}x{cam[1]}  ok")
    print("PASS" if ok else "FAIL")
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description="Check calib.npz, or write a datasheet fallback")
    ap.add_argument("--file", default="calib.npz")
    ap.add_argument("--camera", help="also check against this camera ('2' or /dev/video2)")
    ap.add_argument("--write-spec", action="store_true", help="write a datasheet calib.npz")
    ap.add_argument("--force", action="store_true", help="with --write-spec: overwrite")
    args = ap.parse_args(argv)
    if args.write_spec and not write_spec(args.file, args.force):
        sys.exit(1)
    sys.exit(0 if check(args.file, args.camera) else 1)


if __name__ == "__main__":
    main()
