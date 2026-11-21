# =====================================================================================
# tools/centre_mask.py  -  use only the calibrated middle of the lens             STAGE 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We measured the Justice D's focal length in the MIDDLE of the picture (874 px) and
#   switched the edge correction off because the edges were not measured yet. A wide lens
#   squeezes the edges, so a corner out there seems to move LESS than it really does and
#   would pull our distances down. Simplest honest fix: do not pick landmarks there.
#   vision/feature_filter.py already obeys mask.png (white = use, black = ignore; also
#   used for props and OSD text). This tool ANDs a centre circle into it.
#
#   How big a circle? Measure, do not guess (docs/CAMERAS.md, "trusted zone"): hold the
#   checkerboard at one distance, read the square size in pixels in the middle, half way
#   out and near the edge; where it differs by more than 2 % from the middle, the trusted
#   zone ends. Give that radius as a fraction of the picture width.
#
# HOW TO RUN (from the repo root; needs config.USE_FEATURE_FILTER = True, the default)
#   python -m tools.centre_mask 0.35        # keep a circle of radius 0.35 x width
#   python -m tools.centre_mask --undo      # put back the mask from before
#   The old mask is kept as mask_before_centre.png (rerunning with another radius starts from it). The circle is centred on the lens
#   centre from calib.npz when there is one.
# =====================================================================================
import argparse
import os
import shutil

import cv2
import numpy as np

import config
from vision.feature_filter import BUILD_WIDTH, MASK_FILE

BACKUP = "mask_before_centre.png"
NO_MASK_BEFORE = "mask_before_centre.none"   # marker: there was no mask.png before, so --undo removes it


def centre_mask(shape, radius_frac, centre=None):
    """White disc (255) of radius radius_frac * width on black, same shape as `shape`."""
    h, w = shape
    c = np.zeros((h, w), np.uint8)
    cx, cy = centre if centre else (w / 2, h / 2)
    cv2.circle(c, (int(round(cx)), int(round(cy))), int(round(radius_frac * w)), 255, -1)
    return c


def lens_centre(width, height, calib=None):
    """Principal point from calib.npz scaled to (width, height), or None."""
    calib = calib or config.CALIB_FILE
    if not os.path.exists(calib):
        return None
    d = np.load(calib)
    if "size" not in d.files:
        return None
    sw = float(d["size"][0])
    s = width / sw
    return float((d["K"][0, 2] + 0.5) * s - 0.5), float((d["K"][1, 2] + 0.5) * s - 0.5)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Limit landmarks to the calibrated centre of the lens")
    ap.add_argument("radius", nargs="?", type=float, default=0.35, help="radius as a fraction of picture width")
    ap.add_argument("--undo", action="store_true")
    args = ap.parse_args(argv)
    if args.undo:
        if os.path.exists(BACKUP):
            shutil.copy(BACKUP, MASK_FILE)
            os.remove(BACKUP)
            print(f"{MASK_FILE} restored from {BACKUP}")
        elif os.path.exists(NO_MASK_BEFORE):
            if os.path.exists(MASK_FILE):
                os.remove(MASK_FILE)
            os.remove(NO_MASK_BEFORE)
            print(f"{MASK_FILE} removed (there was none before)")
        else:
            print("Nothing to undo.")
        return
    first = not (os.path.exists(BACKUP) or os.path.exists(NO_MASK_BEFORE))   # keep the ORIGINAL on reruns
    if os.path.exists(MASK_FILE):
        if first:
            shutil.copy(MASK_FILE, BACKUP)
        m = cv2.imread(MASK_FILE, cv2.IMREAD_GRAYSCALE)
        if not first:                                    # rerun with another radius: start from the original
            m = cv2.imread(BACKUP, cv2.IMREAD_GRAYSCALE) if os.path.exists(BACKUP) else np.full_like(m, 255)
    else:
        open(NO_MASK_BEFORE, "w").close()
        w, h = config.CAMERA_SIZE
        m = np.full((round(h * BUILD_WIDTH / w), BUILD_WIDTH), 255, np.uint8)
    h, w = m.shape
    out = cv2.bitwise_and(m, centre_mask((h, w), args.radius, lens_centre(w, h)))
    cv2.imwrite(MASK_FILE, out)
    print(f"{MASK_FILE}: {100 * float((out > 0).mean()):.0f}% of the picture may hold landmarks "
          f"(circle radius {args.radius:.2f} x width).")


if __name__ == "__main__":
    main()
