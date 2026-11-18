# =====================================================================================
# vision/feature_filter.py   (OWNER: Saumya)                              BRIEF ITEM 1
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Not every corner in the picture is a good landmark. The Justice D looks straight
#   down with a very wide lens (114 x 86 degrees), so the drone's own propellers and
#   arms can be in view, and the camera can burn OSD text (voltage, name, timer) into
#   the picture. Those things never move in the image, so corners on them say "the drone
#   hasn't moved" and pull the position towards zero. Glare (pixels near 255) makes
#   corners that slide around as the sun angle changes.
#   This filter keeps home corners OFF those areas:
#     1. a fixed mask image, mask.png (white = ground, black = ignore). Make it once on
#        the real drone with   python -m vision.feature_filter build <video>
#        from a recording where the drone is MOVED around by hand over textured ground:
#        anything that stays the same in every frame is part of the drone (or OSD text).
#     2. a glare check: no corners where the picture is (nearly) white.
#
# HOW IT PLUGS IN
#   vision_hold.py calls  attach(tracker, health)  once (if config.USE_FEATURE_FILTER).
#   If mask.png exists, the mask and the glare check are used whenever home corners are
#   picked (set_reference, so also re-homing), and health.py uses the same mask so the
#   "spread" check only counts grid cells where corners are allowed.
#   No mask.png, or attach() not called = no filter = exactly the old behaviour. Off by
#   default on purpose: the simulated tests must not pick up the real drone's mask.png.
#
# INTERFACE
#   keep_mask(gray, pts) -> np.ndarray[bool]     True = keep the point (the suggested API)
#   PointFilter.mask_for(shape) / .keep(gray, pts) what the tracker calls
#
# COMMANDS (from the repo root)
#   python -m vision.feature_filter build recordings/<folder>/video.avi    -> mask.png
#   python -m vision.feature_filter build /dev/video2 --seconds 20         -> live camera
#   python -m vision.feature_filter show  recordings/<folder>/video.avi    -> mask_preview.png
#
# IDEAS + REFERENCES (not done yet)
#   - Repeated texture (tiles, crops in rows): ratio of best to second-best match.
#     Lowe, "Distinctive Image Features from Scale-Invariant Keypoints", IJCV 2004.
#   - Moving objects: points whose residual after RANSAC stays high over several frames.
#     Torr & Zisserman, "MLESAC", CVIU 2000.
#   - Static mask by temporal variance: the same idea as background subtraction, e.g.
#     Stauffer & Grimson, "Adaptive background mixture models", CVPR 1999 (in reverse:
#     here the STILL pixels are the ones we throw away).
# =====================================================================================

import argparse
import os
import sys
import time

import cv2
import numpy as np

# ---------------- Settings you can tune ----------------
MASK_FILE = "mask.png"   # white (255) = ground, use it; black (0) = ignore
GLARE_LEVEL = 250        # a corner whose neighbourhood reaches this grey level = glare
GLARE_RADIUS = 2         # neighbourhood checked around each corner (pixels)
BUILD_WIDTH = 640        # build the mask at this width (it is resized to any frame later)
STATIC_STD = 3.0         # a pixel that changes less than this (grey levels, std over time)
                         # in a MOVING recording = part of the drone or OSD text
MIN_FRAMES = 60          # fewest frames for a trustworthy mask (~2 s at 30 fps)
GROW_PX = 8              # grow the ignored areas a little (at BUILD_WIDTH), for blur edges
MIN_KEEP_FRACTION = 0.4  # if less than this is "ground", the drone probably didn't move


class PointFilter:
    """Fixed mask + glare check. mask: uint8 image, 255 = keep, 0 = ignore (any size)."""

    def __init__(self, mask=None, glare_level=GLARE_LEVEL):
        self.mask = mask
        self.glare_level = glare_level
        self._resized = {}                       # (h, w) -> mask at that size

    def mask_for(self, shape):
        """The mask at this image size (h, w), or None if there is no mask."""
        if self.mask is None:
            return None
        h, w = shape[:2]
        if (h, w) not in self._resized:
            m = cv2.resize(self.mask, (w, h), interpolation=cv2.INTER_NEAREST)
            self._resized[(h, w)] = np.where(m > 127, 255, 0).astype(np.uint8)
        return self._resized[(h, w)]

    def keep(self, gray, pts):
        """True for every point that is on allowed ground and not on glare."""
        pts = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
        if len(pts) == 0:
            return np.zeros(0, dtype=bool)
        h, w = gray.shape[:2]
        x = np.clip(np.round(pts[:, 0]).astype(int), 0, w - 1)
        y = np.clip(np.round(pts[:, 1]).astype(int), 0, h - 1)
        ok = np.ones(len(pts), dtype=bool)
        m = self.mask_for(gray.shape)
        if m is not None:
            ok &= m[y, x] > 0
        if self.glare_level is not None:
            k = 2 * GLARE_RADIUS + 1
            brightest = cv2.dilate(gray, np.ones((k, k), np.uint8))   # max of each neighbourhood
            ok &= brightest[y, x] < self.glare_level
        return ok


def load_mask(path=MASK_FILE):
    """mask.png as a 0/255 uint8 image, or None if the file isn't there."""
    if not path or not os.path.exists(path):
        return None
    m = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if m is None:
        print(f"WARNING: {path} exists but can't be read as an image - feature filter off")
        return None
    return np.where(m > 127, 255, 0).astype(np.uint8)


def default_filter(path=MASK_FILE):
    """What GroundTracker uses: a PointFilter if mask.png exists, otherwise None (off)."""
    m = load_mask(path)
    return PointFilter(m) if m is not None else None


def attach(tracker, health=None, path=MASK_FILE):
    """Switch the filter on for a tracker (and health checker). Returns True if a mask was found."""
    f = default_filter(path)
    if f is None:
        print(f"Feature filter: no {path} - off (make one: python -m vision.feature_filter build ...)")
        return False
    tracker.point_filter = f
    if health is not None:
        health.mask = f.mask
        health._cells = {}
    print(f"Feature filter ON: {path} ({(f.mask > 0).mean():.0%} of the picture usable) + glare check")
    return True


def keep_mask(gray, pts, mask=None):
    """The suggested interface: True = keep. mask=None -> mask.png (if any) + glare check."""
    f = PointFilter(mask if mask is not None else load_mask())
    return f.keep(gray, pts)


# ---------------- Building mask.png from a recording ----------------
def _to_small_gray(frame):
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    if w > BUILD_WIDTH:
        gray = cv2.resize(gray, (BUILD_WIDTH, round(h * BUILD_WIDTH / w)), interpolation=cv2.INTER_AREA)
    return gray


def build_static_mask(frames):
    """frames: grey (or BGR) images from a recording where the drone MOVES.
    Returns (mask, info): mask is 0/255 at BUILD_WIDTH (None if the recording is unusable),
    info explains the result in words."""
    n, mean, m2 = 0, None, None
    for frame in frames:
        g = _to_small_gray(frame).astype(np.float32)
        if mean is None:
            mean, m2 = np.zeros_like(g), np.zeros_like(g)
        elif g.shape != mean.shape:
            continue                                   # skip frames of another size
        n += 1                                         # Welford: running mean + variance
        delta = g - mean
        mean += delta / n
        m2 += delta * (g - mean)
    if n < MIN_FRAMES:
        return None, f"only {n} frames - record at least {MIN_FRAMES} (about 2 s) while moving"
    std = np.sqrt(m2 / (n - 1))
    static = (std < STATIC_STD).astype(np.uint8) * 255
    # Remove speckles (a few still pixels in random places), then grow the real blobs.
    static = cv2.morphologyEx(static, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    static = cv2.dilate(static, np.ones((2 * GROW_PX + 1,) * 2, np.uint8))
    mask = 255 - static
    keep = float((mask > 0).mean())
    if keep < MIN_KEEP_FRACTION:
        return None, (f"only {keep:.0%} of the picture changed over {n} frames - move the drone "
                      f"around more (by hand, props OFF) over textured ground and record again")
    return mask, f"{n} frames: {keep:.0%} of the picture is ground, {1 - keep:.0%} ignored"


def read_frames(source, seconds=None, max_frames=900):
    """Grey frames from a video file, a recording folder, or a camera ('2' or '/dev/video2')."""
    if os.path.isdir(source):
        source = os.path.join(source, "video.avi")    # a tools/replay.py recording folder
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if not cap.isOpened():
        raise SystemExit(f"Can't open {source}")
    t_end = time.time() + seconds if seconds else None
    n = 0
    try:
        while n < max_frames and (t_end is None or time.time() < t_end):
            ok, frame = cap.read()
            if not ok:
                break
            n += 1
            yield frame
    finally:
        cap.release()


def preview(frame, mask):
    """The frame with ignored areas tinted red (to check the mask by eye)."""
    img = frame if frame.ndim == 3 else cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    m = cv2.resize(mask, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
    red = img.copy()
    red[m == 0] = (0.4 * red[m == 0] + 0.6 * np.array([0, 0, 255])).astype(np.uint8)
    return red


def main(argv=None):
    ap = argparse.ArgumentParser(description="Feature filter: build or check mask.png")
    sub = ap.add_subparsers(dest="cmd", required=True)
    b = sub.add_parser("build", help="make mask.png from a recording where the drone moves")
    b.add_argument("source", help="video file, recording folder, or camera ('2' or /dev/video2)")
    b.add_argument("--out", default=MASK_FILE)
    b.add_argument("--seconds", type=float, default=None, help="camera only: record this long")
    s = sub.add_parser("show", help="save a picture of the mask over a frame")
    s.add_argument("source")
    s.add_argument("--mask", default=MASK_FILE)
    s.add_argument("--out", default="mask_preview.png")
    args = ap.parse_args(argv)

    if args.cmd == "build":
        if args.seconds is None and (args.source.isdigit() or args.source.startswith("/dev/video")):
            args.seconds = 20.0
            print("Recording 20 s from the camera: move the drone around by hand (props OFF).")
        frames = list(read_frames(args.source, args.seconds))
        mask, info = build_static_mask(frames)
        print(info)
        if mask is None:
            sys.exit(1)
        cv2.imwrite(args.out, mask)
        cv2.imwrite(os.path.splitext(args.out)[0] + "_preview.png", preview(frames[len(frames) // 2], mask))
        print(f"Saved {args.out} (+ preview). Check the preview: red = ignored. Commit {args.out}.")
    else:
        mask = load_mask(args.mask)
        if mask is None:
            raise SystemExit(f"No {args.mask} - run the 'build' command first")
        frame = next(read_frames(args.source), None)
        if frame is None:
            raise SystemExit(f"No frames in {args.source}")
        cv2.imwrite(args.out, preview(frame, mask))
        print(f"Saved {args.out} (red = ignored)")


if __name__ == "__main__":
    main()
