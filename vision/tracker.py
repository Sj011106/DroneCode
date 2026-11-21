# vision/tracker.py
# Ground tracker: Harris corners + Lucas-Kanade + forward-backward check + RANSAC.
#
# ======================= TEAM SIGN CONVENTION (read this!) =======================
#  shift_px = how far the DRONE (camera) has moved AWAY FROM HOME, in pixels,
#             in the CURRENT image's axes:  +x = image right,  +y = image down.
#  to_north_east() turns shift_px into metres:  +north, +east.
#  Assumes the camera looks straight down with the TOP of the image toward the
#  drone's nose.
#  Hand test: slide the camera to ITS RIGHT over a table -> shift_px x goes UP (+).
# ==================================================================================
#
# How to run the demo:
#   python -m vision.tracker            -> webcam (point it at the floor/table)
#   python -m vision.tracker video.mp4  -> a video file
# Keys:  r = save HOME (reference)   q = quit

# -------------------------------------------------------------------------------------
# IN PLAIN WORDS (for non-technical readers)
#   When home is saved, the tracker picks ~300 sharp "corners" on the ground picture
#   (pebbles, edges, marks). In every new frame it finds those same corners again, throws
#   away any that don't come back cleanly (forward-backward check) or don't move like the
#   rest (RANSAC), and from the ones that agree works out how far the drone has slid and
#   turned since home. Output: a pixel shift that to_north_east() turns into metres.
# REFERENCES
#   - Harris & Stephens, "A Combined Corner and Edge Detector", Alvey Vision Conf. 1988.
#   - Lucas & Kanade, "An Iterative Image Registration Technique with an Application to
#     Stereo Vision", IJCAI 1981; Bouguet, "Pyramidal Implementation of the Lucas Kanade
#     Feature Tracker", Intel 2000.
#   - Kalal, Mikolajczyk, Matas, "Forward-Backward Error: Automatic Detection of Tracking
#     Failures", ICPR 2010.
#   - Fischler & Bolles, "Random Sample Consensus", Comm. ACM 1981.
#   - Extended (not changed) by vision/rolling_shutter.py (RollingShutterTracker).
# -------------------------------------------------------------------------------------

import math
import os
import sys
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

# ---------------- Settings you can tune ----------------
MAX_CORNERS = 300        # how many Harris corners to keep in the home image
GRID = (4, 4)            # split image into 4x4 cells so corners are spread out
HARRIS_K = 0.04          # Harris "k" (0.04-0.06 is the usual range)
HARRIS_QUALITY = 0.001   # Harris-specific! (Shi-Tomasi values like 0.01 are too strict for Harris)
MIN_DISTANCE = 10        # corners must be at least this many pixels apart
BLOCK_SIZE = 7           # neighbourhood size Harris looks at
BORDER = 15              # ignore corners this close to the image edge

LK_PARAMS = dict(
    winSize=(21, 21),    # patch size LK matches (bigger = handles bigger moves, slower)
    maxLevel=3,          # pyramid levels (3 -> handles roughly 15x bigger motion)
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01),
    minEigThreshold=1e-4,  # LK gives up on flat, textureless patches
)
FB_MAX_ERROR_PX = 1.0    # forward-backward check: point must come back within 1 px
RANSAC_THRESH_PX = 2.0   # RANSAC: a point is an "inlier" if it fits within 2 px
MIN_POINTS = 8           # need at least this many points to try RANSAC
MIN_INLIERS_TO_UPDATE = 15  # only trust a result enough to use it as the next prediction
# Used only if calib.npz is missing. Justice D datasheet: 1280x720, horizontal FOV 114 deg.
# Focal length in pixels = (image width / 2) / tan(HFOV / 2), so it depends on the width
# we track at: ~416 px at 1280 wide, ~208 px at the 640 wide frames vision_hold.py uses.
DEFAULT_HFOV_DEG = 114.0


def focal_from_fov(width_px, hfov_deg=DEFAULT_HFOV_DEG):
    """Pinhole focal length (pixels) of a camera with this horizontal field of view."""
    return (width_px / 2) / math.tan(math.radians(hfov_deg) / 2)


DEFAULT_FOCAL_PX = focal_from_fov(640)   # ~208 px: the 640x360 frames we fly with


# ---------------- The result the tracker hands to everyone else ----------------
@dataclass
class TrackResult:
    t_capture: float                 # when the frame was GRABBED (time.time() seconds)
    attitude: tuple = None           # (roll, pitch, yaw) in radians AT t_capture (for tilt cancel)
    ok: bool = False                 # did we get a transform at all?
    shift_px: tuple = None           # (dx, dy) drone movement from home, see convention above
    rotation_deg: float = 0.0        # how much the ground image has rotated since home
    scale: float = 1.0               # >1 = ground looks bigger = drone is lower than at home
    n_ref: int = 0                   # corners saved in the home image
    n_tracked: int = 0               # LK found them in this frame
    n_fb_ok: int = 0                 # ...and they passed the forward-backward check
    n_inliers: int = 0               # ...and RANSAC agreed with them
    fb_median_px: float = 99.0       # typical forward-backward error of the good points
    inlier_pts: np.ndarray = field(default_factory=lambda: np.empty((0, 2), np.float32))
    reason: str = ""                 # why it failed (empty if ok)


def calibration_size(path="calib.npz"):
    """The (width, height) the calibration was made at, or None if the file doesn't say
    (old files from calibrate.py, and the laptop-webcam calib.npz, have no size)."""
    if not os.path.exists(path):
        return None
    data = np.load(path)
    if "size" not in data.files:
        return None
    w, h = (int(v) for v in np.asarray(data["size"]).ravel()[:2])
    return w, h


def load_calibration(path="calib.npz", image_size=None):
    """Read camera matrix K and lens distortion from calibrate.py's output.

    image_size = (width, height) of the frames K will be used on (optional). If given:
      - the file's saved size matches     -> K as saved
      - same shape, different size        -> K rescaled to image_size (printed)
      - different shape, or no size saved -> warning; the shape mismatch returns
        (None, None) so the tracker falls back to the datasheet focal length instead of
        using a lens file from another camera (wrong metres, silently).
    Without image_size it behaves exactly as before."""
    if not os.path.exists(path):
        return None, None
    data = np.load(path)
    K, dist = data["K"].astype(np.float64), data["dist"].astype(np.float64)
    if image_size is None:
        return K, dist
    saved = calibration_size(path)
    w, h = image_size
    if saved is None:
        print(f"WARNING: {path} has no saved image size - can't check it matches the "
              f"{w}x{h} camera. Recalibrate with calibration.auto_calibrate.")
        return K, dist
    sw, sh = saved
    if (sw, sh) == (w, h):
        return K, dist
    if abs(sw * h - sh * w) > max(sw, w):        # aspect ratio differs (e.g. 4:3 vs 16:9)
        print(f"WARNING: {path} was made at {sw}x{sh} but the camera gives {w}x{h} - "
              f"a different camera or mode. Ignoring it (using the datasheet focal length).")
        return None, None
    s = w / sw
    K = K.copy()
    K[0, 0] *= s
    K[1, 1] *= s
    K[:2, 2] = (K[:2, 2] + 0.5) * s - 0.5        # pixel centres (same rule as cv2.resize)
    print(f"Note: {path} made at {sw}x{sh}, rescaled to {w}x{h}.")
    return K, dist


def to_north_east(shift_px, height_m, focal_px, yaw_rad):
    """
    Turn a pixel shift (team convention) into metres North and East of home.
      height_m : height above the ground (barometer/rangefinder), NOT from vision
      yaw_rad  : drone heading AT t_capture (0 = facing North, +90 deg = facing East)
    """
    dx, dy = shift_px
    metres_per_px = height_m / focal_px      # pinhole camera: metres = pixels * height / focal
    right_m = dx * metres_per_px             # image right = drone's right
    forward_m = -dy * metres_per_px          # image UP = drone's nose, so flip y
    north = forward_m * math.cos(yaw_rad) - right_m * math.sin(yaw_rad)
    east = forward_m * math.sin(yaw_rad) + right_m * math.cos(yaw_rad)
    return north, east


def _per_cell_detect():
    """BETAFLIGHT CHANGE: config.CORNERS_PER_CELL (read at call time so --set and tests can switch it)."""
    try:
        import config
        return bool(getattr(config, "CORNERS_PER_CELL", False))
    except ImportError:
        return False


def _is_still(M, centre, px=1.5):
    """BETAFLIGHT CHANGE: does this home->now motion say 'nothing moved' (centre within px, no turn, no zoom)?"""
    cx, cy = centre
    dx = M[0, 0] * cx + M[0, 1] * cy + M[0, 2] - cx
    dy = M[1, 0] * cx + M[1, 1] * cy + M[1, 2] - cy
    return (math.hypot(dx, dy) < px and abs(math.atan2(M[1, 0], M[0, 0])) < math.radians(0.5)
            and abs(math.hypot(M[0, 0], M[1, 0]) - 1.0) < 0.01)


class GroundTracker:
    def __init__(self, K=None, dist=None, point_filter=None):
        self.K = K
        self.dist = dist
        # Feature filter (vision/feature_filter.py): keeps home corners off the drone's own
        # props/arms, burned-in OSD text and glare. None = off = the tracker behaves exactly
        # as before. vision_hold.py switches it on with feature_filter.attach(tracker, health).
        self.point_filter = point_filter
        self.ref_gray = None     # the HOME image
        self.ref_pts = None      # Harris corners in the home image, shape (N,1,2)
        self.ref_und = None      # same corners with lens distortion removed
        self.M_last = np.eye(2, 3, dtype=np.float64)  # last good home->now transform
        self.centre = None       # (cx, cy) optical centre in pixels
        # BETAFLIGHT CHANGE (vision/frozen.py): home points to leave out (attached to us: shoes, legs, own legs),
        # and where every home point was found in the last frame. None = off = exactly the old behaviour.
        self.exclude = None
        self.last_cur = self.last_valid = None
        self.reject_static = False   # BETAFLIGHT CHANGE: True while the IMU says "we are being carried" (bench):
                                     # a "nothing moved" answer is then checked against the second-best motion

    # ---------- Step 1: Harris corners, spread across the whole image ----------
    def _detect_harris(self, gray):
        h, w = gray.shape
        mask = np.zeros_like(gray)
        mask[BORDER:h - BORDER, BORDER:w - BORDER] = 255   # skip the edges
        if self.point_filter is not None:                  # skip props / arms / OSD text too
            static = self.point_filter.mask_for(gray.shape)
            if static is not None:
                mask = cv2.bitwise_and(mask, static)

        per_cell = max(1, MAX_CORNERS // (GRID[0] * GRID[1]))
        # Ask for LOTS of Harris corners (strongest first)...
        corners = cv2.goodFeaturesToTrack(
            gray, maxCorners=MAX_CORNERS * 4, qualityLevel=HARRIS_QUALITY,
            minDistance=MIN_DISTANCE, mask=mask, blockSize=BLOCK_SIZE,
            useHarrisDetector=True, k=HARRIS_K)
        if _per_cell_detect():
            # BETAFLIGHT CHANGE: top up STARVED cells. The search above takes the strongest corners of the whole
            # picture first, so one strongly textured area (the holder's trousers, shoes, a bag) can take them all
            # and leave a plain floor with none (tests/test_frozen.py: 144 of 144 on the legs). A cell that got
            # fewer than a third of its share is searched again on its OWN strength scale. Cells that were fine are
            # untouched (searching every cell that way added weak, noisy corners: climb test drift 15 -> 31 cm).
            have = {}
            for c in (corners if corners is not None else []):
                x, y = c.ravel()
                cell = (int(x * GRID[0] / w), int(y * GRID[1] / h))
                have[cell] = have.get(cell, 0) + 1
            extra, pad = [], BLOCK_SIZE
            for gy in range(GRID[1]):
                for gx in range(GRID[0]):
                    if have.get((gx, gy), 0) >= max(1, per_cell // 3):
                        continue
                    x0, x1 = gx * w // GRID[0], (gx + 1) * w // GRID[0]
                    y0, y1 = gy * h // GRID[1], (gy + 1) * h // GRID[1]
                    X0, Y0, X1, Y1 = max(0, x0 - pad), max(0, y0 - pad), min(w, x1 + pad), min(h, y1 + pad)
                    sub = np.zeros((Y1 - Y0, X1 - X0), np.uint8)
                    sub[y0 - Y0:y1 - Y0, x0 - X0:x1 - X0] = mask[y0:y1, x0:x1]
                    if not sub.any():
                        continue
                    c = cv2.goodFeaturesToTrack(
                        np.ascontiguousarray(gray[Y0:Y1, X0:X1]), maxCorners=per_cell * 2, qualityLevel=HARRIS_QUALITY,
                        minDistance=MIN_DISTANCE, mask=sub, blockSize=BLOCK_SIZE, useHarrisDetector=True, k=HARRIS_K)
                    if c is not None:
                        extra.append(c + np.float32([X0, Y0]))
            if extra:
                corners = np.concatenate(([corners] if corners is not None else []) + extra)
        if corners is None:
            return np.empty((0, 1, 2), np.float32)
        if self.point_filter is not None:                  # drop corners on glare
            corners = corners[self.point_filter.keep(gray, corners.reshape(-1, 2))]
            if len(corners) == 0:
                return np.empty((0, 1, 2), np.float32)

        # ...then keep only the best few in each grid cell, so they cover the image.
        cell_count = {}
        kept = []
        for c in corners:                         # already sorted strongest -> weakest
            x, y = c.ravel()
            cell = (int(x * GRID[0] / w), int(y * GRID[1] / h))
            if cell_count.get(cell, 0) < per_cell:
                cell_count[cell] = cell_count.get(cell, 0) + 1
                kept.append(c)
        pts = np.array(kept, dtype=np.float32).reshape(-1, 1, 2)

        # Refine each corner to sub-pixel accuracy (from the Harris tutorial)
        crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        cv2.cornerSubPix(gray, pts, (5, 5), (-1, -1), crit)
        return pts

    def _undistort(self, pts):
        """Remove lens bending so straight lines on the ground stay straight."""
        if self.K is None or self.dist is None or len(pts) == 0:
            return pts.reshape(-1, 2)
        return cv2.undistortPoints(pts.reshape(-1, 1, 2), self.K, self.dist, P=self.K).reshape(-1, 2)

    # ---------- Save HOME ----------
    def set_reference(self, gray):
        h, w = gray.shape
        if self.K is None:   # no calibration: datasheet focal length for THIS image width
            f = focal_from_fov(w)
            self.K = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]])
        self.centre = (self.K[0, 2], self.K[1, 2])   # calibrated centre, not w/2, h/2
        self.ref_gray = gray.copy()
        self.ref_pts = self._detect_harris(gray)
        self.ref_und = self._undistort(self.ref_pts)
        self.M_last = np.eye(2, 3, dtype=np.float64)  # at home, nothing has moved
        self.exclude = self.last_cur = self.last_valid = None   # BETAFLIGHT CHANGE: new home, new list
        return len(self.ref_pts)

    def relock(self, M):
        """recovery.py calls this with a home->now transform found by ORB."""
        self.M_last = np.asarray(M, dtype=np.float64).reshape(2, 3)

    @property
    def focal_px(self):
        return (self.K[0, 0] + self.K[1, 1]) / 2 if self.K is not None else DEFAULT_FOCAL_PX

    # ---------- Steps 2-5: track one new frame ----------
    def track(self, gray, t_capture=None, attitude=None):
        if t_capture is None:
            t_capture = time.time()
        res = TrackResult(t_capture=t_capture, attitude=attitude)
        if self.ref_gray is None or len(self.ref_pts) < MIN_POINTS:
            res.reason = "no home image (or too few corners in it)"
            return res
        h, w = gray.shape
        res.n_ref = len(self.ref_pts)

        # Step 2: PRE-ALIGN. Undo the last known shift + rotation, so this frame
        # looks almost like home. Then LK only has to find a tiny leftover move.
        warped = cv2.warpAffine(gray, self.M_last, (w, h),
                                flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)

        # Step 3: Lucas-Kanade FORWARD (home -> now) and BACKWARD (now -> home)
        fwd, st1, _ = cv2.calcOpticalFlowPyrLK(self.ref_gray, warped, self.ref_pts, None, **LK_PARAMS)
        back, st2, _ = cv2.calcOpticalFlowPyrLK(warped, self.ref_gray, fwd, None, **LK_PARAMS)
        fb_err = np.linalg.norm(self.ref_pts - back, axis=2).ravel()   # how far it came back

        # Map tracked points from the pre-aligned image back to the real frame
        cur = cv2.transform(fwd, self.M_last).reshape(-1, 2)
        inside = ((cur[:, 0] > BORDER) & (cur[:, 0] < w - BORDER) &
                  (cur[:, 1] > BORDER) & (cur[:, 1] < h - BORDER))

        lk_ok = (st1.ravel() == 1) & (st2.ravel() == 1) & inside
        good = lk_ok & (fb_err < FB_MAX_ERROR_PX)          # Step 4: forward-backward check
        self.last_cur, self.last_valid = cur, good.copy()  # BETAFLIGHT CHANGE: for vision/frozen.py
        if self.exclude is not None and len(self.exclude) == len(good):
            good &= ~self.exclude                          # BETAFLIGHT CHANGE: attached to us -> not the floor
        res.n_tracked = int(lk_ok.sum())
        res.n_fb_ok = int(good.sum())
        if res.n_fb_ok < MIN_POINTS:
            res.reason = f"only {res.n_fb_ok} points passed forward-backward"
            return res
        res.fb_median_px = float(np.median(fb_err[good]))

        # Step 5: RANSAC finds the shift + rotation + scale most points agree on
        src = self.ref_und[good]
        dst = self._undistort(cur[good].astype(np.float32))
        M, inl = cv2.estimateAffinePartial2D(
            src, dst, method=cv2.RANSAC, ransacReprojThreshold=RANSAC_THRESH_PX,
            maxIters=2000, confidence=0.99, refineIters=10)
        if M is None:
            res.reason = "RANSAC found no consistent motion"
            return res
        inl = inl.ravel().astype(bool)
        # BETAFLIGHT CHANGE (bench, vision/frozen.py): we are being carried but the best motion is "nothing moved"
        # -> those agreeing points are attached to us (holder's shoes, legs, own legs). If the REST of the points
        # agree on a real motion, that is the floor: use it. Never while not carried (reject_static False).
        if self.reject_static and _is_still(M, self.centre) and (~inl).sum() >= MIN_POINTS:
            rest = np.flatnonzero(~inl)
            M2, inl2 = cv2.estimateAffinePartial2D(
                src[rest], dst[rest], method=cv2.RANSAC, ransacReprojThreshold=RANSAC_THRESH_PX,
                maxIters=2000, confidence=0.99, refineIters=10)
            if M2 is not None and not _is_still(M2, self.centre):
                inl2 = inl2.ravel().astype(bool)
                if inl2.sum() >= max(MIN_INLIERS_TO_UPDATE, 0.3 * len(rest)):
                    M, inl = M2, np.zeros_like(inl)
                    inl[rest[inl2]] = True
                    res.reason = "attached points ignored"
                    res.n_fb_ok = len(rest)        # the attached ones were never floor candidates (health's ratio)
        res.n_inliers = int(inl.sum())
        res.inlier_pts = dst[inl]

        # Step 6: the CENTRE trick. Where did home (centre of the home image) go?
        cx, cy = self.centre
        home_now_x = M[0, 0] * cx + M[0, 1] * cy + M[0, 2]
        home_now_y = M[1, 0] * cx + M[1, 1] * cy + M[1, 2]
        # Home appears at (home_now). The drone is above the image centre.
        # So the drone moved (centre - home_now) away from home.
        res.shift_px = (float(cx - home_now_x), float(cy - home_now_y))
        res.rotation_deg = math.degrees(math.atan2(M[1, 0], M[0, 0]))
        res.scale = math.hypot(M[0, 0], M[1, 0])
        res.ok = True

        # Use this as the prediction for the next frame, if we really trust it
        # (BETAFLIGHT CHANGE: while carried, "nothing moved" is never kept as the prediction: it is the holder's legs)
        if (res.n_inliers >= MIN_INLIERS_TO_UPDATE and res.n_inliers >= 0.5 * res.n_fb_ok
                and not (self.reject_static and _is_still(M, self.centre))):
            self.M_last = M
        return res


# ---------------- Demo: webcam / video ----------------
def main():
    from health.health import HealthChecker   # imported here so tracker.py works on its own too

    source = sys.argv[1] if len(sys.argv) > 1 else "0"
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)   # "2" = camera 2, not a file
    K, dist = load_calibration()
    tracker = GroundTracker(K, dist)
    health = HealthChecker()
    height_m = 0.5   # demo only: on the drone this comes from the barometer/rangefinder

    while True:
        ok, frame = cap.read()
        t_capture = time.time()          # stamp the moment the frame arrived
        if not ok:
            break
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if tracker.ref_gray is None:
            text, colour = "Press r to save HOME", (255, 255, 255)
        else:
            res = tracker.track(gray, t_capture, attitude=None)
            rep = health.check(gray, res)
            if rep.quality > 0:
                north, east = to_north_east(res.shift_px, height_m, tracker.focal_px, yaw_rad=0.0)
                text = (f"{rep.status} q={rep.quality}  N={north*100:+.1f}cm E={east*100:+.1f}cm "
                        f"rot={res.rotation_deg:+.1f}  inl={res.n_inliers}")
                colour = (0, 255, 0)
                c = (int(tracker.centre[0]), int(tracker.centre[1]))
                tip = (int(c[0] - res.shift_px[0]), int(c[1] - res.shift_px[1]))
                cv2.arrowedLine(frame, c, tip, colour, 3)     # arrow points back to home
                for x, y in res.inlier_pts.astype(int):
                    cv2.circle(frame, (int(x), int(y)), 2, (0, 255, 255), -1)
            else:
                text, colour = f"{rep.status} q=-1  {', '.join(rep.reasons)}", (0, 0, 255)

        cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, colour, 2)
        cv2.imshow("tracker", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("r"):
            n = tracker.set_reference(gray)
            health.reset()
            print(f"HOME saved with {n} Harris corners")
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
