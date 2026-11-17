# health/health.py
# Decides how much to TRUST each tracker result. Output: a quality score that
# goes straight into the ODOMETRY message's "quality" field:
#   -1      = failed, the autopilot must ignore this measurement
#   1..100  = worst .. best
#
# Checks: lighting, blur, frozen camera, tracker failure, too few points,
#         too many outliers, points bunched in one corner, sudden jumps, height jumps.

# -------------------------------------------------------------------------------------
# IN PLAIN WORDS (for non-technical readers)
#   A referee for every frame: is the picture too dark, too blurry, frozen? Did enough
#   ground points agree, spread across the picture? Did the drone "teleport"? It gives a
#   trust score 1..100 that the autopilot uses to weigh our position, and a status
#   OK / DEGRADED / LOST with a delay (hysteresis) so trust doesn't flicker on and off.
# REFERENCES
#   - Pech-Pacheco et al., "Diatom autofocusing in brightfield microscopy: a comparative
#     study", ICPR 2000 (variance of the Laplacian as a sharpness measure).
#   - MAVLink ODOMETRY quality field: https://mavlink.io/en/messages/common.html#ODOMETRY
#   - Extra checks added AROUND it (not inside): fusion/imu_check.py, recovery/.
# -------------------------------------------------------------------------------------

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

# ---------------- Settings you can tune (on YOUR camera!) ----------------
DARK_LIMIT, BRIGHT_LIMIT = 25, 230   # average brightness outside this = can't see
BLUR_BAD, BLUR_GOOD = 30.0, 150.0    # Laplacian variance: below BAD = too blurry
FROZEN_DIFF = 0.3        # frames this similar (avg grey-level change) = "same picture"
FROZEN_FRAMES = 5        # ...for this many frames in a row = camera is frozen
MIN_INLIERS, GOOD_INLIERS = 15, 80
MIN_INLIER_RATIO, GOOD_INLIER_RATIO = 0.3, 0.8
SPREAD_GRID = 4          # 4x4 = 16 cells
MIN_SPREAD, GOOD_SPREAD = 0.25, 0.6  # fraction of cells that contain an inlier
FB_LIMIT_PX = 1.0        # same limit as tracker.FB_MAX_ERROR_PX
MAX_SPEED_PX_S = 1500.0  # faster than this between frames = a glitch, not real motion
SCALE_MIN, SCALE_MAX = 0.75, 1.33    # ground suddenly ~25% bigger/smaller = bad fit
LOSE_AFTER = 3           # this many bad frames in a row -> status LOST (call recovery)
REGAIN_AFTER = 5         # this many good frames in a row before we trust it again


@dataclass
class HealthReport:
    quality: int = -1             # -1 = failed, 1..100 = worst..best
    status: str = "LOST"          # "OK", "DEGRADED" (brief glitch) or "LOST" (needs recovery)
    reasons: list = field(default_factory=list)   # what went wrong, for the screen and logs
    details: dict = field(default_factory=dict)   # all the raw numbers, for the CSV log


def _score(value, bad, good):
    """Map a number onto 0..1: 'bad' or worse -> 0, 'good' or better -> 1."""
    return float(np.clip((value - bad) / (good - bad), 0.0, 1.0))


def spread_fraction(points, width, height, grid=SPREAD_GRID):
    """What fraction of the grid cells contain at least one point?"""
    if len(points) == 0:
        return 0.0
    col = np.clip((points[:, 0] * grid / width).astype(int), 0, grid - 1)
    row = np.clip((points[:, 1] * grid / height).astype(int), 0, grid - 1)
    return len(set(zip(col.tolist(), row.tolist()))) / (grid * grid)


class HealthChecker:
    def __init__(self):
        self.reset()

    def reset(self):
        """Call this whenever a new HOME is saved."""
        self.prev_gray = None
        self.prev_t = None
        self.prev_shift = None    # last TRUSTED shift...
        self.prev_shift_t = None  # ...and when it was captured
        self.frozen_count = 0
        self.bad_count = 0
        self.good_count = 0
        self.lost = True          # start untrusted until we see a few good frames

    def check(self, gray, res):
        h, w = gray.shape
        fails = []                # anything in here = this frame can't be trusted
        d = {}

        # 1. Lighting
        d["brightness"] = float(gray.mean())
        if d["brightness"] < DARK_LIMIT:
            fails.append("too dark")
        elif d["brightness"] > BRIGHT_LIMIT:
            fails.append("too bright")

        # 2. Blur: sharp images have lots of strong edges -> high Laplacian variance
        d["sharpness"] = float(cv2.Laplacian(gray, cv2.CV_64F).var())
        if d["sharpness"] < BLUR_BAD:
            fails.append("blurry")

        # 3. Frozen camera: "almost no change" for several frames, OR time didn't move
        if self.prev_gray is not None and self.prev_gray.shape == gray.shape:
            d["frame_diff"] = float(cv2.absdiff(gray, self.prev_gray).mean())
            same_time = self.prev_t is not None and res.t_capture <= self.prev_t
            if d["frame_diff"] < FROZEN_DIFF or same_time:
                self.frozen_count += 1
            else:
                self.frozen_count = 0
        if self.frozen_count >= FROZEN_FRAMES:
            fails.append("camera frozen")

        # 4. Did the tracker work at all?
        if not res.ok:
            fails.append(res.reason or "tracker failed")
        else:
            # 5. Enough agreeing points?
            ratio = res.n_inliers / max(res.n_fb_ok, 1)
            spread = spread_fraction(res.inlier_pts, w, h)
            d.update(inliers=res.n_inliers, inlier_ratio=ratio, spread=spread,
                     fb_median=res.fb_median_px, scale=res.scale, rotation=res.rotation_deg)
            if res.n_inliers < MIN_INLIERS:
                fails.append(f"few inliers ({res.n_inliers})")
            if ratio < MIN_INLIER_RATIO:
                fails.append(f"many outliers ({ratio:.0%} agree)")
            # 6. Points bunched in one area -> rotation/shift is unreliable
            if spread < MIN_SPREAD:
                fails.append(f"points bunched ({spread:.0%} of image)")
            # 7. Ground suddenly looks much bigger/smaller -> bad fit (or big height jump)
            if not SCALE_MIN <= res.scale <= SCALE_MAX:
                fails.append(f"scale jump ({res.scale:.2f})")
            # 8. Teleport check: a real drone can't jump hundreds of pixels in one frame
            if self.prev_shift is not None:
                dt = res.t_capture - self.prev_shift_t   # time since the last trusted shift
                if dt > 0:
                    jump = math.hypot(res.shift_px[0] - self.prev_shift[0],
                                      res.shift_px[1] - self.prev_shift[1])
                    d["speed_px_s"] = jump / dt
                    if d["speed_px_s"] > MAX_SPEED_PX_S:
                        fails.append("sudden jump")

        # ---- Combine everything into one 1..100 score ----
        if res.ok:
            raw = (0.30 * _score(res.n_inliers, MIN_INLIERS, GOOD_INLIERS)
                   + 0.20 * _score(d["inlier_ratio"], MIN_INLIER_RATIO, GOOD_INLIER_RATIO)
                   + 0.20 * _score(d["spread"], MIN_SPREAD, GOOD_SPREAD)
                   + 0.15 * (1.0 - min(res.fb_median_px / FB_LIMIT_PX, 1.0))
                   + 0.15 * _score(d["sharpness"], BLUR_BAD, BLUR_GOOD))
            score = int(np.clip(round(100 * raw), 1, 100))
        else:
            score = -1
        d["raw_score"] = score

        frame_good = not fails

        # ---- Hysteresis: don't flicker between trusted / not trusted ----
        if not frame_good:
            self.bad_count += 1
            self.good_count = 0
        else:
            self.good_count += 1
            self.bad_count = 0
        if self.lost and self.good_count >= REGAIN_AFTER:
            self.lost = False
        if not self.lost and self.bad_count >= LOSE_AFTER:
            self.lost = True

        if self.lost:
            status, quality = "LOST", -1
            if not fails:
                fails = [f"re-gaining trust ({self.good_count}/{REGAIN_AFTER})"]
        elif fails:
            status, quality = "DEGRADED", -1   # one bad frame: skip it, keep going
        else:
            status, quality = "OK", score

        # Remember this frame for next time (only trust good shifts for the jump check)
        self.prev_gray = gray.copy()
        self.prev_t = res.t_capture
        if res.ok and frame_good:
            self.prev_shift = res.shift_px
            self.prev_shift_t = res.t_capture

        return HealthReport(quality=quality, status=status, reasons=fails, details=d)
