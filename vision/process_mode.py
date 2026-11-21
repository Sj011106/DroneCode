# =====================================================================================
# vision/process_mode.py  -  low: small fast picture. high: sharp centre crop      STAGE 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We shrink every 1920 x 1080 picture to 640 wide so the APB keeps up. Near the ground
#   that is right: the floor moves fast across the picture, so we need many pictures a
#   second and small ones are enough. High up it is the wrong trade:
#     - one of our 640-wide pixels covers h / 291 metres of ground: 3.4 cm at 10 m,
#       12 cm at 35 m, 26 cm at 75 m. Grass and gravel vanish and the picture LOOKS
#       blurry (that was "blurry at the top"). Full resolution is 3x finer.
#     - the ground moves SLOWLY across the picture up there (1 m/s drift at 50 m is
#       ~17 full-resolution pixels a second), so fewer pictures a second are fine.
#   So, as the mentors said, we "play with the latency": high up we accept a few more
#   milliseconds per picture and use a FULL-RESOLUTION crop from the middle of the
#   picture instead of the shrunk whole picture. The middle is also the part of the lens
#   we calibrated (the edges bend and are not calibrated yet), so this helps twice.
#
#   The choice is made ONCE, when the engage switch saves home (from the barometer
#   height), and never in the middle of a hold: home's landmarks belong to one picture
#   size. Off by default (HIGH_ALT_CROP_FROM_M = None) until it has been flown.
#   Needs a lens file (calib.npz): without a measured focal length a crop's metres
#   would be wrong, so it then stays on the small picture.
#
# HOW IT WORKS (technical)
#   small: downscale() to PROCESS_WIDTH, K scaled by width ratio (as before).
#   crop:  full-resolution window of HIGH_ALT_CROP_SIZE centred on the principal point
#          (cx, cy) and clamped inside the picture. K unchanged except cx -= x0, cy -= y0.
#          Distortion coefficients are unchanged (they act on normalised coordinates).
#   Rolling-shutter row times use row / rows of the processed picture; for a 960-row crop
#   of 1080 rows that is off by <= 6 % of the readout (< 1 ms): negligible.
#   Recording keeps the 640-wide copy (one size per video file), so a replay of a crop
#   flight runs on the small picture.
#
# INTERFACE
#   mode = ProcessMode();  mode.choose(height_m, have_K) -> True if the mode changed
#   mode.image(full_gray) -> picture to track;  mode.camera_matrix(K_full, small_w) -> K
#   mode.name = "small" | "crop".   Proof: tests/test_cameras.py
# =====================================================================================
import numpy as np

import config
from vision.enhance import downscale, scale_camera_matrix


class ProcessMode:
    def __init__(self):
        self.name = "small"
        self.offset = (0, 0)              # crop's top-left corner in the full picture
        self._centre = None               # principal point (cx, cy) in full-resolution pixels

    @property
    def enabled(self):
        return getattr(config, "HIGH_ALT_CROP_FROM_M", None) is not None

    def set_principal_point(self, K_full):
        self._centre = None if K_full is None else (float(K_full[0, 2]), float(K_full[1, 2]))

    def choose(self, height_m, have_K=True):
        """Call when home is about to be saved. Returns True if the picture size changes."""
        want = "small"
        if self.enabled and have_K and height_m is not None and height_m >= config.HIGH_ALT_CROP_FROM_M:
            want = "crop"
        changed = want != self.name
        self.name = want
        return changed

    def _window(self, w, h):
        cw, ch = min(config.HIGH_ALT_CROP_SIZE[0], w), min(config.HIGH_ALT_CROP_SIZE[1], h)
        cx, cy = self._centre if self._centre else ((w - 1) / 2, (h - 1) / 2)
        x0 = int(round(min(max(cx - cw / 2, 0), w - cw)))
        y0 = int(round(min(max(cy - ch / 2, 0), h - ch)))
        return x0, y0, cw, ch

    def image(self, full_gray):
        if self.name == "small":
            self.offset = (0, 0)
            return downscale(full_gray)
        h, w = full_gray.shape[:2]
        x0, y0, cw, ch = self._window(w, h)
        self.offset = (x0, y0)
        return np.ascontiguousarray(full_gray[y0:y0 + ch, x0:x0 + cw])

    def camera_matrix(self, K_full, full_w, processed_w):
        """K for the processed picture. K_full = lens matrix at the camera's full width full_w."""
        if K_full is None:
            return None
        if self.name == "small":
            return scale_camera_matrix(K_full, processed_w / full_w)
        K = K_full.copy()
        K[0, 2] -= self.offset[0]
        K[1, 2] -= self.offset[1]
        return K
