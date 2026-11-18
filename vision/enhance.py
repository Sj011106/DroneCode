# =====================================================================================
# vision/enhance.py  -  prepare each picture: downscale + local contrast boost   BRIEF 1 + 2 + 3
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   From high up, the small sharp details of the ground (pebbles, grass blades) blend
#   together inside each pixel, and in flat light or haze the picture turns grey and
#   soft. The tracker still follows the ground fine, but health.py's sharpness check
#   uses a FIXED threshold and starts calling every frame "blurry", so the drone stops
#   trusting vision for no real reason. Measured in tests/test_altitude.py: low contrast
#   at 40 m and 80 m -> 0% of frames trusted, tracking error still only ~0.6 px.
#   Fix: stretch the contrast locally (each small tile of the picture separately)
#   before tracking and health checks. Same picture, clearer details. Very dark or very
#   bright frames are left untouched, so the "too dark / too bright" alarms still work.
#   Before that, the picture is made SMALLER (1280x720 -> 640x360 by default): the APB's
#   small ARM cores can't track 1280x720 at 30 fps, and halving the width quarters the work.
#   The camera's lens numbers (K) are scaled the same way, so metres come out unchanged.
#
# HOW IT WORKS (technical)
#   CLAHE (Contrast Limited Adaptive Histogram Equalisation): per 8x8 tile histogram
#   equalisation with a clip limit (2.0) that stops noise in flat areas being amplified
#   too much, bilinear blending between tiles. ~1-2 ms at 640x480.
#   Skipped when the mean grey level is outside health.py's DARK/BRIGHT limits.
#   Downscale: cv2.INTER_AREA to config.PROCESS_WIDTH. Scaling an image by s scales the
#   pinhole intrinsics fx, fy, cx, cy by s; lens distortion coefficients are defined on
#   normalised coordinates, so they stay the same. Rolling-shutter row times use the row
#   FRACTION (row / rows), so they are unaffected too. Proof: tests/test_process_scale.py.
#
# WHERE IT PLUGS IN
#   vision_hold.py right after the colour -> grey conversion (feeds tracker + health);
#   vision_hold scales K once from the first frame. Switches: config.USE_CLAHE,
#   config.PROCESS_WIDTH (None = full resolution). health.py is unchanged.
#
# REFERENCES
#   - Zuiderveld, "Contrast Limited Adaptive Histogram Equalization", Graphics Gems IV,
#     Academic Press 1994.
#   - Pizer et al., "Adaptive Histogram Equalization and Its Variations", CVGIP 1987.
#   - OpenCV CLAHE: https://docs.opencv.org/4.x/d5/daf/tutorial_py_histogram_equalization.html
#   - Hartley & Zisserman, "Multiple View Geometry", 2nd ed., sec. 6.1 (intrinsics scale with
#     the image); OpenCV camera model: https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html
# =====================================================================================

import cv2

import config
from health.health import BRIGHT_LIMIT, DARK_LIMIT

_clahe = {}


def enhance(gray):
    """Local contrast boost (if switched on and the frame isn't too dark/bright)."""
    if not config.USE_CLAHE or not DARK_LIMIT <= gray.mean() <= BRIGHT_LIMIT:
        return gray
    key = (config.CLAHE_CLIP, config.CLAHE_TILES)
    if key not in _clahe:
        _clahe[key] = cv2.createCLAHE(clipLimit=config.CLAHE_CLIP, tileGridSize=(config.CLAHE_TILES,) * 2)
    return _clahe[key].apply(gray)


def downscale(gray, width=None):
    """Shrink to config.PROCESS_WIDTH (keeps aspect ratio). No-op if already that small."""
    width = config.PROCESS_WIDTH if width is None else width
    h, w = gray.shape[:2]
    if not width or w <= width:
        return gray
    return cv2.resize(gray, (width, round(h * width / w)), interpolation=cv2.INTER_AREA)


def scale_camera_matrix(K, s):
    """Camera matrix for an image scaled by s (fx, fy, cx, cy scale; distortion doesn't)."""
    if K is None or s == 1.0:
        return K
    K = K.copy()
    K[0, 0] *= s
    K[1, 1] *= s
    K[:2, 2] = (K[:2, 2] + 0.5) * s - 0.5          # pixel centres: exact for cv2.resize
    return K
