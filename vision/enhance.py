# =====================================================================================
# vision/enhance.py  -  boost local contrast for high / hazy / flat-light views   BRIEF 1 + 3
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
#
# HOW IT WORKS (technical)
#   CLAHE (Contrast Limited Adaptive Histogram Equalisation): per 8x8 tile histogram
#   equalisation with a clip limit (2.0) that stops noise in flat areas being amplified
#   too much, bilinear blending between tiles. ~1-2 ms at 640x480.
#   Skipped when the mean grey level is outside health.py's DARK/BRIGHT limits.
#
# WHERE IT PLUGS IN
#   vision_hold.py right after the colour -> grey conversion (feeds tracker + health).
#   Switch: config.USE_CLAHE. health.py is unchanged.
#   Recommendation for Saumya (not applied): health.BLUR_BAD is absolute; a threshold
#   relative to the home frame's sharpness would remove the root cause.
#
# REFERENCES
#   - Zuiderveld, "Contrast Limited Adaptive Histogram Equalization", Graphics Gems IV,
#     Academic Press 1994.
#   - Pizer et al., "Adaptive Histogram Equalization and Its Variations", CVGIP 1987.
#   - OpenCV CLAHE: https://docs.opencv.org/4.x/d5/daf/tutorial_py_histogram_equalization.html
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
