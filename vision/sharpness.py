# =====================================================================================
# vision/sharpness.py  -  never save home from a blurry picture                 STAGE 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   On the hardware day home was once saved from a dark warm-up picture and nothing could
#   be tracked against it. A blurry picture is the same trap: its corners are smeared, so
#   home holds few, weak landmarks. Blur on this aircraft comes mostly from TURNING while
#   a picture is taken (the Justice D has no exposure control), which is the same at
#   every height.
#   This file gives every picture a sharpness number and compares it with the last
#   second or so of pictures:
#     - home: when home is due, wait (a few pictures at most) for one that is at least
#       HOME_SHARP_RATIO as sharp as usual. Never waits longer than
#       HOME_SHARP_MAX_WAIT_FRAMES, so "ready" is late by a fraction of a second at worst.
#     - measuring (OFF until tools/blur_check.py shows motion blur): skip a picture
#       that is much blurrier than usual, at most BLUR_SKIP_MAX_IN_A_ROW in a row
#       (same as a dropped frame: the Betaflight sender keeps the last stick request).
#
# HOW IT WORKS (technical)
#   Sharpness = variance of the Laplacian (second derivative): edges make it large, blur
#   flattens them. It depends on the scene, so it is only compared with the SAME scene a
#   moment ago: ratio = score / median(last WINDOW scores). The median ignores one-off
#   spikes. Cost at 640 x 360: about 1-2 ms on the APB.
#
# INTERFACE
#   sharpness(gray) -> float
#   gate = SharpnessGate(); gate.judge(score)  once per picture (stores last_ratio)
#   gate.home_ok(waited_frames) -> bool;  gate.skip() -> bool;  gate.reset()
#   Proof: tests/test_cameras.py
#
# REFERENCES
#   - Pertuz, Puig, Garcia, "Analysis of focus measure operators for shape-from-focus",
#     Pattern Recognition 46(5), 2013.
# =====================================================================================
from collections import deque

import cv2
import numpy as np

import config


def sharpness(gray):
    """Variance of the Laplacian of a grey picture. Bigger = sharper (for the same scene)."""
    # STAGE 2 (integration): 16-bit + meanStdDev = the same number, 10x cheaper (laptop: 1.3 ms -> 0.12 ms)
    return float(cv2.meanStdDev(cv2.Laplacian(np.ascontiguousarray(gray), cv2.CV_16S))[1][0, 0] ** 2)


class SharpnessGate:
    def __init__(self, window=30, min_history=5):
        self.scores = deque(maxlen=window)
        self.min_history = min_history
        self.last_ratio = None            # this picture's sharpness / usual sharpness (None = no history yet)
        self.skipped_in_a_row = 0
        self.skipped = 0                  # total, for the log

    def reset(self):
        """New scene scale (e.g. the picture size changed): forget the history."""
        self.scores.clear()
        self.last_ratio = None
        self.skipped_in_a_row = 0

    def judge(self, score):
        if len(self.scores) >= self.min_history:
            usual = float(np.median(self.scores))
            self.last_ratio = score / usual if usual > 0 else None
        else:
            self.last_ratio = None
        self.scores.append(score)
        return self.last_ratio

    def home_ok(self, waited_frames):
        """True = save home from this picture now."""
        ratio = getattr(config, "HOME_SHARP_RATIO", None)
        if not getattr(config, "HOME_SHARPEST", False) or ratio is None or self.last_ratio is None:
            return True
        return self.last_ratio >= ratio or waited_frames >= config.HOME_SHARP_MAX_WAIT_FRAMES

    def skip(self):
        """True = do not measure from this picture (much blurrier than usual)."""
        ratio = getattr(config, "BLUR_SKIP_RATIO", None)
        if ratio is None or self.last_ratio is None or self.last_ratio >= ratio \
                or self.skipped_in_a_row >= config.BLUR_SKIP_MAX_IN_A_ROW:
            self.skipped_in_a_row = 0
            return False
        self.skipped_in_a_row += 1
        self.skipped += 1
        return True
