# =====================================================================================
# tests/common.py  -  one fixed test camera for every simulated test
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The tests used to read the repo's calib.npz / timing.npz. Those files will be
#   REPLACED by the real Justice D calibration on hardware day, which would silently
#   change every test number (and maybe break asserts). So the simulated tests use this
#   fixed, distortion-free 640x480 test camera instead (focal 416 px: at 4.16 m height one
#   pixel = 1 cm). vision_hold-based tests point config.CALIB_FILE / TIMING_FILE at the
#   files written by use_test_camera().
# =====================================================================================

import os
import tempfile

import numpy as np

import config

K = np.array([[416.42516071, 0.0, 321.65867311],      # same numbers the tests were written with
              [0.0, 416.12309861, 236.83140405],
              [0.0, 0.0, 1.0]])


def use_test_camera():
    """Point config at a test lens file (no distortion) and a missing timing file (defaults)."""
    folder = tempfile.mkdtemp(prefix="dronecode_testcam_")
    path = os.path.join(folder, "calib_test.npz")
    np.savez(path, K=K, dist=np.zeros((1, 5)))
    config.CALIB_FILE = path
    config.TIMING_FILE = os.path.join(folder, "no_timing.npz")   # missing -> config defaults
    return path
