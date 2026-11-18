# =====================================================================================
# vision/timing.py  -  when exactly was each image row captured?   BRIEF ITEM 1
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   A rolling-shutter camera doesn't take the whole picture at once. It reads row 0,
#   then row 1, ... down to the last row, like a scanner. So every row has its own
#   capture time. To fix the warp (vision/rolling_shutter.py) we need two numbers:
#     readout_s : how long the scan from top row to bottom row takes
#     offset_s  : when the MIDDLE row was captured, relative to the moment our program
#                 received the frame (usually negative: the picture is older than its
#                 arrival, because of exposure, USB/CSI transfer and buffering)
#   Both are measured once by calibration/calibrate_timing.py and saved in timing.npz.
#
# HOW IT WORKS (technical)
#   t_row(v) = t_capture + offset_s + (v / rows - 0.5) * readout_s
#   v = pixel row (raw, distorted image). The middle row is the frame's reference time.
#   offset_s also absorbs the camera-to-IMU clock offset, because both clocks are our
#   computer's time.time() (DroneLink stamps IMU messages on arrival).
#
# REFERENCES
#   - Furgale, Rehder, Siegwart, "Unified Temporal and Spatial Calibration for
#     Multi-Sensor Systems", IROS 2013 (camera-IMU time offset; the Kalibr toolbox).
#   - Oth, Furgale, Kneip, Siegwart, "Rolling Shutter Camera Calibration", CVPR 2013
#     (estimating line delay / readout time).
# =====================================================================================

import os

import numpy as np

import config


class CameraTiming:
    def __init__(self, readout_s, offset_s, source="defaults"):
        self.readout_s = float(readout_s)
        self.offset_s = float(offset_s)
        self.source = source          # "timing.npz" or "defaults", printed at start-up

    def row_time(self, t_capture, row, rows):
        """Capture time of pixel row(s) `row` (number or numpy array)."""
        return t_capture + self.offset_s + (np.asarray(row) / rows - 0.5) * self.readout_s

    def mid_time(self, t_capture):
        """The frame's reference time: when its middle row was captured."""
        return t_capture + self.offset_s

    def __repr__(self):
        return (f"CameraTiming(readout={self.readout_s * 1000:.1f} ms, "
                f"offset={self.offset_s * 1000:+.1f} ms, from {self.source})")


def load_timing(path=None):
    """timing.npz if calibrated, otherwise the defaults in config.py."""
    path = config.TIMING_FILE if path is None else path
    if os.path.exists(path):
        d = np.load(path)
        return CameraTiming(d["readout_s"], d["offset_s"], source=path)
    return CameraTiming(config.READOUT_S, config.CAM_IMU_OFFSET_S)


def save_timing(readout_s, offset_s, path=None):
    np.savez(config.TIMING_FILE if path is None else path, readout_s=readout_s, offset_s=offset_s)
