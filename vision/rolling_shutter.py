# =====================================================================================
# vision/rolling_shutter.py  -  undo the rolling-shutter warp      BRIEF ITEM 1
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The Justice D reads its picture one row at a time (see vision/timing.py). If the
#   drone rotates or vibrates during that scan, the top of the picture is taken from a
#   slightly different angle than the bottom, so straight lines lean and the ground
#   looks stretched ("jello"). The tracker then measures motion that isn't real.
#   The fix: the IMU tells us exactly how the drone was rotating during the scan. For
#   every tracked point we look up the angle at the moment ITS row was captured, and
#   move the point to where it would have been if the whole picture had been taken
#   at one instant (the middle row's time). After that, the image is "global shutter"
#   as far as the tracker's maths is concerned.
#
# HOW IT WORKS (technical)
#   For a point at undistorted pixel p, captured at row time t_r:
#     ray_cam  = K^-1 [p, 1]
#     ray_ned  = R_ned_cam(t_r) * ray_cam                 (where the ray pointed in the world)
#     ray_mid  = R_ned_cam(t_mid)^T * ray_ned             (same ray seen from the mid-time pose)
#     p'       = project(K * ray_mid)
#   i.e. the rotation homography H = K R_rel K^-1 per row. R_ned_cam(t) = R_ned_body(t)
#   * R_body_cam, with roll/pitch/yaw interpolated from the autopilot's ATTITUDE stream
#   (50 Hz). Only ROTATION is corrected: translation during a ~25 ms readout is tiny
#   at altitude (1 m/s at 20 m height ~ 0.5 px), while rotation is not.
#   Points are corrected on BOTH the home image and every new frame, so both are
#   brought to their own mid-row time before RANSAC compares them.
#
# LEVELLING (STAGE 2 fix, config.USE_LEVEL_POINTS)
#   IN PLAIN WORDS: the Justice D sees 114 degrees. When the drone leans, the far side of
#   the picture stretches and the near side squeezes. The tracker only allows "slide,
#   turn, zoom" between home and now, so from about 3 degrees of lean most points stop
#   agreeing and health.py says LOST, although the position is still right (measured:
#   tests/test_level.py). Fix: we know the lean from the IMU, so we move every point to
#   where a camera pointing STRAIGHT DOWN would have seen it. Then "slide, turn, zoom" is
#   exactly true again for flat ground, whatever the lean.
#   TECHNICAL: same ray maths as above, but the target frame is the mid-time attitude with
#   roll = pitch = 0 (yaw kept): p' = K R_level^T R(t_row) K^-1 p. Done for home and for
#   each frame. The image centre is then the point straight below the drone, so the
#   tracker's shift needs no tilt term afterwards: vision_hold.py skips fusion/tilt.py's
#   first-order cancel for a side that was levelled (ref_levelled / now_levelled).
#   Not levelled (no IMU data): behaves as before.
#
# WHERE IT PLUGS IN
#   RollingShutterTracker is Saumya's GroundTracker with ONE extra step: her tracker
#   calls _undistort() on points right before RANSAC; we extend that step. Corner
#   detection, Lucas-Kanade, forward-backward check, RANSAC and health are unchanged.
#   With USE_ROLLING_SHUTTER = False it behaves exactly like GroundTracker.
#   Proof: tests/test_rolling_shutter.py (rendered rolling-shutter frames, error
#   with and without correction).
#
# REFERENCES
#   - Forssen & Ringaby, "Rectifying Rolling Shutter Video from Hand-held Devices",
#     CVPR 2010.
#   - Karpenko, Jacobs, Baek, Levoy, "Digital Video Stabilization and Rolling Shutter
#     Correction using Gyroscopes", Stanford CSTR 2011-03 (gyro-driven per-row warp).
#   - Li & Mourikis, "Real-time Motion Tracking on a Cellphone using Inertial Sensing
#     and a Rolling-Shutter Camera", ICRA 2013 (rolling shutter inside VIO).
#   - Hartley & Zisserman, "Multiple View Geometry", 2nd ed., sec. 8.4 (H = K R K^-1).
# =====================================================================================

import numpy as np

import config
from fusion.tilt import R_BODY_CAM, body_to_ned
from vision.timing import load_timing
from vision.tracker import GroundTracker


def attitude_at(history, t):
    """Roll, pitch, yaw (radians) at time(s) t, linearly interpolated from a list of
    (time, roll, pitch, yaw) samples. A little extrapolation is allowed past the newest
    sample (the IMU message for the last rows may not have arrived yet).
    Returns an (N, 3) array, or None if there are fewer than 2 samples."""
    h = np.asarray(list(history), dtype=float)       # list() = snapshot of a live deque
    if len(h) < 2:
        return None
    ts, ang = h[:, 0], h[:, 1:4].copy()
    ang[:, 2] = np.unwrap(ang[:, 2])                  # yaw: no jump at +-180 degrees
    t = np.atleast_1d(np.asarray(t, dtype=float))
    i = np.clip(np.searchsorted(ts, t), 1, len(ts) - 1)
    w = (t - ts[i - 1]) / np.maximum(ts[i] - ts[i - 1], 1e-6)
    w = np.clip(w, -1.0, 2.0)                         # at most one sample gap of extrapolation
    return ang[i - 1] + (ang[i] - ang[i - 1]) * w[:, None]


def attitude_near(history, t, max_age=None):
    """Nearest (roll, pitch, yaw) sample to time t, or None if no sample is within max_age
    seconds (IMU data missing or stale). Thread-safe: takes a snapshot first, because the
    MAVLink reader thread appends to the live deque (iterating it directly can raise
    "deque mutated during iteration" and crash the main loop)."""
    max_age = config.ATTITUDE_MAX_AGE_S if max_age is None else max_age
    snap = list(history)                              # C-level copy: atomic under the GIL
    if not snap:
        return None
    s = min(snap, key=lambda a: abs(a[0] - t))
    return tuple(s[1:4]) if abs(s[0] - t) <= max_age else None


def correct_points(pts_raw, pts_und, t_capture, rows, history, timing, K, rolling=True, level=False):
    """Move undistorted points to where they'd be if the whole frame were captured at
    its middle-row time. pts_raw gives each point's ROW (raw image); pts_und are the
    same points after lens undistortion (pixel units, P = K).
    rolling=False: skip the per-row part. level=True (STAGE 2): the target is a camera looking
    straight down (roll = pitch = 0 at the middle-row time), see LEVELLING in the header.
    Returns the points unchanged when there is nothing to do or no IMU data."""
    rolling = rolling and timing.readout_s > 0
    if len(pts_und) == 0 or not (rolling or level):
        return pts_und
    att_mid = attitude_at(history, timing.mid_time(t_capture))
    if att_mid is None:
        return pts_und                                # no IMU data yet: nothing to correct with
    if rolling:
        att_rows = attitude_at(history, timing.row_time(t_capture, pts_raw[:, 1], rows))
    else:
        att_rows = np.repeat(att_mid, len(pts_und), axis=0)
    roll, pitch, yaw = att_mid[0]
    if level:
        roll = pitch = 0.0
    R_rows = body_to_ned(*att_rows.T) @ R_BODY_CAM    # (N,3,3) camera -> NED at each row time
    R_mid = body_to_ned(roll, pitch, yaw) @ R_BODY_CAM   # (3,3) camera -> NED at mid time (levelled or not)
    rays = np.c_[pts_und, np.ones(len(pts_und))] @ np.linalg.inv(K).T
    rays_ned = np.einsum("nij,nj->ni", R_rows, rays)
    p = (rays_ned @ R_mid) @ K.T                      # (R_mid^T * ray) for every row, then K
    return (p[:, :2] / p[:, 2:3]).astype(np.float32)


class RollingShutterTracker(GroundTracker):
    """Saumya's GroundTracker + per-row rolling-shutter correction of tracked points."""

    def __init__(self, K=None, dist=None, attitude_history=None, timing=None,
                 enabled=config.USE_ROLLING_SHUTTER, level=False):
        super().__init__(K, dist)
        self.level = level           # STAGE 2: level points with the IMU (see LEVELLING in the header)
        self.ref_levelled = self.now_levelled = self._levelled = False   # did it happen for home / this frame?
        self.history = attitude_history if attitude_history is not None else []
        self.timing = timing or load_timing()
        self.enabled = enabled
        self._frame = None           # (t_capture, rows) of the frame being processed

    def set_reference(self, gray, t_capture=None):
        """Same as GroundTracker.set_reference; pass t_capture to correct the home image too."""
        self._frame = None if t_capture is None else (t_capture, gray.shape[0])
        self._levelled = False
        n = super().set_reference(gray)
        self.ref_levelled = self._levelled
        return n

    def track(self, gray, t_capture=None, attitude=None):
        self._frame = None if t_capture is None else (t_capture, gray.shape[0])
        self._levelled = False
        res = super().track(gray, t_capture, attitude)
        self.now_levelled = self._levelled
        return res

    def _undistort(self, pts):
        und = super()._undistort(pts)
        if not (self.enabled or self.level) or self._frame is None or self.K is None or len(und) == 0:
            return und
        t_capture, rows = self._frame
        out = correct_points(np.asarray(pts).reshape(-1, 2), und, t_capture, rows,
                             self.history, self.timing, self.K, rolling=self.enabled, level=self.level)
        self._levelled = self.level and out is not und       # 'und' itself comes back when there is no IMU data
        return out
