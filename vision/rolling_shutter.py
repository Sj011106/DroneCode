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


def correct_points(pts_raw, pts_und, t_capture, rows, history, timing, K):
    """Move undistorted points to where they'd be if the whole frame were captured at
    its middle-row time. pts_raw gives each point's ROW (raw image); pts_und are the
    same points after lens undistortion (pixel units, P = K)."""
    if timing.readout_s <= 0 or len(pts_und) == 0:
        return pts_und
    t_rows = timing.row_time(t_capture, pts_raw[:, 1], rows)
    att_rows = attitude_at(history, t_rows)
    att_mid = attitude_at(history, timing.mid_time(t_capture))
    if att_rows is None:
        return pts_und                                # no IMU data yet: nothing to correct with
    R_rows = body_to_ned(*att_rows.T) @ R_BODY_CAM    # (N,3,3) camera -> NED at each row time
    R_mid = body_to_ned(*att_mid[0]) @ R_BODY_CAM     # (3,3)   camera -> NED at mid time
    rays = np.c_[pts_und, np.ones(len(pts_und))] @ np.linalg.inv(K).T
    rays_ned = np.einsum("nij,nj->ni", R_rows, rays)
    p = (rays_ned @ R_mid) @ K.T                      # (R_mid^T * ray) for every row, then K
    return (p[:, :2] / p[:, 2:3]).astype(np.float32)


class RollingShutterTracker(GroundTracker):
    """Saumya's GroundTracker + per-row rolling-shutter correction of tracked points."""

    def __init__(self, K=None, dist=None, attitude_history=None, timing=None,
                 enabled=config.USE_ROLLING_SHUTTER):
        super().__init__(K, dist)
        self.history = attitude_history if attitude_history is not None else []
        self.timing = timing or load_timing()
        self.enabled = enabled
        self._frame = None           # (t_capture, rows) of the frame being processed

    def set_reference(self, gray, t_capture=None):
        """Same as GroundTracker.set_reference; pass t_capture to correct the home image too."""
        self._frame = None if t_capture is None else (t_capture, gray.shape[0])
        return super().set_reference(gray)

    def track(self, gray, t_capture=None, attitude=None):
        self._frame = None if t_capture is None else (t_capture, gray.shape[0])
        return super().track(gray, t_capture, attitude)

    def _undistort(self, pts):
        und = super()._undistort(pts)
        if not self.enabled or self._frame is None or self.K is None:
            return und
        t_capture, rows = self._frame
        return correct_points(np.asarray(pts).reshape(-1, 2), und, t_capture, rows,
                              self.history, self.timing, self.K)
