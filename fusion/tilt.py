# =====================================================================================
# fusion/tilt.py  -  tilt cancellation                       BRIEF ITEM 1: sensor fusion
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The camera is bolted to the drone, looking straight down. To move, a multicopter
#   TILTS. When it tilts, the camera looks a bit sideways, so the ground slides across
#   the picture even though the drone hasn't moved. Without a fix, the tracker reports
#   that slide as drift, and the autopilot "corrects" a drift that never happened.
#   We ask the IMU (roll, pitch) how much the drone is tilted and subtract the fake slide.
#
# HOW IT WORKS (technical)
#   The ground point straight below the drone (the nadir) appears in the image at
#       o = f * (v_x / v_z, v_y / v_z),   v = R_cam_body^T * R_ned_body^T * [0, 0, 1]
#   (pinhole projection of the "down" direction). For small angles: o_x ~ f*tan(roll),
#   o_y ~ f*tan(pitch). The tracker's shift (home -> now) contains the real move PLUS
#   (o_home - o_now), so the real move is
#       d = shift + o_now - R(rotation) * o_home
#   R(rotation) turns the home image's axes into the current image's axes.
#   Ignored (2nd order): foreshortening of the tilted view, slant-range change of height.
#   ponytail: first-order fix. Measured residual (tests/test_tilt.py): ~2 px at 5 deg,
#   ~9 px at 10 deg (grows with tilt squared; in metres it grows with height). Hover tilt
#   is usually < 5 deg. Upgrade path if it matters: "level" the points with the
#   homography K * R_level * K^-1 before RANSAC (same maths as rolling_shutter.py), but
#   then the tracker's frame-to-frame prediction must be leveled too.
#
# WHERE IT PLUGS IN
#   vision_hold.py, between tracker.track() and to_north_east(). It uses the attitude
#   Saumya's tracker already stores (TrackResult.attitude). Tracker logic is untouched.
#   Proof: tests/test_tilt.py (rendered tilted views, error with and without the fix).
#
# REFERENCES
#   - Honegger et al., "An Open Source and Open Hardware Embedded Metric Optical Flow
#     CMOS Camera for Indoor and Outdoor Applications", ICRA 2013 (PX4Flow: gyro
#     compensation of rotation-induced flow).
#   - Hartley & Zisserman, "Multiple View Geometry in Computer Vision", 2nd ed., ch. 6
#     (pinhole camera) and sec. 8.4 (a pure rotation moves image points by H = K R K^-1).
#   - Euler angle convention: ZYX (yaw, pitch, roll), same as ArduPilot's ATTITUDE message.
# =====================================================================================

import math

import numpy as np

# Camera axes written in body axes (Forward, Right, Down):
#   image right  = body right    -> column 0 = (0, 1, 0)
#   image down   = body backward -> column 1 = (-1, 0, 0)  (top of image = nose)
#   optical axis = body down     -> column 2 = (0, 0, 1)
R_BODY_CAM = np.array([[0.0, -1.0, 0.0],
                       [1.0, 0.0, 0.0],
                       [0.0, 0.0, 1.0]])


def body_to_ned(roll, pitch, yaw):
    """Rotation matrix body -> North/East/Down (ZYX Euler, radians).
    Works on single numbers (gives 3x3) or arrays of angles (gives N x 3 x 3)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.stack([np.stack([cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy], -1),
                     np.stack([cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy], -1),
                     np.stack([-sp, sr * cp, cr * cp], -1)], -2)


def nadir_offset_px(roll, pitch, focal_px):
    """Where the ground point straight below the drone appears, in pixels from the
    image centre (+x = right, +y = down). Level drone -> (0, 0). Yaw doesn't matter."""
    down_cam = R_BODY_CAM.T @ body_to_ned(roll, pitch, 0.0).T @ np.array([0.0, 0.0, 1.0])
    return focal_px * down_cam[:2] / down_cam[2]


def cancel_tilt(shift_px, att_now, att_home, focal_px, rotation_deg=0.0):
    """Remove the fake shift caused by tilt.
      shift_px     : tracker's home -> now shift (team convention, current image axes)
      att_now      : (roll, pitch, yaw) radians at this frame
      att_home     : (roll, pitch, yaw) radians when HOME was saved
      rotation_deg : tracker's rotation_deg (home image -> current image)
    Returns the corrected (dx, dy) in the same convention, ready for to_north_east()."""
    o_now = nadir_offset_px(att_now[0], att_now[1], focal_px)
    o_home = nadir_offset_px(att_home[0], att_home[1], focal_px)
    a = math.radians(rotation_deg)
    rot = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    d = np.asarray(shift_px, dtype=float) + o_now - rot @ o_home
    return float(d[0]), float(d[1])
