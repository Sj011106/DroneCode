# =====================================================================================
# fusion/imu_check.py  -  does vision agree with the IMU?     BRIEF ITEMS 1 + 3
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Two independent witnesses: the camera sees how much the ground has turned since
#   home, the autopilot's IMU/compass knows how much the drone has turned. If they
#   disagree by more than a few degrees, the camera is being fooled (a moving shadow,
#   water, a repeating pattern, a bad match) and we must not trust its position.
#   This catches failures that look "healthy" to the image-only checks in health.py.
#
# HOW IT WORKS (technical)
#   Nadir camera, top of image = nose: when the drone yaws clockwise by psi, the ground
#   turns counter-clockwise in the image, so the tracker's rotation_deg ~ -(yaw - yaw_home)
#   (sign verified in tests/test_tracker.py: yaw +6.8 deg -> rot -6.8 deg).
#   error = wrap(rotation_deg + (yaw - yaw_home)); fail if |error| > IMU_CHECK_MAX_DEG.
#
# WHERE IT PLUGS IN
#   vision_hold.py, right after health.check(). A disagreement sets quality = -1 for that
#   frame and adds a reason to the log. health.py itself is unchanged.
#   Proof: tests/test_health.py.
#
# REFERENCES
#   - Groves, "Principles of GNSS, Inertial, and Multisensor Integrated Navigation
#     Systems", 2nd ed., Artech House 2013, ch. 17 (fault detection by consistency checks
#     between independent sensors).
#   - Kelly & Sukhatme, "Visual-Inertial Sensor Fusion: Localization, Mapping and
#     Sensor-to-Sensor Self-calibration", IJRR 2011.
# =====================================================================================

import math

import config


def wrap_deg(a):
    """Angle into -180..180 degrees."""
    return (a + 180.0) % 360.0 - 180.0


class ImuCheck:
    def __init__(self, max_deg=None):
        self.max_deg = config.IMU_CHECK_MAX_DEG if max_deg is None else max_deg   # read now: --set works
        self.home_yaw = None

    def set_home(self, yaw_rad):
        """Call when HOME is saved (yaw at that moment, radians). None = no IMU data yet:
        the home yaw is then taken from the first frame that has IMU data (see check)."""
        self.home_yaw = yaw_rad

    def check(self, rotation_deg, yaw_rad):
        """Returns (agree, error_deg). Agrees if there's nothing to compare against."""
        if yaw_rad is None:
            return True, 0.0
        if self.home_yaw is None:                     # home saved without IMU data: start the
            self.home_yaw = yaw_rad + math.radians(rotation_deg)   # check from this frame
            return True, 0.0
        err = wrap_deg(rotation_deg + math.degrees(yaw_rad - self.home_yaw))
        return abs(err) <= self.max_deg, err
