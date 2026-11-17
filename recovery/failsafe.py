# =====================================================================================
# recovery/failsafe.py  -  what to do if vision does NOT come back   BRIEF ITEM 3
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   When vision is lost, the autopilot keeps flying on its IMU and barometer for a
#   short while ("dead reckoning"), but that guess drifts more every second. If vision
#   and recovery haven't fixed things within a few seconds, we stop pretending and
#   make the drone do something safe: LAND (default) or ALT_HOLD for the pilot.
#   If a GPS is fitted and Saumya's GPS fallback has already switched over, there's
#   nothing to do: the failsafe only acts while the autopilot is relying on OUR vision.
#
# HOW IT WORKS (technical)
#   A timer that starts when status becomes LOST while the EKF source set is vision, and
#   resets when vision is trusted again. After FAILSAFE_AFTER_S it fires ONCE: vision_hold
#   switches the flight mode (pymavlink set_mode). Re-armed after vision recovers.
#   ArduPilot's own EKF failsafe (FS_EKF_ACTION / FS_EKF_THRESH) remains as the last line
#   of defence; this one is faster and based on what we know about the camera.
#
# WHERE IT PLUGS IN
#   vision_hold.py, after source switching. Saumya's FALLBACK_TO_GPS logic is unchanged.
#   Proof: tests/test_failsafe.py.
#
# REFERENCES
#   - ArduPilot EKF failsafe: https://ardupilot.org/copter/docs/ekf-inav-failsafe.html
#   - ArduPilot flight modes: https://ardupilot.org/copter/docs/flight-modes.html
# =====================================================================================

import config


class Failsafe:
    def __init__(self, after_s=config.FAILSAFE_AFTER_S, action=config.FAILSAFE_ACTION):
        self.after_s, self.action = after_s, action
        self.lost_since = None
        self.fired = False

    def update(self, lost, now):
        """lost = vision LOST while the autopilot uses vision. Returns the flight mode to
        switch to (once), or None."""
        if not lost:
            self.lost_since, self.fired = None, False
            return None
        if self.lost_since is None:
            self.lost_since = now
        if not self.fired and self.action != "NONE" and now - self.lost_since >= self.after_s:
            self.fired = True
            return self.action
        return None
