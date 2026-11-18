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
#   A timer that starts on the first frame where we could NOT send a valid position while
#   the EKF source set is vision (status LOST, but also IMU disagreement or no height while
#   health still says OK, and a dead camera), and resets on the next valid position.
#   After FAILSAFE_AFTER_S it fires ONCE: vision_hold switches the flight mode and resends
#   it (1/s, FAILSAFE_MODE_RETRIES times) until the autopilot's HEARTBEAT shows the new mode.
#   Re-armed after valid positions resume.
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
    def __init__(self, after_s=None, action=None):     # None = config.py (read now: --set works)
        self.after_s = config.FAILSAFE_AFTER_S if after_s is None else after_s
        self.action = config.FAILSAFE_ACTION if action is None else action
        self.lost_since = None
        self.fired = False
        self.pending, self.tries, self.last_try = None, 0, float("-inf")   # mode change in progress

    def update(self, lost, now):
        """lost = no valid position this frame while the autopilot uses vision. Returns the
        flight mode to switch to (once), or None."""
        if not lost:
            self.lost_since, self.fired = None, False
            return None
        if self.lost_since is None:
            self.lost_since = now
        if not self.fired and self.action != "NONE" and now - self.lost_since >= self.after_s:
            self.fired = True
            return self.action
        return None

    def step(self, lost, now, link):
        """update() + make the mode change stick: resend it once a second (at most
        config.FAILSAFE_MODE_RETRIES times) until the autopilot's HEARTBEAT reports it."""
        action = self.update(lost, now)
        if action:
            print(f"FAILSAFE: no valid vision position for {self.after_s:.0f} s while flying on "
                  f"vision: {link.conn.flightmode} -> {action}")
            self.pending, self.tries, self.last_try = action, 0, float("-inf")
        if self.pending is None:
            return
        if link.conn.flightmode == self.pending:
            print(f"FAILSAFE: autopilot confirmed {self.pending}")
            self.pending = None
        elif self.tries >= config.FAILSAFE_MODE_RETRIES:
            print(f"FAILSAFE: autopilot still in {link.conn.flightmode} after {self.tries} tries - "
                  f"PILOT MUST TAKE OVER")
            self.pending = None
        elif now - self.last_try >= 1.0:
            with link.send_lock:
                link.conn.set_mode(self.pending)
            self.tries, self.last_try = self.tries + 1, now
