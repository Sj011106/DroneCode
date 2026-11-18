"""Pilot handover logic for Part D. Pure decisions, no MAVLink.

The pilot always wins: if the flight mode is not a hold mode, this
module never asks for anything.
"""

MANUAL, HOLDING, DEGRADED, FAILSAFE = "MANUAL", "HOLDING", "DEGRADED", "FAILSAFE"
REANCHOR = "REANCHOR"


class Handover:
    def __init__(self, hold_modes=("LOITER", "POSHOLD"),
                 failsafe_after_s=3.0, failsafe_mode="LAND", good_quality=50,
                 resend_s=1.0, max_sends=5):
        self.hold_modes = set(hold_modes)
        self.failsafe_after_s = failsafe_after_s
        self.failsafe_mode = failsafe_mode      # "NONE" disables it
        self.good_quality = good_quality
        self.state = MANUAL
        self.lost_since = None                  # time vision was first lost
        self.resend_s = resend_s                # resend if the mode didn't change
        self.max_sends = max_sends
        self.sent_at, self.sends = None, 0

    @property
    def engaged(self):
        return self.state != MANUAL

    def update(self, now, flight_mode, quality):
        """now: time.monotonic(). Returns a list of actions:
        "REANCHOR" or ("SET_MODE", mode). Empty list = do nothing."""
        in_hold = flight_mode in self.hold_modes

        # Rule 1: pilot is not in a hold mode -> the pilot wins.
        if not in_hold:
            self.state = MANUAL
            self.lost_since = None
            return []

        # Rule 2: we just entered a hold mode.
        if self.state == MANUAL:
            self.state = HOLDING
            self.lost_since = None
            return [REANCHOR]

        # Failsafe sent but still in a hold mode: resend a few times.
        if self.state == FAILSAFE:
            if self.sends < self.max_sends and now - self.sent_at >= self.resend_s:
                return self._send(now)
            return []

        # Rule 3: vision is OK.
        if quality > 0:
            self.lost_since = None
            self.state = HOLDING if quality >= self.good_quality else DEGRADED
            return []

        # Rule 4: vision is lost while holding.
        self.state = DEGRADED
        if self.lost_since is None:
            self.lost_since = now
        if (now - self.lost_since >= self.failsafe_after_s
                and self.failsafe_mode != "NONE"):
            self.state = FAILSAFE
            self.sends = 0
            return self._send(now)
        return []

    def _send(self, now):
        self.sent_at = now
        self.sends += 1
        return [("SET_MODE", self.failsafe_mode)]
