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
        self.gave_up = False                    # ("GAVE_UP",) is returned once per failsafe

    @property
    def engaged(self):
        return self.state != MANUAL

    def update(self, now, flight_mode, quality, mode_fresh=True):
        """now: the caller's clock (clock() in vision_hold.py). Returns a list of actions:
        "REANCHOR", ("SET_MODE", mode), or ("GAVE_UP",). Empty list = do nothing.
        mode_fresh=False (no recent autopilot heartbeat) never returns SET_MODE."""
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
            if self.sent_at is not None and now - self.sent_at < self.resend_s:
                return []
            if self.sends < self.max_sends:
                return self._send(now, mode_fresh)
            if not self.gave_up:
                self.gave_up = True
                return [("GAVE_UP",)]
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
            self.sends, self.sent_at, self.gave_up = 0, None, False
            return self._send(now, mode_fresh)
        return []

    def _send(self, now, mode_fresh):
        if not mode_fresh:                      # autopilot mode data is stale: wait
            return []
        self.sent_at = now
        self.sends += 1
        return [("SET_MODE", self.failsafe_mode)]
