# =====================================================================================
# vision/frozen.py  -  ignore what moves WITH the camera (bench)          BETAFLIGHT CHANGE
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   On the bench the judge carries the aircraft about 1 m above the floor. The 114 degree
#   lens then also sees the judge's shoes, legs, hands and shadow, and the drone's own
#   legs. All of those move together WITH the camera, so in the picture they stand still,
#   while the real floor slides past. If the floor has little texture, those "still" spots
#   can outvote it and the tracker reports "we did not move" (measured on 21 Nov: legs,
#   a table and bags in view cut the agreeing points to 38% and picked the wrong motion).
#   Rule: while the accelerometer says "we are being carried", the floor MUST slide in the
#   picture. A tracked spot that stays put for FROZEN_WINDOW_S is attached to us: it is
#   dropped for as long as this home picture is used. Standing still: nothing is dropped.
#   Only used while disarmed (bench); in flight there is nothing attached in view.
#
# HOW IT WORKS (technical)
#   Per home-picture point: its pixel position in the current frame (tracker.last_cur) over
#   the last FROZEN_WINDOW_S. moving = BetaflightLink.moving() (raw accelerometer RMS change
#   and gyro rate). If moving and a point's position stayed within FROZEN_STILL_PX ->
#   attached for good. The tracker skips attached points before RANSAC (tracker.exclude).
#   A new home picture (re-home, origin return) starts a fresh list.
#   Prototype simulation: low-texture floor (30 floor points vs 60 on shoes/legs): B1
#   passed 0% without, 100% with (60/60 attached points found, ~1 of 30 floor points lost).
#   Cost: a few numpy operations on ~300 points per frame.
#   Proof: tests/test_frozen.py (through the real tracker).
# =====================================================================================
import numpy as np

import config


class FrozenFilter:
    def __init__(self):
        self.ref = None
        self.attached = None
        self.hist = []                        # (t, points N x 2, valid N)

    def for_tracker(self, tracker):
        """Start a fresh list if the tracker's home picture changed. Returns the 'attached' array."""
        if self.ref is not tracker.ref_pts:
            self.ref = tracker.ref_pts
            n = 0 if tracker.ref_pts is None else len(tracker.ref_pts)
            self.attached = np.zeros(n, bool)
            self.hist = []
        return self.attached

    def update(self, t, cur, valid, moving):
        """cur: N x 2 pixel positions of the home points in this frame; valid: N bools (tracked OK)."""
        if cur is None or self.attached is None or len(cur) != len(self.attached):
            return self.attached
        self.hist.append((t, np.asarray(cur, float).copy(), np.asarray(valid, bool).copy()))
        while self.hist and t - self.hist[0][0] > config.FROZEN_WINDOW_S:
            self.hist.pop(0)
        if not moving or len(self.hist) < 4 or t - self.hist[0][0] < 0.8 * config.FROZEN_WINDOW_S:
            return self.attached
        P = np.stack([h[1] for h in self.hist])
        V = np.all(np.stack([h[2] for h in self.hist]), axis=0)
        span = np.hypot(*(P.max(axis=0) - P.min(axis=0)).T)
        self.attached |= V & (span < config.FROZEN_STILL_PX)
        return self.attached
