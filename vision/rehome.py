# =====================================================================================
# vision/rehome.py  -  keep holding while the drone CLIMBS            BRIEF ITEMS 1 + 3
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The tracker compares every picture with the HOME picture. If the drone climbs, the
#   ground in the picture shrinks (twice as high = half the size). Saumya's health check
#   rightly says "the ground suddenly looks 25% smaller - something is wrong" and stops
#   trusting vision. That's correct for a glitch, but it means we could never climb
#   after saving home, and the trial is won at the HIGHEST altitude.
#   Fix: when the ground has shrunk (or grown) by ~15% and tracking is still healthy,
#   quietly take a NEW home picture right here, and remember how far this new spot is
#   from the ORIGINAL home (in metres North/East). Positions sent to the autopilot keep
#   counting from the original home, so nothing jumps. Like a climber moving the rope's
#   anchor up the wall: the summit is still measured from the ground.
#
# HOW IT WORKS (technical)
#   - Trigger: status OK, valid position, and res.scale outside [REHOME_SCALE_MIN,
#     REHOME_SCALE_MAX] (default 0.85..1.18, well inside health.py's 0.75..1.33 gate),
#     OR (Stage 2 bench) the picture has slid more than REHOME_SHIFT_FRAC of its width
#     or height: low over the floor a 1.5 m carry takes the first picture out of view.
#   - to_origin(): vision_hold found the FIRST home picture again and measures against it
#     (config.USE_ORIGIN_RETURN): offsets go back to 0, the collected drift is gone.
#   - Re-reference: tracker.set_reference(current frame); if too few corners, the old
#     reference is restored (tracker state snapshot) and we try again next frame.
#   - Continuity: offset_ne += position at the switch (already tilt-corrected, in metres,
#     yaw-independent), so position_out = offset_ne + position_vs_new_reference.
#     No EKF reset_counter bump: the position doesn't jump.
#   - Health: its "teleport" check remembers the last trusted shift (old reference);
#     we set it to (0, 0) at the switch time, since we're now at the new reference's
#     centre. health.py's logic is unchanged and no LOST gap occurs.
#   - Cost: each switch freezes in the error of that moment (like visual-odometry
#     keyframes), so error can grow ~ one hold error per switch. Fewer switches = better.
#
# WHERE IT PLUGS IN
#   vision_hold.py after the position is computed; vision_hold also moves home_att
#   (tilt), the IMU-check home yaw and the recovery fingerprints to the new reference.
#   Switch: config.USE_REHOME. Proof: tests/test_rehome.py (climb 10 m -> 40 m) and
#   tests/test_bench.py (1.5 m carry and 3 m loop at 1 m).
#
# REFERENCES
#   - Klein & Murray, "Parallel Tracking and Mapping for Small AR Workspaces", ISMAR 2007
#     (keyframe insertion when the view has changed too much).
#   - Nister, Naroditsky, Bergen, "Visual Odometry", CVPR 2004 (chaining relative motion;
#     error accumulates per link).
#   - Scaramuzza & Fraundorfer, "Visual Odometry: Part I", IEEE RAM 2011 (keyframe selection).
# =====================================================================================

import config


class Rehome:
    def __init__(self, scale_min=None, scale_max=None):
        self.scale_min = config.REHOME_SCALE_MIN if scale_min is None else scale_min
        self.scale_max = config.REHOME_SCALE_MAX if scale_max is None else scale_max
        self.shift_frac = config.REHOME_SHIFT_FRAC
        self.peak = 0                     # BETAFLIGHT CHANGE: most agreeing points seen since this reference
        self.reset()

    def reset(self):
        """Call when a brand-new HOME is saved."""
        self.offset_n = self.offset_e = 0.0
        self.count = 0
        self.peak = 0

    def total(self, north, east):
        """Position vs the ORIGINAL home."""
        return north + self.offset_n, east + self.offset_e

    def to_origin(self):
        """The reference is the ORIGINAL home picture again (the count of switches is kept)."""
        self.offset_n = self.offset_e = 0.0
        self.peak = 0

    def due(self, status, res, size=None, inlier_drop=False):
        """size = picture (width, height) in pixels (None = only the scale trigger, as before).
        inlier_drop (BETAFLIGHT CHANGE, bench / not holding): also when the agreeing points fell below
        REHOME_INLIER_FRAC of the most seen on this reference (the floor is sliding out of view, or under the
        holder's legs). Never during an engaged hold: each switch freezes in a little error."""
        if status != "OK" or not res.ok:
            return False
        self.peak = max(self.peak, res.n_inliers)
        frac = getattr(config, "REHOME_INLIER_FRAC", 0)
        if inlier_drop and frac and self.peak >= 40 and res.n_inliers < frac * self.peak:
            return True
        if not self.scale_min <= res.scale <= self.scale_max:
            return True
        if not size or not self.shift_frac:
            return False
        return (abs(res.shift_px[0]) > self.shift_frac * size[0]      # a quarter of the picture gone on
                or abs(res.shift_px[1]) > self.shift_frac * size[1])  # either axis (it is wider than tall)

    def rehome(self, tracker, health, gray, t_capture, north_total, east_total, min_corners):
        """Make the current frame the new reference. Returns True if it worked."""
        saved = dict(tracker.__dict__)                    # set_reference reassigns, never mutates
        if tracker.set_reference(gray, t_capture) < min_corners:
            tracker.__dict__.update(saved)                # not enough texture: keep the old one
            return False
        self.offset_n, self.offset_e = north_total, east_total
        self.peak = 0
        health.prev_shift, health.prev_shift_t = (0.0, 0.0), t_capture
        self.count += 1
        return True
