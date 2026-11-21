# =====================================================================================
# fusion/position_controller.py  -  turn "we drifted" into stick commands (Betaflight)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   On ArduPilot we hand over a position and the autopilot does the flying. Betaflight
#   has no such thing: with the engage switch on it flies whatever stick values we send.
#   So this file is the pilot's thumbs. Three small rules:
#     sideways:  drifted to the right, or moving right  -> lean a little to the left
#                (same for forward/back). Never more than BF_MAX_TILT_DEG.
#     wind:      (BETAFLIGHT CHANGE) a steady push needs a steady lean. The "I" part slowly
#                builds that lean, and it STARTS at the lean the pilot was holding when
#                the switch went on, so the drone keeps fighting the wind from the very
#                first moment instead of being blown off first.
#     height:    lower than where the pilot engaged, or sinking -> a little more throttle;
#                (BETAFLIGHT CHANGE) plus a slow "I" part for the battery getting weaker.
#   Heading is left alone (yaw stick centred): the camera measures in the drone's own
#   left/right/forward/back, so no compass is needed (the aircraft has none).
#
#   BETAFLIGHT CHANGE (the big one): a lean in degrees is turned into a stick value with
#   Betaflight's OWN curve (link/stick_curve.py), not a straight line. With Betaflight's
#   default rates the straight line asked for ~4x too little lean near the centre.
#   Wind simulation (30 s holds at 10/35/75 m, wind 3-7 m/s + gusts, 12 holds each):
#     old (straight line, PD, 8 deg cap):            0% /  0% /  0% qualifying
#     + Betaflight curve:                            25% / 50% / 33%
#     + curve + I term + 15 deg cap (this file):    100% / 100% / 100%, worst 1.1 / 1.6 / 2.3 m
#   (simulation, not flight: tune on the aircraft, start weaker rather than stronger)
#
# HOW IT WORKS (technical)
#   Horizontal: PD + I in the home-heading frame (north = the nose direction when home was
#     saved), lean = -(KP * error + KD * speed) + I, I += -KI * error * dt (only while not
#     saturated: anti-windup; frozen while disarmed), clipped to +-BF_MAX_TILT_DEG, rotated
#     into the drone's axes with the camera heading, then stick = us_from_angle(lean).
#     KP = 6 deg/m ~ 1 m/s^2 per metre; KD = 10 deg/(m/s) ~ critically damped.
#   Vertical: PD + I on barometer height error and smoothed climb rate, added to the throttle
#     the pilot was holding at engage, clipped to +-BF_THROTTLE_AUTHORITY.
#   The SIGNS are what the bench test checks (Stage 2 section 05 step 5: "roll and pitch
#   oppose the move"); BF_ROLL_SIGN / BF_PITCH_SIGN flip them if the bench shows the opposite.
#   Proof: tests/test_betaflight.py, tests/test_bf_hold.py (wind simulation).
#
# REFERENCES
#   - Stage 2 package section 03 ("your code is the position controller and the height
#     controller") and section 05 step 5.
#   - Astrom & Murray, "Feedback Systems", ch. 10-11 (PD/PID control of a double integrator,
#     integrator anti-windup).
#   - Mahony, Kumar, Corke, "Multirotor Aerial Vehicles: Modeling, Estimation, and Control
#     of Quadrotor", IEEE RAM 2012 (small-angle lean -> horizontal acceleration).
# =====================================================================================
import math

import config
from link import stick_curve


def _clip(x, lim):
    return max(-lim, min(lim, x))


def _stick(lean_deg, sign):
    """Lean in degrees (+ = right / forward) -> stick microseconds through Betaflight's own curve."""
    return round(stick_curve.us_from_angle(sign * lean_deg))


def sticks(forward_m, right_m, v_forward, v_right):
    """PD only (no memory). Position error and speed in the drone's own axes (metres, m/s; + = forward /
    right of home) -> (roll_us, pitch_us). Stick convention: roll > 1500 leans right, pitch > 1500 leans
    forward. Kept for the bench and the tests; the flight uses HoldController below."""
    if not all(map(math.isfinite, (forward_m, right_m, v_forward, v_right))):   # REVIEW M3
        return 1500, 1500
    lean_right = _clip(-(config.BF_KP_DEG_PER_M * right_m + config.BF_KD_DEG_PER_MPS * v_right), config.BF_MAX_TILT_DEG)
    lean_fwd = _clip(-(config.BF_KP_DEG_PER_M * forward_m + config.BF_KD_DEG_PER_MPS * v_forward), config.BF_MAX_TILT_DEG)
    return _stick(lean_right, config.BF_ROLL_SIGN), _stick(lean_fwd, config.BF_PITCH_SIGN)


class HoldController:
    """BETAFLIGHT CHANGE: PD + I, in the home-heading frame (north/east as vision_hold.py measures them)."""

    def __init__(self):
        self.i = [0.0, 0.0]            # degrees of lean held by the I term (north, east)
        self.t = None

    def engage(self, lean_fwd_deg=0.0, lean_right_deg=0.0, heading_rad=0.0):
        """The switch just went on. Start the I term at the lean the pilot was holding (from Betaflight's
        attitude: forward = nose down), turned into the home-heading frame."""
        c, s = math.cos(heading_rad), math.sin(heading_rad)
        self.i = [lean_fwd_deg * c - lean_right_deg * s, lean_fwd_deg * s + lean_right_deg * c]
        self.i = [_clip(v, config.BF_MAX_TILT_DEG) for v in self.i]
        self.t = None

    def update(self, t, north, east, v_north, v_east, heading_rad, integrate=True):
        """-> (roll_us, pitch_us). heading_rad = the drone's heading in the same frame as north/east."""
        if not all(map(math.isfinite, (north, east, v_north, v_east, heading_rad))):
            return 1500, 1500
        dt = 0.0 if self.t is None else max(0.0, min(0.2, t - self.t))
        self.t = t
        lean = []
        for k, (p, v) in enumerate(((north, v_north), (east, v_east))):
            raw = -(config.BF_KP_DEG_PER_M * p + config.BF_KD_DEG_PER_MPS * v) + self.i[k]
            if integrate and abs(raw) < config.BF_MAX_TILT_DEG:            # anti-windup
                self.i[k] = _clip(self.i[k] - config.BF_KI_DEG_PER_M_S * p * dt, config.BF_MAX_TILT_DEG)
            lean.append(_clip(raw, config.BF_MAX_TILT_DEG))
        c, s = math.cos(heading_rad), math.sin(heading_rad)
        lean_fwd = lean[0] * c + lean[1] * s                               # home frame -> the drone's axes
        lean_right = -lean[0] * s + lean[1] * c
        m = config.BF_MAX_TILT_DEG
        return _stick(_clip(lean_right, m), config.BF_ROLL_SIGN), _stick(_clip(lean_fwd, m), config.BF_PITCH_SIGN)


def throttle(hold_us, height_error_m, climb_mps, integral_us=0.0):
    """hold_us = the pilot's throttle at engage. height_error_m = engage height - height now
    (+ = we are too low). integral_us = AltHold's slow part. -> throttle in microseconds."""
    extra = config.BF_ALT_KP_US_PER_M * height_error_m - config.BF_ALT_KD_US_PER_MPS * climb_mps + integral_us
    return round(hold_us + _clip(extra, config.BF_THROTTLE_AUTHORITY))


class AltHold:
    """BETAFLIGHT CHANGE: the height hold with a slow I term (battery sag) and anti-windup."""

    def __init__(self):
        self.i, self.t = 0.0, None

    def reset(self):
        self.i, self.t = 0.0, None

    def update(self, t, hold_us, height_error_m, climb_mps):
        dt = 0.0 if self.t is None else max(0.0, min(0.2, t - self.t))
        self.t = t
        raw = config.BF_ALT_KP_US_PER_M * height_error_m - config.BF_ALT_KD_US_PER_MPS * climb_mps + self.i
        if abs(raw) < config.BF_THROTTLE_AUTHORITY:                        # anti-windup
            self.i = _clip(self.i + config.BF_ALT_KI_US_PER_M_S * height_error_m * dt, config.BF_THROTTLE_AUTHORITY)
        return throttle(hold_us, height_error_m, climb_mps, self.i)
