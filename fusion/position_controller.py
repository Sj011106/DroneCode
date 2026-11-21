# =====================================================================================
# fusion/position_controller.py  -  turn "we drifted" into stick commands (Betaflight)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   On ArduPilot we hand over a position and the autopilot does the flying. Betaflight
#   has no such thing: with the engage switch on it flies whatever stick values we send.
#   So this file is the pilot's thumbs. Two small rules:
#     sideways:  drifted to the right, or moving right  -> lean a little to the left
#                (same for forward/back). Never more than a few degrees.
#     height:    lower than where the pilot engaged, or sinking -> a little more throttle.
#   Heading is left alone (yaw stick centred): the camera measures in the drone's own
#   left/right/forward/back, so no compass is needed (the aircraft has none).
#
# HOW IT WORKS (technical)
#   Horizontal: PD on body-frame position error and velocity, output = lean angle,
#     angle = -(KP * error_m + KD * speed_mps), clipped to +-BF_MAX_TILT_DEG, then to stick
#     microseconds: 1500 + 500 * angle / BF_ANGLE_LIMIT_DEG (Betaflight ANGLE mode: full
#     stick = angle_limit degrees). A point mass leaning by a small angle a accelerates at
#     g * a, so KP = 6 deg/m is about 1 m/s^2 per metre and KD = 10 deg/(m/s) is roughly
#     critically damped (2 * sqrt(1) = 2 m/s^2 per m/s = 11.7 deg).
#   Vertical: PD on barometer height error and climb rate, added to the throttle the pilot
#     was holding at engage, clipped to +-BF_THROTTLE_AUTHORITY.
#   EVERY NUMBER IS A STARTING GUESS (config.py BF_*). No simulator exists for this route:
#   they must be tuned on the real aircraft, starting weaker rather than stronger. The
#   SIGNS are what the bench test checks (Stage 2 section 05 step 5: "roll and pitch oppose
#   the move"); BF_ROLL_SIGN / BF_PITCH_SIGN flip them if the bench shows the opposite.
#   Proof: tests/test_betaflight.py.
#
# REFERENCES
#   - Stage 2 package section 03 ("your code is the position controller and the height
#     controller") and section 05 step 5.
#   - Astrom & Murray, "Feedback Systems", ch. 10-11 (PD control of a double integrator).
#   - Mahony, Kumar, Corke, "Multirotor Aerial Vehicles: Modeling, Estimation, and Control
#     of Quadrotor", IEEE RAM 2012 (small-angle lean -> horizontal acceleration).
# =====================================================================================
import config


def _clip(x, lim):
    return max(-lim, min(lim, x))


def sticks(forward_m, right_m, v_forward, v_right):
    """Position error and speed in the drone's own axes (metres, m/s; + = forward / right of home)
    -> (roll_us, pitch_us). Stick convention: roll > 1500 leans right, pitch > 1500 leans forward."""
    lean_right = _clip(-(config.BF_KP_DEG_PER_M * right_m + config.BF_KD_DEG_PER_MPS * v_right), config.BF_MAX_TILT_DEG)
    lean_fwd = _clip(-(config.BF_KP_DEG_PER_M * forward_m + config.BF_KD_DEG_PER_MPS * v_forward), config.BF_MAX_TILT_DEG)
    per_deg = 500.0 / config.BF_ANGLE_LIMIT_DEG
    return (round(1500 + config.BF_ROLL_SIGN * lean_right * per_deg),
            round(1500 + config.BF_PITCH_SIGN * lean_fwd * per_deg))


def throttle(hold_us, height_error_m, climb_mps):
    """hold_us = the pilot's throttle at engage. height_error_m = engage height - height now
    (+ = we are too low). -> throttle in microseconds."""
    extra = config.BF_ALT_KP_US_PER_M * height_error_m - config.BF_ALT_KD_US_PER_MPS * climb_mps
    return round(hold_us + _clip(extra, config.BF_THROTTLE_AUTHORITY))
