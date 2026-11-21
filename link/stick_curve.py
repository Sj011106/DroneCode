# =====================================================================================
# link/stick_curve.py  -  how far Betaflight REALLY leans for a stick value (ANGLE mode)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We send Betaflight stick values (1000-2000 us, 1500 = centre). In ANGLE mode the stick
#   means "lean this many degrees". The old code assumed a straight line: half stick = half
#   of angle_limit. Betaflight 4.5 and 2025.12 do NOT work like that: the stick first goes
#   through the pilot's "rates" curve, so near the centre the drone leans much less than the
#   straight line says (default Actual rates: about 3 times less). Our 8-degree command came
#   out as about 2 degrees: far too weak to hold position, and in a wind simulation every
#   hold failed. This file holds Betaflight's own curve and its exact inverse, so the
#   controller asks for DEGREES and we send the stick value that gives exactly that lean.
#
# HOW IT WORKS (technical)
#   Betaflight (pid.c, ANGLE mode, 4.5+):  angle = angle_limit * rate(stick) / rate(full stick)
#   rate() = Betaflight's rates function (rc.c applyActualRates / applyBetaflightRates).
#   Check against Betaflight's documentation: with angle_limit 60, default Actual rates give
#   a centre sensitivity of 6.3 deg and default Betaflight rates 18 deg; this file gives 6.4
#   and 18.0 (tests/test_stick_curve.py). The inverse is a bisection (the curve only rises).
#   Deadband (rc deadband) is respected.
#
# CHECK ON THE AIRCRAFT (props off, Betaflight CLI, read only, no `save`):
#   get angle_limit        -> config.BF_ANGLE_LIMIT_DEG
#   get rates_type         -> config.BF_RATES_TYPE ("ACTUAL" or "BETAFLIGHT")
#   get roll_rc_rate, roll_srate, roll_expo (pitch the same) -> config.BF_RC_RATE / BF_SRATE / BF_EXPO
#   get deadband           -> config.BF_DEADBAND_US
#   Then `python -m link.stick_curve` prints the stick value for 1-20 degrees.
#   If the crew cannot give them: ask which rates type is used; the defaults below are
#   Betaflight's own defaults.
#
# REFERENCES
#   - Betaflight source, src/main/fc/rc.c (applyActualRates, applyBetaflightRates) and
#     src/main/flight/pid.c (angle mode target from the setpoint rate), 4.5 / 2025.12.
#   - Betaflight Configurator PR 3910/3927: "Angle Mode centre sensitivity" (6.3 / 18 deg).
# =====================================================================================
from dataclasses import dataclass

import config

RC_RATE_INCREMENTAL = 14.54          # Betaflight constant for rc_rate above 2.0


@dataclass
class Rates:
    rates_type: str = "ACTUAL"       # "ACTUAL" or "BETAFLIGHT"
    rc_rate: int = 7                 # ACTUAL: centre sensitivity / 10 (7 = 70 deg/s). BETAFLIGHT: rc_rate x 100
    srate: int = 67                  # ACTUAL: max rate / 10 (67 = 670 deg/s).        BETAFLIGHT: super rate x 100
    expo: int = 0                    # x 100
    angle_limit: float = 60.0        # degrees of lean at full stick
    deadband: int = 0                # microseconds around 1500 that count as centre


def from_config():
    """The rates the event aircraft uses (config.py BF_*)."""
    return Rates(config.BF_RATES_TYPE, config.BF_RC_RATE, config.BF_SRATE, config.BF_EXPO,
                 config.BF_ANGLE_LIMIT_DEG, config.BF_DEADBAND_US)


def _rate(x, r):
    """Betaflight's setpoint rate (deg/s) for a stick deflection x in [-1, 1]."""
    ax = abs(x)
    if r.rates_type.upper() == "BETAFLIGHT":
        if r.expo:
            e = r.expo / 100.0
            x = x * ax ** 3 * e + x * (1 - e)
        rc = r.rc_rate / 100.0
        if rc > 2.0:
            rc += RC_RATE_INCREMENTAL * (rc - 2.0)
        rate = 200.0 * rc * x
        if r.srate:
            rate *= 1.0 / max(0.01, min(1.0, 1.0 - ax * r.srate / 100.0))
        return rate
    e = r.expo / 100.0                                   # ACTUAL rates
    curve = ax * (x ** 5 * e + x * (1 - e))
    centre = r.rc_rate * 10.0
    movement = max(0.0, r.srate * 10.0 - centre)
    return x * centre + movement * curve


def angle_from_us(us, r=None):
    """Stick value (us) -> the lean Betaflight will hold (degrees, + = roll right / pitch forward)."""
    r = r or from_config()
    d = us - 1500.0
    if abs(d) <= r.deadband:
        return 0.0
    d = (abs(d) - r.deadband) * (1 if d > 0 else -1)
    x = max(-1.0, min(1.0, d / (500.0 - r.deadband)))
    return r.angle_limit * _rate(x, r) / _rate(1.0, r)


def us_from_angle(deg, r=None):
    """Lean wanted (degrees) -> the stick value that gives exactly that lean. Exact inverse (bisection)."""
    r = r or from_config()
    deg = max(-r.angle_limit, min(r.angle_limit, deg))
    if deg == 0.0:
        return 1500.0
    lo, hi = 1000.0, 2000.0
    for _ in range(30):
        mid = (lo + hi) / 2
        if angle_from_us(mid, r) < deg:
            lo = mid
        else:
            hi = mid
    return (lo + hi) / 2


def centre_sensitivity(r=None):
    """The number Betaflight Configurator shows: the lean at full stick if the centre slope continued."""
    r = r or from_config()
    return angle_from_us(1500 + r.deadband + 1, r) * (500 - r.deadband)


if __name__ == "__main__":                              # bench: what will we send for each lean?
    r = from_config()
    print(f"{r}\ncentre sensitivity {centre_sensitivity(r):.1f} deg (Configurator shows the same number)")
    for deg in (1, 2, 3, 5, 8, 10, 15, 20):
        print(f"lean {deg:2d} deg -> stick {us_from_angle(deg, r) - 1500:+6.1f} us"
              f"   (old straight line: {500 * deg / r.angle_limit:+6.1f} us)")
