# =====================================================================================
# tests/test_bf_hold.py  -  30 s holds in wind through the REAL Betaflight-route code (BETAFLIGHT CHANGE)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   No simulator exists for the Betaflight route, so this test is a small one: a drone that
#   leans the way Betaflight's ANGLE mode makes it lean (through its stick curve), blown by
#   wind with gusts, seen by a camera that is 75 ms late at 13 frames a second with a small
#   tilt error. Our REAL code flies it: fusion/position_controller.HoldController, the
#   sender's lean cap (link/msp_sender.cap_stick) and link/stick_curve.py. Judged like the
#   organisers' hold_judge.py: 30 s within 3 m horizontally (Stage 2 section 06).
#     1. sideways hold at 10, 35 and 75 m: every hold qualifies
#     2. the code before these changes (straight-line stick, PD only, 8 deg cap) fails
#     3. engaging is smooth: the I term starts at the pilot's lean into the wind
#     4. height hold (sender's AltHold): noisy barometer (+-0.4 m), battery sag, the pilot's
#        throttle a little off: the height stays within 2 m
#     5. bench step 5: a judge's hand tilt at the engage moment must NOT offset the answer
#        (the I term starts at the pilot's lean only when ARMED)
#   SIMULATION, not flight: it proves the logic and the starting gains are sane; the real
#   aircraft must confirm them (start weaker rather than stronger).
# RUN:  python -m tests.test_bf_hold   (~30 s)
# =====================================================================================
import math

import numpy as np

import config
from fusion.position_controller import AltHold, HoldController
from link import stick_curve
from link.msp_sender import cap_stick

G = 9.81
rng = np.random.default_rng(42)


def ou(n, dt, sigma, tau):
    x, a = np.zeros(n), dt / tau
    for i in range(1, n):
        x[i] = x[i - 1] - a * x[i - 1] + sigma * math.sqrt(2 * a) * rng.normal()
    return x


def hold(h, wind=None, old=False, seconds=30.0):
    dt, n = 1 / 200, int(seconds * 200)
    w = rng.uniform(3, 7) if wind is None else wind
    wd = rng.uniform(0, 2 * np.pi)
    wv = np.c_[w * math.cos(wd) + ou(n, dt, 1.5, 3.0), w * math.sin(wd) + ou(n, dt, 1.5, 3.0)]
    att_err = np.c_[ou(n, dt, math.radians(0.4), 5.0), ou(n, dt, math.radians(0.4), 5.0)]
    cd = 0.28
    tilt = np.arctan(-cd * wv[0] / G)                    # the pilot is holding into the wind at engage
    pos, vel = np.zeros(2), rng.uniform(-0.3, 0.3, 2)
    ctl = HoldController()
    ctl.engage(lean_fwd_deg=math.degrees(tilt[0]), lean_right_deg=math.degrees(tilt[1]))
    sticks = (cap_stick(stick_curve.us_from_angle(math.degrees(tilt[1]))),
              cap_stick(stick_curve.us_from_angle(math.degrees(tilt[0]))))
    queue, prev, vel_est, worst, within = [], None, np.zeros(2), 0.0, seconds
    for k in range(n):
        t = k * dt
        target = np.radians([stick_curve.angle_from_us(sticks[1]), stick_curve.angle_from_us(sticks[0])])  # (N, E)
        tilt += (target - tilt) * dt / 0.12              # Betaflight ANGLE mode follows with a short lag
        acc = G * np.tan(tilt) - cd * (vel - wv[k])
        vel += acc * dt
        pos += vel * dt
        e = float(np.hypot(*pos))
        worst = max(worst, e)
        if within == seconds and e > config.HOLD_MAX_DRIFT_M:
            within = t
        if k % 15 == 0:                                  # camera ~13 fps, result 75 ms late
            queue.append((t + 0.075, t, pos + h * np.tan(att_err[k]) + rng.normal(0, 0.2 * h / 208, 2)))
        while queue and queue[0][0] <= t:
            _, tc, m = queue.pop(0)
            if prev is not None:                         # vision_hold.py's velocity: 0.7 / 0.3 smoothing
                vel_est = 0.7 * vel_est + 0.3 * (m - prev[0]) / (tc - prev[1])
            prev = (m, tc)
            if old:                                      # the code before: straight line, PD, 8 deg
                lean = np.clip(-(6.0 * m + 10.0 * vel_est), -8.0, 8.0)
                sticks = (1500 + 500 * lean[1] / 60, 1500 + 500 * lean[0] / 60)
            else:
                r, p = ctl.update(tc, m[0], m[1], vel_est[0], vel_est[1], 0.0, integrate=True)
                sticks = (cap_stick(r), cap_stick(p))
    return within >= seconds, worst


print("Test 1: 30 s holds, wind 3-7 m/s with gusts, through the real controller + stick curve + sender cap")
for h in (10, 35, 75):
    r = [hold(h) for _ in range(6)]
    q = sum(ok for ok, _ in r)
    print(f"  {h:2d} m: {q}/6 qualifying, worst {max(w for _, w in r):.2f} m")
    assert q == 6

print("\nTest 2: the code before these changes (straight-line stick, PD only, 8 deg cap), same wind")
r = [hold(10, old=True) for _ in range(4)]
print(f"  10 m: {sum(ok for ok, _ in r)}/4 qualifying, worst {max(w for _, w in r):.1f} m")
assert sum(ok for ok, _ in r) <= 1

print("\nTest 3: engaging in a steady 6 m/s wind is smooth (I term starts at the pilot's lean)")
ok, worst = hold(20, wind=6.0, seconds=5.0)
print(f"  first 5 s after engage: worst {worst:.2f} m")
assert worst < 1.0

print("\nTest 4: height hold: barometer +-0.4 m noise at 16 Hz, battery sag, pilot throttle 20 us off")
worst_z = 0.0
for trial in range(4):
    dt, n = 1 / 50, 30 * 50
    hover, k_thr = 1450.0, G / (1450 - 1050)             # m/s^2 per us
    z = vz = 0.0
    alt, climb, alt_t, baro = 0.0, 0.0, None, 0.0
    ah, pilot = AltHold(), hover + rng.choice([-20, 20])
    for k in range(n):
        t = k * dt
        if k % 3 == 0:                                   # the APB barometer at ~16 Hz
            baro = z + rng.normal(0, 0.4)
        new = alt + 0.2 * (baro - alt)                   # the sender's smoothing (msp_sender._height)
        kk = min(1.0, dt / config.BF_CLIMB_SMOOTH_S)
        climb += kk * ((new - alt) / dt - climb)
        alt = new
        thr = ah.update(t, pilot, 0.0 - alt, climb)
        sag = 15.0 * t / 60
        az = (thr - (hover + sag)) * k_thr - 0.3 * vz + rng.normal(0, 0.3)
        vz += az * dt
        z += vz * dt
        worst_z = max(worst_z, abs(z))
print(f"  worst height change over 4 holds: {worst_z:.2f} m (limit {config.HOLD_MAX_VERTICAL_M} m)")
assert worst_z < config.HOLD_MAX_VERTICAL_M
print("\nTest 5: bench step 5 with the judge's hand tilted 4 deg at the engage moment")
from link.betaflight_link import BetaflightLink
for armed in (False, True):
    link = BetaflightLink()
    rc = [1500, 1500, 1500, 1420, 2000 if armed else 1000, 1500, 1500, 1500, 1500, 1000]
    base = dict(t=1.0, att=(1.0, math.radians(4.0), 0.0, 0.0), alt=None, rc=rc)   # tilted 4 deg RIGHT
    link.on_telemetry(dict(base, engaged=False))
    link.send_odometry(1.0, 0.0, 0.0, 0.0, (0, 0, 0), (0, 0, 0), 90, reset_counter=1)
    link.on_telemetry(dict(base, engaged=True))
    link.send_odometry(1.1, 0.0, 0.0, 0.0, (0, 0, 0), (0, 0, 0), 90, reset_counter=2)   # new home saved
    held = link.last_sticks
    link.send_odometry(1.2, 0.0, 0.5, 0.0, (0, 0, 0), (0, 0, 0), 90, reset_counter=2)   # moved 0.5 m RIGHT
    moved = link.last_sticks
    print(f"  {'armed (flight)' if armed else 'disarmed (bench)'}: at home roll {held[0]}, 0.5 m right roll {moved[0]}")
    if armed:
        assert held[0] > 1500, "flight: the pilot's lean is kept at engage (smooth, no wind offset)"
    else:
        assert held[0] == 1500 and moved[0] < 1500, "bench: a hand tilt must not offset the step 5 answer"

print("\nALL BETAFLIGHT HOLD TESTS PASSED (simulation: confirm gains on the aircraft, start weaker)")
