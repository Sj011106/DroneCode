# =====================================================================================
# tests/test_recovery.py  -  camera blackouts of 2 s and 10 s      BRIEF ITEM 3
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The drone hovers, then the camera goes black (2 s, later 10 s) while a gust pushes
#   the drone away from home and turns it. When the picture comes back, can we find
#   home again, and how fast? We run it twice: WITHOUT recovery.py (Saumya's tracker
#   alone) and WITH it, and print a timeline of the health status for the slide.
#
# Run (from the repo root):  python -m tests.test_recovery
#
# REFERENCES: see recovery/recovery.py.
# =====================================================================================

import math

import numpy as np

from health.health import HealthChecker
from recovery.recovery import Recovery
from tools.degrade_video import make_ground, render
from vision.tracker import GroundTracker, to_north_east

W, H = 640, 480
HEIGHT = 4.16
FPS = 30
from tests.common import K                  # fixed test camera (not the repo's calib.npz)
ground = make_ground(3000)
rng = np.random.default_rng(3)

# (start_s, duration_s, drift during blackout (north, east) metres, yaw change deg)
BLACKOUTS = [(1.0, 2.0, (0.6, 0.8), 10.0),       # 1.0 m drift in 2 s
             (5.0, 10.0, (-1.2, 1.6), -25.0)]    # 2.0 m drift in 10 s
END_S = 17.0


def truth(t):
    """True (north, east, yaw) at time t: hover, drift + turn only during blackouts."""
    n = e = yaw = 0.0
    for s, d, (dn, de), dyaw in BLACKOUTS:
        f = np.clip((t - s) / d, 0, 1)
        n, e, yaw = n + f * dn, e + f * de, yaw + f * math.radians(dyaw)
    return n, e, yaw


def blacked_out(t):
    return any(s <= t < s + d for s, d, _, _ in BLACKOUTS)


def run(use_recovery):
    tracker, health = GroundTracker(K, None), HealthChecker()
    rec = Recovery(tracker)
    home = render(ground, K, (W, H), lambda _: (0, 0, HEIGHT, 0, 0, 0), 0, noise=2, rng=rng)
    tracker.set_reference(home)
    rec.set_home(home)
    timeline, back_ok = [], {}
    for i in range(1, int(END_S * FPS)):
        t = i / FPS
        n, e, yaw = truth(t)
        if blacked_out(t):
            img = np.zeros((H, W), np.uint8)
        else:
            img = render(ground, K, (W, H), lambda _: (n, e, HEIGHT, 0, 0, yaw), t, noise=2, rng=rng)
        res = tracker.track(img, t_capture=t)
        rep = health.check(img, res)
        if use_recovery:
            rec.update(img, rep.status == "LOST" and health.bad_count > 0)
        err = None
        if rep.status == "OK":
            vn, ve = to_north_east(res.shift_px, HEIGHT, tracker.focal_px, yaw)
            err = math.hypot(vn - n, ve - e) * 100
        timeline.append((t, rep.status, err))
        for s, d, _, _ in BLACKOUTS:      # first OK frame after each blackout ends
            if t >= s + d and rep.status == "OK" and (s, d) not in back_ok:
                back_ok[(s, d)] = (t - (s + d), err)
    return timeline, back_ok, rec.relocks


def bar(timeline):
    """One character per 0.25 s: O = OK, d = DEGRADED, . = LOST, # = camera black."""
    out = ""
    for t, status, _ in timeline[::FPS // 4]:
        out += "#" if blacked_out(t) else {"OK": "O", "DEGRADED": "d", "LOST": "."}[status]
    return out


print("Timeline, one character per 0.25 s  (O = OK, . = LOST, # = camera black)")
results = {}
for use in (False, True):
    timeline, back_ok, relocks = run(use)
    results[use] = back_ok
    print(f"\n  recovery {'ON ' if use else 'OFF'}: {bar(timeline)}")
    for s, d, (dn, de), _ in BLACKOUTS:
        drift = math.hypot(dn, de)
        if (s, d) in back_ok:
            dt, err = back_ok[(s, d)]
            print(f"    {d:4.0f} s blackout, drifted {drift:.1f} m: trusted again {dt:.2f} s after "
                  f"the picture returned, position error {err:.1f} cm")
        else:
            print(f"    {d:4.0f} s blackout, drifted {drift:.1f} m: NEVER recovered")
    if use:
        print(f"    re-locks performed: {relocks}")

for s, d, _, _ in BLACKOUTS:
    assert (s, d) in results[True], f"recovery failed after the {d} s blackout"
    dt, err = results[True][(s, d)]
    assert dt < 1.0 and err < 5.0, f"{d} s blackout: too slow ({dt:.2f} s) or inaccurate ({err:.1f} cm)"
print("\nALL RECOVERY TESTS PASSED")
