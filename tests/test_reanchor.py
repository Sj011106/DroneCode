# =====================================================================================
# tests/test_reanchor.py  -  pilot flies by hand, vision lost at altitude -> new home saved by itself   (PART D)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   HOME is saved on the ground (HOME_TRIGGER "start"), then the aircraft climbs. Higher up the ground looks
#   much smaller than at home, so vision is LOST and ArduPilot has no position: it refuses LOITER, and with no
#   GNSS there is nothing else to fall back on, so the hold could never be engaged. vision_hold.py now saves a new
#   home by itself, after AUTO_REANCHOR_AFTER_S, while the pilot is NOT in a hold mode.
#   The existing re-home (vision/rehome.py) follows a gradual climb that starts higher than ~0.23 m. It cannot start
#   below that: quality is forced to -1 under MIN_HEIGHT_M (0.3 m) and health drops the scale beyond 1.33x, so the
#   two windows never overlap. A camera on landing gear is that low, so this test saves home at 0.2 m.
#   It records one simulated climb (0.2 m -> 8 m at 2 m/s, then hover) and replays it twice: with the feature on
#   (vision comes back, position stays near 0) and off (--set AUTO_REANCHOR_AFTER_S=0: stays LOST).
#   "Never acts while a hold is engaged" is tested in tests/test_handover.py (AutoReanchor.update).
#
# Run (from the repo root):  python -m tests.test_reanchor        (~1 min)
# =====================================================================================

import csv
import os
import tempfile

import numpy as np

import config
import vision_hold
from tests.common import K, use_test_camera
from tools.degrade_video import make_ground, render
from tools.replay import Recorder

W, H, FPS, T0 = 640, 480, 30, 1000.0
HOME_H, FLY_H, CLIMB_MS = 0.2, 8.0, 2.0              # camera ~20 cm above the ground at home, hover height, climb rate (m/s)
CLIMB_START = 1.0
CLIMB_END = CLIMB_START + (FLY_H - HOME_H) / CLIMB_MS
SECONDS = 14
use_test_camera()                                    # fixed lens + default timing, not the repo's files
ground = make_ground(3000)
rng = np.random.default_rng(11)
tmp = tempfile.mkdtemp(prefix="dronecode_reanchor_")
DEFAULTS = {k: getattr(config, k) for k in dir(config) if k.isupper()}


def height(t):
    t -= T0
    if t < CLIMB_START:
        return HOME_H
    if t < CLIMB_END:
        return HOME_H + CLIMB_MS * (t - CLIMB_START)
    return FLY_H


def pose(t):
    return (0.0, 0.0, height(t), 0.0, 0.0, 0.0)       # straight up, no sideways drift: true position is always home


imu = [(t, 0.0, 0.0, 0.0) for t in np.arange(T0 - 0.5, T0 + SECONDS + 1, 0.02)]
rec_dir = os.path.join(tmp, "rec")
rec = Recorder(rec_dir)
for i in range(int(SECONDS * FPS)):
    t = T0 + i / FPS
    img = render(ground, K, (W, H), pose, t, readout_s=config.READOUT_S, noise=2, rng=rng)
    rec.add(img, t, i == 0, height(t), [s for s in imu if s[0] <= t])   # HOME is saved on frame 0, at 0.2 m
rec.close()


def replay(name, *sets):
    for k, v in DEFAULTS.items():                     # every replay starts from config.py defaults
        setattr(config, k, v)
    log = os.path.join(tmp, name + ".csv")
    argv = ["--replay", rec_dir, "--log", log, "--no-gps-fallback"]
    for s in sets:
        argv += ["--set", s]
    vision_hold.main(argv)
    with open(log, newline="") as f:
        return list(csv.DictReader(f))


def trusted(rows, after_s):
    """Rows later than `after_s` seconds into the flight that carry a valid position."""
    return [r for r in rows if float(r["t_capture"]) - T0 > after_s and int(r["quality"]) > 0 and r["north_m"] != "nan"]


CHECK_FROM = CLIMB_END + 3.0                          # well after the climb and after the 2 s wait
frames_checked = int((SECONDS - CHECK_FROM) * FPS)

print("\nPilot flies by hand: home saved at 0.2 m, climb to 8 m, vision lost -> new home by itself")
on = replay("auto_on")
off = replay("auto_off", "AUTO_REANCHOR_AFTER_S=0")
t_on, t_off = trusted(on, CHECK_FROM), trusted(off, CHECK_FROM)
worst = max((max(abs(float(r["north_m"])), abs(float(r["east_m"]))) for r in t_on), default=float("nan"))
print(f"  frames checked (after {CHECK_FROM:.0f} s): {frames_checked}")
print(f"  feature ON : {len(t_on)} frames with a valid position, worst |north/east| {worst:.2f} m (the aircraft never moved sideways)")
print(f"  feature OFF: {len(t_off)} frames with a valid position")
assert len(t_on) > 0.7 * frames_checked, "with AutoReanchor vision must come back after the climb"
assert worst < 0.3, "after the new home the position must stay near 0 (the aircraft only climbed)"
assert len(t_off) < 0.1 * frames_checked, "without AutoReanchor vision stays lost at altitude (the problem this fixes)"

print("\nALL REANCHOR TESTS PASSED")
