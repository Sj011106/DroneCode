# =====================================================================================
# tests/test_bench.py  -  the Stage 2 BENCH tests, rehearsed in simulation
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   At the event a judge holds the aircraft about 1 m above the floor (props off) and
#   carries it around; our laptop must show the right numbers (Stage 2 package section 05):
#     step 2 (B1)  carry 0.5-1.5 m to a mark        -> displayed distance within 0.25 m
#     step 3 (B2)  walk a loop, stop over the start -> displayed position within 0.30 m of zero
#     step 4 (B3)  raise the aircraft by 0.5 m      -> displayed height change within 0.15 m
#     bonus        camera covered for 3 s mid-loop  -> still within 0.50 m of zero at the start
#   So low over the floor, the first "home" picture slides out of view after about a
#   metre, which the original tracker could not survive. This test plays those four
#   moves through the real program (vision_hold.py) twice: with the bench features OFF
#   (how it was) and ON, and prints the error each time.
#
# HOW IT WORKS (technical)
#   A camera like the Justice D after our downscale (640 x 360, focal 208 px = 114 deg wide)
#   is rendered over a random floor texture at 1 m, with a +-2 deg wobble from the judge's
#   hands. Each move is saved with tools/replay.Recorder and replayed with
#   `--fixed-height 1.0`. Features under test (config.py): REHOME_SHIFT_FRAC (new reference
#   when the picture slid a quarter of its width), USE_ORIGIN_RETURN (measure against the
#   first home again when it is back in view), LOST_KEYFRAME_AFTER_S (new reference after a
#   blind stretch), BENCH_HEIGHT_FROM_SCALE (height from how big the floor looks).
#   SIMULATED: a clean random floor, perfect lens, known attitude. The real floor decides.
#
# RUN:  python -m tests.test_bench
# REFERENCE: Red Team Grand Final Stage 2 package, Rev 2.0, sections 05 and 07.
# =====================================================================================
import csv
import math
import os
import tempfile

import cv2
import numpy as np

import config
import vision_hold
from tests.common import use_test_camera
from tools.degrade_video import make_ground, render
from tools.replay import Recorder

W, H, FPS, HEIGHT, T0 = 640, 360, 30, 1.0, 1000.0
F = 320 / math.tan(math.radians(57))                 # 114 deg across 640 px = 207.8 px
K = np.array([[F, 0, 319.5], [0, F, 179.5], [0, 0, 1.0]])
M_PER_TEXEL = 0.004                                  # floor detail of 4 mm (about 1 pixel at 1 m)
SPEED = 0.4                                          # m/s: a judge walking slowly
YAW = 0.3                                            # aircraft not pointing North

use_test_camera()
tmp = tempfile.mkdtemp(prefix="dronecode_bench_")
config.CALIB_FILE = os.path.join(tmp, "calib_bench.npz")
np.savez(config.CALIB_FILE, K=K, dist=np.zeros((1, 5)), size=np.array([W, H]))
DEFAULTS = {k: getattr(config, k) for k in dir(config) if k.isupper()}
ground = make_ground(3000)                           # 12 m square of floor
rng = np.random.default_rng(11)
OFF = ["REHOME_SHIFT_FRAC=0", "USE_ORIGIN_RETURN=False", "LOST_KEYFRAME_AFTER_S=0", "BENCH_HEIGHT_FROM_SCALE=False"]


def path_pose(corners, hold_s=1.0, heights=None):
    """pose(t) walking the corner list at SPEED, then standing still. heights = (from, to) for a raise."""
    legs = [math.dist(a, b) for a, b in zip(corners, corners[1:])]
    total = sum(legs) / SPEED if sum(legs) else 2.0

    def pose(t):
        t -= T0
        d, (n, e) = min(max(t - 0.5, 0.0), total) * SPEED, corners[0]
        for (a, b), leg in zip(zip(corners, corners[1:]), legs):
            if d <= leg and leg:
                n, e = a[0] + (b[0] - a[0]) * d / leg, a[1] + (b[1] - a[1]) * d / leg
                break
            d, (n, e) = d - leg, b
        h = HEIGHT if not heights else heights[0] + (heights[1] - heights[0]) * min(max(t - 0.5, 0.0) / total, 1.0)
        return (n, e, h, math.radians(2) * math.sin(math.pi * t), math.radians(2) * math.sin(0.8 * math.pi * t + 1), YAW)
    return pose, 0.5 + total + hold_s


def record(name, pose, seconds, cover=None):
    """Save the move as a replayable recording. cover = (start_s, length_s): camera covered (black)."""
    folder = os.path.join(tmp, name)
    rec = Recorder(folder)
    imu = [(t, *pose(t)[3:]) for t in np.arange(T0 - 0.5, T0 + seconds + 0.5, 0.02)]   # 50 Hz attitude
    for i in range(int(seconds * FPS)):
        t = T0 + i / FPS
        if cover and cover[0] <= t - T0 < cover[0] + cover[1]:
            img = np.zeros((H, W), np.uint8)
        else:
            img = render(ground, K, (W, H), pose, t, readout_s=config.READOUT_S, noise=2, rng=rng,
                         m_per_texel=M_PER_TEXEL)
        rec.add(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), t, i == 0, HEIGHT, [s for s in imu if t - 1.0 <= s[0] <= t])
    rec.close()
    return folder


def replay(folder, name, sets=()):
    for k, v in DEFAULTS.items():                    # each replay starts from config.py defaults
        setattr(config, k, v)
    log = os.path.join(tmp, name + ".csv")
    argv = ["--replay", folder, "--log", log, "--no-gps-fallback", "--fixed-height", str(HEIGHT)]
    for s in sets:
        argv += ["--set", s]
    vision_hold.main(argv)
    with open(log, newline="") as f:
        return list(csv.DictReader(f))


def shown(rows):
    """What the laptop shows at the end: (north, east, height) averaged over the ready frames of the
    last half second, or None if the program was not ready for most of it."""
    last = [r for r in rows[-15:] if r["north_m"] != "nan"]
    if len(last) < 8:
        return None
    return tuple(float(np.mean([float(r[k]) for r in last])) for k in ("north_m", "east_m", "height_m"))


def run(title, name, pose, seconds, truth, limit, measure, cover=None):
    """Replay one move OFF and ON. measure(shown) -> error in metres. Returns the ON error."""
    folder = record(name, pose, seconds, cover)
    print(f"\n{title}   (pass at the event: within {limit:.2f} m)")
    errs = {}
    for label, sets in (("OFF", OFF), ("ON ", ())):
        rows = replay(folder, f"{name}_{label.strip()}", sets)
        s = shown(rows)
        errs[label] = None if s is None else measure(s)
        trusted = 100 * sum(r["north_m"] != "nan" for r in rows) / len(rows)
        rehomes = rows[-1]["rehomes"]
        text = "NOT READY at the end" if s is None else f"error {errs[label] * 100:5.1f} cm"
        print(f"  bench features {label}: {text:22s} ready {trusted:5.1f}% of the time, {rehomes} reference switches")
    assert errs["ON "] is not None and errs["ON "] < limit, f"{title}: must be within {limit} m (truth {truth})"
    return errs


print("Stage 2 bench rehearsal: Justice D-like camera 1 m above a random floor (SIMULATED)")
results = {}

# Step 2 (B1): 1.5 m carry, the longest the judge may choose, in a diagonal direction
end = (1.5 * math.cos(0.7), 1.5 * math.sin(0.7))
pose, secs = path_pose([(0, 0), end])
results["B1"] = run("Step 2 (B1): carry 1.5 m", "carry", pose, secs, 1.5, 0.25,
                    lambda s: abs(math.hypot(s[0], s[1]) - 1.5))

# Step 3 (B2): a square loop with 1.5 m sides (harder than "about 3 m" round), back over the start mark
loop = [(0, 0), (1.5, 0), (1.5, 1.5), (0, 1.5), (0, 0)]
pose, secs = path_pose(loop)
results["B2"] = run("Step 3 (B2): loop, back to the start", "loop", pose, secs, 0.0, 0.30,
                    lambda s: math.hypot(s[0], s[1]))

# Step 4 (B3): raise 0.5 m over the start mark
pose, secs = path_pose([(0, 0), (0, 0)], heights=(1.0, 1.5))
results["B3"] = run("Step 4 (B3): raise 0.5 m", "raise", pose, secs, 0.5, 0.15,
                    lambda s: abs((s[2] - HEIGHT) - 0.5))

# Recovery bonus: the same loop, camera covered for 3 s on the second side (the aircraft keeps moving)
pose, secs = path_pose(loop)
results["bonus"] = run("Recovery bonus: loop with the camera covered for 3 s", "cover", pose, secs, 0.0, 0.50,
                       lambda s: math.hypot(s[0], s[1]), cover=(5.5, 3.0))

for k, v in DEFAULTS.items():
    setattr(config, k, v)
assert results["B1"]["OFF"] is None or results["B1"]["OFF"] > results["B1"]["ON "], \
    "the 1.5 m carry should be lost or worse without the bench features"
print("\nALL BENCH TESTS PASSED (simulated floor: repeat on the real floor before claiming an item)")
