# =====================================================================================
# tests/test_replay.py  -  record once, replay many times              (all brief items)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Test 1: a simulated flight (rolling-shutter camera, tilting and vibrating drone,
#   known true position) is saved with the recorder, then replayed through the real
#   vision_hold.py: twice with the same settings (must give identical results), then
#   with the tilt fix OFF and with the rolling-shutter fix OFF. This is how the
#   "with vs without" slides are made from ONE real flight.
#   Test 2: a live run (pretend autopilot + video, rocking attitude, rolling-shutter fix ON)
#   is recorded with --record and replayed: the replay must reproduce the live positions
#   exactly (each frame gets exactly the IMU samples it had live).
#   Test 3: --set reaches settings that our classes read when they're created.
#
# Run (from the repo root):  python -m tests.test_replay      (~1 min)
# REFERENCES: see tools/replay.py.
# =====================================================================================

import csv
import math
import os
import tempfile
import threading
import time
from collections import deque

import cv2
import numpy as np

import config
import vision_hold
from tools.degrade_video import make_ground, render
from tests.common import K, use_test_camera
from tools.replay import Recorder

W, H, FPS, HEIGHT, T0 = 640, 480, 30, 6.0, 1000.0
use_test_camera()                                    # fixed lens + default timing, not the repo's files
ground = make_ground(3000)
rng = np.random.default_rng(7)
tmp = tempfile.mkdtemp(prefix="dronecode_replay_")
DEFAULTS = {k: getattr(config, k) for k in dir(config) if k.isupper()}


def pose(t):
    """Hover with slow lean (+-6 deg, 0.5 Hz), vibration (2 deg, 6 Hz) and a little drift."""
    t -= T0
    return (0.10 * math.sin(0.4 * t), 0.05 * t, HEIGHT,
            math.radians(2) * math.sin(2 * math.pi * 6 * t) + math.radians(4) * math.sin(math.pi * t),
            math.radians(6) * math.sin(math.pi * t + 0.5), math.radians(5) * math.sin(0.3 * t))


def read_log(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def replay(folder, name, *sets):
    for k, v in DEFAULTS.items():                    # each replay starts from config.py defaults
        setattr(config, k, v)
    log = os.path.join(tmp, name + ".csv")
    argv = ["--replay", folder, "--log", log, "--no-gps-fallback"]
    for s in sets:
        argv += ["--set", s]
    vision_hold.main(argv)
    return read_log(log)


def error_cm(rows):
    errs = []
    n0, e0 = pose(float(rows[0]["t_capture"]) - 1 / FPS)[:2]     # home = first frame
    for r in rows:
        if r["status"] == "OK" and r["north_m"] != "nan":
            p = pose(float(r["t_capture"]))
            errs.append(math.hypot(float(r["north_m"]) - (p[0] - n0), float(r["east_m"]) - (p[1] - e0)) * 100)
    return errs


def positions(rows):
    return [(r["status"], r["north_m"], r["east_m"]) for r in rows]


# ---------------- Test 1: synthetic recording, replays + toggles ----------------
print("Test 1: record a simulated flight, replay it with different settings")
rec_dir = os.path.join(tmp, "rec_sim")
rec = Recorder(rec_dir)
imu = [(t, *pose(t)[3:]) for t in np.arange(T0 - 0.5, T0 + 8.0, 0.02)]    # 50 Hz attitude stream
for i in range(int(4 * FPS)):
    t = T0 + i / FPS
    img = render(ground, K, (W, H), pose, t, readout_s=config.READOUT_S, noise=2, rng=rng)
    rec.add(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), t, i == 0, HEIGHT, [s for s in imu if s[0] <= t])
rec.close()

runs = {"all fixes ON": replay(rec_dir, "a"),
        "same again": replay(rec_dir, "b"),
        "tilt fix OFF": replay(rec_dir, "c", "USE_TILT=False"),
        "rolling-shutter fix OFF": replay(rec_dir, "d", "USE_ROLLING_SHUTTER=False")}
print()
for name, rows in runs.items():
    e = error_cm(rows)
    print(f"  {name:24s}: {len(e):3d} trusted frames, error mean {np.mean(e):5.1f} cm, max {max(e):5.1f} cm")
assert positions(runs["all fixes ON"]) == positions(runs["same again"]), "replay must be repeatable"
on = np.mean(error_cm(runs["all fixes ON"]))
assert np.mean(error_cm(runs["tilt fix OFF"])) > 3 * on, "turning the tilt fix off should hurt a lot"
# Without the RS fix the warp makes health.py reject many frames (fewer trusted frames);
# the frames it still trusts are the calm ones, so their error stays similar.
assert len(error_cm(runs["rolling-shutter fix OFF"])) < 0.8 * len(error_cm(runs["all fixes ON"])), \
    "turning the RS fix off should cost trusted frames"

# ---------------- Test 2: live run with --record, then replay ----------------
print("\nTest 2: live run recorded with --record, then replayed")
video = os.path.join(tmp, "live.avi")
out = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"FFV1"), FPS, (W, H))
for i in range(int(3 * FPS)):
    out.write(cv2.cvtColor(render(ground, K, (W, H), lambda _: (0.1 * i / FPS, 0, 4.16, 0, 0, 0), 0,
                                  noise=2, rng=rng), cv2.COLOR_GRAY2BGR))
out.release()


def rocking(t):
    """Pretend IMU: roll/pitch rocking +-2 deg at 3/2 Hz, heading fixed."""
    return (math.radians(2) * math.sin(6 * math.pi * t), math.radians(2) * math.sin(4 * math.pi * t), 0.0)


class FakeLink:
    """Pretend autopilot: rocking attitude at 50 Hz in a thread, fixed height."""
    def __init__(self, connection):
        self.conn = type("C", (), {"flightmode": "LOITER", "address": "fake", "set_mode": lambda s, m: None})()
        self.send_lock, self.attitudes = threading.Lock(), deque(maxlen=200)
        self.rc, self.origin_set, self.acked_source, self.local_z, self.running = {}, True, None, None, True

    def start(self):
        def feed():
            while self.running:
                now = time.time()
                self.attitudes.append((now, *rocking(now)))
                time.sleep(0.02)
        threading.Thread(target=feed, daemon=True).start()
        time.sleep(0.1)

    def check_params(self): pass
    def send_heartbeat(self): pass
    def set_origin(self): pass
    def set_source(self, n): self.acked_source = n
    def send_odometry(self, *a, **k): pass
    def attitude_at(self, t): return min(list(self.attitudes), key=lambda a: abs(a[0] - t))[1:]
    def height(self, t): return 4.16


for k, v in DEFAULTS.items():
    setattr(config, k, v)
real_link = vision_hold.DroneLink
vision_hold.DroneLink = FakeLink
live_dir = os.path.join(tmp, "rec_live")
live_log = os.path.join(tmp, "live.csv")
vision_hold.main(["--camera", video, "--trigger", "start", "--record", live_dir, "--log", live_log,
                  "--no-gps-fallback"])                         # rolling-shutter fix ON
vision_hold.DroneLink = real_link
live = read_log(live_log)
again = replay(live_dir, "live_replay")
same = sum(a == b for a, b in zip(positions(live), positions(again)))
trusted = sum(r["status"] == "OK" and r["north_m"] != "nan" for r in live)
print(f"\n  live frames {len(live)}, replayed frames {len(again)}, identical positions {same}/{len(live)} "
      f"({trusted} trusted, attitude rocking, rolling-shutter fix on)")
assert len(live) == len(again) and same == len(live), "replay must reproduce the live run"
assert trusted > len(live) // 2, "the live run must actually produce positions (not a trivial all-nan match)"

# ---------------- Test 3: --set reaches settings read at creation time ----------------
print("\nTest 3: --set changes settings used when our objects are created")
from fusion.height import HeightFilter
from fusion.imu_check import ImuCheck
from recovery.failsafe import Failsafe
from recovery.recovery import Recovery
config.IMU_CHECK_MAX_DEG, config.HEIGHT_MAX_RATE_MPS, config.RECOVERY_MIN_INLIERS = 20.0, 99.0, 5
config.FAILSAFE_AFTER_S = 7.0
got = (ImuCheck().max_deg, HeightFilter().max_rate, Recovery(None).min_inliers, Failsafe().after_s)
print(f"  ImuCheck {got[0]}, HeightFilter {got[1]}, Recovery {got[2]}, Failsafe {got[3]} (should be 20, 99, 5, 7)")
assert got == (20.0, 99.0, 5, 7.0)
for k, v in DEFAULTS.items():
    setattr(config, k, v)

# ---------------- Test 4: failsafe when health says OK but no valid position is sent -------------
# (review finding: the height disappears -> quality -1 while health stays "OK"; the old
#  failsafe only watched for LOST and never fired)
print("\nTest 4: height lost after 2 s while flying on vision (health still OK) -> failsafe")
import contextlib
import io
rec_dir = os.path.join(tmp, "rec_noheight")
rec = Recorder(rec_dir)
for i in range(int(7 * FPS)):
    t = T0 + i / FPS
    img = render(ground, K, (W, H), pose, t, readout_s=config.READOUT_S, noise=2, rng=rng)
    rec.add(img, t, i == 0, HEIGHT if i < 2 * FPS else None, [s for s in imu if s[0] <= t])
rec.close()
out = io.StringIO()
with contextlib.redirect_stdout(out):
    rows = replay(rec_dir, "noheight")
text = out.getvalue()
ok_no_pos = sum(r["status"] == "OK" and r["north_m"] == "nan" for r in rows)
fired = [l for l in text.splitlines() if l.startswith("FAILSAFE")]
print(f"  frames with status OK but no position: {ok_no_pos}; failsafe messages: {fired[:2]}")
assert ok_no_pos > 2 * FPS, "health should still say OK while the height is missing"
assert any("-> LAND" in l for l in fired), "failsafe must fire when no valid position is sent"
print("\nALL REPLAY TESTS PASSED")
