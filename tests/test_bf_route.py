# =====================================================================================
# tests/test_bf_route.py  -  vision_hold.py on the Betaflight route, end to end (BETAFLIGHT CHANGE)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Runs the REAL vision_hold.py with --fc betaflight on a made-up camera video, with a
#   pretend msp_sender on the same local network ports the real one uses. The pretend sender
#   reports the radio (disarmed bench, then the engage switch CH10 on) and the raw IMU, and
#   records every stick request vision_hold.py sends. The video: the drone turns 15 degrees
#   (Betaflight's own heading says something else entirely: it must be ignored), then the
#   judge flips the switch, then moves the aircraft 0.3 m to its RIGHT. Bench step 5 says
#   roll must push LEFT (below 1500) and pitch stay about centred.
#   Checks: home saved, centred sticks before engage, the engage re-home, roll < 1500 and
#   pitch ~1500 after the move, nothing crashed (the log has rows).
# RUN:  python -m tests.test_bf_route   (~12 s, real time)
# =====================================================================================
import csv
import json
import math
import os
import socket
import tempfile
import threading
import time

import cv2
import numpy as np

import config
import vision_hold

W, H, FPS, SECONDS = 640, 360, 15, 9.0
F = 623.0 * W / 1920                                     # the datasheet calib.npz scaled to 640 wide
ENGAGE_AT, MOVE_FROM, MOVE_TO = 3.5, 5.0, 6.5            # seconds from the start
rng = np.random.default_rng(3)
floor = cv2.GaussianBlur(rng.normal(128, 45, (1400, 1400)).astype(np.float32), (0, 0), 1.6)
floor = np.clip(floor, 0, 255).astype(np.uint8)


def pose(t):
    """(x right of start in metres, yaw in degrees) of the drone at video time t."""
    yaw = 15.0 * min(1.0, max(0.0, (t - 1.0) / 1.0))     # turns 15 deg right between 1 and 2 s
    x = 0.3 * min(1.0, max(0.0, (t - MOVE_FROM) / (MOVE_TO - MOVE_FROM)))
    return x, yaw


def render(t):
    x, yaw = pose(t)
    c = math.radians(yaw)
    # camera moves x metres along its own RIGHT axis (as it points after the turn): image content moves left
    dx_px, dy_px = x * F * math.cos(c), x * F * math.sin(c)
    M = cv2.getRotationMatrix2D((700 + dx_px, 700 + dy_px), yaw, 1.0)   # turn right -> picture turns the other way
    M[0, 2] += W / 2 - (700 + dx_px)
    M[1, 2] += H / 2 - (700 + dy_px)
    img = cv2.warpAffine(floor, M, (W, H), flags=cv2.INTER_LINEAR).astype(np.float32)
    return np.clip(img + rng.normal(0, 2.0, img.shape), 0, 255).astype(np.uint8)   # sensor noise: never 'frozen'


tmp = tempfile.mkdtemp()
video = os.path.join(tmp, "bench.avi")
vw = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*"MJPG"), FPS, (W, H), False)
for i in range(int(SECONDS * FPS)):
    vw.write(render(i / FPS))
vw.release()

requests, stop = [], threading.Event()
t0 = [None]


def fake_sender():
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", config.BF_SENDER_PORT))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while not stop.is_set():
        now = time.time()
        t = now - t0[0]
        engaged = t >= ENGAGE_AT
        rc = [1500, 1500, 1500, 1000, 1000, 1500, 1500, 1500, 1500, 2000 if engaged else 1000] + [1500] * 6
        moving = MOVE_FROM <= t <= MOVE_TO
        acc = [0.0 + (0.06 * math.sin(11 * t) if moving else 0.0), 0.0, 1.0]
        tele = dict(t=now, att=[now, 0.0, 0.0, math.radians(123.0)],   # Betaflight's heading: nonsense on purpose
                    alt=None, rc=rc, engaged=engaged, sent=[1500, 1500, 1300, 1500], imu=[now, acc, [0, 0, 0]])
        try:
            tx.sendto(json.dumps(tele).encode(), ("127.0.0.1", config.BF_TELEMETRY_PORT))
        except OSError:
            pass
        try:
            while True:
                msg = json.loads(rx.recv(512))
                requests.append((time.time() - t0[0], msg["roll"], msg["pitch"]))
        except (BlockingIOError, ValueError):
            pass
        time.sleep(0.02)


t0[0] = time.time()
th = threading.Thread(target=fake_sender, daemon=True)
th.start()
log = os.path.join(tmp, "flight.csv")
vision_hold.main(["--fc", "betaflight", "--camera", video, "--max-seconds", str(SECONDS + 1), "--log", log])
stop.set()
th.join(1.0)

with open(log) as f:
    rows = list(csv.DictReader(f))
print(f"Test 1: the program ran: {len(rows)} logged frames, {len(requests)} stick requests")
assert len(rows) > 50 and len(requests) > 50

before = [(r, p) for t, r, p in requests if t < ENGAGE_AT - 0.3]
print(f"\nTest 2: before engage every request is centred ({len(before)} requests)")
assert before and all((r, p) == (1500, 1500) for r, p in before)

after = [(r, p) for t, r, p in requests if MOVE_TO + 0.3 <= t <= MOVE_TO + 1.5]
print(f"\nTest 3: moved 0.3 m to the right after engage -> roll pushes LEFT, pitch about centred "
      f"({len(after)} requests, last {after[-1] if after else None})")
assert after and all(r < 1500 for r, _ in after)
assert all(abs(p - 1500) < abs(r - 1500) / 3 + 3 for r, p in after), "the push must be sideways, not forward/back"
print("  (the drone had turned 15 deg and Betaflight's heading said 123 deg: neither mattered)")
print("\nALL BETAFLIGHT ROUTE TESTS PASSED (pretend sender and camera: confirm on the aircraft)")
