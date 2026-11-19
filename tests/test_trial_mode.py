# =====================================================================================
# tests/test_trial_mode.py  -  in trial mode, GPS can NEVER take over   (OWNER: Ayush, task A1)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Test 1: who decides trial vs practice (config switch, --practice flag).
#   Test 2: we build Saumya's real DroneLink, catch every BYTE it would put on the radio,
#           and try every way of asking for GPS (set_source, command, raw COMMAND_LONG,
#           COMMAND_INT, parameter writes, RC override). In trial mode none of them may
#           leave; "use vision" still must. In practice mode the same calls DO leave, which
#           proves the test can see them.
#   Test 3: the real vision_hold.py runs a recorded flight with a 1.5 s camera blackout
#           (vision LOST) and then shuts down: the two places where Saumya's logic asks for
#           GPS. Trial mode (the default): zero GPS requests. --practice: GPS is requested.
#
# Run (from the repo root):  python -m tests.test_trial_mode      (~20 s)
# REFERENCES: safety/trial_mode.py
# =====================================================================================

import os
import tempfile

import cv2
import numpy as np

import config
from link.drone_link import DroneLink, SRC_GPS, SRC_VISION   # sets MAVLink 2 before pymavlink loads
from pymavlink import mavutil
from safety import trial_mode
from safety.trial_mode import MAV_CMD_SET_EKF_SOURCE_SET

# ---------------- Test 1: the decision ----------------
print("Test 1: trial or practice?")
cases = [((False, True), True), ((True, True), False), ((False, False), False), ((True, False), False)]
for (practice, cfg), expect in cases:
    t = trial_mode.decide(practice, cfg)
    print(f"  --practice={practice!s:5s} config.TRIAL_MODE={cfg!s:5s} -> trial {t.active}  ({t.reason})")
    assert t.active == expect


class Args:
    practice, no_gps_fallback = False, False


a = Args()
trial_mode.decide(False, True).apply_to_args(a)
assert a.no_gps_fallback, "trial mode must set --no-gps-fallback"
assert config.TRIAL_MODE is True, "config.py must ship with TRIAL_MODE = True"


# ---------------- Test 2: byte-level guard on Saumya's DroneLink ----------------
def radio_bytes(active):
    """Every way of asking for GPS, plus the things that must still work. Returns decoded messages."""
    link = DroneLink("udpout:127.0.0.1:14599")             # nothing listens: we only catch bytes
    wire = []
    link.conn.write = lambda buf: wire.append(bytes(buf))  # what would go on the radio
    trial = trial_mode.TrialMode(active, "test")
    trial.guard(link)
    ts, tc = link.conn.target_system, link.conn.target_component
    link.set_source(SRC_GPS)
    after_gps_pending = link.pending_source
    link.set_source(SRC_VISION)                                       # must still work
    link.set_source(3)
    link.command(MAV_CMD_SET_EKF_SOURCE_SET, 1)
    link.conn.mav.command_long_send(ts, tc, MAV_CMD_SET_EKF_SOURCE_SET, 0, 1, 0, 0, 0, 0, 0, 0)
    link.conn.mav.command_int_send(ts, tc, 0, MAV_CMD_SET_EKF_SOURCE_SET, 0, 0, 1, 0, 0, 0, 0, 0, 0)
    link.conn.mav.param_set_send(ts, tc, b"EK3_SRC2_POSXY", 3, 9)
    link.conn.mav.param_set_send(ts, tc, b"RC8_OPTION", 90, 9)
    link.conn.mav.rc_channels_override_send(ts, tc, *([0] * 7), 1900)
    link.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 30, 20000)   # normal traffic: allowed
    link.send_heartbeat()
    link.send_odometry(0.0, 1.0, 2.0, 0.0, (0, 0, 0), (0, 0, 0), 80, 1)
    link.conn.mav.param_set_send(ts, tc, b"VISO_DELAY_MS", 50, 9)           # harmless param: allowed
    parser = mavutil.mavlink.MAVLink(None)
    msgs = parser.parse_buffer(b"".join(wire)) or []
    link.conn.close()
    return msgs, trial, after_gps_pending


def gps_requests(msgs):
    bad = []
    for m in msgs:
        kind = m.get_type()
        if kind in ("COMMAND_LONG", "COMMAND_INT") and m.command == MAV_CMD_SET_EKF_SOURCE_SET and int(m.param1) != 2:
            bad.append(f"{kind} source set {int(m.param1)}")
        elif kind == "PARAM_SET" and trial_mode.blocked_param(m.param_id):
            bad.append(f"PARAM_SET {m.param_id}")
        elif kind == "RC_CHANNELS_OVERRIDE":
            bad.append("RC_CHANNELS_OVERRIDE")
    return bad


print("\nTest 2: every way of asking for GPS, caught at the radio (bytes decoded back to MAVLink)")
msgs_t, trial_t, pend_t = radio_bytes(True)
msgs_p, _, pend_p = radio_bytes(False)
kinds_t = [m.get_type() for m in msgs_t]
vision_t = [m for m in msgs_t if m.get_type() == "COMMAND_LONG" and m.command == MAV_CMD_SET_EKF_SOURCE_SET]
print(f"  trial mode   : {len(gps_requests(msgs_t))} GPS requests on the radio, {len(trial_t.blocked)} blocked, "
      f"{len(vision_t)} 'use vision' sent, ODOMETRY sent: {'ODOMETRY' in kinds_t}")
print(f"  practice mode: {len(gps_requests(msgs_p))} GPS requests on the radio (the test can see them)")
assert gps_requests(msgs_t) == [], gps_requests(msgs_t)
assert len(trial_t.blocked) == 8, trial_t.blocked
assert len(vision_t) == 1 and int(vision_t[0].param1) == SRC_VISION, "'use vision' must still go out"
assert "ODOMETRY" in kinds_t and "HEARTBEAT" in kinds_t, "normal traffic must still go out"
assert any(m.get_type() == "PARAM_SET" and m.param_id == "VISO_DELAY_MS" for m in msgs_t)
assert pend_t != SRC_GPS, "a blocked set_source must not even record GPS as pending"
assert len(gps_requests(msgs_p)) == 8 and pend_p == SRC_GPS, "practice mode must let them through"

# ---------------- Test 3: the real vision_hold.py, vision LOST and shutdown ----------------
import tools.replay as replay_mod
import vision_hold
from tests.common import K, use_test_camera
from tools.degrade_video import make_ground, render
from tools.replay import Recorder

use_test_camera()
W, H, FPS, HEIGHT, T0 = 640, 480, 30, 6.0, 1000.0
ground = make_ground(3000)
rng = np.random.default_rng(3)
tmp = tempfile.mkdtemp(prefix="dronecode_trial_")
rec_dir = os.path.join(tmp, "rec")


def pose(t):
    return (0.05 * np.sin(t - T0), 0.0, HEIGHT, 0.0, 0.0, 0.0)


rec = Recorder(rec_dir)
imu = [(t, 0.0, 0.0, 0.0) for t in np.arange(T0 - 0.5, T0 + 8.0, 0.02)]
for i in range(int(6 * FPS)):
    t = T0 + i / FPS
    dark = 2.0 <= i / FPS < 3.5                                     # 1.5 s camera blackout
    img = np.zeros((H, W), np.uint8) if dark else render(ground, K, (W, H), pose, t, noise=2, rng=rng)
    rec.add(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), t, i == 0, HEIGHT, [s for s in imu if s[0] <= t])
rec.close()

requests = []
real_set_source = replay_mod.ReplayLink.set_source


def recording_set_source(self, n):
    requests.append(n)
    real_set_source(self, n)


replay_mod.ReplayLink.set_source = recording_set_source
DEFAULTS = {k: getattr(config, k) for k in dir(config) if k.isupper()}


def run(*extra):
    for k, v in DEFAULTS.items():
        setattr(config, k, v)
    requests.clear()
    log = os.path.join(tmp, "log.csv")
    vision_hold.main(["--replay", rec_dir, "--log", log, *extra])
    with open(log) as f:
        statuses = [line.split(",")[1] for line in f.readlines()[1:]]
    return list(requests), statuses


print("\nTest 3: vision_hold.py on a recording with a 1.5 s blackout, then a normal shutdown")
req_trial, st = run()
req_prac, _ = run("--practice")
print(f"  frames LOST during the blackout: {st.count('LOST')}")
print(f"  trial mode (default): source requests {req_trial}  -> GPS (1) requested {req_trial.count(1)} times")
print(f"  --practice          : source requests {req_prac}  -> GPS (1) requested {req_prac.count(1)} times")
assert st.count("LOST") > 0, "the blackout must make vision LOST (else the test proves nothing)"
assert SRC_VISION in req_trial, "trial mode must still switch the autopilot to vision"
assert SRC_GPS not in req_trial, "trial mode must never ask for GPS"
assert SRC_GPS in req_prac, "practice mode should ask for GPS (proves the paths were exercised)"
replay_mod.ReplayLink.set_source = real_set_source

print("\nALL TRIAL-MODE TESTS PASSED")
