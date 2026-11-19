# =====================================================================================
# tests/test_gnss_monitor.py  -  jammed or lying GPS is flagged, and NOTHING else changes
#                                                                     (OWNER: Ayush, task A5)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Vision says we hold still (a few cm of noise). We feed the monitor GPS reports:
#     honest GPS, 10 minutes with realistic 1.5 m wander     -> 0 false alarms
#     no 3D fix / 4 satellites / HDOP 4 / reports stop        -> JAMMED, each with its reason
#     the position jumps 300 m                                -> SPOOF SUSPECTED at once
#     the position is dragged away at 0.5 m/s (a "slow" spoof) -> SPOOF SUSPECTED in ~23 s
#     the same drag while vision is LOST                      -> no verdict (can't judge)
#     the drone really moves 3 m (GPS and vision agree)       -> no alarm
#   Then: it reads Saumya's link cache with no change to drone_link.py, asks only for the
#   GPS stream, and contains no steering call at all (checked in its source code).
#   Finally the real vision_hold.py replays a flight while a fake GPS teleports: the side
#   log must say SPOOF SUSPECTED and every ODOMETRY message must be identical to the run
#   with the monitor OFF.
#
# Run (from the repo root):  python -m tests.test_gnss_monitor      (~15 s)
# REFERENCES: safety/gnss_monitor.py
# =====================================================================================

import ast
import contextlib
import csv
import io
import math
import os
import tempfile

import cv2
import numpy as np

import config
from safety import gnss_monitor as gm

LAT0, LON0 = 45.4215, -75.6972
M_PER_DEG_LAT = math.pi / 180 * gm.EARTH_R


def gps(n, e, fix=3, sats=14, hdop=0.9):
    lat = LAT0 + n / M_PER_DEG_LAT
    lon = LON0 + e / (M_PER_DEG_LAT * math.cos(math.radians(LAT0)))
    return fix, sats, None if hdop is None else int(hdop * 100), int(lat * 1e7), int(lon * 1e7)


def fly(seconds, gps_at, vision_at=lambda t: (0.0, 0.0), trusted_at=lambda t: True, mon=None,
        gps_hz=5, fps=30, reset_at=lambda t: 1):
    """Run the monitor: GPS reports at gps_hz, frames at fps. Returns (monitor, first SPOOF time)."""
    mon = mon or gm.GnssMonitor()
    rng = np.random.default_rng(0)
    next_gps, first_spoof = 0.0, None
    for i in range(int(seconds * fps)):
        t = i / fps
        if t >= next_gps:
            g = gps_at(t)
            if g is not None:
                mon.feed(t, *g)
            next_gps += 1 / gps_hz
        vn, ve = vision_at(t)
        vn, ve = vn + rng.normal(0, 0.03), ve + rng.normal(0, 0.03)
        ok = trusted_at(t)
        mon.update(t, vn if ok else float("nan"), ve if ok else float("nan"), 80 if ok else -1,
                   "OK" if ok else "LOST", reset_at(t))
        if mon.state == gm.SPOOF and first_spoof is None:
            first_spoof = t
    return mon, first_spoof


print("Test 1: honest GPS for 10 minutes (1.5 m random wander, HDOP 0.9, 14 satellites)")
rng = np.random.default_rng(42)
wn = we = 0.0
noise = []
for _ in range(10 * 60 * 5):                                    # AR(1): GPS error drifts slowly
    wn = 0.98 * wn + rng.normal(0, 1.5 * math.sqrt(1 - 0.98 ** 2))
    we = 0.98 * we + rng.normal(0, 1.5 * math.sqrt(1 - 0.98 ** 2))
    noise.append((wn, we))
mon, spoof = fly(600, lambda t: gps(*noise[min(int(t * 5), len(noise) - 1)]))
alarms = [x for x in mon.transitions if x[1] in (gm.SPOOF, gm.JAMMED)]
print(f"  GPS wander RMS {math.sqrt(np.mean(np.square(noise))) * math.sqrt(2):.2f} m; "
      f"final state {mon.state}; false alarms {len(alarms)}")
assert mon.state == gm.OK and not alarms, alarms

print("\nTest 2: jamming, each with its reason")
for label, g in (("no 3D fix", gps(0, 0, fix=1)), ("4 satellites", gps(0, 0, sats=4)),
                 ("HDOP 4.0", gps(0, 0, hdop=4.0))):
    m, _ = fly(3, lambda t, g=g: gps(0, 0) if t < 1 else g)
    print(f"  {label:13s} -> {m.state}: {m.reason}")
    assert m.state == gm.JAMMED
m, _ = fly(5, lambda t: gps(0, 0) if t < 1 else None)
print(f"  {'reports stop':13s} -> {m.state}: {m.reason}")
assert m.state == gm.JAMMED and "no GPS report" in m.reason
assert gm.GnssMonitor().update(0, 0, 0, 80, "OK", 1)["gnss_state"] == gm.NO_DATA

print("\nTest 3: spoofing")
m, t_sp = fly(20, lambda t: gps(0, 0) if t < 10 else gps(300, 0))
print(f"  300 m teleport at t = 10.0 s -> {gm.SPOOF} at t = {t_sp:.1f} s: {m.spoof_reason}")
assert m.state == gm.SPOOF and 10.0 <= t_sp < 10.5
m, t_sp = fly(50, lambda t: gps(0, 0) if t < 10 else gps(0.5 * (t - 10), 0))
print(f"  0.5 m/s drag from t = 10 s -> {gm.SPOOF} at t = {t_sp:.1f} s ({t_sp - 10:.1f} s later): {m.spoof_reason}")
assert m.state == gm.SPOOF and 18 <= t_sp - 10 <= 30
m, t_sp = fly(50, lambda t: gps(0, 0) if t < 10 else gps(0.5 * (t - 10), 0), trusted_at=lambda t: t < 10)
print(f"  same drag, vision LOST       -> {m.state} (no drift verdict without trusted vision)")
assert t_sp is None
m, t_sp = fly(40, lambda t: gps(min(3.0, 0.2 * max(0, t - 10)), 0),
              vision_at=lambda t: (min(3.0, 0.2 * max(0, t - 10)), 0.0))
print(f"  drone really moves 3 m       -> {m.state} (GPS and vision agree)")
assert t_sp is None and m.state == gm.OK
m, t_sp = fly(40, lambda t: gps(0 if t < 15 else 4.0, 0), vision_at=lambda t: (0.0, 0.0) if t < 15 else (4.0, 0.0),
              reset_at=lambda t: 1 if t < 15 else 2)
print(f"  new home saved at 15 s       -> {m.state} (re-anchored, no false alarm)")
assert t_sp is None

print("\nTest 4: reads Saumya's link cache, asks only for the GPS stream, never steers")


class Msg:
    def __init__(self, t, n=0.0):
        self.fix_type, self.satellites_visible, self.eph, self.lat, self.lon = gps(n, 0)
        self.time_usec = int(t * 1e6)


class FakeLink:
    def __init__(self):
        self.conn = type("Conn", (), {})()
        self.conn.messages = {}
        self.commands = []

    def command(self, *a):
        self.commands.append(a)


link = FakeLink()
mon = gm.GnssMonitor.attach(link)
for i in range(10):
    link.conn.messages["GPS_RAW_INT"] = Msg(i * 0.2)
    mon.update(i * 0.2, 0.0, 0.0, 80, "OK", 1)
print(f"  commands sent: {link.commands}  (511 = SET_MESSAGE_INTERVAL, 24 = GPS_RAW_INT, 200000 us = 5 Hz)")
assert link.commands == [(gm.MAV_CMD_SET_MESSAGE_INTERVAL, 24, 200000)] and mon.state == gm.OK
tree = ast.parse(open(gm.__file__).read())
calls = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
steering = {c for c in calls if c.endswith("_send") or c in ("set_source", "set_mode", "send_odometry", "arducopter_arm")}
print(f"  steering calls in gnss_monitor.py: {sorted(steering) or 'none'}")
assert not steering

print("\nTest 5: real vision_hold.py replay, fake GPS teleports 300 m: monitor shouts, ODOMETRY unchanged")
import tools.replay as replay_mod
import vision_hold
from tests.common import K, use_test_camera
from tools.degrade_video import make_ground, render
from tools.replay import Recorder

use_test_camera()
tmp = tempfile.mkdtemp(prefix="dronecode_gnss_")
ground = make_ground(3000)
rng = np.random.default_rng(9)
rec_dir = os.path.join(tmp, "rec")
rec = Recorder(rec_dir)
pose = lambda t: (0.05 * math.sin(t - 1000), 0.0, 8.0, 0.0, 0.0, 0.0)
imu = [(t, 0.0, 0.0, 0.0) for t in np.arange(999.5, 1004.0, 0.02)]
for i in range(105):
    t = 1000 + i / 30
    img = render(ground, K, (640, 480), pose, t, noise=2, rng=rng)
    rec.add(cv2.cvtColor(img, cv2.COLOR_GRAY2BGR), t, i == 0, 8.0, [s for s in imu if s[0] <= t])
rec.close()

sent = []
real_send, real_init, real_read = replay_mod._ReplayConn.odometry_send, replay_mod._ReplayConn.__init__, replay_mod.Replay.read


def init_with_cache(self):
    real_init(self)
    self.messages = {}                     # like pymavlink's cache


def read_with_gps(self):
    ok, frame = real_read(self)
    if ok:
        self.link.conn.messages["GPS_RAW_INT"] = Msg(self.t, 0.0 if self.t < 1002.0 else 300.0)
    return ok, frame


replay_mod._ReplayConn.odometry_send = lambda self, *a: sent.append(a)
replay_mod._ReplayConn.__init__ = init_with_cache
replay_mod.Replay.read = read_with_gps
DEFAULTS = {k: getattr(config, k) for k in dir(config) if k.isupper()}


def run(on):
    for k, v in DEFAULTS.items():
        setattr(config, k, v)
    config.GNSS_MONITOR = on
    sent.clear()
    log = os.path.join(tmp, f"flight_{on}.csv")
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        vision_hold.main(["--replay", rec_dir, "--log", log])
    return list(sent), log, out.getvalue()


def same(a, b):
    flat = lambda m: np.array([float(v) for x in m for v in (np.ravel(x) if np.ndim(x) else [x])], float)
    return len(a) == len(b) and all(np.array_equal(flat(x), flat(y), equal_nan=True) for x, y in zip(a, b))


on_msgs, on_log, on_text = run(True)
off_msgs, _, _ = run(False)
replay_mod._ReplayConn.odometry_send, replay_mod._ReplayConn.__init__, replay_mod.Replay.read = real_send, real_init, real_read
with open(os.path.splitext(on_log)[0] + "_safety.csv", newline="") as f:
    states = [row["gnss_state"] for row in csv.DictReader(f)]
shout = [ln for ln in on_text.splitlines() if ln.startswith("GNSS: SPOOF")]
n = len(on_msgs) - 5
print(f"  side log states: {states.count('OK')} OK, {states.count(gm.SPOOF)} SPOOF SUSPECTED")
print(f"  console: {shout[0][:110] if shout else 'none'}")
print(f"  ODOMETRY: {len(on_msgs)} vs {len(off_msgs)} messages, in-flight identical: {same(on_msgs[:n], off_msgs[:n])}")
assert gm.SPOOF in states and "OK" in states and shout
assert n > 50 and same(on_msgs[:n], off_msgs[:n]) and len(on_msgs) == len(off_msgs)

print("\nALL GNSS MONITOR TESTS PASSED")
