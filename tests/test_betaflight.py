# =====================================================================================
# tests/test_betaflight.py  -  the Betaflight route, against the Stage 2 rules
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We have no Betaflight aircraft and the organisers give no simulator, so this test
#   plays the flight controller: a pretend serial cable that records every message we
#   send and answers with tilt and radio channels. It checks what the judge will check on
#   the bench (Stage 2 section 05) and the safety rule (section 03):
#     1. our messages are well-formed, and a GPS request cannot even be built
#     2. not ready            -> roll, pitch, yaw exactly 1500, throttle never minimum
#     3. ready + engaged      -> roll and pitch push AGAINST the move, all four directions
#     4. vision goes silent   -> back to 1500 within 0.3 s (the timeout Betaflight lacks)
#     5. engage switch        -> no stale command at the engage moment; home is re-saved
#     6. throttle             -> the pilot's value at engage, nudged to hold the height, bounded
#     7. the lean we ask for is capped, even if something asks for more
#     8. the onboard barometer / gyro files are read correctly (a pretend sysfs folder)
#     9. nothing we ever sent was a GPS request
#   SIMULATED: it proves logic and signs as WE understand Betaflight. The real aircraft
#   must confirm the stick directions and the tilt signs (config.py BF_* settings).
#
# RUN:  python -m tests.test_betaflight      (~1 s)
# REFERENCE: Red Team Grand Final Stage 2 package, Rev 2.0, sections 03, 04, 05.
# =====================================================================================
import math
import os
import struct
import tempfile

import config
from fusion import position_controller as pc
from link import msp, stick_curve
from link.apb_sensors import Baro, Gyro, pressure_to_height
from link.betaflight_link import BetaflightLink
from link.msp_sender import Sender


def reply(command, payload):
    body = bytes([len(payload), command]) + payload
    check = 0
    for b in body:
        check ^= b
    return b"$M>" + body + bytes([check])


class FakeFC:
    """Pretend serial cable + Betaflight. Remembers every request; answers attitude and channels."""

    def __init__(self):
        self.sent, self.buf, self._in = [], b"", b""
        self.roll, self.pitch, self.heading = 2.0, -3.0, 90          # degrees, Betaflight's own signs
        self.rc = [1500, 1500, 1500, 1420, 1000, 1500, 1500, 1500, 1500, 1000, 1500, 1500]   # R P Y T, CH5 arm, CH10 engage
        self.status = None                                           # None = an FC that never answers MSP_STATUS

    def write(self, data):
        self._in += data
        while len(self._in) >= 6 and len(self._in) >= 6 + self._in[3]:
            n = 6 + self._in[3]
            msg, self._in = self._in[:n], self._in[n:]
            command, payload = msg[4], msg[5:-1]
            self.sent.append((command, payload))
            if command == msp.MSP_ATTITUDE:
                self.buf += reply(command, struct.pack("<3h", int(self.roll * 10), int(self.pitch * 10), self.heading))
            elif command == msp.MSP_RC:
                self.buf += reply(command, struct.pack(f"<{len(self.rc)}H", *self.rc))
            elif command == msp.MSP_SET_RAW_RC:
                self.buf += reply(command, b"")
            elif command == msp.MSP_STATUS and self.status is not None:
                self.buf += reply(command, struct.pack("<3HI", 126, 0, 43, self.status) + bytes(16))

    def read(self, n):
        out, self.buf = self.buf, b""
        return out

    def sticks(self):
        """The last stick message, as (roll, pitch, throttle, yaw)."""
        last = [p for c, p in self.sent if c == msp.MSP_SET_RAW_RC][-1]
        return struct.unpack("<8H", last)[:4]


class FakeBaro:
    def __init__(self):
        self.p0, self.h = None, 0.0

    def zero(self):
        self.p0 = 1.0

    def height(self):
        return self.h if self.p0 else None


def run(sender, t0, seconds):
    t = t0
    while t < t0 + seconds:
        sender.step(t)
        t += 0.02
    return t


print("Test 1: messages are well-formed; a GPS request cannot be built")
f = msp.set_raw_rc(1500, 1490, 1300, 1500)
assert f[:3] == b"$M<" and f[3] == 16 and f[4] == 200 and len(f) == 22
check = 0
for b in f[3:-1]:
    check ^= b
assert check == f[-1], "checksum = XOR of size, command, payload"
A = config.BF_AUX_US                                                # BETAFLIGHT CHANGE: AUX "off" (1000), not 1500
assert struct.unpack("<8H", f[5:-1]) == (1500, 1490, 1300, 1500, A, A, A, A), "order roll, pitch, throttle, yaw"
for gps in (106, 107, 164, 166):                                    # MSP_RAW_GPS, COMP_GPS, GPSSVINFO, GPSSTATISTICS
    try:
        msp.request(gps)
        raise AssertionError("a GPS request must be refused")
    except ValueError:
        pass
p = msp.Parser()
got = p.feed(b"junk" + reply(108, struct.pack("<3h", 25, -40, 270))[:7])       # half a message...
got += p.feed(reply(108, struct.pack("<3h", 25, -40, 270))[7:] + b"$M>\x01\x69\x00\xff")   # ...the rest + a bad checksum
assert got == [(108, struct.pack("<3h", 25, -40, 270))] and msp.parse_attitude(got[0][1]) == (2.5, -4.0, 270.0)
print("  22-byte stick message, checksum right, GPS commands refused, parser survives junk and split messages")

print("\nTest 2: not ready -> centred sticks, throttle never minimum (bench step 6)")
fc, baro = FakeFC(), FakeBaro()
s = Sender(fc, baro)
s.step(0.0)
assert fc.sticks() == (1500, 1500, config.BF_HOVER_THROTTLE, 1500), "very first message, before any reply: safe values"
t = run(s, 0.02, 0.5)
assert fc.sticks() == (1500, 1500, 1420, 1500), "disengaged: centred, and the pilot's own throttle"
fc.rc[9] = 2000                                                     # engage, vision silent
t = run(s, t, 0.5)
assert fc.sticks() == (1500, 1500, 1420, 1500), "engaged but not ready: 1500s, throttle that holds height"
print(f"  first message {(1500, 1500, config.BF_HOVER_THROTTLE, 1500)}, then {fc.sticks()} (pilot's throttle 1420)")

print("\nTest 3: ready + engaged -> roll and pitch oppose the move (bench step 5)")
moves = {"right": (0.0, 0.5), "left": (0.0, -0.5), "forward": (0.5, 0.0), "back": (-0.5, 0.0)}
for name, (fwd, right) in moves.items():
    roll, pitch = pc.sticks(fwd, right, 0.0, 0.0)
    s.set_target(roll, pitch, t)
    s.step(t)
    r, pch, thr, yaw = fc.sticks()
    print(f"  aircraft moved 0.5 m {name:7s} -> roll {r}, pitch {pch}, yaw {yaw}")
    assert yaw == 1500
    assert (r < 1500) if name == "right" else (r > 1500) if name == "left" else r == 1500
    assert (pch < 1500) if name == "forward" else (pch > 1500) if name == "back" else pch == 1500
    t += 0.02
roll, pitch = pc.sticks(0.0, 0.0, 0.0, 0.8)                         # at home but sliding right: brake left
assert roll < 1500 and pitch == 1500

print("\nTest 4: vision goes silent -> 1500s within the timeout (the timeout Betaflight lacks)")
s.set_target(1450, 1540, t)
s.step(t)
assert fc.sticks()[:2] == (1450, 1540)
t = run(s, t + 0.02, config.BF_TARGET_TIMEOUT_S + 0.04)
assert fc.sticks()[:2] == (1500, 1500), "stale request must be dropped"
fc.rc[9] = 1000                                                     # pilot switches off: nothing of ours survives
s.set_target(1450, 1540, t)
t = run(s, t, 0.1)
assert fc.sticks()[:2] == (1500, 1500)
print(f"  centred {config.BF_TARGET_TIMEOUT_S} s after the last request; centred whenever the switch is off")

# REVIEW M2: the wall clock jumping BACK must not revive a dead request (freshness is on its own clock)
fc.rc[9] = 2000
s.step(t, fresh_now=100.0)
s.set_target(1450, 1550, 100.0)
s.step(t, fresh_now=100.1)
assert fc.sticks()[:2] == (1450, 1550)
s.step(t - 3600.0, fresh_now=101.0)                                 # wall clock 1 h back, request 1 s old
assert fc.sticks()[:2] == (1500, 1500), "stale request used after a clock jump"
# REVIEW M3: a NaN request is ignored, never turned into the cap
s.set_target(float("nan"), float("nan"), 101.0)
s.step(t, fresh_now=101.01)
assert fc.sticks()[:2] == (1500, 1500) and pc.sticks(float("nan"), 0.0, 0.0, 0.0) == (1500, 1500)
print("  wall clock jumps back 1 h: still centred; NaN request: centred")
fc.rc[9] = 1000
t = run(s, t, 0.1)

print("\nTest 5: the lean is capped twice (controller, then sender)")
# BETAFLIGHT CHANGE: the cap is in DEGREES of real lean, through Betaflight's own stick curve (link/stick_curve.py)
lean = lambda us: abs(stick_curve.angle_from_us(us))
most = config.BF_MAX_TILT_DEG + 0.1
roll, pitch = pc.sticks(50.0, -50.0, 0.0, 0.0)                      # 50 m away: still only BF_MAX_TILT_DEG
assert lean(roll) <= most and lean(pitch) <= most
fc.rc[9] = 2000
t = run(s, t, 0.1)
s.set_target(1000, 2000, t)                                         # something asks for full stick
s.step(t)
r, pch = fc.sticks()[:2]
assert lean(r) <= most and lean(pch) <= most and r > 1000 and pch < 2000 and lean(r) > config.BF_MAX_TILT_DEG - 0.2
print(f"  asked for (1000, 2000), sent ({r}, {pch}) = {lean(r):.1f} deg of real lean "
      f"(cap {config.BF_MAX_TILT_DEG:g} deg, Betaflight curve, angle_limit {config.BF_ANGLE_LIMIT_DEG:g})")

print("\nTest 6: throttle = pilot's value at engage, nudged to hold the engage height, bounded")
fc2, baro2 = FakeFC(), FakeBaro()
s2 = Sender(fc2, baro2)
fc2.rc[4] = 2000                                                    # armed: the barometer keeps its zero
baro2.zero()
baro2.h = 10.0
t = run(s2, 0.0, 2.0)
fc2.rc[3] = 1380                                                    # the pilot's hover throttle
t = run(s2, t, 0.3)
fc2.rc[9] = 2000                                                    # engage at 10 m
t = run(s2, t, 0.5)
level = fc2.sticks()[2]
fc2.rc[3] = 1000                                                    # (the FC now reports OUR throttle: must not be learned)
baro2.h = 9.0                                                       # sank 1 m
t = run(s2, t, 2.0)
low = fc2.sticks()[2]
baro2.h = 30.0                                                      # absurd reading: authority is limited
t = run(s2, t, 2.0)
high = fc2.sticks()[2]
print(f"  at the engage height {level}, 1 m too low {low}, sensor says +20 m {high} (pilot 1380, authority +-{config.BF_THROTTLE_AUTHORITY})")
assert level == 1380 and 1380 < low <= 1380 + config.BF_THROTTLE_AUTHORITY
assert high == 1380 - config.BF_THROTTLE_AUTHORITY >= config.BF_THROTTLE_MIN
s3 = Sender(FakeFC(), None)                                         # no barometer on this board: pilot's value only
s3.port.rc[9] = 2000
run(s3, 0.0, 0.3)
assert s3.port.sticks()[2] == 1420

print("\nTest 7: tilt and channels reach the vision side in OUR conventions")
tele = s.step(t)
_, roll, pitch, yaw = tele["att"]
print(f"  Betaflight says roll +2.0, pitch -3.0, heading 90 -> ours roll {math.degrees(roll):+.1f}, "
      f"pitch {math.degrees(pitch):+.1f} (nose up +), yaw {math.degrees(yaw):.0f}")
assert abs(math.degrees(roll) - 2.0) < 0.01 and abs(math.degrees(pitch) - 3.0) < 0.01 and abs(math.degrees(yaw) - 90) < 0.5

print("\nTest 8: the link: engage -> POSHOLD, no stale command at engage, bench height when disarmed")
link = BetaflightLink()
att = (0.0, 0.0, math.radians(40))                                  # heading must not matter
base = dict(t=1.0, att=(1.0, 0.0, 0.0, att[2]), alt=None, rc=list(FakeFC().rc))
link.on_telemetry(dict(base, engaged=False))
assert link.conn.flightmode == "MANUAL" and link.bench_height() == config.BENCH_HEIGHT_M
n, e = 2.0 * math.cos(att[2]), 2.0 * math.sin(att[2])               # 2 m FORWARD of the old home
link.send_odometry(1.0, n, e, 0.0, att, (0, 0, 0), 90, reset_counter=4)
assert link.last_sticks == (1500, 1500), "switch off: we ask for nothing"
link.on_telemetry(dict(base, engaged=True))
assert link.conn.flightmode == "POSHOLD", "handover.py sees a hold mode -> saves a new home"
link.send_odometry(1.1, n, e, 0.0, att, (0, 0, 0), 90, reset_counter=4)
assert link.last_sticks == (1500, 1500), "old home still in use: no command yet"
link.send_odometry(1.2, 0.0, 0.0, 0.0, att, (0, 0, 0), 90, reset_counter=5)
assert link.last_sticks == (1500, 1500), "new home, zero error: centred"
link.send_odometry(1.3, 0.3 * math.cos(att[2]), 0.3 * math.sin(att[2]), 0.0, att, (0, 0, 0), 90, reset_counter=5)
assert link.last_sticks[0] == 1500 and link.last_sticks[1] < 1500, "0.3 m forward of home -> pitch back, no roll"
link.send_odometry(1.4, 0.3, 0.0, 0.0, att, (0, 0, 0), -1, reset_counter=5)
assert link.last_sticks == (1500, 1500), "quality <= 0 (camera covered): centred at once (bench step 7)"
armed = dict(base, engaged=True, alt=12.0)
armed["rc"][4] = 2000
link.on_telemetry(armed)
assert link.bench_height() is None and abs(link.height(1.0) - 12.0) < 1e-9, "armed: the barometer height is used"
print("  MANUAL -> POSHOLD on CH10; centred until the new home is saved; then opposes the move; centred when not ready")

import numpy as np                                                  # the camera's raw bytes -> grey picture
from compute.budget import to_gray
_w, _h = config.CAMERA_SIZE
_raw = np.zeros((_h, _w, 2), np.uint8)
_raw[:, :, 0], _raw[:, :, 1] = 77, 128                              # Y = 77 everywhere, colour bytes 128
for _f in (_raw, _raw.reshape(1, -1), np.full((_h, _w, 3), 77, np.uint8), np.full((_h, _w), 77, np.uint8)):
    _g = to_gray(_f)
    assert _g.shape == (_h, _w) and int(_g.min()) == int(_g.max()) == 77, _f.shape
print("  raw YUYV (2-channel or flat), colour and grey frames all give the same grey picture")

from compute.budget import LatestFrame as _LF                       # a slow camera must not slow the loop
class _SlowCam:
    n = 0
    def read(self):
        _time_mod.sleep(0.03); _SlowCam.n += 1
        return True, _SlowCam.n
    def release(self): pass
import time as _time_mod
_lf = _LF(_SlowCam())
_time_mod.sleep(0.2)
_t0 = _time_mod.time(); _ok, _f1 = _lf.read(); _dt = _time_mod.time() - _t0
_ok, _f2 = _lf.read()
assert _ok and _dt < 0.01 and _f1 >= 5 and _f2 == _f1 + 1 and _lf.t > 0, (_dt, _f1, _f2)
_lf.release()
print("  camera on a side thread: newest frame at once (frame %d after 0.2 s), then the next one, never a repeat" % _f1)

import time as _time                                                # a slow barometer must not slow the caller
from link.apb_sensors import Baro as _Baro
_b = _Baro(root="no_such_folder")
_b.dir = "x"
_b._measure = lambda: (_time.sleep(0.5), 101.3)[1]                  # like the real one: blocks
_b.start()
_time.sleep(0.7)
_t = _time.time()
assert _b.pressure() == 101.3 and _time.time() - _t < 0.05
print("  barometer read that blocks 0.5 s: pressure() answers in under 50 ms")

import preflight                                                    # the GO / NO-GO list's Betaflight check
no_sleep = lambda s: None
silent = BetaflightLink()
assert preflight.check_betaflight(silent, 0, no_sleep).level == preflight.NOGO, "sender not running"
silent.on_telemetry(dict(t=1.0, att=None, alt=None, rc=[], engaged=False))
assert preflight.check_betaflight(silent, 0, no_sleep).level == preflight.NOGO, "sender runs, FC silent"
good = preflight.check_betaflight(link, 0, no_sleep)
assert good.level == preflight.GO, good
print(f"  preflight: sender missing -> NO-GO, FC silent -> NO-GO, both alive -> GO ({good.reason[:40]}...)")

print("\nTest 9: onboard barometer and gyro files (pretend sysfs)")
root = tempfile.mkdtemp(prefix="dronecode_iio_")
for i, (name, files) in enumerate((("icm42605-gyro", {"in_anglvel_scale": "0.001", "in_anglvel_x_raw": "100",
                                                      "in_anglvel_y_raw": "-50", "in_anglvel_z_raw": "0"}),
                                   ("dps310", {"in_pressure_input": "101.325"}))):
    d = os.path.join(root, f"iio_device{i}")          # real name: iio:device0 (":" is not allowed on Windows)
    os.makedirs(d)
    for k, v in dict(files, name=name).items():
        with open(os.path.join(d, k), "w") as fh:
            fh.write(v + "\n")
b, g = Baro(root), Gyro(root)
b.zero()
assert abs(b.height()) < 1e-6 and g.rates() == (0.1, -0.05, 0.0)
with open(os.path.join(b.dir, "in_pressure_input"), "w") as fh:
    fh.write("101.205\n")                                           # 0.12 kPa lower = about 10 m higher
print(f"  pressure 101.325 -> 101.205 kPa = {b.height():.1f} m higher; gyro {g.rates()} rad/s")
assert 9.5 < b.height() < 10.5 and abs(pressure_to_height(89.87, 101.325) - 1000) < 15
assert Baro(os.path.join(root, "nothing")).height() is None and Gyro(os.path.join(root, "nothing")).rates() is None

# MEASURED 21 Nov 2026: radio off -> arm channel 1775, but the flight controller says DISARMED. Its word wins.
fc4 = FakeFC(); fc4.rc[4] = 1775; fc4.status = 0
s4 = Sender(fc4, FakeBaro())
tele4 = None
for _i in range(30):
    tele4 = s4.step(1000.0 + _i * 0.02)
assert tele4["armed"] is False
bl = BetaflightLink(); bl.on_telemetry(dict(tele4, att=None))
assert bl.bench_height() == config.BENCH_HEIGHT_M
fc4.status = 1
for _i in range(30, 60):
    tele4 = s4.step(1000.0 + _i * 0.02)
bl.on_telemetry(dict(tele4, att=None))
assert tele4["armed"] is True and bl.bench_height() is None
print("  radio off (arm channel 1775) but FC says disarmed: bench height used; FC says armed: barometer")

print("\nTest 10: nothing we ever sent asked for GNSS (Stage 2 section 04)")
commands = {c for port in (fc, fc2, s3.port) for c, _ in port.sent}
print(f"  MSP commands sent in this whole test: {sorted(commands)} (200 sticks, 108 tilt, 105 radio channels, 102 raw IMU, 101 armed)")
assert commands - {msp.MSP_STATUS} == {msp.MSP_SET_RAW_RC, msp.MSP_ATTITUDE, msp.MSP_RC, msp.MSP_RAW_IMU}
for port in (fc, fc2, s3.port):
    for c, payload in port.sent:
        if c == msp.MSP_SET_RAW_RC:
            r, pch, thr, yaw = struct.unpack("<8H", payload)[:4]
            assert yaw == 1500 and config.BF_THROTTLE_MIN <= thr <= config.BF_THROTTLE_MAX
            assert lean(r) <= most and lean(pch) <= most, "never anywhere near minimum values"
print("\nALL BETAFLIGHT TESTS PASSED (simulated flight controller: confirm signs on the real aircraft)")
