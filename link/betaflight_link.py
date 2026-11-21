# =====================================================================================
# link/betaflight_link.py  -  lets vision_hold.py fly a Betaflight aircraft unchanged
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   vision_hold.py was written for ArduPilot: it hands a position to "the link" and reads
#   tilt, height and the flight mode from it. This file is a second link with the same
#   handles, for Betaflight (the firmware the event aircraft ships with):
#     - "here is our position"  -> becomes stick requests (fusion/position_controller.py)
#       sent to the always-on sender program (link/msp_sender.py), which owns the cable.
#     - tilt, height, radio channels come back from that sender.
#     - "which mode is the pilot in?" -> POSHOLD while the engage switch (CH10) is on, so
#       Praise's handover code saves a fresh home at the engage moment, exactly as it does
#       when an ArduPilot pilot selects LOITER.
#   Not ready (quality <= 0) -> we simply send nothing: the sender centres the sticks by
#   itself after 0.3 s (and at once when we say so).
#
# HOW IT WORKS (technical)
#   Subclass of Saumya's DroneLink without its MAVLink connection (same trick as
#   tools/replay.ReplayLink), so attitude_at() and height() are her code.
#   A thread reads the sender's telemetry (UDP 127.0.0.1:BF_TELEMETRY_PORT, JSON) into
#   self.attitudes / self.local_z / self.rc. send_odometry() rotates the North/East
#   position and velocity into the drone's own axes with the SAME yaw vision_hold used to
#   make them (so the heading value cancels: no compass needed), runs the PD controller
#   and sends {"roll", "pitch"} to BF_SENDER_PORT.
#   ENGAGE GUARD: when the switch goes on, the position on hand is still measured from the
#   OLD home (possibly metres away). We send centred sticks until vision_hold has saved
#   the new home (it tells us by changing reset_counter).
#   No GNSS anywhere on this route: nothing here or in the sender can request it.
#   Proof: tests/test_betaflight.py.
#
# BETAFLIGHT CHANGES (21 Nov, after Stage 2 + the first hardware session)
#   - The controller is fusion/position_controller.HoldController (PD + I, Betaflight's own
#     stick curve). Its I term starts at the lean the pilot holds at the engage moment.
#   - HEADING: Betaflight corrects its heading with GPS course over ground (no compass), so we
#     do not use it (config.BF_HEADING_FROM_VISION). The yaw in self.attitudes is our own:
#     Betaflight's RAW gyro integrated (only if config.BF_GYRO_YAW_SIGN was checked on the
#     bench; else 0). vision_hold.py takes the heading from the camera.
#   - Raw accelerometer/gyro samples are kept (self.imu) for moving() ("are we being carried?",
#     vision/frozen.py) and imu_positions() (optional height from motion, fusion/scale.py).
#
# REFERENCES: Stage 2 package Rev 2.0 sections 03-05; handover.py (Part D).
# =====================================================================================
import json
import math
import socket
import threading
import time
from collections import deque

import config
from fusion import position_controller
from fusion.scale import ImuIntegrator
from link.drone_link import DroneLink

NEUTRAL = (1500, 1500)


class _Conn:
    """The few things vision_hold.py and handover.py expect from a MAVLink connection."""
    flightmode = "MANUAL"
    address = "Betaflight through link/msp_sender.py"
    target_system = target_component = 1

    def set_mode(self, mode):
        print(f"(Betaflight: no flight mode to set, ignoring '{mode}'. Not ready = centred sticks.)")


class BetaflightLink(DroneLink):
    def __init__(self, connection=None):                   # no super(): there is no MAVLink connection
        self.conn = _Conn()
        self.send_lock = threading.Lock()
        self.attitudes = deque(maxlen=200)                 # (local time, roll, pitch, yaw), like DroneLink
        self.rangefinder = self.local_z = None
        self.rc, self.params = {}, {}
        self.origin_set, self.running = True, True
        self.acked_source = self.pending_source = None
        self.mode_time = None
        self.engaged = False
        self._wait_reset = None                            # ENGAGE GUARD: reset_counter seen at engage
        self._last_reset = None
        self.last_sticks = NEUTRAL                         # for tests and the log
        self.slew = position_controller.SlewLimiter()      # STAGE 2: no jerks = no smeared pictures
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # BETAFLIGHT CHANGES
        self.hold = position_controller.HoldController()
        self.armed = False
        self.imu = deque(maxlen=200)                       # (t, acc in g [3], gyro in deg/s [3]) ~8 s at 25 Hz
        self.integ = ImuIntegrator()                       # double-integrated raw accel, for fusion/scale.py
        self.gyro_yaw = 0.0                                # integrated raw gyro yaw (rad), our own heading source
        self._gyro_bias = 0.0
        self._last_imu_t = None
        self._last_att = None

    # ----- telemetry from the sender -----
    def on_telemetry(self, tele):
        """One telemetry packet from link/msp_sender.py."""
        self.mode_time = time.monotonic()
        imu = tele.get("imu")
        if imu and imu[0] != self._last_imu_t:             # BETAFLIGHT CHANGE: raw sensors (never GPS-corrected)
            self._on_imu(*imu)
        if tele.get("att"):
            t_att, roll, pitch, bf_yaw = tele["att"]
            self._last_att = (roll, pitch)
            if config.BF_HEADING_FROM_VISION:              # never Betaflight's GPS-corrected heading
                bf_yaw = self.gyro_yaw if config.BF_GYRO_YAW_SIGN else 0.0
            self.attitudes.append((t_att, roll, pitch, bf_yaw))
        if tele.get("alt") is not None:
            self.local_z = (tele["t"], -tele["alt"])       # DroneLink.height() reads this (down = +)
        self.rc = {i + 1: v for i, v in enumerate(tele.get("rc") or [])}
        engaged = bool(tele.get("engaged"))
        arm = self.rc.get(config.BF_ARM_CHANNEL)
        fc_armed = tele.get("armed")                     # STAGE 2 (measured): the FC's own flag first; with the
        self.armed = fc_armed if fc_armed is not None else (arm is not None and arm >= 1500)   # radio off CH5 reads 1775
        if engaged and not self.engaged:
            self._wait_reset = self._last_reset            # centred sticks until the new home is saved
            # BETAFLIGHT CHANGE: in flight the I term starts at the lean the pilot holds against the wind. On the
            # bench (disarmed) it starts at 0: a judge's hand tilt must not offset the step 5 response.
            roll, pitch = (self._last_att or (0.0, 0.0)) if self.armed else (0.0, 0.0)
            self.hold.engage(lean_fwd_deg=-math.degrees(pitch), lean_right_deg=math.degrees(roll), heading_rad=0.0)
        self.engaged = engaged
        self.conn.flightmode = "POSHOLD" if engaged else "MANUAL"

    def _on_imu(self, t, acc, gyro):
        """BETAFLIGHT CHANGE: one raw accelerometer (g) + gyro (deg/s) sample from Betaflight."""
        dt = 0.0 if self._last_imu_t is None else max(0.0, min(0.2, t - self._last_imu_t))
        self._last_imu_t = t
        self.imu.append((t, list(acc), list(gyro)))
        z = gyro[2] if len(gyro) > 2 else 0.0
        if not self.armed and not self.moving(t):          # at rest on the bench: learn the gyro offset
            self._gyro_bias += 0.05 * (z - self._gyro_bias)
        self.gyro_yaw += math.radians(config.BF_GYRO_YAW_SIGN * (z - self._gyro_bias)) * dt
        self.integ.add(t, [a * 9.80665 for a in acc])

    def moving(self, t=None, window=None):
        """BETAFLIGHT CHANGE: True if the raw accelerometer wobbled more than FROZEN_MOVING_ACC (m/s^2 RMS of
        the change in |acc|) or the gyro turned faster than 8 deg/s within the last `window` seconds."""
        window = config.FROZEN_WINDOW_S if window is None else window
        if not self.imu:
            return False
        t = self.imu[-1][0] if t is None else t
        recent = [s for s in list(self.imu) if t - window <= s[0] <= t + 0.05]   # list(): one atomic copy (threads)
        if len(recent) < 4:
            return False
        mags = [9.80665 * math.sqrt(sum(a * a for a in s[1])) for s in recent]
        mean = sum(mags) / len(mags)
        rms = math.sqrt(sum((m - mean) ** 2 for m in mags) / len(mags))
        spin = max(math.sqrt(sum(g * g for g in s[2])) for s in recent)
        return rms >= config.FROZEN_MOVING_ACC or spin >= 8.0

    def _reader(self):
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.bind(("127.0.0.1", config.BF_TELEMETRY_PORT))
        rx.settimeout(0.5)
        while self.running:
            try:
                self.on_telemetry(json.loads(rx.recv(4096)))
            except (socket.timeout, ValueError):
                pass

    def start(self):
        print("Betaflight route: waiting for link/msp_sender.py telemetry (is the msp_sender service running?)")
        threading.Thread(target=self._reader, daemon=True).start()

    # ----- the parts of DroneLink that have no meaning on Betaflight -----
    def check_params(self):
        print("Betaflight route: sticks through MSP OVERRIDE, engage = CH10, no GNSS requested anywhere.")

    def command(self, *args, **kw): pass
    def send_heartbeat(self): pass
    def set_origin(self): pass

    def set_source(self, n):
        self.acked_source = n                              # nothing to switch: we ARE the controller

    def bench_height(self):
        """STAGE 2 bench: disarmed (CH5 low; props off, a judge carries the aircraft about 1 m up) ->
        config.BENCH_HEIGHT_M. Armed, or no radio data yet -> None (the barometer height is used)."""
        if self.rc:                                        # armed already prefers the FC's own flag
            return None if self.armed else config.BENCH_HEIGHT_M
        arm = self.rc.get(config.BF_ARM_CHANNEL)
        return config.BENCH_HEIGHT_M if arm is not None and arm < 1500 else None

    def mode_age(self):
        return float("inf") if self.mode_time is None else time.monotonic() - self.mode_time

    # ----- position in, stick request out -----
    def send_odometry(self, t_capture, north, east, down, att, vel_body, quality, reset_counter, metres_per_px=None):
        self._last_reset = reset_counter
        if self._wait_reset is not None and reset_counter != self._wait_reset:
            self._wait_reset = None                        # the engage home is saved: our position counts from it
        if quality <= 0 or self._wait_reset is not None or not self.engaged:
            self.last_sticks = NEUTRAL                     # centred AT ONCE (never slewed)
            self.slew.reset()
        else:
            # BETAFLIGHT CHANGE: PD + I in the home-heading frame, Betaflight's own stick curve. att[2] is the
            # heading vision_hold.py made north/east with (the camera's, by default), so it cancels exactly.
            yaw = att[2]
            vf, vr = float(vel_body[0]), float(vel_body[1])
            v_n, v_e = vf * math.cos(yaw) - vr * math.sin(yaw), vf * math.sin(yaw) + vr * math.cos(yaw)
            wanted = self.hold.update(t_capture, north, east, v_n, v_e, yaw, integrate=self.armed)
            self.last_sticks = self.slew(t_capture, wanted)   # STAGE 2: at most BF_MAX_TILT_RATE_DEG_S
        try:
            self._tx.sendto(json.dumps(dict(roll=self.last_sticks[0], pitch=self.last_sticks[1])).encode(),
                            ("127.0.0.1", config.BF_SENDER_PORT))
        except OSError:
            pass                                           # sender not up: it is already sending centred sticks
