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
        self._tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # ----- telemetry from the sender -----
    def on_telemetry(self, tele):
        """One telemetry packet from link/msp_sender.py."""
        self.mode_time = time.monotonic()
        if tele.get("att"):
            self.attitudes.append(tuple(tele["att"]))
        if tele.get("alt") is not None:
            self.local_z = (tele["t"], -tele["alt"])       # DroneLink.height() reads this (down = +)
        self.rc = {i + 1: v for i, v in enumerate(tele.get("rc") or [])}
        engaged = bool(tele.get("engaged"))
        if engaged and not self.engaged:
            self._wait_reset = self._last_reset            # centred sticks until the new home is saved
        self.engaged = engaged
        self.conn.flightmode = "POSHOLD" if engaged else "MANUAL"

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
            self.last_sticks = NEUTRAL
        else:
            yaw = att[2]
            forward = north * math.cos(yaw) + east * math.sin(yaw)
            right = -north * math.sin(yaw) + east * math.cos(yaw)
            self.last_sticks = position_controller.sticks(forward, right, float(vel_body[0]), float(vel_body[1]))
        try:
            self._tx.sendto(json.dumps(dict(roll=self.last_sticks[0], pitch=self.last_sticks[1])).encode(),
                            ("127.0.0.1", config.BF_SENDER_PORT))
        except OSError:
            pass                                           # sender not up: it is already sending centred sticks
