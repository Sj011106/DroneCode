# =====================================================================================
# link/msp_sender.py  -  the small program that ALWAYS talks to Betaflight
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Betaflight has no safety timeout (Stage 2 section 03): with the engage switch on it
#   flies our last stick message for ever, and before our first message it reads "throttle
#   cut, full tilt". So one tiny program must send stick values 50 times a second from the
#   moment the board boots, whether the vision program is ready, busy, or has crashed.
#   That is this file. It runs as its own program (its own boot service) and owns the
#   cable to the flight controller. Its rules:
#     - no fresh request from the vision program in the last 0.3 s  -> sticks centred
#       (roll, pitch, yaw 1500) and a throttle that holds height. This is the timeout
#       Betaflight lacks, and what the bench test calls "not ready".
#     - vision ready and engage switch (CH10) on -> the vision program's roll and pitch,
#       never more than a few degrees.
#     - throttle: what the pilot's own stick was giving just before engaging, nudged up or
#       down to keep the barometer height of the engage moment.
#     - yaw stick always centred.
#   It also reads the drone's tilt and the radio channels and passes them to the vision
#   program. It never asks for GPS (link/msp.py cannot build that request).
#
# HOW IT WORKS (technical)
#   Serial /dev/ttymxc2 at 115200, MSP v1 (link/msp.py). Every 20 ms: read replies, decide
#   the four channels, write MSP_SET_RAW_RC, ask for MSP_ATTITUDE (every tick) and MSP_RC
#   (every other tick). Local UDP on 127.0.0.1: targets in on BF_SENDER_PORT
#   ({"roll": us, "pitch": us}), telemetry out to BF_TELEMETRY_PORT (attitude in radians,
#   aerospace signs; height; channels; engaged).
#   Height: the APB's own DPS310 (link/apb_sensors.py), zeroed while disarmed (CH5 low),
#   smoothed; climb rate from its change. No barometer found -> throttle stays at the
#   pilot's value (no height hold). The flight controller's altitude is NOT used: it may
#   blend GPS, which Stage 2 forbids during a hold.
#   Any error inside the decision step still ends in a centred-sticks message.
#   Clean stop (systemctl stop): three centred-sticks messages, then exit.
#   Proof: tests/test_betaflight.py (pretend serial port and barometer).
#
# RUN
#   venv/bin/python -m link.msp_sender                 # the boot service does this
#   venv/bin/python -m link.msp_sender --monitor       # bench: also print tilt + channels twice a second
#
# REFERENCES
#   - Red Team Grand Final Stage 2 package, Rev 2.0: section 03 (safety rule), section 04
#     (GNSS rule), section 05 steps 5-8 (what the judge reads on the Receiver tab).
# =====================================================================================
import argparse
import json
import math
import signal
import socket
import time

import config
from fusion import position_controller
from link import msp


class Sender:
    """All the decisions, with the cable and the barometer handed in (so tests can fake them)."""

    def __init__(self, port, baro=None):
        self.port, self.baro, self.parser = port, baro, msp.Parser()
        self.target, self.target_t = (1500, 1500), -math.inf
        self.rc, self.att, self.yaw = [], None, None
        self.engaged = False
        self.hold_throttle = None                    # the pilot's throttle just before engage
        self.alt = self.alt_t = self.alt_engage = None
        self.climb = 0.0
        self.tick = 0
        self.last = (1500, 1500, config.BF_HOVER_THROTTLE, 1500)

    def set_target(self, roll_us, pitch_us, now):
        """The vision program's request. Only used while fresh and while the pilot has engaged.
        `now` must come from the SAME clock as decide()'s `fresh_now` (main: time.monotonic())."""
        roll_us, pitch_us = float(roll_us), float(pitch_us)
        if not (math.isfinite(roll_us) and math.isfinite(pitch_us)):     # REVIEW M3: NaN would pass min/max as the cap
            return
        self.target, self.target_t = (roll_us, pitch_us), now

    def _read(self, now):
        for command, payload in self.parser.feed(self.port.read(4096) or b""):
            if command == msp.MSP_ATTITUDE and len(payload) >= 6:
                roll, pitch, heading = msp.parse_attitude(payload)
                yaw = math.radians(heading)
                if self.yaw is None:
                    self.yaw = yaw
                else:                                # heading arrives in whole degrees: smooth the steps away
                    self.yaw += 0.2 * math.atan2(math.sin(yaw - self.yaw), math.cos(yaw - self.yaw))
                self.att = (now, math.radians(config.BF_ATT_ROLL_SIGN * roll),
                            math.radians(config.BF_ATT_PITCH_SIGN * pitch), self.yaw)
            elif command == msp.MSP_RC:
                self.rc = msp.parse_rc(payload)

    def _height(self, now, armed):
        if self.baro is None:
            return
        if not armed or self.baro.p0 is None:
            self.baro.zero()                         # on the ground: this is height 0
        h = self.baro.height()
        if h is None:
            return
        if self.alt is None:
            self.alt, self.climb = h, 0.0
        else:
            new = self.alt + 0.2 * (h - self.alt)
            dt = max(now - self.alt_t, 1e-3)
            self.climb += 0.2 * ((new - self.alt) / dt - self.climb)
            self.alt = new
        self.alt_t = now

    def decide(self, now, fresh_now=None):
        """-> (roll, pitch, throttle, yaw) in microseconds for this tick.
        REVIEW M2: `fresh_now` (monotonic) judges whether the vision request is still fresh. The wall clock
        (`now`) jumps on the APB when network time arrives; a backward jump would revive a dead request."""
        fresh_now = now if fresh_now is None else fresh_now
        rc = self.rc
        engaged = len(rc) >= config.BF_ENGAGE_CHANNEL and rc[config.BF_ENGAGE_CHANNEL - 1] >= config.BF_ENGAGE_PWM
        armed = len(rc) >= config.BF_ARM_CHANNEL and rc[config.BF_ARM_CHANNEL - 1] >= 1500
        self._height(now, armed)
        if len(rc) > config.BF_RC_THROTTLE_INDEX:
            pilot = rc[config.BF_RC_THROTTLE_INDEX]
            # While engaged the FC reports OUR throttle here, so only learn it before engage (or once,
            # if we started while already engaged: then it is the value the FC is holding).
            if pilot >= config.BF_THROTTLE_MIN and (not engaged or self.hold_throttle is None):
                self.hold_throttle = pilot
        if engaged and not self.engaged:
            self.alt_engage = self.alt               # keep THIS height
        self.engaged = engaged
        hold = self.hold_throttle if self.hold_throttle is not None else config.BF_HOVER_THROTTLE

        roll = pitch = 1500.0
        if engaged and 0.0 <= fresh_now - self.target_t <= config.BF_TARGET_TIMEOUT_S:
            most = 500.0 * config.BF_MAX_TILT_DEG / config.BF_ANGLE_LIMIT_DEG    # second cap, whatever was asked
            roll = 1500 + max(-most, min(most, self.target[0] - 1500))
            pitch = 1500 + max(-most, min(most, self.target[1] - 1500))
        throttle = hold
        if engaged and config.BF_HEIGHT_HOLD and self.alt is not None and self.alt_engage is not None:
            throttle = position_controller.throttle(hold, self.alt_engage - self.alt, self.climb)
        throttle = max(config.BF_THROTTLE_MIN, min(config.BF_THROTTLE_MAX, throttle))
        return round(roll), round(pitch), round(throttle), 1500

    def step(self, now, fresh_now=None):
        """One 20 ms tick: read, decide, send. Returns what to tell the vision program."""
        try:
            self._read(now)
            out = self.decide(now, fresh_now)
        except Exception as e:                       # a bug here must never freeze the sticks
            print(f"msp_sender: decision error ({e!r}) - sending centred sticks", flush=True)
            out = (1500, 1500, self.last[2], 1500)
        self.last = out
        self.port.write(msp.set_raw_rc(*out) + msp.request(msp.MSP_ATTITUDE)
                        + (msp.request(msp.MSP_RC) if self.tick % 2 == 0 else b""))
        self.tick += 1
        return dict(t=now, att=self.att, alt=self.alt, rc=self.rc, engaged=self.engaged, sent=out)

    def neutral(self):
        """Centred sticks at the last throttle (used on a clean stop)."""
        self.port.write(msp.set_raw_rc(1500, 1500, self.last[2], 1500))


def _stop(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description="Always-on Betaflight MSP sender")
    ap.add_argument("--port", default=config.BF_SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=config.BF_SERIAL_BAUD)
    ap.add_argument("--monitor", action="store_true", help="print tilt, channels and what we send, twice a second")
    args = ap.parse_args(argv)
    import serial                                    # pyserial (in requirements.txt)
    from link.apb_sensors import Baro
    signal.signal(signal.SIGTERM, _stop)

    while True:                                      # the cable may not be ready at boot: keep trying
        try:
            port = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0.05)
            break
        except serial.SerialException as e:
            print(f"msp_sender: cannot open {args.port} ({e}); retrying", flush=True)
            time.sleep(1.0)
    baro = Baro()
    print(f"msp_sender: {args.port} at {args.baud}, barometer {'found' if baro.dir else 'NOT found (no height hold)'}", flush=True)
    sender = Sender(port, baro if baro.dir else None)
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", config.BF_SENDER_PORT))
    rx.setblocking(False)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    next_tick, last_print = time.monotonic(), 0.0
    try:
        while True:
            now = time.time()
            try:
                while True:                          # newest request wins
                    msg = json.loads(rx.recv(512))
                    sender.set_target(msg["roll"], msg["pitch"], time.monotonic())
            except (BlockingIOError, ValueError, KeyError, TypeError):
                pass
            try:
                tele = sender.step(now, time.monotonic())
            except serial.SerialException as e:      # cable hiccup: keep trying, never exit
                print(f"msp_sender: serial error ({e})", flush=True)
                tele = None
            if tele:
                try:
                    tx.sendto(json.dumps(tele).encode(), ("127.0.0.1", config.BF_TELEMETRY_PORT))
                except OSError:
                    pass
                if args.monitor and now - last_print >= 0.5:
                    last_print = now
                    a = tele["att"]
                    tilt = "no reply from the FC" if a is None else \
                        f"roll {math.degrees(a[1]):+6.1f} pitch {math.degrees(a[2]):+6.1f} yaw {math.degrees(a[3]):6.1f}"
                    print(f"{tilt} | height {tele['alt']} | engaged {tele['engaged']} | sent {tele['sent']} | rc {tele['rc']}", flush=True)
            next_tick += 1.0 / config.BF_SEND_HZ
            time.sleep(max(0.0, next_tick - time.monotonic()))
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(3):
            try:
                sender.neutral()
            except Exception:
                pass
            time.sleep(0.02)
        print("msp_sender: stopped (sent centred sticks)", flush=True)


if __name__ == "__main__":
    main()
