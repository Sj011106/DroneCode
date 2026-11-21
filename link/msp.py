# =====================================================================================
# link/msp.py  -  the Betaflight "language" (MSP v1): build and read its messages
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Betaflight (the firmware on the event aircraft) does not understand "be at this
#   position". It only understands stick positions, like a pilot's thumbs. We talk to it
#   over a serial cable in a small protocol called MSP. This file only knows how to WRITE
#   one message ("here are the stick values") and how to ASK for and READ two answers
#   ("how are you tilted?", "what is the pilot's radio sending?"). Nothing else can be
#   built with it, on purpose: in particular there is no way to ask Betaflight for GPS.
#
# HOW IT WORKS (technical)
#   MSP v1 frame:  '$' 'M' '<'  size  command  payload...  checksum
#     checksum = XOR of size, command and every payload byte. Replies use '>' ('!' = error).
#   Commands used (Stage 2 package section 03 + the public MSP protocol):
#     200 MSP_SET_RAW_RC   8 x uint16 (microseconds), order roll, pitch, throttle, yaw, AUX1-4.
#                          The FC uses the first four while the engage switch (CH10) is on.
#     108 MSP_ATTITUDE     int16 roll (0.1 deg), int16 pitch (0.1 deg), int16 heading (deg)
#     105 MSP_RC           uint16 per channel: what the receiver sees (CH10 = engage switch)
#     102 MSP_RAW_IMU      BETAFLIGHT CHANGE: 9 x int16: accelerometer (raw units, ~2048 = 1 g), gyro (deg/s),
#                          magnetometer (none fitted). Plain sensor readings: Betaflight never mixes GPS into
#                          them (unlike its heading, which it corrects with GPS course over ground).
#   GNSS RULE (Stage 2 section 04): MSP_RAW_GPS (106) and friends must never be requested.
#   request() refuses any command that is not in ALLOWED, so no caller can do it by mistake.
#   Proof: tests/test_betaflight.py.
#
# CHECK ON THE REAL AIRCRAFT (not verifiable without it): the channel order MSP_RC answers
#   in, and the sign of pitch in MSP_ATTITUDE. Both have a setting in config.py (BF_*).
#
# REFERENCES
#   - Red Team Grand Final Stage 2 package, Rev 2.0, section 03 (link, command, safety rule).
#   - MSP protocol: https://github.com/betaflight/betaflight/blob/master/src/main/msp/msp_protocol.h
# =====================================================================================
import struct

MSP_STATUS, MSP_RAW_IMU, MSP_RC, MSP_ATTITUDE, MSP_SET_RAW_RC = 101, 102, 105, 108, 200
ALLOWED = frozenset({MSP_STATUS, MSP_RAW_IMU, MSP_RC, MSP_ATTITUDE, MSP_SET_RAW_RC})   # anything else (GPS!) is refused


def frame(command, payload=b""):
    """One MSP v1 message to the flight controller."""
    if command not in ALLOWED:
        raise ValueError(f"MSP command {command} is not allowed (Stage 2: no GNSS, nothing we don't need)")
    body = bytes([len(payload), command]) + payload
    check = 0
    for b in body:
        check ^= b
    return b"$M<" + body + bytes([check])


def request(command):
    """Ask the flight controller to send us `command`'s data."""
    return frame(command)


def set_raw_rc(roll, pitch, throttle, yaw, aux=None):
    """The stick message. Values in microseconds (1500 = stick centred). AUX1-4: the flight controller ignores
    them (Stage 2 section 03). BETAFLIGHT CHANGE: sent as config.BF_AUX_US (1000, "off"), not 1500, so a wrong
    override mask could never arm the aircraft (see config.py)."""
    import config
    a = int(config.BF_AUX_US if aux is None else aux)
    return frame(MSP_SET_RAW_RC, struct.pack("<8H", int(roll), int(pitch), int(throttle), int(yaw), a, a, a, a))


class Parser:
    """Feed it whatever bytes arrived; it returns the complete, checksum-correct replies."""

    def __init__(self):
        self.buf = b""

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            start = self.buf.find(b"$M")
            if start < 0:
                self.buf = self.buf[-1:]                  # keep a possible lone '$'
                return out
            self.buf = self.buf[start:]
            if len(self.buf) < 5:
                return out
            size = self.buf[3]
            if len(self.buf) < 6 + size:
                return out
            msg, self.buf = self.buf[:6 + size], self.buf[6 + size:]
            check = 0
            for b in msg[3:-1]:
                check ^= b
            if msg[2:3] == b">" and check == msg[-1]:     # '!' (error reply) and bad checksums are dropped
                out.append((msg[4], msg[5:-1]))


def parse_attitude(payload):
    """-> (roll_deg, pitch_deg, heading_deg) exactly as Betaflight reports them."""
    roll, pitch, heading = struct.unpack("<3h", payload[:6])
    return roll / 10.0, pitch / 10.0, float(heading)


def parse_status(payload):
    """MSP_STATUS -> True if the flight controller says it is ARMED, None if the reply is too short.
    Bytes 6..9 are the active flight-mode boxes; ARM is box 0. MEASURED 21 Nov 2026: disarmed reads 0
    even with no radio (when the arm CHANNEL reads 1775). Armed = 1 is from the protocol file, not yet seen."""
    return bool(struct.unpack_from("<I", payload, 6)[0] & 1) if len(payload) >= 10 else None


def parse_raw_imu(payload):
    """BETAFLIGHT CHANGE -> (acc_x, acc_y, acc_z) in Betaflight's raw units, (gyro_x, gyro_y, gyro_z) in deg/s."""
    v = struct.unpack("<9h", payload[:18])
    return v[0:3], v[3:6]


def parse_rc(payload):
    """-> list of channel values in microseconds."""
    n = len(payload) // 2
    return list(struct.unpack(f"<{n}H", payload[:2 * n]))
