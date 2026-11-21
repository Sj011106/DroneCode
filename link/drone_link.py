# link/drone_link.py
# Connection to ArduPilot over MAVLink: keeps the newest autopilot data (attitude, height,
# RC, params) up to date in a background thread, and sends our ODOMETRY + source switches.
# Split out of vision_hold.py so check_link.py and tools can reuse it.

# -------------------------------------------------------------------------------------
# IN PLAIN WORDS (for non-technical readers)
#   The phone line to the autopilot. A background helper listens all the time and keeps
#   the latest tilt, height, switch positions and settings; the main program sends our
#   position and "use vision / use GPS" requests through it.
# REFERENCES
#   - MAVLink protocol: https://mavlink.io/en/  |  pymavlink: https://mavlink.io/en/mavgen_python/
#   - SET_GPS_GLOBAL_ORIGIN, ODOMETRY, MAV_CMD_SET_EKF_SOURCE_SET:
#     https://mavlink.io/en/messages/common.html
# -------------------------------------------------------------------------------------

import os
os.environ["MAVLINK20"] = "1"      # ODOMETRY's "quality" field only exists in MAVLink 2

import math
import threading
import time
from collections import deque

from pymavlink import mavutil

# ---------------- Settings ----------------
BAUD = 921600                        # only used for serial connections
HEIGHT_TIMEOUT_S = 0.5   # height readings older than this are not trusted

# EKF origin: ANY fixed place works, it just has to be set once without a GPS.
ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT_M = 45.4215, -75.6972, 70.0   # Ottawa

SRC_GPS, SRC_VISION = 1, 2


class LockedDeque(deque):
    """A deque that is safe to read while another thread appends to it.

    Why: the reader thread appends ATTITUDE samples ~50 times a second while the main
    loop copies the history (list(link.attitudes), min(...)). Iterating a plain deque
    while it is being appended raises "RuntimeError: deque mutated during iteration",
    which would stop vision_hold in flight. Appending and iterating now share one lock,
    and iterating returns a frozen copy, so every existing caller is safe unchanged."""

    def __init__(self, iterable=(), maxlen=None):
        self._lock = threading.Lock()
        super().__init__(iterable, maxlen)

    def append(self, item):
        with self._lock:
            super().append(item)

    def __iter__(self):
        with self._lock:
            items = list(super().__iter__())
        return iter(items)

    def snapshot(self):
        """A list copy of the history, taken safely in one go."""
        return list(self)


def euler_to_quaternion(roll, pitch, yaw):
    """(roll, pitch, yaw) in radians -> quaternion [w, x, y, z] (what MAVLink wants)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def odometry_sigma_px(quality):
    """NEW: expected position error IN PIXELS for a quality score (1..100).
    Multiplied by metres-per-pixel (height / focal length) this becomes metres, so the
    confidence we send shrinks automatically as we climb. ArduPilot's EKF uses
    sqrt(var_x + var_y + var_z) from our covariance as its noise, never lower than
    VISO_POS_M_NSE (AP_VisualOdom_MAV.cpp: constrain_float(posErr, pos_noise, 100))."""
    import config
    return config.ODOM_PX_NOISE_BEST + (config.ODOM_PX_NOISE_WORST - config.ODOM_PX_NOISE_BEST) * (1 - quality / 100)


# ---------------- Talking to the autopilot ----------------
class DroneLink:
    """Keeps the newest autopilot data up to date in the background, and sends our messages."""

    def __init__(self, connection):
        self.conn = mavutil.mavlink_connection(connection, baud=BAUD, autoreconnect=True)
        self.send_lock = threading.Lock()
        self.attitudes = LockedDeque(maxlen=200)   # (local time, roll, pitch, yaw) ~4 s; thread-safe
        self.rangefinder = None              # (local time, metres)
        self.local_z = None                  # (local time, z) from LOCAL_POSITION_NED (down = +)
        self.rc = {}                         # channel number -> PWM
        self.origin_set = False
        self.acked_source = None             # source set the autopilot CONFIRMED
        self.pending_source = None           # source set we last ASKED for
        self.params = {}
        self.mode_time = None                # monotonic time of the last autopilot HEARTBEAT
        self.running = True

    def mode_age(self):
        """Seconds since the last autopilot HEARTBEAT (inf if none yet)."""
        return float("inf") if self.mode_time is None else time.monotonic() - self.mode_time

    def is_autopilot_heartbeat(self, msg):
        """True only for the flight controller's own HEARTBEAT (not a ground station, not our own onboard heartbeat)."""
        return (msg.get_srcSystem() == self.conn.target_system
                and msg.type != mavutil.mavlink.MAV_TYPE_GCS
                and msg.autopilot != mavutil.mavlink.MAV_AUTOPILOT_INVALID)

    def start(self):
        print(f"Connecting to {self.conn.address} ...")
        self.send_heartbeat()                # say hello first (needed for UDP links)
        self.conn.wait_heartbeat()
        print(f"Connected to system {self.conn.target_system}")
        threading.Thread(target=self._reader, daemon=True).start()
        # Ask ONLY for what we need (not MAV_DATA_STREAM_ALL, which floods slow radio links)
        for msg_id, hz in ((30, 50), (32, 10), (132, 20), (65, 10)):  # ATTITUDE, LOCAL_POS, DIST, RC
            self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, msg_id, int(1e6 / hz))

    def _reader(self):
        """Background thread: read every message and keep the newest values."""
        while self.running:
            msg = self.conn.recv_match(blocking=True, timeout=0.5)
            if msg is None:
                continue
            kind, now = msg.get_type(), time.time()
            if kind == "ATTITUDE":
                self.attitudes.append((now, msg.roll, msg.pitch, msg.yaw))
            elif kind == "DISTANCE_SENSOR":
                d = msg.current_distance / 100.0
                if msg.min_distance / 100.0 <= d <= msg.max_distance / 100.0:
                    self.rangefinder = (now, d)
            elif kind == "LOCAL_POSITION_NED":
                self.local_z = (now, msg.z)
            elif kind == "RC_CHANNELS":
                self.rc = {i: getattr(msg, f"chan{i}_raw") for i in range(1, 17)}
            elif kind == "HEARTBEAT":
                if self.is_autopilot_heartbeat(msg):
                    self.mode_time = time.monotonic()      # only the autopilot's heartbeat
            elif kind == "GPS_GLOBAL_ORIGIN":
                self.origin_set = True
            elif kind == "PARAM_VALUE":
                self.params[msg.param_id] = msg.param_value
            elif kind == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    self.acked_source = self.pending_source
            elif kind == "STATUSTEXT":
                print(f"[drone] {msg.text}")

    # ----- sending -----
    def command(self, cmd, p1=0, p2=0):
        with self.send_lock:
            self.conn.mav.command_long_send(self.conn.target_system, self.conn.target_component,
                                            cmd, 0, p1, p2, 0, 0, 0, 0, 0)

    def send_heartbeat(self):
        with self.send_lock:
            self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)

    def set_origin(self):
        """Tell the EKF where 'zero' is. Without a GPS nobody else will."""
        with self.send_lock:
            self.conn.mav.set_gps_global_origin_send(
                self.conn.target_system, int(ORIGIN_LAT * 1e7), int(ORIGIN_LON * 1e7),
                int(ORIGIN_ALT_M * 1000), int(time.time() * 1e6))

    def set_source(self, n):
        """Replaces the 'v' / 'g' keys: 1 = GPS, 2 = our vision."""
        self.pending_source = n
        self.command(mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET, n)
        print(f">>> asked autopilot to use {'VISION' if n == SRC_VISION else 'GPS'} (source set {n})")

    def send_odometry(self, t_capture, north, east, down, att, vel_body, quality, reset_counter,
                      metres_per_px=None):
        roll, pitch, yaw = att
        if quality > 0:
            sigma = 0.02 + 0.3 * (1 - quality / 100)        # metres: better quality = smaller
            if metres_per_px:   # NEW: higher up, one pixel = more metres -> same pixel error = more metres
                sigma = max(sigma, odometry_sigma_px(quality) * metres_per_px)
            pose_cov = [0.0] * 21
            pose_cov[0] = pose_cov[6] = pose_cov[11] = sigma ** 2     # x, y, z variance
            pose_cov[15] = pose_cov[18] = pose_cov[20] = 0.01         # roll, pitch, yaw variance
        else:
            pose_cov = [float("nan")] + [0.0] * 20                    # "unknown"
        with self.send_lock:
            self.conn.mav.odometry_send(
                int(t_capture * 1e6),
                mavutil.mavlink.MAV_FRAME_LOCAL_FRD, mavutil.mavlink.MAV_FRAME_BODY_FRD,
                north, east, down, euler_to_quaternion(roll, pitch, yaw),
                float(vel_body[0]), float(vel_body[1]), float(vel_body[2]),
                float("nan"), float("nan"), float("nan"),
                pose_cov, [float("nan")] + [0.0] * 20,
                reset_counter, mavutil.mavlink.MAV_ESTIMATOR_TYPE_VISION, int(quality))

    # ----- reading -----
    def attitude_at(self, t):
        """The roll/pitch/yaw reading closest in time to t (the frame's capture time)."""
        history = list(self.attitudes)           # one safe copy (LockedDeque: taken under its lock;
                                                 # replay's plain deque: single-threaded anyway)
        if not history:
            return None
        return min(history, key=lambda a: abs(a[0] - t))[1:]

    def height(self, now):
        """Height above ground: rangefinder if fresh, else the autopilot's barometer height."""
        if self.rangefinder and now - self.rangefinder[0] < HEIGHT_TIMEOUT_S:
            return self.rangefinder[1]
        if self.local_z and now - self.local_z[0] < HEIGHT_TIMEOUT_S:
            return -self.local_z[1]
        return None

    def check_params(self):
        """Read a few autopilot settings and warn if they look wrong. Never blocks flight."""
        wanted = {"VISO_TYPE": "not 0", "EK3_SRC2_POSXY": 6, "EK3_SRC2_POSZ": "1 or 2"}
        for name in wanted:
            with self.send_lock:
                self.conn.mav.param_request_read_send(self.conn.target_system,
                                                      self.conn.target_component,
                                                      name.encode(), -1)
        time.sleep(1.5)
        p = self.params
        if p.get("VISO_TYPE", 1) == 0:
            print("WARNING: VISO_TYPE = 0 -> the autopilot will IGNORE our position")
        if "EK3_SRC2_POSXY" in p and int(p["EK3_SRC2_POSXY"]) != 6:
            print("WARNING: EK3_SRC2_POSXY should be 6 (ExternalNav)")
        if "EK3_SRC2_POSZ" in p and int(p["EK3_SRC2_POSZ"]) not in (1, 2):
            print("WARNING: EK3_SRC2_POSZ should be 1 (Baro) or 2 (Rangefinder), never our vision")
        if not p:
            print("(could not read parameters - check them by hand)")
