# vision_hold.py
# The AUTOMATIC version of the keys. Runs on the drone's computer, no keyboard needed:
#   'r' (save home)     -> saved by itself when the HOME TRIGGER fires (see HOME_TRIGGER)
#   'v' (use vision)    -> switches by itself once health.py says tracking is trusted (OK)
#   'g' (back to GPS)   -> switches back by itself if tracking is LOST (if a GPS is fitted)
#   'q' (quit)          -> Ctrl+C, the camera dying, or MAX_SECONDS: shuts down SAFELY
#
# Every frame:  camera -> tracker.py -> health.py -> ODOMETRY message -> autopilot
#
# How to run:
#   python vision_hold.py                         -> simulator (tcp:127.0.0.1:5762), camera 0
#   python vision_hold.py --camera 1              -> a different camera
#   python vision_hold.py --camera video.mp4      -> a recorded video (played in real time)
#   python vision_hold.py --show                  -> also show a preview window
#   python vision_hold.py --fixed-height 0.5      -> bench test: webcam 0.5 m above a table
#
# Autopilot settings this expects (source set 2 = our vision, set 1 = GPS):
#   VISO_TYPE = 1, EK3_SRC2_POSXY = 6 (ExternalNav), EK3_SRC2_POSZ = 1 (Baro),
#   EK3_SRC2_VELXY = 0 to start with, VISO_QUAL_MIN = lowest quality you trust (e.g. 30)
# The program reads some of these at start-up and warns you if they look wrong.

import os
os.environ["MAVLINK20"] = "1"      # ODOMETRY's "quality" field only exists in MAVLink 2

import argparse
import csv
import math
import threading
import time
from collections import deque

import cv2
import numpy as np
from pymavlink import mavutil

from tracker import GroundTracker, load_calibration, to_north_east
from health import HealthChecker

# ---------------- Settings ----------------
CONNECTION = "tcp:127.0.0.1:5762"   # simulator. Real drone e.g. "/dev/ttyAMA0" or "COM5"
BAUD = 921600                        # only used for serial connections

HOME_TRIGGER = "mode"   # when to save HOME (replaces the 'r' key):
                        #  "mode"  = when the pilot switches into a mode in HOLD_MODES
                        #  "rc"    = when a switch on the remote (HOME_RC_CHANNEL) is flipped ON
                        #  "start" = as soon as the camera sees enough ground (bench tests)
HOLD_MODES = {"LOITER", "POSHOLD"}  # good for the simulator (it has GPS to enter LOITER)
HOME_RC_CHANNEL = 7                 # for "rc": real GPS-denied flights (fly ALT_HOLD, flip switch)
RC_ON_PWM = 1700                    # switch counts as ON above this
MIN_HOME_CORNERS = 50               # don't save home over a blank floor

FALLBACK_TO_GPS = True   # LOST -> switch the autopilot back to GPS (needs a GPS fitted)
MIN_HEIGHT_M = 0.3       # below this the picture is too close / we're landing -> don't send
HEIGHT_TIMEOUT_S = 0.5   # height readings older than this are not trusted
MAX_SECONDS = None       # stop after this long (None = run until Ctrl+C)

# EKF origin: ANY fixed place works, it just has to be set once without a GPS.
ORIGIN_LAT, ORIGIN_LON, ORIGIN_ALT_M = 45.4215, -75.6972, 70.0   # Ottawa

SRC_GPS, SRC_VISION = 1, 2


# ---------------- Small maths helpers ----------------
def euler_to_quaternion(roll, pitch, yaw):
    """(roll, pitch, yaw) in radians -> quaternion [w, x, y, z] (what MAVLink wants)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def ned_to_body(v_ned, roll, pitch, yaw):
    """Turn a North/East/Down velocity into the drone's Forward/Right/Down axes."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],   # body -> NED
                  [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                  [-sp, sr * cp, cr * cp]])
    return R.T @ np.asarray(v_ned)                                           # NED -> body


# ---------------- Talking to the autopilot ----------------
class DroneLink:
    """Keeps the newest autopilot data up to date in the background, and sends our messages."""

    def __init__(self, connection):
        self.conn = mavutil.mavlink_connection(connection, baud=BAUD, autoreconnect=True)
        self.send_lock = threading.Lock()
        self.attitudes = deque(maxlen=200)   # (local time, roll, pitch, yaw) ~4 s of history
        self.rangefinder = None              # (local time, metres)
        self.local_z = None                  # (local time, z) from LOCAL_POSITION_NED (down = +)
        self.rc = {}                         # channel number -> PWM
        self.origin_set = False
        self.acked_source = None             # source set the autopilot CONFIRMED
        self.pending_source = None           # source set we last ASKED for
        self.params = {}
        self.running = True

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

    def send_odometry(self, t_capture, north, east, down, att, vel_body, quality, reset_counter):
        roll, pitch, yaw = att
        if quality > 0:
            sigma = 0.02 + 0.3 * (1 - quality / 100)        # metres: better quality = smaller
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
        if not self.attitudes:
            return None
        return min(self.attitudes, key=lambda a: abs(a[0] - t))[1:]

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


# ---------------- The home trigger (replaces the 'r' key) ----------------
class HomeTrigger:
    """Fires ONCE each time its condition turns on (a 'rising edge'), like a key press."""

    def __init__(self, kind):
        self.kind = kind
        self.was_on = False

    def fired(self, link):
        if self.kind == "start":
            on = True
        elif self.kind == "mode":
            on = link.conn.flightmode in HOLD_MODES
        else:  # "rc"
            on = link.rc.get(HOME_RC_CHANNEL, 0) > RC_ON_PWM
        fired = on and not self.was_on
        self.was_on = on
        return fired


# ---------------- Main loop ----------------
def main():
    ap = argparse.ArgumentParser(description="Automatic vision position hold")
    ap.add_argument("--connect", default=CONNECTION)
    ap.add_argument("--camera", default="0", help="camera number or video file")
    ap.add_argument("--trigger", default=HOME_TRIGGER, choices=["mode", "rc", "start"])
    ap.add_argument("--fixed-height", type=float, default=None,
                    help="bench test only: pretend the camera is this many metres up")
    ap.add_argument("--no-gps-fallback", action="store_true")
    ap.add_argument("--show", action="store_true", help="show a preview window")
    ap.add_argument("--max-seconds", type=float, default=MAX_SECONDS)
    args = ap.parse_args()

    link = DroneLink(args.connect)
    link.start()
    link.check_params()

    is_camera = args.camera.isdigit()
    cap = cv2.VideoCapture(int(args.camera) if is_camera else args.camera)
    if not cap.isOpened():
        print(f"Could not open camera/video '{args.camera}'. Check the number or file name.")
        link.running = False
        return
    frame_gap = 0.0 if is_camera else 1.0 / (cap.get(cv2.CAP_PROP_FPS) or 30)  # play videos in real time

    K, dist = load_calibration()
    tracker, health = GroundTracker(K, dist), HealthChecker()
    trigger = HomeTrigger(args.trigger)
    fallback = FALLBACK_TO_GPS and not args.no_gps_fallback

    os.makedirs("logs", exist_ok=True)
    log_path = time.strftime("logs/flight_%Y%m%d_%H%M%S.csv")
    log_file = open(log_path, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t_capture", "status", "quality", "north_m", "east_m", "height_m", "shift_x_px",
                  "shift_y_px", "rotation_deg", "inliers", "source", "reasons"])

    home_saved, home_pending, reset_counter, last_texture_warn = False, False, 0, 0.0
    want_source, last_source_try = None, 0.0
    last_heartbeat, last_origin_try, last_frame_ok = 0.0, 0.0, time.time()
    prev_ne, vel_ned = None, np.zeros(3)
    started, next_frame = time.time(), time.time()
    print(f"Running. Home trigger = '{args.trigger}'. Logging to {log_path}. Ctrl+C to stop.")

    try:
        while args.max_seconds is None or time.time() - started < args.max_seconds:
            now = time.time()
            if now - last_heartbeat > 1.0:                       # "I'm alive" once a second
                link.send_heartbeat()
                last_heartbeat = now
            if not link.origin_set and now - last_origin_try > 2.0:   # keep trying until confirmed
                link.set_origin()
                last_origin_try = now

            # ---------- 1. Grab a frame, stamp the time immediately ----------
            if frame_gap:                                        # video file: wait for its moment
                time.sleep(max(0.0, next_frame - time.time()))
                next_frame += frame_gap
            ok, frame = cap.read()
            t_capture = time.time()
            if not ok:
                if not is_camera:
                    print("Video finished.")
                    break
                if t_capture - last_frame_ok > 5.0:
                    print("Camera lost for 5 s - stopping.")
                    break
                if home_saved:                                   # tell the autopilot: no data
                    att = link.attitude_at(t_capture) or (0.0, 0.0, 0.0)
                    link.send_odometry(t_capture, 0.0, 0.0, 0.0, att, (0, 0, 0), -1, reset_counter)
                time.sleep(0.05)
                continue
            last_frame_ok = t_capture
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # ---------- 2. Home trigger (replaces 'r') ----------
            if trigger.fired(link):
                home_pending = True                              # try on this frame and the next ones
            if home_pending:
                n = tracker.set_reference(gray)
                if n >= MIN_HOME_CORNERS:
                    health.reset()
                    home_saved, home_pending = True, False
                    reset_counter = (reset_counter + 1) % 256    # tells the EKF "position jumped to 0"
                    prev_ne, vel_ned = None, np.zeros(3)
                    print(f"HOME saved automatically ({n} corners, trigger '{args.trigger}')")
                elif t_capture - last_texture_warn > 2.0:
                    print(f"Not enough ground texture for home ({n} corners) - retrying")
                    last_texture_warn = t_capture
            if not home_saved:
                continue

            # ---------- 3. Measure + judge ----------
            att = link.attitude_at(t_capture) or (0.0, 0.0, 0.0)
            res = tracker.track(gray, t_capture, attitude=att)
            rep = health.check(gray, res)
            height = args.fixed_height if args.fixed_height else link.height(t_capture)
            quality = rep.quality
            if height is None or height < MIN_HEIGHT_M:
                quality = -1                                     # can't turn pixels into metres

            north = east = float("nan")
            if quality > 0:
                north, east = to_north_east(res.shift_px, height, tracker.focal_px, att[2])
                if prev_ne is not None and t_capture > prev_ne[2]:   # smooth velocity estimate
                    dt = t_capture - prev_ne[2]
                    v = np.array([(north - prev_ne[0]) / dt, (east - prev_ne[1]) / dt, 0.0])
                    vel_ned = 0.7 * vel_ned + 0.3 * v
                prev_ne = (north, east, t_capture)

            # ---------- 4. Send to the autopilot ----------
            down = link.local_z[1] if link.local_z else 0.0      # ignored: height source is Baro
            send_n, send_e = (north, east) if quality > 0 else (0.0, 0.0)
            link.send_odometry(t_capture, send_n, send_e, down, att,
                               ned_to_body(vel_ned, *att), quality, reset_counter)

            # ---------- 5. Source switching (replaces 'v' and 'g') ----------
            if rep.status == "OK" and quality > 0:
                want_source = SRC_VISION
            elif rep.status == "LOST" and fallback and link.acked_source == SRC_VISION:
                want_source = SRC_GPS                            # only if we had switched to vision
            if want_source and link.acked_source != want_source and now - last_source_try > 1.0:
                link.set_source(want_source)                     # retried until the autopilot says OK
                last_source_try = now

            log.writerow([f"{t_capture:.3f}", rep.status, quality,
                          f"{north:.3f}", f"{east:.3f}", height,
                          f"{res.shift_px[0]:.2f}" if res.ok else "", f"{res.shift_px[1]:.2f}" if res.ok else "",
                          f"{res.rotation_deg:.2f}", res.n_inliers, link.acked_source, "; ".join(rep.reasons)])

            if args.show:
                text = f"{rep.status} q={quality} N={north:+.2f} E={east:+.2f} src={link.acked_source}"
                cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0) if quality > 0 else (0, 0, 255), 2)
                cv2.imshow("vision hold", frame)
                cv2.waitKey(1)

    except KeyboardInterrupt:
        print("\nCtrl+C pressed.")
    finally:
        # ---------- Safe shutdown (replaces 'q') ----------
        print("Shutting down safely...")
        if home_saved:
            att = link.attitude_at(time.time()) or (0.0, 0.0, 0.0)
            for _ in range(5):                                   # "our data is no longer valid"
                link.send_odometry(time.time(), 0.0, 0.0, 0.0, att, (0, 0, 0), -1, reset_counter)
                time.sleep(0.02)
        if fallback and link.acked_source == SRC_VISION:
            link.set_source(SRC_GPS)
            time.sleep(0.5)
        link.running = False
        cap.release()
        log_file.close()
        cv2.destroyAllWindows()
        print(f"Done. Log saved to {log_path}")


if __name__ == "__main__":
    main()
