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

# -------------------------------------------------------------------------------------
# IN PLAIN WORDS (for non-technical readers)
#   The main program that flies. For every camera frame: work out how far we've slid
#   from home, decide how much to trust it, correct it for tilt and rolling shutter, and
#   send it to the autopilot, which blends it with its own IMU and barometer (EKF3) to
#   hold position. Lines marked "NEW" were added on top of Saumya's original flow; each
#   addition has an on/off switch in config.py.
# PIPELINE
#   camera -> tracker (+ rolling-shutter fix) -> health -> IMU check -> recovery
#          -> tilt cancel -> metres (filtered height) -> ODOMETRY -> ArduPilot EKF3
# REFERENCES
#   - ArduPilot non-GPS navigation: https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html
#   - ArduPilot EKF source switching: https://ardupilot.org/copter/docs/common-ekf-sources.html
#   - MAVLink ODOMETRY: https://mavlink.io/en/messages/common.html#ODOMETRY
# -------------------------------------------------------------------------------------

import argparse
import csv
import os
import time
import math

import cv2
import numpy as np

from vision.tracker import load_calibration, to_north_east
from health.health import HealthChecker
from link.drone_link import DroneLink, SRC_GPS, SRC_VISION

# NEW (added on top of Saumya's pipeline; each one can be switched off in config.py):
import config
from compute.budget import FrameBudget, make_low_latency
from compute.profiler import Profiler
from fusion.height import HeightFilter
from fusion.imu_check import ImuCheck
from fusion.tilt import cancel_tilt
from recovery.failsafe import Failsafe
from recovery.recovery import Recovery
from vision.rolling_shutter import RollingShutterTracker
from vision.timing import load_timing

# ---------------- Settings ----------------
CONNECTION = "tcp:127.0.0.1:5762"   # simulator. Real drone e.g. "/dev/ttyAMA0" or "COM5"

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
MAX_SECONDS = None       # stop after this long (None = run until Ctrl+C)


# ---------------- Small maths helpers ----------------
def ned_to_body(v_ned, roll, pitch, yaw):
    """Turn a North/East/Down velocity into the drone's Forward/Right/Down axes."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([[cp * cy, sr * sp * cy - cr * sy, cr * sp * cy + sr * sy],   # body -> NED
                  [cp * sy, sr * sp * sy + cr * cy, cr * sp * sy - sr * cy],
                  [-sp, sr * cp, cr * cp]])
    return R.T @ np.asarray(v_ned)                                           # NED -> body


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
    if is_camera:                                                # NEW: no stale frames in the queue
        print(f"Camera 1-frame queue: {'yes' if make_low_latency(cap) else 'not supported by this backend'}")

    K, dist = load_calibration()
    # NEW: Saumya's GroundTracker + rolling-shutter correction (identical if switched off)
    timing = load_timing()
    print(f"Rolling shutter: {'ON' if config.USE_ROLLING_SHUTTER else 'OFF'}, {timing}")
    tracker = RollingShutterTracker(K, dist, attitude_history=link.attitudes, timing=timing)
    health = HealthChecker()
    imu_check, height_filter = ImuCheck(), HeightFilter()        # NEW
    recovery, failsafe = Recovery(tracker), Failsafe()           # NEW
    prof, budget = Profiler(budget_ms=1000 / config.TARGET_FPS), FrameBudget()   # NEW
    home_att = None                                              # NEW: attitude when HOME was saved
    trigger = HomeTrigger(args.trigger)
    fallback = FALLBACK_TO_GPS and not args.no_gps_fallback

    os.makedirs("logs", exist_ok=True)
    log_path = time.strftime("logs/flight_%Y%m%d_%H%M%S.csv")
    log_file = open(log_path, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t_capture", "status", "quality", "north_m", "east_m", "height_m", "shift_x_px",
                  "shift_y_px", "rotation_deg", "inliers", "source", "reasons",
                  # NEW columns (tools/analyze_log.py plots these)
                  "corr_x_px", "corr_y_px", "imu_err_deg", "relocks",
                  "ms_grab", "ms_track", "ms_health", "ms_fusion", "ms_send", "ms_total"])

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
            prof.start_frame()                                   # NEW: after the video pacing wait
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
            t_frame = timing.mid_time(t_capture)                 # NEW: real capture time (= t_capture until calibrated)
            prof.lap("grab")                                     # NEW

            # ---------- 2. Home trigger (replaces 'r') ----------
            if trigger.fired(link):
                home_pending = True                              # try on this frame and the next ones
            if home_pending:
                n = tracker.set_reference(gray, t_capture)       # NEW: t_capture -> rolling-shutter fix on home too
                if n >= MIN_HOME_CORNERS:
                    health.reset()
                    home_att = link.attitude_at(t_frame)         # NEW: for tilt + IMU check
                    imu_check.set_home(home_att[2] if home_att else None)
                    recovery.set_home(gray)                      # NEW: fingerprints for re-finding home
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
            att_real = link.attitude_at(t_frame)                 # NEW: t_frame, and None = no IMU data
            att = att_real or (0.0, 0.0, 0.0)
            res = tracker.track(gray, t_capture, attitude=att)
            prof.lap("track")                                    # NEW
            rep = health.check(gray, res)
            prof.lap("health")                                   # NEW
            height = args.fixed_height if args.fixed_height else link.height(t_capture)
            if not args.fixed_height and config.USE_HEIGHT_FILTER:   # NEW: smooth, ignore spikes
                height = height_filter.update(t_capture, height)
            quality = rep.quality
            if height is None or height < MIN_HEIGHT_M:
                quality = -1                                     # can't turn pixels into metres

            # NEW: vision must agree with the IMU about how much we turned since home
            imu_err = 0.0
            if config.USE_IMU_CHECK and res.ok and att_real:
                agree, imu_err = imu_check.check(res.rotation_deg, att_real[2])
                if not agree:
                    quality = -1
                    rep.reasons.append(f"vision/IMU disagree ({imu_err:+.1f} deg)")

            # NEW: lost and failing -> try to re-find home (tracker.relock)
            if config.USE_RECOVERY and recovery.update(gray, rep.status == "LOST" and health.bad_count > 0):
                print(f"Re-found HOME (re-lock #{recovery.relocks})")

            # NEW: remove the fake shift caused by roll/pitch
            shift = res.shift_px
            if config.USE_TILT and res.ok and att_real and home_att:
                shift = cancel_tilt(res.shift_px, att_real, home_att, tracker.focal_px, res.rotation_deg)
            prof.lap("fusion")                                   # NEW

            north = east = float("nan")
            if quality > 0:
                north, east = to_north_east(shift, height, tracker.focal_px, att[2])   # NEW: shift (was res.shift_px)
                if prev_ne is not None and t_capture > prev_ne[2]:   # smooth velocity estimate
                    dt = t_capture - prev_ne[2]
                    v = np.array([(north - prev_ne[0]) / dt, (east - prev_ne[1]) / dt, 0.0])
                    vel_ned = 0.7 * vel_ned + 0.3 * v
                prev_ne = (north, east, t_capture)

            # ---------- 4. Send to the autopilot ----------
            down = link.local_z[1] if link.local_z else 0.0      # ignored: height source is Baro
            send_n, send_e = (north, east) if quality > 0 else (0.0, 0.0)
            link.send_odometry(t_capture, send_n, send_e, down, att,
                               ned_to_body(vel_ned, *att), quality, reset_counter,
                               metres_per_px=height / tracker.focal_px if quality > 0 else None)   # NEW
            prof.lap("send")                                     # NEW

            # ---------- 5. Source switching (replaces 'v' and 'g') ----------
            if rep.status == "OK" and quality > 0:
                want_source = SRC_VISION
            elif rep.status == "LOST" and fallback and link.acked_source == SRC_VISION:
                want_source = SRC_GPS                            # only if we had switched to vision
            if want_source and link.acked_source != want_source and now - last_source_try > 1.0:
                link.set_source(want_source)                     # retried until the autopilot says OK
                last_source_try = now

            # NEW: vision lost for too long while the autopilot relies on it -> safe mode
            action = failsafe.update(rep.status == "LOST" and link.acked_source == SRC_VISION, now)
            if action:
                print(f"FAILSAFE: vision lost for {config.FAILSAFE_AFTER_S:.0f} s -> switching to {action}")
                with link.send_lock:
                    link.conn.set_mode(action)

            loop_ms = prof.end_frame()                           # NEW
            ms = prof.current
            log.writerow([f"{t_capture:.3f}", rep.status, quality,
                          f"{north:.3f}", f"{east:.3f}", height,
                          f"{res.shift_px[0]:.2f}" if res.ok else "", f"{res.shift_px[1]:.2f}" if res.ok else "",
                          f"{res.rotation_deg:.2f}", res.n_inliers, link.acked_source, "; ".join(rep.reasons),
                          # NEW columns
                          f"{shift[0]:.2f}" if res.ok else "", f"{shift[1]:.2f}" if res.ok else "",
                          f"{imu_err:.2f}", recovery.relocks,
                          *(f"{ms.get(s, 0.0):.2f}" for s in ("grab", "track", "health", "fusion", "send", "total"))])
            if is_camera:                                        # NEW: fell behind -> drop stale frames
                for _ in range(budget.frames_to_drop(loop_ms / 1000)):
                    cap.grab()

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
        if prof.frames:                                          # NEW: compute summary for the slide
            print(f"\n{prof.table()}\nRe-locks: {recovery.relocks}, stale frames dropped: {budget.dropped}")
        print(f"Done. Log saved to {log_path}")


if __name__ == "__main__":
    main()
