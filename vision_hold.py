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
import ast
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
import signal
import config
from compute.budget import FrameBudget, make_low_latency
from compute.profiler import Profiler
from fusion.height import HeightFilter
from fusion.imu_check import ImuCheck
from fusion.tilt import cancel_tilt
from recovery.failsafe import Failsafe
from recovery.recovery import Recovery
from tools.replay import Recorder, Replay, auto_folder
from vision.enhance import downscale, enhance, scale_camera_matrix
from vision.rehome import Rehome
from vision.rolling_shutter import RollingShutterTracker, attitude_near
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


def _stop_on_sigterm(signum, frame):                            # NEW
    """systemctl stop sends SIGTERM; turn it into Ctrl+C so the safe shutdown below runs."""
    raise KeyboardInterrupt


# ---------------- Main loop ----------------
def main(argv=None):                                             # NEW: argv for tests
    ap = argparse.ArgumentParser(description="Automatic vision position hold")
    ap.add_argument("--connect", default=CONNECTION)
    ap.add_argument("--camera", default="0", help="camera number or video file")
    ap.add_argument("--trigger", default=HOME_TRIGGER, choices=["mode", "rc", "start"])
    ap.add_argument("--fixed-height", type=float, default=None,
                    help="bench test only: pretend the camera is this many metres up")
    ap.add_argument("--no-gps-fallback", action="store_true")
    ap.add_argument("--show", action="store_true", help="show a preview window")
    ap.add_argument("--max-seconds", type=float, default=MAX_SECONDS)
    # NEW: record a flight, replay it through this same code, toggle settings for replays
    ap.add_argument("--record", metavar="DIR", nargs="?", const="auto",   # NEW
                    help="save video + times + attitude + height (no DIR: /sdcard/rec_<time> or logs/)")
    ap.add_argument("--replay", metavar="DIR", help="run on a recording instead of camera + drone")
    ap.add_argument("--set", action="append", default=[], metavar="NAME=VALUE",
                    help="override a config.py setting, e.g. --set USE_TILT=False")
    ap.add_argument("--log", metavar="PATH", help="flight log file name (default logs/flight_<time>.csv)")
    args = ap.parse_args(argv)
    for item in args.set:                                        # NEW
        name, value = item.split("=", 1)
        if not hasattr(config, name):
            raise SystemExit(f"--set: config.py has no setting {name}")
        setattr(config, name, ast.literal_eval(value))
        print(f"config.{name} = {getattr(config, name)!r}")
    signal.signal(signal.SIGTERM, _stop_on_sigterm)              # NEW: 'systemctl stop' -> safe shutdown
    replay = Replay(args.replay) if args.replay else None        # NEW
    clock = replay.clock if replay else time.time                # NEW: recorded time during replay

    link = replay.link if replay else DroneLink(args.connect)    # NEW: replay link
    link.start()
    link.check_params()

    is_camera = (args.camera.isdigit() or args.camera.startswith("/dev/video")) and not replay   # NEW
    cap = replay.cap if replay else cv2.VideoCapture(int(args.camera) if args.camera.isdigit() else args.camera)
    if not cap.isOpened():
        print(f"Could not open camera/video '{args.camera}'. Check the number or file name.")
        link.running = False
        return
    frame_gap = 0.0 if is_camera else 1.0 / (cap.get(cv2.CAP_PROP_FPS) or 30)  # play videos in real time
    if replay:
        frame_gap = 0.0                                          # NEW: replay as fast as possible
    low_latency = is_camera and make_low_latency(cap)            # NEW: no stale frames in the queue
    if is_camera:                                                # NEW
        print(f"Camera 1-frame queue: {'yes' if low_latency else 'not supported by this backend'}")

    K, dist = load_calibration()
    if config.CALIB_FILE != "calib.npz":                         # NEW: tests use their own lens file
        K, dist = load_calibration(config.CALIB_FILE)
    # NEW: Saumya's GroundTracker + rolling-shutter correction (identical if switched off)
    timing = load_timing()
    print(f"Rolling shutter: {'ON' if config.USE_ROLLING_SHUTTER else 'OFF'}, {timing}")
    tracker = RollingShutterTracker(K, dist, attitude_history=link.attitudes, timing=timing,
                                    enabled=config.USE_ROLLING_SHUTTER)   # NEW: explicit, so --set works
    health = HealthChecker()
    if getattr(config, "USE_FEATURE_FILTER", False):             # Saumya: ignore props/OSD (mask.png)
        from vision.feature_filter import attach
        attach(tracker, health)
    imu_check, height_filter = ImuCheck(), HeightFilter()        # NEW
    recovery, failsafe = Recovery(tracker), Failsafe()           # NEW
    rehome = Rehome()                                            # NEW: new reference while climbing
    prof = Profiler(budget_ms=1000 / config.TARGET_FPS)          # NEW
    budget = FrameBudget(camera_fps=cap.get(cv2.CAP_PROP_FPS) or None,          # NEW
                         queue_len=1 if low_latency else 4)
    home_att = None                                              # NEW: attitude when HOME was saved
    trigger = replay.trigger if replay else HomeTrigger(args.trigger)   # NEW: recorded trigger in replay
    rec_dir = auto_folder() if args.record == "auto" else args.record   # NEW
    recorder = Recorder(rec_dir) if rec_dir else None            # NEW
    if recorder:                                                 # NEW
        print(f"Recording to {rec_dir}")
    fallback = FALLBACK_TO_GPS and not args.no_gps_fallback

    os.makedirs("logs", exist_ok=True)
    log_path = args.log or time.strftime("logs/flight_%Y%m%d_%H%M%S.csv")   # NEW: --log
    log_file = open(log_path, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t_capture", "status", "quality", "north_m", "east_m", "height_m", "shift_x_px",
                  "shift_y_px", "rotation_deg", "inliers", "source", "reasons",
                  # NEW columns (tools/analyze_log.py plots these)
                  "corr_x_px", "corr_y_px", "imu_err_deg", "relocks",
                  "ms_grab", "ms_track", "ms_health", "ms_fusion", "ms_send", "ms_total", "rehomes"])

    home_saved, home_pending, reset_counter, last_texture_warn = False, False, 0, 0.0
    want_source, last_source_try = None, 0.0
    last_heartbeat, last_origin_try, last_frame_ok = 0.0, 0.0, time.time()
    prev_ne, vel_ned = None, np.zeros(3)
    started, next_frame = time.time(), time.time()
    k_scaled, imu_bad_streak = False, 0                          # NEW
    print(f"Running. Home trigger = '{args.trigger}'. Logging to {log_path}. Ctrl+C to stop.")

    try:
        while args.max_seconds is None or time.time() - started < args.max_seconds:
            now = clock()                                        # NEW: was time.time()
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
            t_read = time.time()                                 # NEW: how long read() waits (budget)
            ok, frame = cap.read()
            t_capture = clock()                                  # NEW: was time.time()
            if not ok and is_camera and home_saved:              # NEW: dead camera while on vision ->
                failsafe.step(link.acked_source == SRC_VISION, t_capture, link)   # failsafe timer runs
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
            small = downscale(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))   # NEW: PROCESS_WIDTH (APB CPU)
            gray = enhance(small)                                # NEW: enhance() = contrast boost
            if not k_scaled:                                     # NEW: lens K for the processed size
                orig_w = (replay.orig_width if replay else None) or frame.shape[1]
                orig_h = round(frame.shape[0] * orig_w / frame.shape[1])
                K, dist = load_calibration(config.CALIB_FILE, image_size=(orig_w, orig_h))   # size check
                tracker.dist = dist
                tracker.K = scale_camera_matrix(K, small.shape[1] / orig_w)
                k_scaled = True
            t_frame = timing.mid_time(t_capture)                 # NEW: real capture time (= t_capture until calibrated)
            atts = list(link.attitudes)                          # NEW: ONE frozen IMU snapshot per frame:
            tracker.history = atts                               # thread-safe, and replay sees exactly this
            prof.lap("grab")                                     # NEW

            # ---------- 2. Home trigger (replaces 'r') ----------
            fired = trigger.fired(link)                          # NEW: kept for the recorder
            if recorder:                                         # NEW: record raw inputs
                recorder.add(small, t_capture, fired, link.height(t_capture), atts,
                             orig_size=(frame.shape[1], frame.shape[0]))
            if fired:
                home_pending = True                              # try on this frame and the next ones
            old_ref = dict(tracker.__dict__) if home_pending and home_saved else None   # NEW
            if home_pending:
                n = tracker.set_reference(gray, t_capture)       # NEW: t_capture -> rolling-shutter fix on home too
                if n >= MIN_HOME_CORNERS:
                    health.reset()
                    home_att = attitude_near(atts, t_frame)   # NEW: tilt + IMU check (thread-safe)
                    imu_check.set_home(home_att[2] if home_att else None)
                    recovery.set_home(gray)                      # NEW: fingerprints for re-finding home
                    rehome.reset()                               # NEW: positions count from THIS home
                    home_saved, home_pending = True, False
                    reset_counter = (reset_counter + 1) % 256    # tells the EKF "position jumped to 0"
                    prev_ne, vel_ned = None, np.zeros(3)
                    print(f"HOME saved automatically ({n} corners, trigger '{args.trigger}')")
                elif t_capture - last_texture_warn > 2.0:
                    print(f"Not enough ground texture for home ({n} corners) - retrying")
                    last_texture_warn = t_capture
            if old_ref is not None and home_pending:             # NEW: new home failed (poor texture):
                tracker.__dict__.update(old_ref)                 # keep tracking the OLD home meanwhile
            if not home_saved:
                continue

            # ---------- 3. Measure + judge ----------
            att_real = attitude_near(atts, t_frame)    # NEW: thread-safe; None = missing/stale IMU
            att = att_real or (0.0, 0.0, 0.0)                    # NEW
            if home_att is None and att_real:                    # NEW: home saved before IMU data arrived:
                home_att = att_real                              # use the first attitude (approximation)
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
            imu_err, disagree = 0.0, False
            if config.USE_IMU_CHECK and res.ok and att_real:
                agree, imu_err = imu_check.check(res.rotation_deg, att_real[2])
                if not agree:
                    quality, disagree = -1, True
                    rep.reasons.append(f"vision/IMU disagree ({imu_err:+.1f} deg)")
            imu_bad_streak = imu_bad_streak + 1 if disagree else 0
            if att_real is None and quality > 0:                 # NEW: no IMU -> heading unknown: N/E
                quality = -1                                     # would be rotated by a fake yaw 0
                rep.reasons.append("no IMU attitude (missing or stale)")

            # NEW: lost and failing (or locked onto something the IMU disagrees with) -> re-find home
            failing = ((rep.status == "LOST" and health.bad_count > 0)
                       or imu_bad_streak >= config.IMU_DISAGREE_RECOVER_FRAMES)
            if config.USE_RECOVERY and recovery.update(gray, failing):
                print(f"Re-found HOME (re-lock #{recovery.relocks})")

            # NEW: remove the fake shift caused by roll/pitch
            shift = res.shift_px
            if config.USE_TILT and res.ok and att_real and home_att:
                shift = cancel_tilt(res.shift_px, att_real, home_att, tracker.focal_px, res.rotation_deg)
            prof.lap("fusion")                                   # NEW

            north = east = float("nan")
            if quality > 0:
                north, east = to_north_east(shift, height, tracker.focal_px, att[2])   # NEW: shift (was res.shift_px)
                north, east = rehome.total(north, east)          # NEW: + offset of the current reference
                if prev_ne is not None and t_capture > prev_ne[2]:   # smooth velocity estimate
                    dt = t_capture - prev_ne[2]
                    v = np.array([(north - prev_ne[0]) / dt, (east - prev_ne[1]) / dt, 0.0])
                    vel_ned = 0.7 * vel_ned + 0.3 * v
                prev_ne = (north, east, t_capture)

            # NEW: climbed/descended ~15% -> new reference picture here, position stays continuous
            if config.USE_REHOME and quality > 0 and att_real and rehome.due(rep.status, res):
                if rehome.rehome(tracker, health, gray, t_capture, north, east, MIN_HOME_CORNERS):
                    home_att = att_real
                    imu_check.set_home(att_real[2])              # rotation restarts at 0 here
                    recovery.set_home(gray)
                    print(f"Re-homed at {height:.1f} m (#{rehome.count}), offset N={north:+.2f} E={east:+.2f}")

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

            # NEW: no VALID position for too long while the autopilot relies on vision -> safe mode
            # (covers LOST, but also IMU disagreement / no height while health still says OK)
            failsafe.step(quality <= 0 and link.acked_source == SRC_VISION, now, link)

            prof.end_frame()                                     # NEW
            ms = prof.current
            log.writerow([f"{t_capture:.3f}", rep.status, quality,
                          f"{north:.3f}", f"{east:.3f}", height,
                          f"{res.shift_px[0]:.2f}" if res.ok else "", f"{res.shift_px[1]:.2f}" if res.ok else "",
                          f"{res.rotation_deg:.2f}", res.n_inliers, link.acked_source, "; ".join(rep.reasons),
                          # NEW columns
                          f"{shift[0]:.2f}" if res.ok else "", f"{shift[1]:.2f}" if res.ok else "",
                          f"{imu_err:.2f}", recovery.relocks,
                          *(f"{ms.get(s, 0.0):.2f}" for s in ("grab", "track", "health", "fusion", "send", "total")),
                          rehome.count])
            if is_camera:                                        # NEW: fell behind -> drop stale frames
                for _ in range(budget.frames_to_drop(time.time() - t_capture, t_capture - t_read)):
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
        if recorder:                                             # NEW
            recorder.close()
            print(f"Recording saved in {rec_dir} ({recorder.n} frames)")
        cv2.destroyAllWindows()
        if prof.frames:                                          # NEW: compute summary for the slide
            print(f"\n{prof.table()}\nRe-locks: {recovery.relocks}, stale frames dropped: {budget.dropped}")
        print(f"Done. Log saved to {log_path}")


if __name__ == "__main__":
    main()
