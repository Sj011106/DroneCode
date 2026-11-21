# vision_hold.py
# The AUTOMATIC version of the keys. Runs on the drone's computer, no keyboard needed:
#   'r' (save home)     -> saved by itself when the HOME TRIGGER fires (see HOME_TRIGGER)
#   'v' (use vision)    -> switches by itself once health.py says tracking is trusted (OK)
#   'g' (back to GPS)   -> switches back by itself if tracking is LOST (if a GPS is fitted)
#                          NEW (Ayush A1): only with --practice. TRIAL MODE (the default) makes this
#                          impossible: GNSS must not aid the scored hold (safety/trial_mode.py).
#   'q' (quit)          -> Ctrl+C, the camera dying, or MAX_SECONDS: shuts down SAFELY
#
# Every frame:  camera -> tracker.py -> health.py -> ODOMETRY message -> autopilot
#
# How to run:
#   python vision_hold.py                         -> simulator (tcp:127.0.0.1:5762), camera 0
#   python vision_hold.py --camera 1              -> a different camera
#   python vision_hold.py --camera video.mp4      -> a recorded video (played in real time)
#   python vision_hold.py --show                  -> also show a preview window
#   python vision_hold.py --fc betaflight         -> STAGE 2: fly the stock Betaflight firmware (sticks over MSP;
#                                                    start link/msp_sender.py first, see deploy/)
#   python vision_hold.py --fixed-height 1.0      -> bench test: the camera is 1.0 m above the floor when
#                                                    home is saved (height changes are then read from how
#                                                    much smaller/bigger the floor looks)
# STAGE 2 BENCH (judge carries the aircraft, our laptop shows the estimate):
#   watch the estimate:      journalctl -u vision_hold -f        (READY / NOT READY lines, 5 per second)
#   zero at the start mark:  systemctl kill -s USR1 vision_hold  (saves a new home right here)
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
from handover import AutoReanchor, Handover, REANCHOR

import cv2
import numpy as np

from vision.tracker import load_calibration, to_north_east
from health.health import HealthChecker
from link.drone_link import DroneLink, SRC_GPS, SRC_VISION
from link.betaflight_link import BetaflightLink              # STAGE 2: the firmware the aircraft ships with
from pymavlink import mavutil

# NEW (added on top of Saumya's pipeline; each one can be switched off in config.py):
import signal
import config
from compute.budget import FrameBudget, LatestFrame, camera_fps, make_low_latency, open_camera, to_gray
from compute.profiler import Profiler
from fusion.height import HeightFilter
from fusion.imu_check import ImuCheck
from fusion.tilt import cancel_tilt
from recovery.recovery import Recovery
from tools.replay import Recorder, Replay, auto_folder
from vision.enhance import downscale, enhance, scale_camera_matrix
from vision.rehome import Rehome
from vision.rolling_shutter import RollingShutterTracker, attitude_near
from vision.timing import load_timing
from safety import trial_mode                                    # NEW (Ayush A1): GPS can't take over
from safety.side_monitors import SideMonitors                    # NEW (Ayush A4/A5): watch-only advisors

# ---------------- Settings ----------------
CONNECTION = "tcp:127.0.0.1:5762"   # simulator. Real drone e.g. "/dev/ttymxc2" or "COM5"

HOME_TRIGGER = "start"  # when to save HOME (replaces the 'r' key):
                        #  "mode"  = when the pilot switches into a mode in HOLD_MODES
                        #  "rc"    = when a switch on the remote (HOME_RC_CHANNEL) is flipped ON
                        #  "start" = as soon as the camera sees enough ground (bench tests)
HOLD_MODES = {"LOITER", "POSHOLD"}  # good for the simulator (it has GPS to enter LOITER)
HOME_RC_CHANNEL = 7                 # for "rc": real GPS-denied flights (fly ALT_HOLD, flip switch)
RC_ON_PWM = 1700                    # switch counts as ON above this
MIN_HOME_CORNERS = 50               # don't save home over a blank floor

FALLBACK_TO_GPS = False  # GPS is jammed in this challenge (LOST -> switch back to GPS needs a GPS fitted)
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
    ap.add_argument("--fc", default=None, choices=["ardupilot", "betaflight"],   # STAGE 2
                    help="flight controller route (default: config.FC). betaflight needs link/msp_sender.py running")
    ap.add_argument("--trigger", default=HOME_TRIGGER, choices=["mode", "rc", "start"])
    ap.add_argument("--fixed-height", type=float, default=None,
                    help="bench test only: the camera is this many metres above the floor when home is saved")
    ap.add_argument("--no-gps-fallback", action="store_true")
    ap.add_argument("--practice", action="store_true",           # NEW (Ayush A1)
                    help="practice only: allow the GPS fallback (NEVER for the scored trial)")
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
    trial = trial_mode.from_args(args)                           # NEW (Ayush A1): sets --no-gps-fallback
    signal.signal(signal.SIGTERM, _stop_on_sigterm)              # NEW: 'systemctl stop' -> safe shutdown
    zero_here = []                                               # STAGE 2 bench: "zero at the start mark"
    if hasattr(signal, "SIGUSR1"):                               # (Linux only) systemctl kill -s USR1 vision_hold
        signal.signal(signal.SIGUSR1, lambda *_: zero_here.append(1))
    replay = Replay(args.replay) if args.replay else None        # NEW
    clock = replay.clock if replay else time.time                # NEW: recorded time during replay

    betaflight = (args.fc or config.FC) == "betaflight" and not replay   # STAGE 2
    link = replay.link if replay else (BetaflightLink() if betaflight else DroneLink(args.connect))   # NEW: replay link
    trial.guard(link)                                            # NEW (Ayush A1): drops any "use GPS" command
    link.start()
    link.check_params()

    is_camera = (args.camera.isdigit() or args.camera.startswith("/dev/video")) and not replay   # NEW
    cap = replay.cap if replay else open_camera(args.camera, raw=config.CAMERA_RAW_GREY and not args.show)   # STAGE 2
    if not cap.isOpened():
        print(f"Could not open camera/video '{args.camera}'. Check the number or file name.")
        link.running = False
        return
    frame_gap = 0.0 if is_camera else 1.0 / camera_fps(cap, 30)  # play videos in real time
    if replay:
        frame_gap = 0.0                                          # NEW: replay as fast as possible
    low_latency = is_camera and make_low_latency(cap)            # NEW: no stale frames in the queue
    if is_camera:                                                # STAGE 2 (measured): the first frames are dark
        warm_until = time.time() + config.CAMERA_WARMUP_S
        while time.time() < warm_until:
            cap.read()
        if config.CAMERA_THREAD:                                 # STAGE 2 (measured): never wait for the driver
            cap = LatestFrame(cap, clock)
    if is_camera:                                                # NEW
        print(f"Camera 1-frame queue: {'yes' if low_latency else 'not supported by this backend'}")

    K, dist = load_calibration()
    if config.CALIB_FILE != "calib.npz":                         # NEW: tests use their own lens file
        K, dist = load_calibration(config.CALIB_FILE)
    # NEW: Saumya's GroundTracker + rolling-shutter correction (identical if switched off)
    timing = load_timing()
    print(f"Rolling shutter: {'ON' if config.USE_ROLLING_SHUTTER else 'OFF'}, {timing}")
    tracker = RollingShutterTracker(K, dist, attitude_history=link.attitudes, timing=timing,
                                    enabled=config.USE_ROLLING_SHUTTER,   # NEW: explicit, so --set works
                                    level=config.USE_TILT and config.USE_LEVEL_POINTS)   # STAGE 2: wide-lens tilt fix
    health = HealthChecker()
    if getattr(config, "USE_FEATURE_FILTER", False):             # Saumya: ignore props/OSD (mask.png)
        from vision.feature_filter import attach
        attach(tracker, health)
    imu_check, height_filter = ImuCheck(), HeightFilter()        # NEW
    recovery = Recovery(tracker)                                 # NEW
    rehome = Rehome()                                            # NEW: new reference while climbing
    prof = Profiler(budget_ms=1000 / config.TARGET_FPS)          # NEW
    budget = FrameBudget(camera_fps=camera_fps(cap, config.CAMERA_FPS if is_camera else None),   # NEW (STAGE 2: -1 on the APB)
                         queue_len=1 if low_latency else 4)
    home_att = None                                              # NEW: attitude when HOME was saved
    trigger = replay.trigger if replay else HomeTrigger(args.trigger)   # NEW: recorded trigger in replay
    rec_dir = auto_folder() if args.record == "auto" else args.record   # NEW
    recorder = Recorder(rec_dir) if rec_dir else None            # NEW
    if recorder:                                                 # NEW
        print(f"Recording to {rec_dir}")
    fallback = FALLBACK_TO_GPS and not args.no_gps_fallback

    handover = Handover(hold_modes=HOLD_MODES, failsafe_after_s=config.FAILSAFE_AFTER_S,
                        # STAGE 2: Betaflight has no LAND to command; not ready = centred sticks (msp_sender)
                        failsafe_mode="NONE" if betaflight else config.FAILSAFE_ACTION,
                        max_sends=config.FAILSAFE_MODE_RETRIES)
    auto_reanchor = AutoReanchor(config.AUTO_REANCHOR_AFTER_S)
    # Part D: handover timers run on a monotonic clock, so a wall-clock correction (the APB's time is wrong until
    # NTP, APB manual p.10) cannot jump the failsafe timer. A replay uses its recorded time.
    hclock = replay.clock if replay else time.monotonic
    home_height = None                                       # Part D: height when HOME was saved (scale reference)
    # STAGE 2 bench: the FIRST home picture (tracker state, attitude, height) so we can measure against it again
    # when it comes back into view; at_origin = the tracker's reference IS that first picture right now.
    origin, at_origin, frame_i, last_status = None, True, 0, 0.0
    bench_height = args.fixed_height                         # STAGE 2 bench: height now, from the picture's scale
    from link.apb_sensors import Gyro                        # STAGE 2 bench step 1: the APB's own IMU, read in
    apb_gyro = Gyro()                                        # THIS process (None-safe off the board)
    from tools.live_view import LiveView                     # STAGE 2: live view, laptop browser http://192.168.75.1:8088
    live = LiveView()                                        # (never raises; no work while no browser is open)

    def act_on(actions):
        """Do what handover.py asked. Returns True if it asked for a new HOME (REANCHOR)."""
        want_home = False
        for action in actions:
            if action == REANCHOR:
                want_home = True
            elif action[0] == "GAVE_UP":
                text = f"FAILSAFE: {handover.failsafe_mode} ignored. PILOT TAKE OVER"   # <= 50 chars: STATUSTEXT limit
                print(f"{text} (after {handover.max_sends} tries, flight mode {link.conn.flightmode})")
                with link.send_lock:
                    link.conn.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_CRITICAL, text[:50].encode())
            else:
                print(f"FAILSAFE: no valid vision position for {config.FAILSAFE_AFTER_S:g} s while holding: {link.conn.flightmode} -> {action[1]}")
                with link.send_lock:
                    link.conn.set_mode(action[1])
        return want_home

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
    reanchor_request = False                                 # Part D: set by handover, recorded like a trigger
    last_cam_warn = 0.0                                      # Part D: camera-dead warning at most every 5 s
    last_hb_warn = 0.0                                       # Part D: no-heartbeat warning at most every 5 s
    want_source, last_source_try = None, 0.0
    last_heartbeat, last_origin_try, last_frame_ok = 0.0, 0.0, time.time()
    prev_ne, vel_ned = None, np.zeros(3)
    started, next_frame = time.time(), time.time()
    k_scaled, imu_bad_streak = False, 0                          # NEW
    print(f"Running. Home trigger = '{args.trigger}'. Logging to {log_path}. Ctrl+C to stop.")
    monitors = SideMonitors(link, log_path)                      # NEW (Ayush A4/A5): never steers

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
            t_capture = getattr(cap, "t", 0.0) or clock()         # NEW: was time.time(). STAGE 2: the grab thread's time
            if not ok and is_camera and home_saved:              # Part D: dead camera = quality 0
                if act_on(handover.update(hclock(), link.conn.flightmode, 0,
                                          mode_fresh=link.mode_age() < 1.5)):
                    reanchor_request = True
            if not ok:
                if not is_camera:
                    print("Video finished.")
                    break
                if t_capture - last_frame_ok > 5.0 and t_capture - last_cam_warn >= 5.0:
                    print("Camera lost - sending no-position to autopilot; check the camera.")
                    last_cam_warn = t_capture
                if home_saved and (betaflight or not config.ARDUPILOT_SILENT_WHEN_NOT_READY):   # tell the autopilot: no data
                    att = link.attitude_at(t_capture) or (0.0, 0.0, 0.0)
                    link.send_odometry(t_capture, 0.0, 0.0, 0.0, att, (0, 0, 0), -1, reset_counter)
                time.sleep(0.05)
                continue
            last_frame_ok = t_capture
            if config.CAMERA_UPSIDE_DOWN:                        # STAGE 2: camera mounted the other way round
                frame = cv2.rotate(frame, cv2.ROTATE_180)
            small = downscale(to_gray(frame))                    # NEW: PROCESS_WIDTH (APB CPU). STAGE 2: grey directly
            gray = enhance(small)                                # NEW: enhance() = contrast boost
            live.frame(gray)                                     # STAGE 2: live view shows the picture the tracker sees
            if not k_scaled:                                     # NEW: lens K for the processed size
                full_w, full_h = (config.CAMERA_SIZE if frame.ndim == 2 and frame.shape[0] == 1 else frame.shape[1::-1])
                orig_w = (replay.orig_width if replay else None) or full_w
                orig_h = round(full_h * orig_w / full_w)
                K, dist = load_calibration(config.CALIB_FILE, image_size=(orig_w, orig_h))   # size check
                tracker.dist = dist
                tracker.K = scale_camera_matrix(K, small.shape[1] / orig_w)
                k_scaled = True
            t_frame = timing.mid_time(t_capture)                 # NEW: real capture time (= t_capture until calibrated)
            atts = list(link.attitudes)                          # NEW: ONE frozen IMU snapshot per frame:
            tracker.history = atts                               # thread-safe, and replay sees exactly this
            prof.lap("grab")                                     # NEW

            # STAGE 2 bench: no --fixed-height needed on the Betaflight route. Disarmed (props off, judge carrying
            # the aircraft) -> the link says "bench height"; armed -> None -> the barometer height is used.
            fixed_height = args.fixed_height or (link.bench_height() if betaflight else None)

            # ---------- 2. Home trigger (replaces 'r') ----------
            fired = trigger.fired(link) or reanchor_request or bool(zero_here)   # Part D: engage re-anchor is
            reanchor_request = False                             # recorded too. STAGE 2: so is "zero here"
            zero_here.clear()
            if recorder:                                         # NEW: record raw inputs
                recorder.add(small, t_capture, fired, link.height(t_capture), atts,
                             orig_size=(orig_w, orig_h))
            if fired:
                home_pending = True                              # try on this frame and the next ones
            old_ref = dict(tracker.__dict__) if home_pending and home_saved else None   # NEW
            if home_pending:
                n = tracker.set_reference(gray, t_capture)       # NEW: t_capture -> rolling-shutter fix on home too
                if n >= MIN_HOME_CORNERS:
                    health.reset()
                    home_att = attitude_near(atts, t_frame)   # NEW: tilt + IMU check (thread-safe)
                    imu_check.set_home(home_att[2] if home_att else None)
                    recovery.set_home(gray, origin=True)         # NEW: fingerprints for re-finding home
                    rehome.reset()                               # NEW: positions count from THIS home
                    home_saved, home_pending = True, False
                    home_height = fixed_height or link.height(t_capture)   # Part D: scale reference for AutoReanchor
                    bench_height = fixed_height
                    origin, at_origin = dict(ref=dict(tracker.__dict__), att=home_att, height=home_height), True
                    reset_counter = (reset_counter + 1) % 256    # tells the EKF "position jumped to 0"
                    prev_ne, vel_ned = None, np.zeros(3)
                    print(f"HOME saved automatically ({n} corners, trigger '{args.trigger}')")
                elif t_capture - last_texture_warn > 2.0:
                    print(f"Not enough ground texture for home ({n} corners) - retrying")
                    last_texture_warn = t_capture
            if old_ref is not None and home_pending:             # NEW: new home failed (poor texture):
                tracker.__dict__.update(old_ref)                 # keep tracking the OLD home meanwhile
            if not home_saved:
                live.state(ready=False, status="NO HOME", reason="no home picture yet (not enough ground texture)",
                           engaged=getattr(link, "engaged", False), sticks=getattr(link, "last_sticks", None))   # STAGE 2: live view
                if config.STATUS_LINE_HZ and not replay and t_capture - last_status >= 1.0:
                    last_status = t_capture
                    print("NOT READY  no home picture yet (not enough ground texture)", flush=True)
                continue

            # ---------- 3. Measure + judge ----------
            att_real = attitude_near(atts, t_frame)    # NEW: thread-safe; None = missing/stale IMU
            att = att_real or (0.0, 0.0, 0.0)                    # NEW
            if home_att is None and att_real:                    # NEW: home saved before IMU data arrived:
                home_att = att_real                              # use the first attitude (approximation)
                if at_origin:
                    origin["att"] = att_real
            res = tracker.track(gray, t_capture, attitude=att)
            prof.lap("track")                                    # NEW
            rep = health.check(gray, res)
            prof.lap("health")                                   # NEW
            if fixed_height and config.BENCH_HEIGHT_FROM_SCALE:
                # STAGE 2 bench: no rangefinder and the judge raises the aircraft. --fixed-height is the height
                # where the reference was saved; the floor looking smaller/bigger gives the height now.
                if res.ok:
                    bench_height = (home_height or fixed_height) / res.scale
                height = bench_height or fixed_height
            else:
                height = fixed_height if fixed_height else link.height(t_capture)
            if not fixed_height and config.USE_HEIGHT_FILTER:   # NEW: smooth, ignore spikes
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
                # STAGE 2: a side whose points the tracker already levelled with the IMU (USE_LEVEL_POINTS) has
                # no tilt shift left: give cancel_tilt roll = pitch = 0 for that side.
                a_now = (0.0, 0.0, att_real[2]) if tracker.now_levelled else att_real
                a_home = (0.0, 0.0, home_att[2]) if tracker.ref_levelled else home_att
                shift = cancel_tilt(res.shift_px, a_now, a_home, tracker.focal_px, res.rotation_deg)
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
            # STAGE 2 bench: also when the picture slid sideways by REHOME_SHIFT_FRAC of its width (1.5 m carry at 1 m)
            if config.USE_REHOME and quality > 0 and att_real and rehome.due(rep.status, res, gray.shape[1::-1]):
                if rehome.rehome(tracker, health, gray, t_capture, north, east, MIN_HOME_CORNERS):
                    home_att = att_real
                    if not (fixed_height and rehome.scale_min <= res.scale <= rehome.scale_max):
                        home_height = height                     # Part D: new scale reference. (STAGE 2 bench: a
                        # switch caused by sliding keeps the old height, or scale noise would pile up per switch)
                    imu_check.set_home(att_real[2])              # rotation restarts at 0 here
                    recovery.set_home(gray)
                    at_origin = False
                    print(f"Re-homed at {height:.1f} m (#{rehome.count}), offset N={north:+.2f} E={east:+.2f}")

            # STAGE 2 bench (steps 2-3 + recovery bonus), two additions around the re-home above:
            frame_i += 1
            # (a) Blind for a while and nothing we know is in view (the camera was covered while the aircraft was
            #     carried on): take a new reference HERE and keep counting from the last trusted position. Never
            #     while the pilot is holding on us: there, LOST must stay LOST (failsafe, handover.py).
            if (config.USE_REHOME and config.LOST_KEYFRAME_AFTER_S and rep.status == "LOST" and not handover.engaged
                    and prev_ne is not None and t_capture - prev_ne[2] > config.LOST_KEYFRAME_AFTER_S
                    and att_real and height is not None and frame_i % 5 == 0):
                if rehome.rehome(tracker, health, gray, t_capture, prev_ne[0], prev_ne[1], MIN_HOME_CORNERS):
                    home_att, home_height, at_origin = att_real, height, False
                    imu_check.set_home(att_real[2])
                    recovery.set_home(gray)
                    prev_ne, vel_ned = (prev_ne[0], prev_ne[1], t_capture), np.zeros(3)
                    print(f"LOST for {config.LOST_KEYFRAME_AFTER_S:g} s with nothing known in view: new reference here, "
                          f"position continues from N={prev_ne[0]:+.2f} E={prev_ne[1]:+.2f}")
            # (b) The reference is a later picture and the FIRST home picture is in view again: measure against it
            #     again. Whatever drift the chain of references collected on the way is gone.
            if (config.USE_ORIGIN_RETURN and origin and not at_origin and att_real
                    and frame_i % config.ORIGIN_CHECK_EVERY_N == 0):
                M = recovery.find_origin(gray)
                if M is not None:                                # where we are vs the first home, in pixels
                    cx, cy = origin["ref"]["centre"]
                    back = (float(cx - (M[0, 0] * cx + M[0, 1] * cy + M[0, 2])),
                            float(cy - (M[1, 0] * cx + M[1, 1] * cy + M[1, 2])))
                    # only when well inside the re-home distance, or the two switches would take turns
                    if math.hypot(*back) > 0.5 * (config.REHOME_SHIFT_FRAC or 0.25) * gray.shape[0]:
                        M = None
                if M is not None:
                    tracker.__dict__.update(origin["ref"])       # set_reference reassigns, never mutates
                    tracker.relock(M)
                    recovery.use_origin()
                    rehome.to_origin()
                    home_att, home_height, at_origin = origin["att"], origin["height"], True
                    imu_check.set_home(home_att[2] if home_att else None)
                    health.prev_shift, health.prev_shift_t = back, t_capture   # health's jump check starts here
                    prev_ne = None                               # the position may step: no fake velocity from it
                    print("First home picture is in view again: measuring against it (drift removed)")

            # ---------- 4. Send to the autopilot ----------
            down = link.local_z[1] if link.local_z else 0.0      # ignored: height source is Baro
            send_n, send_e = (north, east) if quality > 0 else (0.0, 0.0)
            # STAGE 2 (bench steps 6-7): not ready on ArduCopter = "receives no position" -> send nothing at all.
            # Betaflight always gets the call: its link turns "not ready" into centred sticks.
            if quality > 0 or betaflight or not config.ARDUPILOT_SILENT_WHEN_NOT_READY:
                link.send_odometry(t_capture, send_n, send_e, down, att,
                                   ned_to_body(vel_ned, *att), quality, reset_counter,
                                   metres_per_px=height / tracker.focal_px if quality > 0 else None)   # NEW
            prof.lap("send")                                     # NEW
            monitors.frame(t_capture, height, tracker.focal_px, quality, north, east,   # NEW (Ayush A4/A5):
                           rep.status, reset_counter)            # after the send; can't change it
            prof.lap("safety")                                   # NEW (Ayush): its cost per frame

            # ---------- 5. Source switching (replaces 'v' and 'g') ----------
            if rep.status == "OK" and quality > 0:
                want_source = SRC_VISION
            elif rep.status == "LOST" and fallback and link.acked_source == SRC_VISION:
                want_source = SRC_GPS                            # only if we had switched to vision
            if want_source and link.acked_source != want_source and now - last_source_try > 1.0:
                link.set_source(want_source)                     # retried until the autopilot says OK
                last_source_try = now

            # Part D: pilot handover. Only acts while the pilot has selected a hold mode.
            age = link.mode_age()
            if link.conn.flightmode in HOLD_MODES and age > 3.0 and now - last_hb_warn >= 5.0:
                print(f"WARNING: no fresh autopilot heartbeat for {age:.1f} s - failsafe cannot act")
                last_hb_warn = now
            hnow = hclock()
            if act_on(handover.update(hnow, link.conn.flightmode, quality, mode_fresh=age < 1.5)):
                reanchor_request = True                          # next frame saves home + bumps reset_counter
            # Part D: pilot flying by hand + vision LOST + the ground looks far from home's scale (home was saved on
            # the ground, then we climbed) -> save a new home HERE, so vision is ready BEFORE the pilot engages.
            # Without this ArduPilot never gets a position and refuses LOITER (no GNSS to fall back on).
            h_ok = height is not None and height >= MIN_HEIGHT_M
            scale_far = h_ok and (home_height is None or home_height < MIN_HEIGHT_M
                                  or not 0.7 <= height / home_height <= 1.4)
            if auto_reanchor.update(hnow, handover.engaged, rep.status == "LOST" and scale_far):
                reanchor_request = True
                print(f"Vision LOST while the pilot flies by hand (height {height:.1f} m, home was at "
                      f"{home_height or 0:.1f} m): saving a new home here")

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
            # STAGE 2 bench: "your laptop shows your live estimate". forward/right are along the aircraft's nose
            # as it pointed when home was saved.
            if config.STATUS_LINE_HZ and not replay and t_capture - last_status >= 1.0 / config.STATUS_LINE_HZ:
                last_status = t_capture
                g = apb_gyro.rates()
                imu = "" if g is None else f"   imu gyro {g[0]:+.2f} {g[1]:+.2f} {g[2]:+.2f} rad/s"
                if quality > 0:
                    yaw0 = origin["att"][2] if origin["att"] else att[2]
                    fwd, right = (north * math.cos(yaw0) + east * math.sin(yaw0),
                                  -north * math.sin(yaw0) + east * math.cos(yaw0))
                    print(f"READY      distance {math.hypot(north, east):5.2f} m   forward {fwd:+5.2f}   right {right:+5.2f}"
                          f"   height change {height - (origin['height'] or height):+5.2f} m   q={quality}{imu}", flush=True)
                else:
                    print(f"NOT READY  {rep.status}: {'; '.join(rep.reasons) or 'no height'}{imu}", flush=True)
            live.state(ready=quality > 0, status=rep.status,           # STAGE 2: live view (the same estimate as
                       reason="; ".join(rep.reasons) or ("" if quality > 0 else "no height"),   # the status line)
                       quality=quality, north_m=north, east_m=east,
                       yaw0=(origin["att"][2] if origin and origin["att"] else att[2]),
                       height_m=height,
                       height_change_m=(height - origin["height"]) if (origin and origin["height"] and height) else None,
                       engaged=getattr(link, "engaged", False), sticks=getattr(link, "last_sticks", None),
                       relocks=recovery.relocks, rehomes=rehome.count)
            if is_camera and not isinstance(cap, LatestFrame):   # (the grab thread already drops them)                                        # NEW: fell behind -> drop stale frames
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
        if home_saved and (betaflight or not config.ARDUPILOT_SILENT_WHEN_NOT_READY):
            att = link.attitude_at(time.time()) or (0.0, 0.0, 0.0)
            for _ in range(5):                                   # "our data is no longer valid"
                link.send_odometry(time.time(), 0.0, 0.0, 0.0, att, (0, 0, 0), -1, reset_counter)
                time.sleep(0.02)
        if fallback and link.acked_source == SRC_VISION:
            link.set_source(SRC_GPS)
            time.sleep(0.5)
        link.running = False
        monitors.close()                                         # NEW (Ayush A4/A5): before anything that
        print(trial.summary())                                   # NEW (Ayush A1)     can raise (see below)
        cap.release()
        log_file.close()
        if recorder:                                             # NEW
            recorder.close()
            print(f"Recording saved in {rec_dir} ({recorder.n} frames)")
        if args.show:                                            # PROPOSED (C6): no window was opened
            cv2.destroyAllWindows()                              # without --show; headless OpenCV raises here
        if prof.frames:                                          # NEW: compute summary for the slide
            print(f"\n{prof.table()}\nRe-locks: {recovery.relocks}, stale frames dropped: {budget.dropped}")
        print(f"Done. Log saved to {log_path}")


if __name__ == "__main__":
    main()
