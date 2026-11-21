# =====================================================================================
# config.py  -  settings for the NEW pieces (rolling shutter, tilt, recovery, ...)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   One place for every knob we added on top of Saumya's code. Each improvement has an
#   on/off switch, so we can fly (or replay) WITH and WITHOUT it and show the difference
#   on a slide. Saumya's own settings stay in her files (tracker.py, health.py,
#   vision_hold.py, link/drone_link.py).
#
# WHEN TO EDIT
#   - After calibration/calibrate_timing.py has run: nothing, it saves timing.npz itself.
#   - When the technical spec arrives (72 h before): failsafe action, frame rate.
# =====================================================================================

# ---------------- On/off switches (for "with vs without" proof) ----------------
USE_ROLLING_SHUTTER = True   # vision/rolling_shutter.py: undo the row-by-row warp
USE_TILT = True              # fusion/tilt.py: remove fake motion caused by roll/pitch
USE_LEVEL_POINTS = False     # vision/rolling_shutter.py (needs USE_TILT): do it the exact way: turn every tracked
                             # point into what a camera looking STRAIGHT DOWN would see (IMU roll/pitch) BEFORE
                             # matching. With the Justice D's 114 deg lens the old way (match first, subtract one
                             # shift after) goes LOST from 3 deg of tilt. False = the old first-order fix only
USE_IMU_CHECK = True         # fusion/imu_check.py: reject frames where vision and IMU disagree
USE_HEIGHT_FILTER = True     # fusion/height.py: smooth height, ignore spikes
USE_RECOVERY = True          # recovery/recovery.py: re-find home after LOST
USE_FEATURE_FILTER = True    # vision/feature_filter.py: ignore props/OSD text (needs mask.png)
USE_REHOME = True            # vision/rehome.py: new reference picture while climbing/descending
USE_CLAHE = True             # vision/enhance.py: local contrast boost (high / hazy / flat light)

# ---------------- Contrast boost (CLAHE) ----------------
CLAHE_CLIP = 2.0             # higher = stronger boost (and more noise)
CLAHE_TILES = 8              # picture split into 8 x 8 tiles

# ---------------- Re-home during climbs ----------------
REHOME_SCALE_MIN = 0.85      # ground looks 15% smaller than in the reference -> new reference
REHOME_SCALE_MAX = 1.18      # ground looks 18% bigger (descending) -> new reference
                             # (health.py rejects outside 0.75..1.33, so switch well before)
REHOME_SHIFT_FRAC = 0.25     # Stage 2 bench: the picture slid by this fraction of its width or height -> new
                             # reference too (at 1 m a 1.5 m carry takes the first picture out of view).
                             # In flight this is 4.3 m or more from 10 m up, outside the 3 m hold limit, so a
                             # hold that is still scoring never triggers it. 0 = off
USE_ORIGIN_RETURN = True     # after re-homes: when the FIRST home picture is in view again, measure against it
                             # again (drift collected on the way is removed). Bench loop + recovery bonus
ORIGIN_CHECK_EVERY_N = 15    # look for the first home every Nth frame (ORB, ~10-20 ms) while away from it
LOST_KEYFRAME_AFTER_S = 1.0  # LOST this long while the pilot is NOT holding and nothing known is in view (camera
                             # was covered while the aircraft moved on): new reference here, position continues
                             # from the last trusted one. 0 = off
BENCH_HEIGHT_FROM_SCALE = True   # with --fixed-height H: H is the height where the reference was saved; the
                             # height now = H / (how much bigger the ground looks). Gives the bench its height
                             # change with no rangefinder (Stage 2 bench step 4)
STATUS_LINE_HZ = 5           # print READY / NOT READY + displacement + height change this often (0 = off).
                             # Watch it on the laptop: journalctl -u vision_hold -f

# ---------------- Rolling shutter timing ----------------
# Used ONLY if timing.npz is missing. calibration/calibrate_timing.py measures the real ones.
READOUT_S = 0.015            # top row -> bottom row read time. Justice D runs at 60 fps, so the
                             # readout must be < 1/60 s = 16.7 ms. Measure the real one!
CAM_IMU_OFFSET_S = 0.0       # middle row capture time minus our timestamp (negative = earlier)
TIMING_FILE = "timing.npz"
CALIB_FILE = "calib.npz"     # lens calibration (vision_hold loads this one; tests point it elsewhere)

# ---------------- Camera (Justice D datasheet + APB manual) ----------------
# 1920 x 1080 @ 60 fps YUYV (Stage 2; the datasheet said 1280 x 720), HFOV 114 deg / VFOV 86 deg, MIPI CSI-1 = /dev/video2 (CSI-2 = /dev/video3),
# no exposure/gain control. Use --camera 2 (or --camera /dev/video2: both are treated as a live camera).
PROCESS_WIDTH = 640          # downscale every frame to this width before tracking (None = full
                             # resolution). The camera matrix K is scaled to match, so metres are
                             # unchanged; 1920 -> 640 is ~9x less work on the APB's Cortex-A53s.
ATTITUDE_MAX_AGE_S = 0.1     # an attitude sample further than this from the frame time = no IMU data

# ---------------- IMU cross-check ----------------
IMU_CHECK_MAX_DEG = 5.0      # vision rotation vs IMU yaw change may differ by at most this
IMU_CHECK_DRIFT_DEG_S = 0.5  # STAGE 2, MEASURED 21 Nov 2026: with no compass, Betaflight's heading drifts while the
                             # drone sits still (0.16 deg/s on our aircraft). The check follows a slow drift up to
                             # this rate and still catches a sudden disagreement. 0 = the old fixed comparison

# ---------------- Height filter ----------------
HEIGHT_MAX_RATE_MPS = 3.0    # height can't change faster than this (else it's a spike)
HEIGHT_SMOOTH = 0.3          # 0..1, higher = follows faster, lower = smoother
HEIGHT_MAX_REJECTS = 5       # after this many spikes in a row, accept the new value (it's real)

# ---------------- Recovery (re-find home) ----------------
RECOVERY_EVERY_N_FRAMES = 3  # while LOST, try every Nth frame (ORB costs ~10-20 ms)
RECOVERY_MIN_INLIERS = 25    # matches that must agree before we trust a re-lock
RECOVERY_ORB_FEATURES = 1000

# ---------------- Failsafe ----------------
FAILSAFE_AFTER_S = 3.0       # no VALID position sent for this long while the autopilot flies on
                             # vision (LOST, IMU disagreement, no height, camera dead) -> act
FAILSAFE_ACTION = "LAND"     # "LAND", "ALT_HOLD" or "NONE". LAND matches ArduPilot's own
                             # EKF failsafe default (FS_EKF_ACTION = 1).
FAILSAFE_MODE_RETRIES = 5    # resend the mode change (1 per second) until the autopilot reports it
IMU_DISAGREE_RECOVER_FRAMES = 3   # this many IMU-disagreement frames in a row -> try recovery (relock)
AUTO_REANCHOR_AFTER_S = 2.0  # pilot flying by hand, vision LOST and the picture is far from home's scale (home saved on the
                             # ground, then climbed): save a new home here after this many seconds. 0 = off. Never acts while a
                             # hold is engaged. Without it LOITER cannot be engaged without GNSS (see handover.py AutoReanchor).

# ---------------- Autopilot ----------------
MIN_ARDUPILOT_VERSION = (4, 5, 0)   # TEAM DECISION: ArduPilot only, >= 4.5. Before 4.5 the autopilot
                             # ignores ODOMETRY's quality field (VISO_QUAL_MIN) and would fuse our
                             # "no data" packets as real positions. link/check_link.py enforces this.

# ---------------- Recording (--record) ----------------
# The APB has only 4 GB eMMC: record the downscaled grey frames, and to /sdcard when present.
RECORD_CODEC = "FFV1"        # lossless (replay sees identical pixels), ~250 MB/min worst case at
                             # 640x360 30 fps; "MJPG" ~90 MB/min but lossy (measured)
RECORD_MIN_FREE_MB = 300     # stop recording (not flying) when the disk has less free space than this

# ---------------- Confidence we send to ArduPilot ----------------
# Position error in PIXELS at the best / worst quality. link/drone_link.py multiplies it by
# metres-per-pixel (height / focal), so ArduPilot trusts us less the higher we fly.
# Saumya's quality-only formula stays the MINIMUM (0.17 m per axis at quality 50), so this
# only takes over higher up. Example at quality 50: up to ~40 m -> 0.17 m (hers),
# 60 m -> 0.25 m, 100 m -> 0.42 m. Tune from the tracker's real jitter in flight logs.
ODOM_PX_NOISE_BEST = 0.5     # quality 100
ODOM_PX_NOISE_WORST = 3.0    # quality 1

# ---------------- Compute budget ----------------
TARGET_FPS = 30              # processing rate we aim for; budget per frame = 1 / TARGET_FPS.
                             # (The camera runs at 60 fps; stale frames are dropped by frame age.)

# ---------------- Trial safety lock (Ayush, task A1: safety/trial_mode.py) ----------------
TRIAL_MODE = True            # True = GPS can NEVER take over: vision_hold sets --no-gps-fallback by itself
                             # and a guard on the link drops every "use GPS" command. Only
                             # `vision_hold.py --practice` turns it off (loud banner). The rules: GNSS must
                             # not aid the scored hold (Grand Final package Stage 1 Rev 1.3, p.2, p.7 Q14).
                             # preflight.py says NO-GO if this is False. Keep it True.

# ---------------- Pre-flight GO / NO-GO (Ayush, task A2: preflight.py) ----------------
PREFLIGHT_STRICT = False     # True = every WARN counts as NO-GO (set True, or pass --strict, for the trial)
PREFLIGHT_CAMERA_SIZE = (1920, 1080)  # Stage 2 package section 02: Justice D gives 1920x1080 @ 60 fps
PREFLIGHT_MIN_FPS = 50       # 60 fps nominal: below 50 measured -> WARN
PREFLIGHT_TEMP_WARN_C = 75   # the fan holds ~62 C, the board shuts itself down at 95 C (APB manual p.11)
PREFLIGHT_TEMP_NOGO_C = 85   # our choice: 10 C of margin below the 95 C shutdown
PREFLIGHT_SD_PATH = "/sdcard"   # APB manual p.11
PREFLIGHT_SD_WARN_MB = 2000  # ~8 min of lossless recording (RECORD_CODEC above: ~250 MB/min worst case)

# ---------------- Hold limits -> PASS / FAIL (Ayush, task A3: tools/hold_score.py) ----------------
# OFFICIAL since Stage 2 (Rev 2.0, section 06): a hold qualifies when, for 30 s from the engage switch going
# on, the GPS distance from the engage point stays within 3 m and the barometer height change within 2 m.
# RMS / time-inside are not judged (None). The 2 m vertical rule is not checked by hold_score.py: run the
# judges' own tools/hold_judge.py (Stage 2 download) on the FC log for the real verdict.
USE_HOLD_SCORE = True        # print PASS/FAIL at the end of tools/analyze_log.py and tools/truth_drift.py
HOLD_LIMITS_OFFICIAL = True  # the numbers below are copied from the Stage 2 package
HOLD_LIMITS_SOURCE = "Stage 2 Rev 2.0 (20 Nov 2026), section 06"
HOLD_MIN_SECONDS = 30.0      # a qualifying hold lasts 30 s from hold start
HOLD_MAX_DRIFT_M = 3.0       # largest horizontal distance from the engage point (m)
HOLD_MAX_VERTICAL_M = 2.0    # largest barometric height change (m): judged by hold_judge.py, not by hold_score.py
HOLD_RMS_DRIFT_M = None      # not judged in Stage 2
HOLD_RADIUS_M = None         # not judged in Stage 2
HOLD_MIN_INSIDE_PCT = None   # not judged in Stage 2
HOLD_MAX_SPREAD_M = None     # "stability" as wobble around the average position (m); None = not judged

# ---------------- Altitude Ceiling Advisor (Ayush, task A4: safety/ceiling.py) ----------------
# Watches only: predicts our position error from height, focal length and quality, and warns
# before a climb step that would fail the HOLD_* limits above. Never changes what is sent.
CEILING_ADVISOR = True       # on/off (log: logs/flight_<time>_safety.csv)
CEILING_TILT_SIGMA_DEG = 0.5 # leftover roll/pitch error after tilt cancel, per axis. SIMULATED: ArduCopter
                             # SITL EKF attitude vs truth (roll 0.62-0.69, pitch 0.25 deg). Measure in flight!
CEILING_MARGIN = 1.5         # vision may use only 1/1.5 of each limit (the rest: wind, controller)
CEILING_CLIMB_STEPS_M = (10, 20, 40, 60, 80, 100)   # climb ladder, docs/WORKLIST.md task 3.8
CEILING_WINDOW = 30          # median over this many trusted frames (1 s at 30 fps)

# ---------------- GNSS integrity monitor (Ayush, task A5: safety/gnss_monitor.py) ----------------
# WATCH ONLY: flags JAMMED / SPOOF SUSPECTED from MAVLink GPS_RAW_INT. Never used to navigate.
# OFF since Stage 2: "from engage to disengage your solution must not use GNSS (... ArduCopter GPS
# messages ...)" and judges inspect code and logs. Even watching could be read as use. Practice only.
# The thresholds are our choices (not in the PDFs); tune them from practice-flight logs.
GNSS_MONITOR = False         # on/off (log: logs/flight_<time>_safety.csv)
GNSS_REQUEST_HZ = 5          # ask the autopilot to stream GPS_RAW_INT this often (0 = don't ask)
GNSS_MIN_SATS = 6            # fewer satellites -> JAMMED (degraded)
GNSS_MAX_HDOP = 2.5          # worse horizontal dilution of precision -> JAMMED (degraded)
GNSS_TIMEOUT_S = 2.0         # no new GPS report for this long (after we had some) -> JAMMED
GNSS_JUMP_MIN_M = 20.0       # a GPS jump bigger than this ...
GNSS_JUMP_SPEED_MPS = 25.0   # ... and faster than this (90 km/h; no hovering drone does it) -> SPOOF SUSPECTED
GNSS_DISAGREE_M = 10.0       # GPS vs vision displacement differ by more than max(this, 4 x GPS accuracy) ...
GNSS_DISAGREE_S = 3.0        # ... for this long while vision is trusted -> SPOOF SUSPECTED
                             # (5 m / 2 s gave 3 false alarms in 10 min of honest, wandering GPS: measured)
GNSS_ANCHOR_S = 5.0          # GPS reference point = median of the first 5 s of fixes after home is saved
GNSS_ALARM_HOLD_S = 10.0     # keep showing SPOOF SUSPECTED this long after the last evidence

# ---------------- Camera format (STAGE 2, MEASURED on the APB 21 Nov 2026) ----------------
# Opened without asking, /dev/video2 gives 640x480. Asked for YUYV 1920x1080 it delivers 1920x1080.
# The driver does not report its frame rate (OpenCV returns -1), so we state it here.
CAMERA_SIZE = (1920, 1080)   # what compute/budget.open_camera() asks a Linux camera for
CAMERA_FOURCC = "YUYV"
CAMERA_FPS = 60
CAMERA_WARMUP_S = 1.5        # MEASURED 21 Nov 2026: the first frames after opening are dark (auto-exposure still
                             # settling). Home was saved from one of them and nothing could be tracked against
                             # it. Frames are thrown away for this long before the first one is used
CAMERA_THREAD = True         # MEASURED: read() always waits ~33 ms for the NEXT frame, on top of our own work
                             # (9-12 fps in total). True = a side thread keeps grabbing, the main loop takes the
                             # newest frame with its own capture time. False = the old way
CAMERA_RAW_GREY = True       # MEASURED: letting OpenCV turn every 1920x1080 frame into colour cost 38 ms a frame
                             # (12 fps in total). True = take the camera's own grey channel (the Y of YUYV)
                             # directly. False = the old way

# ---------------- Flight controller route (STAGE 2) ----------------
# "ardupilot"  = MAVLink ODOMETRY to ArduCopter (link/drone_link.py). Needs the team to flash and tune it.
# "betaflight" = stick commands over MSP to the firmware the aircraft ships with (link/betaflight_link.py +
#                the always-on sender link/msp_sender.py). vision_hold.py --fc overrides this.
FC = "betaflight"            # TEAM DECISION 21 Nov 2026: Betaflight. "ardupilot" = unused fallback (--fc ardupilot)
ARDUPILOT_SILENT_WHEN_NOT_READY = True   # Stage 2 bench steps 6-7: "not ready: ArduCopter receives no position".
                             # True = send NOTHING while not ready (was: zeros marked quality -1 = "ignore me").
                             # TO VERIFY IN THE SIMULATOR: with no messages ArduPilot may refuse to arm
                             # ("PreArm: VisOdom: not healthy") until vision is ready. False = the old way.
CAMERA_UPSIDE_DOWN = False   # True = turn every picture 180 deg first. Our maths assumes "top of the picture =
                             # nose". If bench step 5 shows BOTH roll and pitch pushing the wrong way, flip this.

# ---------------- Betaflight route: link ----------------
BF_SERIAL_PORT = "/dev/ttymxc2"   # Stage 2 section 03
BF_SERIAL_BAUD = 115200
BF_SEND_HZ = 50              # Stage 2: "50 Hz recommended"
BF_SENDER_PORT = 14601       # local UDP: vision program -> sender (stick requests)
BF_TELEMETRY_PORT = 14602    # local UDP: sender -> vision program (tilt, height, channels)
BF_TARGET_TIMEOUT_S = 0.3    # no fresh request from the vision program for this long -> centred sticks
BF_ENGAGE_CHANNEL = 10       # CH10 (TL2) = engage switch (Stage 2), on at or above:
BF_ENGAGE_PWM = 1800
BF_ARM_CHANNEL = 5           # CH5 = arm (MRM2-10AI manual p.13). Disarmed -> the barometer re-zeroes
BENCH_HEIGHT_M = 1.0         # Betaflight route, DISARMED = bench: the camera is this far above the floor when home
                             # is saved (Stage 2 section 05: "about 1 m"). Height changes then come from the
                             # picture's scale. Armed -> the barometer is used instead. No flag to forget
# CHECK ON THE AIRCRAFT with `python -m link.msp_sender --monitor` (not knowable without it):
BF_RC_THROTTLE_INDEX = 3     # where the throttle sits in Betaflight's MSP_RC answer (0-based). Betaflight's own
                             # order is roll, pitch, yaw, THROTTLE -> 3. If moving the throttle stick changes
                             # the 3rd number instead, set 2.
BF_ATT_ROLL_SIGN = 1         # MSP_ATTITUDE -> our convention (roll + = right side down, pitch + = nose UP).
BF_ATT_PITCH_SIGN = -1       # Betaflight reports nose DOWN as + (our reading of its source; verify: lift the nose,
                             # --monitor must show pitch going positive, else set 1)

# ---------------- Betaflight route: controller. EVERY NUMBER IS A STARTING GUESS, tune on the aircraft ----------------
BF_ANGLE_LIMIT_DEG = 60.0    # Betaflight `angle_limit`: degrees of lean at full stick. ASK THE CREW for the real
                             # value (do not plug in our own Configurator: the manual warns it can reset the FC)
BF_MAX_TILT_DEG = 8.0        # we never ask for more lean than this (also enforced again inside the sender)
BF_KP_DEG_PER_M = 6.0        # lean per metre of position error
BF_KD_DEG_PER_MPS = 10.0     # lean per m/s of speed (the brake)
BF_ROLL_SIGN = 1             # flip to -1 if bench step 5 shows roll pushing WITH the move
BF_PITCH_SIGN = 1            # flip to -1 if bench step 5 shows pitch pushing WITH the move
BARO_SAMPLING_HZ = 16        # MEASURED 21 Nov 2026: the APB's barometer is set to 1 reading a second and one
                             # read BLOCKS for 1-2 s. That froze the sender to 1 message a second. The sender now
                             # reads it on a side thread and asks the sensor for this rate (lost at reboot).
                             # 0 = leave the sensor's rate alone
BF_HEIGHT_HOLD = True        # nudge the throttle to keep the engage height (APB barometer). False = pilot's value
BF_ALT_KP_US_PER_M = 40.0    # throttle microseconds per metre too low
BF_ALT_KD_US_PER_MPS = 30.0  # throttle microseconds per m/s of sinking
BF_THROTTLE_AUTHORITY = 50   # the height hold may move the throttle at most this far from the pilot's value.
                             # Small on purpose: 1.36 kg on 10" props hovers low and reacts strongly
BF_HOVER_THROTTLE = 1300     # ONLY used if we have never seen the pilot's throttle. MEASURE at check-in
BF_THROTTLE_MIN = 1100       # we never send less (Stage 2: "never minimum values") ...
BF_THROTTLE_MAX = 1800       # ... or more than this
