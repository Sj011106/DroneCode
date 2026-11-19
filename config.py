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

# ---------------- Rolling shutter timing ----------------
# Used ONLY if timing.npz is missing. calibration/calibrate_timing.py measures the real ones.
READOUT_S = 0.015            # top row -> bottom row read time. Justice D runs at 60 fps, so the
                             # readout must be < 1/60 s = 16.7 ms. Measure the real one!
CAM_IMU_OFFSET_S = 0.0       # middle row capture time minus our timestamp (negative = earlier)
TIMING_FILE = "timing.npz"
CALIB_FILE = "calib.npz"     # lens calibration (vision_hold loads this one; tests point it elsewhere)

# ---------------- Camera (Justice D datasheet + APB manual) ----------------
# 1280 x 720 @ 60 fps, HFOV 114 deg / VFOV 86 deg, MIPI CSI-1 = /dev/video2 (CSI-2 = /dev/video3),
# no exposure/gain control. Use --camera 2 (or --camera /dev/video2: both are treated as a live camera).
PROCESS_WIDTH = 640          # downscale every frame to this width before tracking (None = full
                             # resolution). The camera matrix K is scaled to match, so metres are
                             # unchanged; 1280 -> 640 is ~4x less work on the APB's Cortex-A53s.
ATTITUDE_MAX_AGE_S = 0.1     # an attitude sample further than this from the frame time = no IMU data

# ---------------- IMU cross-check ----------------
IMU_CHECK_MAX_DEG = 5.0      # vision rotation vs IMU yaw change may differ by at most this

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
PREFLIGHT_CAMERA_SIZE = (1280, 720)   # Justice D datasheet; APB manual p.12: fixed 1280x720 @ 60 fps
PREFLIGHT_MIN_FPS = 50       # 60 fps nominal: below 50 measured -> WARN
PREFLIGHT_TEMP_WARN_C = 75   # the fan holds ~62 C, the board shuts itself down at 95 C (APB manual p.11)
PREFLIGHT_TEMP_NOGO_C = 85   # our choice: 10 C of margin below the 95 C shutdown
PREFLIGHT_SD_PATH = "/sdcard"   # APB manual p.11
PREFLIGHT_SD_WARN_MB = 2000  # ~8 min of lossless recording (RECORD_CODEC above: ~250 MB/min worst case)

# ---------------- Hold limits -> PASS / FAIL (Ayush, task A3: tools/hold_score.py) ----------------
# NOT IN THE PDFs YET. The Grand Final package (Stage 1 Rev 1.3) says Stage 2 will define "the approved
# altitude levels, qualifying hold and measurement rules" (p.2; also p.3 and p.8). Every number below
# is a PLACEHOLDER and is printed as such until HOLD_LIMITS_OFFICIAL = True. When Stage 2 arrives:
# copy each number, write the page into HOLD_LIMITS_SOURCE, set a limit to None if it is not judged.
USE_HOLD_SCORE = True        # print PASS/FAIL at the end of tools/analyze_log.py and tools/truth_drift.py
HOLD_LIMITS_OFFICIAL = False # True ONLY when the numbers below are copied from the Stage 2 package
HOLD_LIMITS_SOURCE = "Stage 2 package not published yet"
HOLD_MIN_SECONDS = 30.0      # PLACEHOLDER: shortest hold that counts (seconds)
HOLD_MAX_DRIFT_M = 1.0       # PLACEHOLDER: largest horizontal distance from the engage point (m)
HOLD_RMS_DRIFT_M = 0.5       # PLACEHOLDER: RMS distance from the engage point (m)
HOLD_RADIUS_M = 1.0          # PLACEHOLDER: radius for the "time inside" rule (m)
HOLD_MIN_INSIDE_PCT = 95.0   # PLACEHOLDER: % of the hold inside HOLD_RADIUS_M
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
