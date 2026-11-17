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

# ---------------- Rolling shutter timing ----------------
# Used ONLY if timing.npz is missing. calibration/calibrate_timing.py measures the real ones.
READOUT_S = 0.025            # top row -> bottom row read time. Typical for 30 fps CMOS FPV cams.
CAM_IMU_OFFSET_S = 0.0       # middle row capture time minus our timestamp (negative = earlier)
TIMING_FILE = "timing.npz"

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
FAILSAFE_AFTER_S = 3.0       # vision LOST this long (while flying on vision) -> act
FAILSAFE_ACTION = "LAND"     # "LAND", "ALT_HOLD" or "NONE". LAND matches ArduPilot's own
                             # EKF failsafe default (FS_EKF_ACTION = 1).

# ---------------- Compute budget ----------------
TARGET_FPS = 30              # camera frame rate; the time budget per frame is 1 / TARGET_FPS
