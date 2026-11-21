# Architecture: how the code works

Describes `main` at commit `be0d084` (20 Nov 2026, after Ayush's safety layer and Praise's Part D).
Line numbers drift; file and function names are the stable handles. Verify against the code before
quoting a threshold.

## Contents
1. Big picture
2. One frame in `vision_hold.py`, in order
3. Tracking (Saumya's tracker) and the sign convention
4. Corrections added around it: rolling shutter, tilt, IMU check, height, CLAHE, re-home
5. Trust: `health.py`
6. Recovery
7. Talking to ArduPilot: `link/drone_link.py`, ODOMETRY, source sets, parameters
8. Handover and failsafe (`handover.py`)
9. Safety layer (`safety/`, `preflight.py`)
10. Compute
11. Record, replay and analysis tools
12. Tests
13. Worked numbers

## 1. Big picture
Two computers on the APB board. The i.MX8M Plus (Linux) runs our Python; the STM32H743 runs the
flight controller firmware. They talk over the serial port `/dev/ttymxc2`.

```
Justice D -> vision_hold.py (i.MX8) --position + trust--> flight controller --> motors
                    ^-------- attitude, height, mode, RC ---------'
```

The code as written targets **ArduPilot**: it sends MAVLink `ODOMETRY` messages, ArduPilot's EKF3
fuses them with its own IMU and barometer, and ArduPilot's LOITER mode flies. There is **no
Betaflight / MSP code in the repo**. The vision part (camera to trusted metres) does not depend on
the flight controller; only the output layer (`link/`, `handover.py`, `ardupilot/`) does.

## 2. One frame in `vision_hold.py`
Housekeeping each loop: heartbeat to the autopilot once a second; `SET_GPS_GLOBAL_ORIGIN` every 2 s
until the autopilot confirms an origin (so the EKF works with no GPS fix).

1. **Grab and timestamp.** `cap.read()`, then `t_capture` immediately. Camera queue is set to one
   frame (`compute/budget.make_low_latency`). If the read fails on a live camera: handover is fed
   quality 0, an ODOMETRY with quality -1 is sent (if home exists), and the loop keeps trying; it
   no longer exits.
2. **Downscale and enhance.** Grey, shrink to `config.PROCESS_WIDTH` (640) with `vision/enhance.downscale`,
   then CLAHE. On the first frame the lens file is loaded with a size check and K is scaled to the
   processed width.
3. **Attitude snapshot.** One frozen copy of the autopilot attitude history for this frame.
4. **Home.** `HOME_TRIGGER = "start"`: home is saved on the first frame with at least 50 corners.
   A new home is also saved when `handover.py` asks (`REANCHOR`: the pilot entered a hold mode) or
   when `AutoReanchor` fires. Saving home resets health, tilt reference, IMU-check yaw, recovery
   fingerprints and re-home offset, and bumps `reset_counter` (tells the EKF "position restarts at 0").
5. **Track** (`RollingShutterTracker.track`): shift, rotation, scale since home.
6. **Health** (`HealthChecker.check`): quality and status.
7. **Height**: `link.height()` (rangefinder if fresh, else the autopilot's local z), through
   `fusion/height.HeightFilter`. No height, or below 0.3 m: quality -1.
8. **IMU check**: vision rotation vs autopilot yaw since home. Disagreement or no attitude: quality -1.
9. **Recovery**: if LOST and failing, or 3 IMU disagreements in a row, ORB search and relock.
10. **Tilt cancel**, then **pixels to North/East metres** plus the re-home offset.
11. **Re-home** if the scale has left 0.85..1.18 while trusted.
12. **Send** `ODOMETRY` (position if quality > 0, else zeros with quality -1).
13. **Side monitors** (ceiling advisor, GNSS monitor): after the send, watch only.
14. **Source switching**: status OK and quality > 0 -> ask for EKF source set 2 (vision), retried
    once a second until acknowledged.
15. **Handover / failsafe** and **AutoReanchor** (section 8).
16. **Log** one CSV row (`logs/flight_<time>.csv`), drop stale frames if behind.

Shutdown (Ctrl+C or SIGTERM from systemd): five "no data" ODOMETRY packets, close files, print the
profiler table.

## 3. Tracking and the sign convention
`vision/tracker.py` (Saumya). At home: up to 300 Harris corners, kept per cell of a 4 x 4 grid so
they cover the image, refined to sub-pixel. Every frame:
1. Pre-align: warp the new frame by the last trusted home->now transform, so Lucas-Kanade only has
   to find a small leftover motion.
2. Pyramidal Lucas-Kanade forward and backward (21 x 21 window, 3 levels); drop points that don't
   come back within 1 px.
3. Undistort (lens file), then RANSAC `estimateAffinePartial2D`: a similarity (shift, rotation,
   uniform scale), inlier threshold 2 px.
4. **Centre trick**: where did the home image's optical centre go? `shift_px = centre - M(centre)`.
   A pure turn on the spot gives zero shift.

Convention (top of `tracker.py`): `shift_px` = how far the DRONE moved from home, in the CURRENT
image's axes, +x = image right, +y = image down, **top of the image = the drone's nose**, camera
straight down. `to_north_east(shift, height, focal, yaw)` converts to metres. `scale > 1` = ground
looks bigger = lower than at home.

Datasheet focal length: `focal_from_fov(width)` = (width / 2) / tan(57 deg): about 208 px at the
640-wide processing size. Whether the 1920 x 1080 mode has the same 114 deg field of view is not
stated anywhere; it must be measured by calibration.

## 4. Corrections added around the tracker
- **Rolling shutter** (`vision/rolling_shutter.py`, `vision/timing.py`): subclass of the tracker
  that extends its undistort step. Row time = `t_capture + offset + (row/rows - 0.5) * readout`.
  Each point's ray is rotated from its row-time attitude to the mid-row attitude (H = K R K^-1 per
  row). Attitude is interpolated from the autopilot's 50 Hz `ATTITUDE` stream, stamped on arrival.
  Defaults 15 ms readout, 0 offset until `calibration/calibrate_timing.py` writes `timing.npz`
  (it estimates offset by matching image motion to IMU motion, then readout by minimising the
  similarity-fit residual). Rotation only.
- **Tilt** (`fusion/tilt.py`): the nadir point appears at `o = f * (tan roll, tan pitch)` off
  centre; corrected shift `d = shift + o_now - R(rotation) * o_home`. First order: about 2 px
  residual at 5 deg, 9 px at 10 deg (simulated).
- **IMU check** (`fusion/imu_check.py`): `rotation_deg + (yaw - yaw_home)` must be within 5 deg.
- **Height filter** (`fusion/height.py`): reject steps faster than 3 m/s, smooth with alpha 0.3,
  accept a persistent step after 5 rejects.
- **CLAHE** (`vision/enhance.py`): local contrast boost (clip 2.0, 8 x 8 tiles), skipped on very
  dark or bright frames. Its measured benefit is on health's fixed blur threshold in flat light,
  not on tracking accuracy.
- **Re-home** (`vision/rehome.py`): new reference picture when scale leaves 0.85..1.18 while
  trusted; `offset += position at the switch`, so the reported position is continuous and no EKF
  reset is signalled. **It triggers on scale only**, not on how far the drone has moved sideways.
- **Feature filter** (`vision/feature_filter.py`, Saumya): mask (`mask.png`) for props, arms and
  burned-in OSD text, plus a glare check. Off until a `mask.png` is built from a recording.

## 5. Trust (`health/health.py`, Saumya)
A frame fails on any of: mean grey outside 25..230; Laplacian variance below 30; frozen (mean
change below 0.02 or non-advancing time, 5 frames); tracker failed; fewer than 15 inliers; under
30 % of points agree; inliers in under 25 % of the 4 x 4 cells; scale outside 0.75..1.33; shift
change faster than 1500 px/s. Score = 30 % inliers + 20 % ratio + 20 % spread + 15 %
forward-backward error + 15 % sharpness, mapped to 1..100. Hysteresis: 3 bad frames -> LOST;
5 good frames to regain. DEGRADED = a bad frame while not lost. Quality is -1 unless status is OK.

## 6. Recovery (`recovery/recovery.py`)
ORB fingerprints (1000) of the current reference picture. While failing, every 3rd frame: ORB on
the frame, brute-force Hamming match with cross-check, RANSAC similarity, at least 25 inliers ->
`tracker.relock(M)`. Health then needs its 5 good frames. Fingerprints are replaced at every
re-home, so it searches for the **latest** reference only.

## 7. Talking to ArduPilot
`link/drone_link.py` (Saumya): pymavlink, MAVLink 2, a reader thread that keeps the newest
attitude history (thread-safe `LockedDeque`), rangefinder, local z, RC channels, parameters, the
acknowledged EKF source, and the time of the last autopilot heartbeat (`mode_age()`, only the
flight controller's own heartbeat counts). Serial baud constant: 921600. It requests ATTITUDE
50 Hz, LOCAL_POSITION_NED 10 Hz, DISTANCE_SENSOR 20 Hz, RC_CHANNELS 10 Hz.

`send_odometry`: frame LOCAL_FRD / BODY_FRD, position North/East, the autopilot's own attitude
echoed back, velocity (not fused: `EK3_SRC2_VELXY = 0`), `quality`, and a covariance from
`sigma = max(0.02 + 0.3 * (1 - q/100), sigma_px(q) * height / focal)` where sigma_px runs from
0.5 px (quality 100) to 3 px (quality 1). ArduPilot gates on `VISO_QUAL_MIN = 30` and uses the
covariance as noise, never below `VISO_POS_M_NSE`.

`ardupilot/vision_hold.param` (loaded and verified by `ardupilot/load_params.py`, which refuses
while armed, maps renamed parameters on newer firmware, and never overwrites an RC option without
`--force`):
- EKF3 on; `AHRS_GPS_USE 0`.
- `VISO_TYPE 1`, `VISO_DELAY_MS 50`, `VISO_POS_M_NSE 0.1`, `VISO_QUAL_MIN 30`.
- Source set 1: GPS position and velocity, baro height, **compass yaw**.
- Source set 2: position ExternalNav (6), velocity none, baro height, **compass yaw**; `EK3_SRC_OPTIONS 0`.
- `ANGLE_MAX 2000`, `LOIT_SPEED 500`, `LOIT_ACC_MAX 250`, `FS_EKF_ACTION 1` (LAND), `RC7_OPTION 0`.
- Serial, GPS type, rangefinder, flight-mode channels: commented out, never set.

With `VISO_TYPE = 1` ArduPilot refuses to arm until ODOMETRY is arriving ("PreArm: VisOdom: not
healthy"), so the program must be running before the pilot arms.

`link/check_link.py`: read-only pre-flight check (firmware is ArduPilot >= 4.5, attitude rate
>= 40 Hz, height present, every parameter matches the file).

## 8. Handover and failsafe (`handover.py`, Praise)
Pure decision logic, no MAVLink. Hold modes = LOITER, POSHOLD.
- Not in a hold mode: do nothing (the pilot wins), state MANUAL.
- Entering a hold mode: return `REANCHOR` (vision_hold saves a new home at that spot).
- In a hold with quality > 0: HOLDING (or DEGRADED below quality 50).
- In a hold with quality <= 0 for `FAILSAFE_AFTER_S` (3 s): send `FAILSAFE_ACTION` (LAND) once,
  resend each second up to 5 times while the mode hasn't changed, then `GAVE_UP` (a STATUSTEXT
  "PILOT TAKE OVER"). A mode change is only sent when the autopilot heartbeat is fresh (< 1.5 s).
- `AutoReanchor`: pilot flying by hand, vision LOST and the height is far from home's (ratio
  outside 0.7..1.4, or home was below 0.3 m) for 2 s -> save a new home, so vision is trusted
  before the pilot engages. Never acts while engaged.

`recovery/failsafe.py` is no longer imported by `vision_hold.py`; it remains for its test.

## 9. Safety layer (Ayush)
- `safety/trial_mode.py`: default on. Forces `--no-gps-fallback` and wraps the link so anything
  that could select GPS is dropped (source set other than 2, raw COMMAND_LONG/INT, writes to
  `EK3_SRC*`, `AHRS_GPS_USE`, `GPS_TYPE`, `RCx_OPTION`, RC override). Off only with `--practice`.
- `safety/side_monitors.py`: runs the watchers after the send, in try/except, writes
  `logs/flight_<time>_safety.csv`.
- `safety/ceiling.py`: predicted RMS error = h * sqrt((sigma_px / f)^2 + 2 tan^2(sigma_tilt));
  warns before a climb step predicted to fail the `config.HOLD_*` limits.
- `safety/gnss_monitor.py`: reads `GPS_RAW_INT` (asks the autopilot to stream it at 5 Hz) and
  reports OK / JAMMED / SPOOF SUSPECTED / NO DATA. Watch only.
- `preflight.py`: GO / WARN / NO-GO for trial mode, busy devices, temperature, SD space,
  calibration (uses `check_calib`), timing file, camera size and frame rate, `check_link`, and
  autopilot safety parameters. `--bench` skips the autopilot; `--strict` turns WARN into NO-GO.
- `tools/hold_score.py`: PASS/FAIL against `config.HOLD_*` (still placeholders in the code).

## 10. Compute
`compute/profiler.py` times each stage per frame (grab, track, health, fusion, send, safety) into
the flight log and a shutdown table. `compute/budget.py` drops stale frames by frame age.
`compute/benchmark.py` runs the pipeline flat out on simulated or recorded video and writes
`logs/benchmark_<time>.md`. Laptop: about 7 ms per frame. Never run on the APB.

## 11. Record, replay, analysis
- `vision_hold.py --record [DIR]`: grey processed frames (`video.avi`, FFV1 lossless),
  `frames.csv`, `attitude.csv`, `meta.csv`. `--replay DIR` runs the same code on it with a fake
  link and recorded time; results are identical run to run. `--set NAME=VALUE` overrides a config
  switch for one run.
- `tools/analyze_log.py`: position, health and compute plots from a flight CSV.
- `tools/truth_drift.py`: real drift from an ArduPilot `.bin` log with GPS as referee.
- `tools/degrade_video.py`: the camera simulator used by the tests (`render`), and blur / wobble /
  blackout effects for real video.
- `tools/sitl_link_check.py`, `sitl_odometry_check.py`, `sitl_odometry_follow.py`,
  `sitl_handover_check.py`: checks against a running ArduPilot simulator.
- `tools/discover_hardware.py`: finds CPU, cameras, serial ports on the board.

## 12. Tests
24 files in `tests/`, each runs as a script and prints its numbers; `python -m tests.run_all`
runs them all (`--fast` skips the slow simulated flights). Simulated tests use the fixed test
camera in `tests/common.py`, except `test_tracker.py`, which reads the repo's `calib.npz`.
`tests/test_preflight.py` creates symbolic links and fails on Windows without Developer Mode.

## 13. Worked numbers (f = 208 px at 640 wide)
- Ground per pixel = height / f: 1 m -> 0.5 cm, 10 m -> 4.8 cm, 35 m -> 17 cm, 75 m -> 36 cm.
- At 1 m, the 114 x 86 deg view covers about +-1.5 m by +-0.9 m of floor.
- 5 deg of tilt = 208 * tan(5 deg) = 18 px of fake shift = 1.75 m at 20 m if uncorrected.
- 2 deg vibration at 6 Hz = up to 75 deg/s = 1.1 deg during a 15 ms readout = about 4 px of skew.
- Simulated results quoted in the docs: rolling-shutter fix removes 92 % of its error; tilt fix
  94 cm -> 9.8 cm at 10 deg; 10 -> 40 m climb with re-homing 6.7 to 8.0 cm mean error; trusted
  0.2 s after 2 s and 10 s blackouts; replay 90/90 identical.

## Added on 21 Nov 2026 for Stage 2 (branch `stage2-fixes`)

### Levelling: the wide-lens tilt fix (`config.USE_LEVEL_POINTS`, needs `USE_TILT`)
- Problem found: the tracker fits "slide, turn, zoom" (similarity) between home and now. With the
  real 114 deg lens a lean stretches one side of the picture, so from a few degrees of difference in
  tilt most points disagree and health says LOST, although the position is still right. Older
  tests used a narrower test lens (f = 416 px at 640 wide, not 208) and never showed it.
- Fix in `vision/rolling_shutter.py`: `correct_points(..., rolling, level)` rotates every point's
  ray to a camera looking straight down (roll = pitch = 0 at the middle-row time, yaw kept), for the
  home picture and every frame. `RollingShutterTracker(level=True)` sets `ref_levelled` /
  `now_levelled`. `vision_hold.py` then gives `cancel_tilt` zero roll/pitch for a levelled side, so
  nothing is corrected twice. No IMU data -> not levelled -> the old first-order `cancel_tilt`.
- The constructor default is `level=False` (older tests call `cancel_tilt` themselves); only
  `vision_hold.py` turns it on.
- Proof: `tests/test_level.py` (0 to 15 deg at 1 m and 20 m). Simulated result: trusted up to 10 deg
  with errors under 1 cm; the old way lost trust at 8 deg. Known limit: at 15 deg both fail, because
  the frame-to-frame search (pre-align warp + Lucas-Kanade) still runs on the un-levelled picture.

### Bench features (Stage 2 section 05)
- `vision/rehome.py` `due(status, res, size)`: also fires when the shift exceeds
  `REHOME_SHIFT_FRAC` (0.25) of the picture width or height. A switch caused by sliding keeps the
  old reference height (bench mode), so scale noise does not pile up.
- Origin return (`USE_ORIGIN_RETURN`): `recovery.set_home(gray, origin=True)` keeps the first home
  picture's ORB fingerprints; `vision_hold.py` keeps `origin = {ref: tracker state, att, height}`.
  Every `ORIGIN_CHECK_EVERY_N` frames while the reference is a later picture it calls
  `recovery.find_origin()`; when the first home is in view (scale 0.85..1.18, and closer than half
  the re-home distance, to avoid ping-pong) the tracker state is restored, `rehome.to_origin()`
  zeroes the offsets and the collected drift is gone.
- Lost keyframe (`LOST_KEYFRAME_AFTER_S` = 1 s): LOST, pilot not holding, nothing known in view ->
  new reference here, position continues from the last trusted one. Never while engaged.
- Bench height (`BENCH_HEIGHT_FROM_SCALE`): with a fixed height H (flag `--fixed-height`, or
  `config.BENCH_HEIGHT_M` on the Betaflight route while disarmed), height now = reference height /
  `res.scale`. Gives bench step 4 its height change with no rangefinder.
- Status line (`STATUS_LINE_HZ` = 5): `READY distance / forward / right / height change / q` or
  `NOT READY <reason>`, plus the APB gyro. Watch with `journalctl -u vision_hold -f`.
- Zero here: `SIGUSR1` saves a new home (`systemctl kill -s USR1 vision_hold`).
- Proof: `tests/test_bench.py` replays four moves through `vision_hold.main` with a Justice D-like
  camera at 1 m: 1.5 m carry, square loop, 0.5 m raise, loop with a 3 s cover.

### Betaflight route (`--fc betaflight`, the boot service default)
Two processes:
1. `link/msp_sender.py` (service `msp_sender`): owns `/dev/ttymxc2` (115200, MSP v1). Every 20 ms it
   sends `MSP_SET_RAW_RC` (roll, pitch, throttle, yaw) and asks for `MSP_ATTITUDE` and `MSP_RC`.
   - Not engaged, or no fresh request for `BF_TARGET_TIMEOUT_S` (0.3 s): 1500, 1500, 1500 and the
     hold throttle. This is the timeout Betaflight lacks.
   - Engaged (CH10 >= 1800) with a fresh request: the requested roll/pitch, capped again at
     `BF_MAX_TILT_DEG`.
   - Hold throttle = the pilot's throttle read just before engage (while engaged the FC reports
     our own value, so it is not re-learned). Never seen any -> `BF_HOVER_THROTTLE`.
   - Height hold (`BF_HEIGHT_HOLD`): PD on the APB's own DPS310 (`link/apb_sensors.py`, IIO sysfs),
     zeroed while disarmed (CH5 low), authority +-`BF_THROTTLE_AUTHORITY` (50). The FC's altitude
     is not used because it may blend GPS.
   - Telemetry to the vision process on UDP 127.0.0.1:14602; requests in on 14601.
   - `--monitor` prints tilt, channels and what is sent (bench sign check).
2. `vision_hold.py --fc betaflight` uses `link/betaflight_link.BetaflightLink`, a `DroneLink`
   subclass without MAVLink: `send_odometry()` rotates N/E into body axes with the same yaw that
   made them (so heading cancels: no compass needed), runs `fusion/position_controller.sticks()`
   (PD: 6 deg/m, 10 deg per m/s, cap 8 deg) and sends the request. Flight mode reads `POSHOLD`
   while engaged, so `handover.py` re-anchors home at engage. Engage guard: centred sticks until
   `reset_counter` changes (the new home is saved). Failsafe mode is `NONE` on this route.
- `link/msp.py` can only build commands 105, 108 and 200; a GPS request raises `ValueError`.
- Signs and unknowns that only the aircraft can confirm are settings: `BF_ROLL_SIGN`,
  `BF_PITCH_SIGN`, `BF_ATT_ROLL_SIGN`, `BF_ATT_PITCH_SIGN` (-1: Betaflight reports nose down as
  positive, by our reading), `BF_RC_THROTTLE_INDEX` (3), `BF_ANGLE_LIMIT_DEG` (60),
  `BF_HOVER_THROTTLE` (1300), `CAMERA_UPSIDE_DOWN`. All controller gains are starting guesses.
- Heading from MSP comes in whole degrees; the sender smooths it. Tilt comes at 0.1 deg, 50 Hz.
- Proof: `tests/test_betaflight.py` (pretend flight controller on a pretend serial port).

### ArduCopter route after Stage 2
- `ARDUPILOT_SILENT_WHEN_NOT_READY = True`: nothing is sent when not ready (was zeros with
  quality -1). To verify in SITL: ArduPilot may refuse to arm until vision is ready.
- `RC10_OPTION,56` (Loiter on CH10) added to `ardupilot/vision_hold.param`; verify in SITL.
- Still open: heading without a compass (`EK3_SRC*_YAW` is still 1).
