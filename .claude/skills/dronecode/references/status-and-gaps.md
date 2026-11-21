# Status, limitations and milestones

**Snapshot: 21 November 2026, `main` at `be0d084`, the day the Stage 2 package arrived.**
This file goes stale fastest. Check `git log`, run `scripts/stage2_check.py`, and read
`docs/WORKLIST.md` before repeating anything here; update this file when things change.

## Contents
1. Where the project stands
2. Stage 2 clashes: what in the repo now breaks a rule or an assumption
3. What the bench tests need that doesn't exist yet
4. The open decision: Betaflight or ArduCopter
5. Other known limitations
6. Questions for the organisers
7. Milestones
8. Docs that are out of date

## 0. UPDATE 21 Nov 2026 (read this first; it overrides the older sections below)

Branch `stage2-fixes` (from `main` be0d084), uncommitted. `scripts/stage2_check.py` shows the live state.

Done in code (all SIMULATED; nothing has run on the aircraft or the APB):
- Section 2 clashes 1 to 7: fixed. No gpioset anywhere, install to `/data` with offline wheels,
  1920 x 1080 everywhere (datasheet `calib.npz`, f = 623 px), ArduCopter baud 230400, GNSS monitor off,
  official hold limits (30 s, 3 m horizontal, 2 m vertical).
- Bench (section 3): re-home on sideways shift, return to the first home, lost keyframe, height from
  scale, READY / NOT READY status line, SIGUSR1 "zero here". `tests/test_bench.py`.
- Wide-lens tilt fix (levelling). `tests/test_level.py`.
- Section 4 decision: **the team chose Betaflight on 21 Nov 2026** (ArduCopter is an unused fallback; do not spend time on its open items). Both routes exist in code. Betaflight is the boot default (`--fc betaflight`):
  `link/msp.py`, `link/msp_sender.py`, `link/apb_sensors.py`, `link/betaflight_link.py`,
  `fusion/position_controller.py`, `deploy/msp_sender.service`, `tests/test_betaflight.py`.
  ArduCopter stays available with `--fc ardupilot` (silent when not ready, CH10 = Loiter).
  Design details: `architecture.md`, last section. Commands: `commands.md`, last section.

Still open, in order of risk:
1. Betaflight signs and constants are our reading of MSP, not measured: tilt signs, MSP_RC channel
   order, stick directions, `angle_limit`, hover throttle, camera orientation. All are `BF_*` settings
   in `config.py`; the bench procedure is in `commands.md`. First hour on the aircraft goes here.
2. Betaflight controller gains are untuned guesses and there is no simulator for this route. Flying
   it is a real risk; bench points (C) only need the signs.
3. APB sensor file names (IIO) are not checked on the board: `python -m link.apb_sensors`.
4. Frame time stamps use `time.time()`; the board's wall clock jumps when NTP arrives. Attitude
   stamps use the same clock in the sender, so they stay consistent with each other, but a jump
   mid-run would upset speeds for a moment.
5. No `timing.npz` (rolling-shutter timing bonus needs a measurement) and no `mask.png`.
6. ArduCopter route only: heading without a compass (`EK3_SRC*_YAW` = 1), "silent when not ready"
   and `RC10_OPTION,56` not yet tried in SITL.
7. Levelling fails at about 15 deg of lean; origin return can mismatch on identical floor tiles.
8. `tests/test_preflight` cannot run on the Windows laptop (symlink privilege); it runs on Linux.
9. Start-up time on the board (bench E2: ready within 60 s) is unmeasured.

Teammates' files touched on this branch: Saumya - `link/drone_link.py` (baud), `calibration/check_calib.py`,
`tests/test_tracker.py`, parts of `vision_hold.py`, `ardupilot/vision_hold.param`;
Ayush - `config.py`, `preflight.py` (comment), `tests/test_preflight.py`, `tests/test_hold_score.py`;
Praise - none (`recovery/`, `vision/rehome.py`, `vision/rolling_shutter.py` are Divyanshi's by git log).

## 0b. FIRST REAL MEASUREMENTS on the APB (21 Nov 2026, USB-C, props off)

- Install works: `/data/DroneCode`, venv with system site packages, offline wheels. The board needs
  `opencv-python-headless` (no libxcb); `requirements.txt` now picks it by platform.
- Self-tests pass on the board: `test_tracker`, `test_handover`, `test_betaflight`.
- IIO names are as guessed: `dps310` (`in_pressure_input`), `icm42605-gyro`, `icm42605-accel`.
  Barometer noise at rest about +-0.3 to 0.6 m. Gyro reads about -0.2 rad/s at rest (display only).
- Camera: opened without a format request `/dev/video2` gives 640x480. Asked for YUYV 1920x1080 it
  delivers it, at 31 fps through OpenCV's colour conversion. The driver reports fps as -1.
  `compute/budget.open_camera()` and `camera_fps()` handle both.
- Betaflight answers MSP on `/dev/ttymxc2` at 115200 (attitude and 16 channels). Throttle is at
  index 3 of MSP_RC. Heading drifts about 0.16 deg/s at rest (no compass): `imu_check` now follows
  slow drift (`IMU_CHECK_DRIFT_DEG_S`).
- With the radio apparently off, MSP_RC read throttle 885, CH5 1775, the rest 1500. CH5 >= 1500
  means "armed" to our code (barometer height instead of bench mode): check with the radio on.
- Not yet confirmed: tilt signs, engage switch, stick directions, start-up time, bench accuracy.
- Board wall clock is months off after a USB-only boot.
- Teammate review fixes applied: services never give up (`StartLimitIntervalSec=0`), sender
  freshness on the monotonic clock, NaN requests ignored.

## 1. Where the project stands
Built and passing in simulation (no real hardware yet):
- the whole vision pipeline: tracker, rolling-shutter correction, tilt cancel, IMU check, height
  filter, CLAHE, re-homing on climbs, health, ORB recovery;
- the ArduPilot output path: ODOMETRY with quality and height-scaled noise, EKF source switching,
  parameter file and loader, link check;
- pilot handover and failsafe (Part D), trial mode, preflight, side monitors, hold scoring, CI;
- record / replay, log analysis, simulator checks against ArduPilot SITL.

Test suite at `be0d084` on a Windows laptop (21 Nov): 23 of 24 files pass in about 4 min.
The one failure, `test_preflight`, is the Windows symbolic-link privilege (WinError 1314), not a
code fault. `scripts/stage2_check.py` on the same commit: 3 FAIL, 12 WARN.

Checked against a real ArduPilot simulator (ArduCopter 4.8.0 in Mission Planner, 21 Nov, the
user's laptop): `load_params` + `check_link` PASS; `sitl_link_check` 4/4; `sitl_odometry_check`
PASS (EKF accepted our ODOMETRY, source switch acknowledged in 0.02 s, max drift 1.1 cm).

Never done: running on the APB, opening the real Justice D, reading the IIO IMU, any calibration
on the real camera, any bench rehearsal, any flight. Every result number in the docs is simulated.

Decided before Stage 2 and now in question: "ArduPilot only (>= 4.5), no Betaflight". ArduCopter
4.7.2 satisfies the version, but it arrives untuned and unflown (see section 4).

## 2. Stage 2 clashes in the repo
| # | What | Where | Why it matters |
|---|---|---|---|
| 1 | `gpioset` commands | `deploy/vision_hold.service` (two `ExecStartPre` lines), `tools/discover_hardware.py --route-fc-uart` | Stage 2: do not change APB GPIO lines; they route the FC link and the pilot video |
| 2 | Camera assumed 1280 x 720 | `config.PREFLIGHT_CAMERA_SIZE`, `calibration/check_calib.py` (`SPEC_W, SPEC_H`), docs | The camera delivers 1920 x 1080 YUYV; preflight would say NO-GO; the datasheet fallback calibration would be written at the wrong size |
| 3 | Serial baud 921600 | `link/drone_link.py` `BAUD` (Saumya's file) | ArduCopter link is 230400; Betaflight is 115200 MSP |
| 4 | No Betaflight support at all | whole repo | Betaflight is the installed firmware; the code only speaks MAVLink to ArduPilot |
| 5 | Engage = entering LOITER/POSHOLD | `handover.py`, `vision_hold.HOLD_MODES` | Engage is CH10 (TL2) high. On ArduCopter CH10 must be mapped to a hold mode (to be worked out and verified in SITL); on Betaflight the switch hands the sticks to our commands |
| 6 | Compass yaw | `ardupilot/vision_hold.param` (`EK3_SRC1_YAW 1`, `EK3_SRC2_YAW 1`) | There is no magnetometer. The EKF needs another yaw arrangement; our North/East conversion uses the autopilot's yaw |
| 7 | "Not ready" still sends ODOMETRY (zeros, quality -1) | `vision_hold.py` send step | Bench D and E1 on ArduCopter: "receives no position". Sending nothing is the unambiguous reading |
| 8 | GNSS monitor reads GPS during the hold | `safety/gnss_monitor.py`, `config.GNSS_MONITOR = True` | Stage 2 forbids using "ArduCopter GPS messages" from engage to disengage; judges inspect code. Watch-only may still be read as use |
| 9 | Install location | `deploy/install.sh` docs say copy to `~` | Files must live in `/data`; the root filesystem is almost full |
| 10 | Hold limits are placeholders | `config.HOLD_*` (1.0 m max, 0.5 m RMS), `HOLD_LIMITS_OFFICIAL = False` | Official: 30 s within 3 m horizontal and 2 m vertical. The ceiling advisor uses these numbers too |
| 11 | Docs describe RC7 home switch, ArduPilot >= 4.5 only, 1280 x 720 | README, WORKLIST, hardware.md | Out of date after Part D and Stage 2 |

Also still true from before Stage 2:
- `calib.npz` is a 640 x 480 laptop-webcam file with no saved size. `vision_hold.py` only warns
  and uses it. Measured effect on Justice D frames: optical centre lands at (161, 118) instead of
  about (320, 180), undistortion moves points 25 px (median) up to 507 px, and a 10 deg turn
  produces about 2.9 m of fake shift at 20 m. `preflight.py` says NO-GO for it.
- No `timing.npz` (rolling-shutter timing is the 15 ms / 0 ms guess) and no `mask.png` (feature
  filter is off).

## 3. What the bench tests need
70 of 100 base points are bench items, and D and E must be banked before any flight.

- **B1 / B2 (20 pts): the tracker loses home on the bench.** It compares every frame with one home
  picture. At about 1 m the view covers roughly +-1.5 m by +-0.9 m of floor, so a 1.5 m carry or a
  3 m loop takes home out of the picture. `vision/rehome.py` takes a new reference only when the
  scale changes. It needs a second trigger on how far the picture has shifted (same offset
  bookkeeping), i.e. keyframe-style odometry.
- **Height for scale on the bench.** `link.height()` is the autopilot's height relative to where
  it started, not above the floor, and below 0.3 m the code refuses to output. The bench is "about
  1 m above the floor": needs a bench height source (a set value, or estimated).
- **B3 (5 pts): a displayed height change** of 0.5 m within 0.15 m. Nothing displays height change
  today. Candidates: barometer difference (APB DPS310 or FC), cross-checked with the tracker's
  scale (`h_new = h_old / scale`).
- **A live display on the team's laptop** of displacement and height change. Today the program
  only prints events; `--show` needs a screen on the board.
- **C (10 pts): works with the laptop unplugged**, started by the boot service.
- **D (15 pts): engage on CH10, clean disengage, true "not ready"** (see clash 5 and 7).
- **E1 (8 pts): camera covered -> not ready within 1 s.** The health logic already does this
  (dark frame fails at once); the output behaviour must match the chosen route.
- **E2 (7 pts): APB power-cycle -> ready within 60 s** by itself. Needs the service enabled, a
  measured start-up time on the board, and (Betaflight) safe values from the first command.
- **Recovery bonus (+5):** camera covered for 3 s mid-loop, position within 0.5 m at the start
  mark. Recovery searches only the latest reference; searching the original home as well would
  snap the position back when the drone returns over the start mark.

## 4. The open decision: Betaflight or ArduCopter
| | Betaflight (installed) | ArduCopter 4.7.2 (optional) |
|---|---|---|
| Airframe readiness | tuned and flying | default parameters; frame, motor order, receiver, modes and tune are the team's work; not flown by the organisers; crew approval needed; flash risk |
| What must be written | MSP link; a position controller (position error -> roll/pitch) and a height controller (barometer -> throttle); continuous safe output from start-up (no command timeout) | CH10 -> hold-mode mapping; a no-compass yaw setup; baud; "no position when not ready" |
| What is reused | vision, health, recovery, replay, tests | nearly everything, including handover, trial mode and the SITL checks |
| Practice | no simulator | ArduPilot SITL (already in use) |
| Bonus | +5 for a qualifying hold | none |

The vision estimator is the same on both routes; only the output layer differs. Nobody on the
team is a hardware person, which weighs against tuning an untuned airframe with little flight time.
The team has not decided. Don't assume either route; ask, or state which route an answer assumes.

## 5. Other known limitations
- Tilt fix is first order (residual grows with tilt squared).
- Attitude comes at 50 Hz over MAVLink, stamped on arrival: it can't follow vibration above about
  25 Hz. Stage 2 offers better: the APB's ICM-42605 at up to 4 kHz on the same clock as the camera,
  and camera timestamps marking the end of the frame. Not used yet (this is the timing bonus).
- Position only; velocity is computed but not fused.
- Each re-home freezes in the error of that moment (keyframe drift).
- Health's blur threshold is absolute, tuned on a webcam; CLAHE works around it.
- Feature tracking needs texture; the judging floor is "as it is" and unseen.
- 1920 x 1080 YUYV at 60 fps is more data to convert and shrink than assumed; untested on the A53s.
- If the service restarts the program mid-flight, the reset counter starts again at 0 and the new
  process doesn't know the autopilot is still on vision until it re-acknowledges.
- The ceiling advisor's 0.5 deg tilt error is a simulator number and dominates its prediction.
- Camera mount angle (0 to 30 deg adjustable) and image orientation relative to the nose are unknown.

## 6. Questions for the organisers
1. Does an EKF heading that was set with GPS before engage count as "GPS-aided state" during the hold?
2. Betaflight: what throttle is expected at engage; may we read the pilot's sticks over MSP?
3. ArduCopter: which RC channels carry arm and flight mode on the event radio; is mapping CH10 to a hold mode acceptable?
4. Is a watch-only GPS integrity monitor during the hold allowed?
5. On vision loss during a hold, may the solution command LAND, or must it only hand back?
6. What is the judging floor like; what is the camera mount angle and orientation?

## 7. Milestones
| # | Milestone | Done when | Unlocks |
|---|---|---|---|
| M0 | Choose Betaflight or ArduCopter; one owner for the airframe | written in `docs/WORKLIST.md` | everything after M4 |
| M1 | Make the repo legal for Stage 2: remove `gpioset`, install to `/data`, 1920 x 1080, baud, GNSS monitor off at engage | `stage2_check.py` shows no FAIL | no rule violations |
| M2 | Run on the APB: offline wheels for Python 3.12 aarch64 (opencv headless, pymavlink, fastcrc), open `/dev/video2`, read the IIO IMU, benchmark | one process shows camera and IMU live | bench A (5) |
| M3 | Calibrate the real camera: lens at 1920 x 1080, timing, image orientation | `check_calib` passes; readout under 16.7 ms | correct metres |
| M4 | Bench estimation: re-home on shift, bench height, height-change display, live laptop display, recovery to the original home | at home, 5 times each: 1 m carry within 0.25 m; 3 m loop within 0.3 m; 0.5 m raise within 0.15 m; cover mid-loop within 0.5 m | bench B (25) + recovery bonus (5) |
| M5 | Output adapter for the chosen FC | laptop unplugged, correct sign for all four 0.5 m moves | bench C (10) |
| M6 | Engage on CH10, disengage, true not-ready | 10 clean on/off cycles ready and not ready | bench D (15) |
| M7 | Camera cover and APB power-cycle | not ready within 1 s; ready within 60 s after power-up, 5 times each | bench E (15); **permission to fly** |
| M8 | Judged bench session(s): declare only what passes reliably | D and E banked | flight |
| M9 | First development flight at 10 m, recorded | `hold_judge.py` shows a 30 s hold within 3 m / 2 m | G level 1 (12) |
| M10 | Climb the ladder: 20, 35, 50, 75 m | known ceiling; 2 scored flights used | G up to 20; bonuses |
| M11 | Presentation with real numbers, ours vs libraries, what failed and why | rehearsed in time | F (10) + timing bonus (5) |

## 8. Docs that are out of date
README (feature filter "to write", RC7 trigger wording in places, test list, "tests never read
calib.npz"), `docs/hardware.md` (1280 x 720, gpioset advice, rangefinder and compass assumptions),
`docs/WORKLIST.md` phases 2 to 4, `docs/EVIDENCE.md` (areas now have concrete pass conditions),
`ardupilot/vision_hold.param` header ("Copter 4.3+").
