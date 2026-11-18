# Evidence for the judges (areas A–G)

The Stage 1 package says credit follows **demonstrated, evidenced** progress in seven areas, even without a
qualifying hold, and that "claims alone will not substitute for evidence". This page maps each area to
**what we show, where it lives, and what is still missing**. Keep it up to date after every test session:
paste the real numbers and the file names of the logs and plots.

> **Team decision: flight controller firmware = ArduPilot (≥ 4.5). Not pursuing PX4, INAV or the optional
> Betaflight bonus route.**

Legend: ✅ have it (simulated or real, as stated) · 🟡 tool ready, real data missing · ⬜ not started

## A. Sensor data and calibration

| What judges want | Our evidence | Status |
|---|---|---|
| Usable camera + inertial data | `tools/discover_hardware.py` output (cameras, sample frames, FC link, IMU rate); `link/check_link.py` (attitude Hz, height) | 🟡 run on the APB |
| Understandable calibration approach | Lens: `calibration/auto_calibrate.py` → `calib.npz` (Zhang's method). Timing: `calibration/calibrate_timing.py` → `timing.npz` (camera–IMU offset + readout from video and IMU alone) | 🟡 the repo's calib.npz is a webcam's: redo on the Justice D |
| Data quality and timing account | `tests/test_rolling_shutter.py`: recovers a 30 ms readout as **29 ms** and a −20 ms delay as **−19 ms** (simulated) | ✅ sim / 🟡 real |

## B. Visual-inertial estimation

| What judges want | Our evidence | Status |
|---|---|---|
| Motion estimates backed by measurements | `tools/truth_drift.py` (vision and EKF error vs GPS per altitude band, practice flights with GPS as referee only) | 🟡 needs flights |
| Explain drift | Absolute reference to home (no frame-to-frame accumulation); re-homing adds ~1 hold error per switch (`tests/test_rehome.py`: 10→40 m, **6.7–8.0 cm** mean error) | ✅ sim |
| Explain scale | Pixels → metres from barometer/rangefinder height (`fusion/height.py`); EKF trust falls with height (`link/drone_link.py` noise) | ✅ |
| Rolling-shutter effects | Per-row IMU correction (`vision/rolling_shutter.py`): removes **92%** of the RS error (`test_rolling_shutter`); replay of one flight with the fix off: trusted frames **94 → 56** (`test_replay`) | ✅ sim / 🟡 real |
| Tilt | `fusion/tilt.py`: fake drift at 10° **94 → 9.8 cm** (`test_tilt`); replay with the fix off: **3.8 → 50.2 cm** (`test_replay`) | ✅ sim |

## C. Onboard execution

| What judges want | Our evidence | Status |
|---|---|---|
| Runs on the APB | `deploy/install.sh`, `deploy/vision_hold.service` (boot, restart) | 🟡 |
| Processing performance | `compute/benchmark.py` table (ms per stage, FPS, CPU, memory); every flight log has ms per stage (`tools/analyze_log.py` compute plot). Laptop: ~7 ms/frame | 🟡 run on the APB |
| Limited resources / competing services | Process at 640×360 (`config.PROCESS_WIDTH`, same metres: `tests/test_process_scale.py`); stale-frame dropping by age (`tests/test_budget.py`: 20 ms/frame on a 1-frame queue → **20 → 50 fps** vs the old rule) | ✅ sim / 🟡 APB |
| Repeatable execution | Record once, replay identically (`test_replay`: **90/90** positions identical) | ✅ |

## D. Flight controller integration and pilot handover

| What judges want | Our evidence | Status |
|---|---|---|
| Integration with the event FC | ArduPilot ≥ 4.5 (`check_link` enforces), `ardupilot/vision_hold.param` + `load_params.py` (verify / write / renamed params) | 🟡 needs the event build |
| Engage / disengage without disruption | Vision runs from takeoff, EKF switched to vision **before** the pilot engages; the engage switch only changes the flight mode (LOITER holds where engaged). Re-homing keeps the position continuous while climbing | 🟡 procedure to confirm with Stage 2 |
| Evidence | Flight video + `analyze_log` health timeline + ArduPilot .bin log around the switch | ⬜ |

## E. Reliability and reproducibility

| What judges want | Our evidence | Status |
|---|---|---|
| Identifies degraded results | `health/health.py` (blur, dark, frozen, outliers, jumps) + `fusion/imu_check.py` (vision vs IMU yaw) + missing/stale IMU → quality −1 | ✅ |
| Behaves safely | Failsafe on **no valid position for 3 s while on vision** (`recovery/failsafe.py`, mode change resent until confirmed; `test_failsafe`, `test_replay` Test 4); ArduPilot EKF failsafe as backup | ✅ sim |
| Handles interruptions | `tests/test_recovery.py`: 2 s and 10 s blackouts with 1–2 m drift, trusted again **0.2 s** after the picture returns (never without recovery) | ✅ sim / 🟡 real |
| Reproduces results across runs | Replay toolchain + the same tests rerun on every change; climb ladder table in `docs/WORKLIST.md` 3.8 (several flights per altitude) | 🟡 |

## F. Engineering contribution and explanation

### Our contribution vs existing tools

| Existing tool (used as is) | What it gives us | What we built on top |
|---|---|---|
| **OpenCV** | corner detection, Lucas-Kanade, RANSAC, ORB, CLAHE, video I/O | the hold pipeline: reference-image tracking, rolling-shutter per-row IMU correction, tilt cancellation, timing calibration, health scoring, recovery, re-homing, replay |
| **NumPy** | arrays and maths | — |
| **pymavlink** | MAVLink messages | the link (`link/drone_link.py`), ODOMETRY with quality and height-scaled noise, param loader / checker |
| **ArduPilot** (EKF3 + LOITER controller) | final sensor fusion and flying | its configuration (source sets, VISO, limits, failsafe) and the decision to let a proven controller fly |
| **matplotlib** | plots | the analysis tools (`analyze_log`, `truth_drift`) |

Team code: everything in `vision/`, `fusion/`, `health/`, `recovery/`, `compute/`, `link/`, `ardupilot/`,
`calibration/`, `tools/`, `tests/`, `vision_hold.py`. The tracker, health checker, MAVLink link and lens
calibration are Saumya's; the rest builds around them.

### Experiments, decisions, limitations
- Experiments: every `tests/test_*.py` prints a number for a slide; real flights add `analyze_log` / `truth_drift` plots.
- Decisions: ArduPilot only; don't write our own flight controller; absolute reference to home + re-homing;
  first-order tilt fix (residual ~2 px at 5°, documented upgrade path); process at 640×360.
- Limitations to state honestly: simulated numbers until real flights; tilt fix second-order residual; re-homing
  error grows per switch; 50 Hz attitude can't correct vibration above ~25 Hz; webcam calib until recalibrated.

## G. Live position hold

| What judges want | Our evidence | Status |
|---|---|---|
| Controlled holds at increasing altitudes | Climb ladder (`docs/WORKLIST.md` 3.8): per height % time OK, real drift from `truth_drift` | ⬜ |
| Under vibration, wind, changing light | `test_altitude` (flat light 40–80 m: contrast boost **0% → 91%** trusted, the maximum possible after the 5-frame trust delay), real flights | ✅ sim / ⬜ real |

## Bonus routes (Stage 1 outline; points set in Stage 2)

| Route | Plan | Evidence needed | Status |
|---|---|---|---|
| GNSS-free initialisation | `vision_hold.py` sets the EKF origin itself (no GPS needed); vision initialises during the pilot's manual setup | a full flight with GPS disabled from power-on: log + video | 🟡 |
| Measured timing / rolling-shutter improvement | `calibrate_timing.py` + replay of one flight with `USE_ROLLING_SHUTTER` on/off | real recording, before/after numbers and plots | 🟡 |
| Demonstrated recovery | blackout recovery + failsafe | in-flight lens-cover test (2 s and 10 s), health timeline plot | 🟡 |
| Betaflight integration | **not pursued (team decision: ArduPilot only)** | — | — |
