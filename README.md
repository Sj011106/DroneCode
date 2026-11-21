# DroneCode: Vision Position Hold Without GPS

**Red Team Hackathon Grand Final: Contested Battlefield Aerial Operations Challenge**

Hold an Orqa M2-AI drone over one spot with **no GPS**, using its downward camera (Justice D, rolling
shutter), its motion sensors and the APB onboard computer. The pilot flies up, flips the **engage
switch** (CH10), and from that moment our code holds the position until the pilot switches off.

> **Status, 21 Nov 2026 (Stage 2, Rev 2.0).** **Team decision: we fly Betaflight**, the firmware the
> aircraft ships with. The ArduCopter route stays in the code as an unused fallback (`--fc ardupilot`).
> **Every number in this file is simulated.** Nothing has run on the real aircraft or the APB yet.
> What must be checked on the real hardware first is listed in [section 9](#9-status-and-what-is-still-open).
>
> **Betaflight changes (21 Nov, local copy, not pushed):** Betaflight's real stick curve, an I term that starts at
> the pilot's lean, heading from the camera (Betaflight's heading is GPS-corrected), AUX off, engage hysteresis,
> a sender watchdog, bench protection against the holder's legs, optional height from motion, a heat guard.
> What changed, why, and what to check on the aircraft: [docs/BETAFLIGHT_CHANGES.md](docs/BETAFLIGHT_CHANGES.md).

---

## Contents

1. [The idea in plain words](#1-the-idea-in-plain-words)
2. [Two ways to fly it](#2-two-ways-to-fly-it)
3. [What happens every frame](#3-what-happens-every-frame)
4. [Folder structure](#4-folder-structure)
5. [Setup](#5-setup)
6. [How to use it](#6-how-to-use-it)
7. [Tests and results](#7-tests-and-results)
8. [Stage 2 rules and how the code meets them](#8-stage-2-rules-and-how-the-code-meets-them)
9. [Status and what is still open](#9-status-and-what-is-still-open)
10. [Settings](#10-settings)
11. [Team rules](#11-team-rules)
12. [Glossary](#12-glossary)

Other documents: [`docs/WORKLIST.md`](docs/WORKLIST.md) (what to do next, in order),
[`docs/EVIDENCE.md`](docs/EVIDENCE.md) (what we show the judges per scoring item),
[`docs/hardware.md`](docs/hardware.md) (hardware facts), [`docs/presentation_notes.md`](docs/presentation_notes.md),
[`docs/CAMERAS.md`](docs/CAMERAS.md) (front camera, blur, lens centre: tests to run on the APB).

---

## 1. The idea in plain words

1. **Take a photo of the ground** at the moment the pilot engages. That spot is **home**.
2. **Every frame, find the same ground features again** and measure how far the picture has slid.
   If the ground slid left, the drone moved right.
3. **Remove what the camera gets wrong.** A leaning drone sees a slid and stretched picture without
   having moved. The camera also scans row by row (rolling shutter), which warps the picture when
   the drone shakes. The motion sensors tell us how the drone leaned and shook, so we rotate every
   tracked point back to what a camera looking **straight down** would have seen ("levelling").
4. **Turn pixels into metres** with the height (barometer; on the bench, the picture's zoom).
5. **When the picture has moved or zoomed too far**, take a new reference photo and keep counting
   from the original home. When the original home comes back into view, snap back to it, which
   removes the error collected on the way.
6. **Grade every measurement.** Not trusted means "not ready", and the aircraft gets nothing from
   us that could move it.
7. **Act on it**, in one of the two ways in section 2.

---

## 2. Two ways to fly it

| | **Betaflight** (shipped firmware, **our choice**) | **ArduCopter 4.7.2** (fallback only, not planned) |
|---|---|---|
| Start with | `vision_hold.py --fc betaflight` + the `msp_sender` service | `vision_hold.py --fc ardupilot --connect /dev/ttymxc2` |
| We send | stick values (roll, pitch, throttle, yaw) 50 times a second, MSP at 115200 | our position (MAVLink ODOMETRY) at 230400 |
| Who steers | **our code**: `fusion/position_controller.py` | ArduCopter's own position hold (EKF3 + LOITER) |
| Height | our code: APB barometer, small throttle nudges | ArduCopter |
| Not ready | sticks centred (1500) + a throttle that holds height | nothing is sent |
| Simulator | none (a pretend flight controller in `tests/test_betaflight.py`) | ArduPilot SITL (`tools/sitl_*.py`) |
| Biggest risk | controller gains are untuned guesses; stick signs unconfirmed | firmware is untuned and unflown; no compass on the aircraft |
| Bonus | +5 for a Betaflight hold | |

### The Betaflight route in detail

```
 camera ─> vision_hold.py ── position error ──> betaflight_link.py ── wanted roll/pitch (UDP) ──┐
            (tracking, health)                    (position_controller.py)                       v
                    ^                                                                    msp_sender.py  ── MSP, 50 Hz ──> Betaflight
                    └──────────── tilt, height, radio channels (UDP) ─────────────────── (own boot service,
                                                                                          owns /dev/ttymxc2)
```

Betaflight has **no safety timeout**: it keeps flying the last stick message for ever, and before
the first message it reads "throttle cut, full tilt". So the sender is a separate small program:

- It starts at boot, before the vision program, and restarts by itself.
- Its first message is already safe: roll, pitch, yaw 1500 and a hold throttle.
- No fresh request from the vision program for 0.3 s, or not ready, or the engage switch off:
  centred sticks. This is the timeout Betaflight lacks.
- The lean we ask for is capped at 8 degrees, twice (in the controller and again in the sender).
- Throttle = what the pilot's own stick was giving just before engaging, nudged by at most ±50 to
  keep the height of the engage moment.
- `link/msp.py` can only build three messages (sticks, "how are you tilted?", "what does the radio
  send?"). A GPS request cannot be built, so the no-GNSS rule cannot be broken by mistake.
- Heading is not needed: the camera measures in the drone's own forward/right, and the sticks act
  in the same axes. The aircraft has no compass.

---

## 3. What happens every frame

| Step | What | Code |
|---|---|---|
| 1 | Grab the newest frame (drop stale ones), shrink 1920 x 1080 to 640 x 360, boost local contrast | `vision_hold.py`, `compute/budget.py`, `vision/enhance.py` |
| 2 | Home trigger: at start, when the pilot engages, or on "zero here" | `handover.py` |
| 3 | Track corners from the home picture; rotate each point to a straight-down camera using the tilt at the moment its row was scanned | `vision/tracker.py`, `vision/rolling_shutter.py` |
| 4 | Fit slide + turn + zoom, ignoring points that disagree | `vision/tracker.py` (RANSAC) |
| 5 | Grade the frame: OK / DEGRADED / LOST, and cross-check the turn against the motion sensors | `health/health.py`, `fusion/imu_check.py` |
| 6 | Height: barometer in flight, picture zoom on the bench | `fusion/height.py`, `link/apb_sensors.py` |
| 7 | New reference picture if needed; return to the first home when it is in view; search for home after a blackout | `vision/rehome.py`, `recovery/recovery.py` |
| 8 | Pixels to metres, then send (position on ArduCopter, stick request on Betaflight) | `link/drone_link.py`, `link/betaflight_link.py` |
| 9 | Print `READY distance / forward / right / height change` or `NOT READY <reason>` 5 times a second, log everything | `vision_hold.py` |

---

## 4. Folder structure

```
vision_hold.py            the main program (one loop, both routes)
config.py                 every setting and on/off switch
handover.py               when to save home; what to do when the pilot engages
preflight.py              one GO / NO-GO list to run on the APB before flying
calib.npz                 lens numbers (1920 x 1080, from the datasheet: 114 deg, f = 623 px)

vision/        tracker, rolling_shutter (+ levelling), timing, rehome, enhance, feature_filter
fusion/        tilt, height, imu_check, position_controller (Betaflight)
health/        health (trust score per frame)
recovery/      recovery (find home again), failsafe
link/          drone_link (ArduCopter, MAVLink) · msp, msp_sender, betaflight_link (Betaflight)
               apb_sensors (the APB's own barometer and gyro) · check_link
safety/        trial_mode (no GPS aid), ceiling, gnss_monitor, side_monitors
compute/       profiler, budget, benchmark
calibration/   auto_calibrate, calibrate, calibrate_timing, check_calib
ardupilot/     vision_hold.param + load_params.py (ArduCopter route only)
deploy/        install.sh, vision_hold.service, msp_sender.service
tools/         replay, analyze_log, truth_drift, hold_score, discover_hardware, sitl_* ...
tests/         one file per feature; run_all.py runs everything
docs/          WORKLIST, EVIDENCE, hardware, presentation_notes
```

Every code file starts with a header: IN PLAIN WORDS, HOW IT WORKS, where it plugs in, REFERENCES.

---

## 5. Setup

Laptop (Windows shown; on Linux use `venv/bin/python`):
```
python -m venv venv
venv\Scripts\pip install -r requirements.txt
venv\Scripts\python -m tests.run_all --fast
```

The APB has Python 3.12 with numpy, **no internet and no compiler**. On a laptop with internet,
download the packages for it once:
```
pip download -r requirements.txt -d wheels --only-binary=:all: --platform manylinux2014_aarch64 --python-version 3.12
```
Then copy the folder to **`/data`** (the only place that survives updates and has space) and install:
```
rsync -avP DroneCode root@192.168.75.1:/data/
ssh root@192.168.75.1
cd /data/DroneCode && sh deploy/install.sh
```
`install.sh` builds the venv, installs from `wheels/`, runs a self-test and installs (but does not
enable) the two boot services.

---

## 6. How to use it

### First hour on the real aircraft (props OFF, battery in, video transmitter on low power)

Nothing below has been done yet. The settings named are in `config.py`.

```
systemctl stop vision_hold msp_sender             # always, before any manual run
venv/bin/python -m link.apb_sensors               # are the barometer and gyro found? do the numbers move?
venv/bin/python -m link.msp_sender --monitor      # tilt, radio channels, and what we send, twice a second
```
| Do this | Must show | If not |
|---|---|---|
| lift the nose | pitch goes positive | `BF_ATT_PITCH_SIGN = 1` |
| right side down | roll goes positive | `BF_ATT_ROLL_SIGN = -1` |
| move the throttle stick | the 4th channel number changes | `BF_RC_THROTTLE_INDEX = 2` |
| flip TL2 (CH10) | `engaged True` | check `BF_ENGAGE_CHANNEL` |

Then start both programs, engage, and carry the aircraft while an organiser laptop shows the
Betaflight Receiver tab: carried right, roll goes **below** 1500; carried forward, pitch goes
**below** 1500. If one is wrong flip `BF_ROLL_SIGN` or `BF_PITCH_SIGN`; if both are wrong set
`CAMERA_UPSIDE_DOWN = True`. Ask the crew for Betaflight's `angle_limit` and hover throttle
(`BF_ANGLE_LIMIT_DEG`, `BF_HOVER_THROTTLE`). **Do not plug our own Betaflight Configurator into the
aircraft**: the drone manual warns that a wrong version resets the flight controller.

### Event set-up (Betaflight)
```
systemctl enable --now msp_sender vision_hold     # start now and at every boot
journalctl -u vision_hold -f                      # the live estimate for the judge
# STAGE 2 live view: laptop browser -> http://192.168.75.1:8088 (camera + estimate + track; docs/LIVE_VIEW.md)
systemctl kill -s USR1 vision_hold                # "zero here": save a new home at the start mark
```
On the bench (disarmed) the height is taken as `BENCH_HEIGHT_M` (1 m) when home is saved, and
height changes come from the picture's zoom. Armed, the barometer is used.

### ArduCopter instead (fallback only; the team chose Betaflight)
Flash it, load `ardupilot/vision_hold.param` (`python -m ardupilot.load_params`), swap the
`ExecStart` line in `deploy/vision_hold.service` (the other line is there as a comment), rerun
`install.sh` and enable **only** `vision_hold` (two programs on one serial cable corrupt each other).

### Once per camera
```
venv/bin/python -m calibration.calibrate_timing --camera 2   # rolling-shutter timing -> timing.npz (props off,
                                                   # msp_sender running: the tilt comes from it)
venv/bin/python -m compute.benchmark               # milliseconds per stage on the APB
venv/bin/python preflight.py                       # GO / NO-GO list (Betaflight: checks the sender is running
                                                   # and the flight controller answers; stop vision_hold first)
```

### Practice and replay
`vision_hold.py --record DIR` saves a run; `--replay DIR --set NAME=False` replays the same run
with a feature switched off. That is how every "with vs without" number is made.

---

## 7. Tests and results

```
python -m tests.run_all            # everything, about 5 minutes
python -m tests.run_all --fast     # skip the slow simulated flights
python -m tests.test_bench         # one test by name
```
The tests draw what the downward camera would see for any position, tilt and shutter timing
(`tools/degrade_video.py`), so each test knows the true answer. **All results are simulated.**

**New for Stage 2 (run 21 Nov 2026):**

| Test | What it plays | Result |
|---|---|---|
| `test_bench` step 2 | carry 1.5 m at 1 m height | error 0.3 cm (limit 25 cm). Without the bench features: not ready at the end |
| `test_bench` step 3 | square loop, back to the start | error 0.0 cm (limit 30 cm), 8 reference switches |
| `test_bench` step 4 | raise 0.5 m | error 0.0 cm (limit 15 cm). Without height-from-zoom: 50 cm |
| `test_bench` recovery | loop with the camera covered for 3 s | error 0.0 cm (bonus limit 50 cm) |
| `test_level` | lean 0 to 15 degrees with the real 114 degree lens | trusted up to 10 degrees, error at most 0.5 cm. The old tilt fix lost trust at 8 degrees and was up to 20.6 cm off. At 15 degrees both fail |
| `test_betaflight` | a pretend Betaflight on a pretend cable | first message is safe; moved right/left/forward/back gives roll 1475 / 1525, pitch 1475 / 1525; centred 0.3 s after vision goes silent; full-stick request is cut to 1433..1567; GPS request refused; only MSP 105, 108, 200 ever sent |
| `test_replay` | one recorded run, features on and off | all on: 2.2 cm mean error. Tilt fix off: 50.2 cm. Rolling-shutter fix off: trusted frames 74 to 63 |

**From before Stage 2** (still passing; numbers last printed with the earlier test camera):

| Test | Result |
|---|---|
| Tracker | worst error 0.01 px |
| Rolling shutter | readout 30 ms measured as 29 ms, delay -20 ms as -19 ms; 92% of the error removed |
| Recovery | 2 s and 10 s blackouts with 1 to 2 m drift: trusted again 0.2 s after the picture returns |
| Re-home while climbing 10 to 40 m | without: lost at 13.4 m; with: 98.8% trusted, error 6.7 to 8.0 cm |
| Altitude sweep | about 0.6 px at every height: 0.7 cm at 5 m, 12 cm at 80 m |
| Budget, process scale | 20 to 50 fps on a 1-frame queue; full size vs 640 x 360 agree within 0.02 cm |

`test_preflight` needs Linux (it makes symbolic links); it cannot run on a Windows laptop without
admin rights. GitHub Actions runs it.

---

## 8. Stage 2 rules and how the code meets them

| Rule (Stage 2, Rev 2.0) | How | State |
|---|---|---|
| Never change APB GPIO lines (`gpioset`), device tree, boot config | all `gpioset` calls removed | done |
| Do not enable `orqa-digital-osd` | never referenced | done |
| Keep files in `/data` | `install.sh` warns when run elsewhere | done |
| Camera is 1920 x 1080 YUYV 60 fps | `calib.npz`, `preflight.py`, `check_calib` | done (datasheet lens numbers, not measured) |
| No GNSS from engage to disengage | Betaflight: a GPS request cannot be built. ArduCopter: trial mode blocks GPS source switches. GNSS monitor off | done |
| Betaflight: send from start-up, never minimum values | `msp_sender` boot service | simulated |
| Betaflight: not ready = 1500, 1500, 1500 | sender timeout 0.3 s | simulated |
| ArduCopter: not ready = no position | `ARDUPILOT_SILENT_WHEN_NOT_READY` | coded, not yet tried in SITL |
| Engage = CH10 (TL2) | Betaflight: read over MSP. ArduCopter: `RC10_OPTION,56` | simulated / not yet tried in SITL |
| Hold = 30 s within 3 m horizontal, 2 m vertical | `config.HOLD_*`, `tools/hold_score.py` | done |
| No magnetometer, no rangefinder | Betaflight route needs neither. ArduCopter route still expects a compass | **open on ArduCopter** |

| Scoring item | Points | What we have | State |
|---|---:|---|---|
| A: camera and IMU live in our process | 5 | status line shows the estimate and the APB gyro | needs the board |
| B1 carry within 0.25 m, B2 loop within 0.3 m, B3 raise within 0.15 m | 25 | `test_bench` | simulated floor |
| C: commands oppose the move, laptop unplugged | 10 | boot services, `test_betaflight` | signs unconfirmed |
| D: engage, disengage, not ready | 15 | sender rules, engage guard | simulated |
| E1: camera covered, not ready within 1 s | 8 | health goes LOST, sticks centred | simulated |
| E2: power-cycle, ready within 60 s | 7 | both services restart | start-up time unmeasured |
| F: explanation | 10 | this file, `docs/` | |
| G: flight, 30 s hold at 10 / 20 / 35 / 50 / 75 m | 20 | re-home, altitude tests | untuned controller |
| Bonus +5 each | 20 | Betaflight hold · GNSS-free start · timing and rolling shutter · recovery in the bench loop | first and third need the aircraft |

D and E must be passed on the bench before the team may fly.

---

## 9. Status and what is still open

Done in code and passing in simulation: Stage 2 rule fixes, bench features, the wide-lens tilt fix,
the complete Betaflight route, the ArduCopter route's Stage 2 changes.

Open, most risky first:

1. **Betaflight signs and constants are our reading of the protocol, not measurements**: tilt
   signs, channel order, stick directions, `angle_limit`, hover throttle, which way up the camera
   is. All are `BF_*` settings; the checks are in section 6.
2. **Betaflight controller gains are starting guesses** and there is no simulator for this route.
   The bench items only need the signs. A first flight needs tuning time, starting weaker.
3. **The APB sensor file names** are not checked on the board (`python -m link.apb_sensors`).
4. **Start-up time** on the board is unmeasured (limit 60 s).
5. **No `timing.npz`** (rolling-shutter timing has to be measured on the real camera) and no
   `mask.png`. The lens numbers are from the datasheet; a real calibration would be better.
6. Frame time stamps use the wall clock, which jumps on the board when the network time arrives.
7. **ArduCopter only**: heading without a compass, "silent when not ready" and the CH10 mapping
   are untested in SITL.
8. Levelling fails at about 15 degrees of lean. Returning to the first home can mismatch on
   identical floor tiles.

---

## 10. Settings

Everything is in `config.py`, with a comment per line. The main switches:

| Switch | What |
|---|---|
| `FC` / `--fc` | `"betaflight"` (default, team decision) or `"ardupilot"` (fallback) |
| `USE_ROLLING_SHUTTER`, `USE_TILT`, `USE_LEVEL_POINTS` | row-by-row correction, tilt fix, levelling |
| `USE_IMU_CHECK`, `USE_HEIGHT_FILTER`, `USE_CLAHE`, `USE_RECOVERY`, `USE_REHOME` | as named |
| `REHOME_SHIFT_FRAC`, `USE_ORIGIN_RETURN`, `LOST_KEYFRAME_AFTER_S`, `BENCH_HEIGHT_FROM_SCALE`, `BENCH_HEIGHT_M` | bench features |
| `BF_*` | the Betaflight route: port, signs, gains, limits. Marked "check on the aircraft" where unmeasured |
| `ARDUPILOT_SILENT_WHEN_NOT_READY`, `FAILSAFE_ACTION` | ArduCopter route |
| `HOLD_*` | the official hold limits |

Replay a recording with one switched off (`--replay DIR --set NAME=False`) for a "with vs without" number.

---

## 11. Team rules

- **Anyone may edit any file** (agreed 21 Nov 2026), but keep changes small and mark them with a
  `# STAGE 2` comment so the owner can find them.
- **Sign convention**: `shift_px` is how far the drone moved from home in picture axes (+x right,
  +y down). The camera points straight down, **top of the picture = nose**.
- **Angles**: radians, roll + = right side down, pitch + = nose up, same as ArduPilot.
- **Every new feature gets** a header, an on/off switch in `config.py` and a test that prints a number.
- **Say whether a number is simulated or real** whenever you quote it.
- **Run from the repo root** with `python -m ...`.
- **Stop the boot services before any manual run** (`systemctl stop vision_hold msp_sender`).
- **Never run `gpioset`** or touch the APB's GPIO, device tree or boot files.

---

## 12. Glossary

| Term | Meaning |
|---|---|
| **GNSS / GPS** | Satellite positioning. Not allowed during the hold |
| **VIO** | Visual-inertial odometry: estimating motion from a camera and motion sensors |
| **IMU** | Motion sensors: gyroscopes and accelerometers |
| **APB** | The onboard computer that runs our code |
| **Nadir camera** | A camera pointing straight down |
| **Rolling shutter** | A camera that captures rows one after another, so fast motion warps the picture |
| **Levelling** | Rotating tracked points to what a straight-down camera would have seen |
| **Roll / pitch / yaw** | Tilting sideways / nose up or down / turning left or right |
| **Home** | The spot, and its ground photo, that the drone must hold over |
| **Re-home** | Taking a new reference photo while keeping the original home position |
| **RANSAC** | Finds the motion most points agree on and ignores the rest |
| **OK / DEGRADED / LOST** | Trusted / one bad frame / not trusted until several good frames in a row |
| **Ready / not ready** | Whether we currently have a trusted position |
| **Betaflight / ArduCopter** | The two possible flight controller firmwares |
| **MSP** | Betaflight's message protocol. We send stick values with it |
| **MAVLink / ODOMETRY** | ArduCopter's protocol / the message our position goes in |
| **Engage switch** | CH10 (TL2) on the pilot's radio: on = our code holds |
| **SITL** | The ArduPilot simulator on a PC |

---

## Team

Praise Chopde · Ayush Ahir · Saumya J · Divyanshi Kashyap
