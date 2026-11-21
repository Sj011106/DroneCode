# Evidence for the judges (Stage 2 scoring, areas A to G)

Stage 2 (Rev 2.0, 20 Nov 2026) scores pass or fail per item, read by an organiser from organiser
equipment. This page maps each item to **what we show, where it lives and what is still missing**.
Update it after every session on the real aircraft: paste the real numbers and the log file names.

**Every number here is simulated unless it says "real".** As of 21 Nov 2026 nothing has run on the
aircraft or the APB.

Legend: ✅ simulated and passing · 🟡 code ready, needs the real hardware · ⬜ not started

Route: **Betaflight** (team decision, 21 Nov 2026; the firmware the aircraft ships with). The
ArduCopter code remains as an unused fallback (`--fc ardupilot`).

## Bench (70 points). D and E must be passed before we may fly

| Step | Item | Pts | Pass condition | What we show | Proof so far | Status |
|---|---|---:|---|---|---|---|
| 1 | A | 5 | camera and IMU live at once in our process | `journalctl -u vision_hold -f`: the status line carries the position estimate and the APB gyro rates | `link/apb_sensors.py`, `tests/test_betaflight.py` test 9 (pretend sensor files) | 🟡 sensor file names unchecked on the board |
| 2 | B1 | 10 | carry 0.5 to 1.5 m: shown distance within 0.25 m | status line `distance` | `tests/test_bench.py`: 1.5 m carry, error **0.3 cm** (without the bench features: not ready) | ✅ sim floor |
| 3 | B2 | 10 | loop of about 3 m, back at start: within 0.3 m of zero | status line | `test_bench`: error **0.0 cm**, 8 reference switches, back on the first home picture | ✅ sim floor |
| 4 | B3 | 5 | raise 0.5 m: shown height change within 0.15 m | status line `height change` (from the picture's zoom) | `test_bench`: error **0.0 cm** (without: 50 cm) | ✅ sim floor |
| 5 | C | 10 | laptop unplugged, engaged: roll and pitch oppose each 0.5 m move | Betaflight Receiver tab on the organiser laptop | `test_betaflight` test 3: right 1475, left 1525, forward 1475, back 1525; boot services need no laptop | 🟡 **real stick signs unconfirmed** |
| 6 | D | 15 | ready + on: commands. Off: pilot at once. Not ready: 1500, 1500, 1500, never minimum | same tab | `test_betaflight` tests 2, 4, 8, 10: first message safe, centred when off or silent, no stale command at engage, throttle never below 1100 | ✅ sim |
| 7 | E1 | 8 | camera covered 3 s: not ready within 1 s, 1500s while covered | status line `NOT READY`, Receiver tab | health goes LOST on a dark frame; link sends centred sticks at once; sender times out in 0.3 s | ✅ sim |
| 8 | E2 | 7 | power-cycle: restarts by itself, ready within 60 s, safe values from the first command | both `systemd` services | `deploy/*.service` (`Restart=always`); first message tested | 🟡 **start-up time unmeasured** |

ArduCopter route for steps 5 to 7: the position follows the move; not ready = no position sent
(`ARDUPILOT_SILENT_WHEN_NOT_READY`); engage = `RC10_OPTION,56`. 🟡 not yet tried in SITL.

## F. Engineering and explanation (10 points)

What is ours and what is borrowed:

| Existing tool (used as is) | What it gives | What we built on top |
|---|---|---|
| OpenCV | corner detection, Lucas-Kanade, RANSAC, ORB, CLAHE | reference-picture tracking, per-row rolling-shutter correction, levelling, health scoring, recovery, re-homing, origin return, replay |
| NumPy | arrays | |
| pymavlink / pyserial | MAVLink messages / the serial port | both links, the MSP framing, the always-on sender |
| Betaflight (ANGLE mode) | keeps the lean angle we ask for | the position and height controller that decides the lean and throttle |
| ArduPilot (EKF3 + LOITER) | fusion and flying on that route | its configuration and the choice to let a proven controller fly |

Experiments that print a number (all simulated):

| Claim | Test | Number |
|---|---|---|
| Wide-lens tilt problem and its fix | `test_level` | old fix loses trust at 8 degrees of lean and is up to 20.6 cm off; levelled points: trusted to 10 degrees, at most 0.5 cm |
| Tilt fix on a recorded run | `test_replay` | 2.2 cm with, 50.2 cm without |
| Rolling shutter | `test_rolling_shutter`, `test_replay` | 92% of the error removed; timing measured to 1 ms; fix off costs trusted frames (74 to 63) |
| Blackout recovery | `test_recovery`, `test_bench` | trusted 0.2 s after the picture returns; loop with a 3 s cover ends 0.0 cm from start |
| Climbing | `test_rehome`, `test_altitude` | 10 to 40 m: 98.8% trusted with re-homing, lost at 13.4 m without; error 0.7 cm at 5 m to 12 cm at 80 m |
| Compute | `test_budget`, `test_process_scale`, `compute.benchmark` | laptop about 8 to 10 ms per frame; APB unmeasured |
| No GNSS aid | `test_betaflight` test 1 and 10, `test_trial_mode` | GPS request cannot be built (Betaflight); 0 GPS requests on the wire (ArduCopter) |

Safety and evidence layer (Ayush): trial mode, `preflight.py` GO / NO-GO, hold PASS / FAIL with
the official limits (`tools/hold_score.py`), altitude ceiling advisor, CI on every push.

Limits we state openly:
- All numbers are simulated; the simulated floor is a model of a textured floor.
- The Betaflight controller gains are starting guesses; no simulator exists for that route.
- Stick and tilt signs are our reading of the MSP protocol until checked on the aircraft.
- Lens numbers are from the datasheet (114 degrees, f = 623 px at 1920), not a calibration.
- Rolling-shutter timing is not measured on the real camera yet (`timing.npz` missing).
- Levelling fails at about 15 degrees of lean; origin return can mismatch on identical tiles.
- Attitude arrives at 50 Hz, so vibration above about 25 Hz is not corrected.

## G. Live position hold (20 points)

Ladder: 10, 20, 35, 50, 75 m. A hold counts when the aircraft stays within 3 m horizontally and
2 m vertically for 30 s, measured by the organisers' GPS. Two scored flights.

| What | Our evidence | Status |
|---|---|---|
| Holds at increasing height | re-homing and altitude tests (above); `safety/ceiling.py` predicts the highest safe step | ✅ sim / ⬜ real |
| Verdict after each hold | `tools/hold_score.py` with the official limits | ✅ tool |
| Real drift | `tools/truth_drift.py` on a flight log (ArduCopter route) | ⬜ |

## Bonuses (+5 each)

| Bonus | Plan | Status |
|---|---|---|
| Hold on Betaflight | our default route | 🟡 needs tuning on the aircraft |
| GNSS-free start | the Betaflight route never uses GNSS at any time | 🟡 show a full run with GPS never requested |
| Timing / rolling shutter | `calibration/calibrate_timing.py`, then one replay with `USE_ROLLING_SHUTTER` on and off | 🟡 needs a real recording |
| Recovery in the bench loop (within 0.5 m) | lost-keyframe + origin return; `test_bench` recovery case: 0.0 cm | ✅ sim floor |
