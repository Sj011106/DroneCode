# Betaflight changes: what changed, why, and why it matters

**Made 21 Nov 2026, on top of `main` at `753ffff`** (the Stage 2 Betaflight route and the first hardware-session fixes).
Nothing was pushed: this is a local copy for the team to review and merge. Every change is marked `BETAFLIGHT CHANGE`
in the code, has an on/off switch or a setting in `config.py`, and has a test.

**All results below are SIMULATED or synthetic.** They prove the logic. The real aircraft must confirm the numbers.
Section 4 lists the checks for the aircraft, with props off.

---

## 1. The short version

| # | Problem found on `main` | What it would have cost | Fix |
|---|---|---|---|
| 1 | Stick values assumed a **straight line** from stick to lean. Betaflight's ANGLE mode sends the stick through the pilot's **rates curve**: our "8 degrees" came out as about **2 degrees** | Every flight hold fails (simulation: 0% at 10, 35 and 75 m) | `link/stick_curve.py`: Betaflight's own curve and its exact inverse |
| 2 | **No I term and an 8 degree cap**: wind needs a steady lean, and 8 degrees only holds about 5 m/s | Blown off the spot in normal wind | I term that **starts at the pilot's own lean** at engage; 15 degree cap |
| 3 | **Heading came from Betaflight**, which corrects it with **GPS course over ground** (no compass on board) | "GPS-aided state" during the hold (Stage 2 section 04): flight voided, GNSS-free bonus lost | Heading from the **camera**; Betaflight's heading is no longer used anywhere |
| 4 | **AUX channels sent as 1500**. If the override mask were ever wrong, 1500 on the ARM channel could arm | A hidden safety trap | AUX sent as 1000 ("off"): a wrong mask can only show up loudly on the bench |
| 5 | **No hysteresis** on the engage switch | A noisy switch reading could engage, disengage and re-home over and over | On at 1800, off below 1700 |
| 6 | If the stick sender ever **hangs**, Betaflight flies our last command for ever | The one failure Betaflight cannot see | systemd watchdog: hung for 2 s, then killed and restarted |
| 7 | **The holder's legs fool the tracker** (measured by the team on 21 Nov: legs, a table and bags in view cut agreement to 38%) | B1, B2, C fail on the bench | Top up corner-starved cells + frozen-point filter + "second-best motion" check + re-home when points are lost |
| 8 | Bench height is a **guess** (1.0 m) | B1 fails if the judge holds it at 0.8 or 1.2 m | Height from motion (optional, OFF until a rehearsal confirms it) |
| 9 | Height hold had **no I term** and a **raw, noisy** climb rate | Slow sink as the battery sags; throttle twitch | I term + climb rate smoothed over 0.5 s |
| 10 | No protection against a **hot board** | The board shuts down at 95 C | Fewer frames above 80 C / 88 C; the sender is never slowed |
| 11 | ORB re-find could lock **one tile off** on identical floor tiles | B2 / recovery bonus end in the wrong place | Lowe ratio test |

---

## 2. Every file that changed, and why

### New files

**`link/stick_curve.py`: how far Betaflight really leans for a stick value**
- *What:* Betaflight's own rates formulas (Actual and Betaflight rates) and their exact inverse.
- *Why:* the old code assumed full stick = `angle_limit`, in a straight line. Betaflight 4.5 and 2025.12 shape the
  stick with the rates curve first. With Betaflight's default rates, an "8 degree" command produced 1.8 degrees
  (`tests/test_stick_curve.py` prints the table).
- *Proof:* it reproduces the centre sensitivities Betaflight documents (6.3 and 18 degrees), and the inverse is
  exact to 0.001 degrees.
- *Importance:* **critical.** Without it, the controller is about 4 times too weak near the centre and every hold
  fails in simulation.
- *To check:* the crew's CLI values (section 4); then `python -m link.stick_curve`.

**`fusion/scale.py`: height above the floor from motion (bench)**
- *What:* the accelerometer feels each sway in metres, the camera sees it in "heights". The ratio is the height.
- *Why:* B1 is judged to 0.25 m over 0.5 to 1.5 m, and the judge holds the aircraft only "about 1 m" up. A 20%
  height error is a 20% distance error.
- *How:* both signals go through the SAME filters. Accelerometer axes are matched automatically, so its mounting
  and signs do not matter. A bug was found and fixed while testing: the camera path first lacked the gravity
  filter (0.50 m measured for a true 0.70 m); now it is within 4.5%.
- *Importance:* medium. **OFF by default** (`VIO_HEIGHT = False`). The status line always prints its estimate
  (`vio h=`). Turn it on only after a rehearsal shows it matches a tape-measured height.

**`vision/frozen.py`: ignore what moves with the camera (bench)**
- *What:* while the raw accelerometer says "we are being carried", a tracked point that stays still in the picture
  for 0.4 s is attached to us (shoes, legs, hands, shadow, the drone's own legs) and is dropped.
- *Why:* the team measured it on 21 Nov: legs in view made the tracker pick the wrong motion.
- *Proof:* `tests/test_frozen.py`, through the real tracker, re-homes and health. A 1.2 m carry with legs in
  36% / 61% of the picture: right 1/4 / 0/4 without, 4/4 / 4/4 with. Standing still: nothing dropped.
- *Importance:* high for the bench (B1, B2, C are 30 points). Only active while **disarmed**.

**`compute/thermal.py`: less work when the board is hot**
- *What:* above 80 C process every 2nd frame; above 88 C every 3rd and no ORB searches.
- *Why:* the board shuts down at 95 C. The prototype showed the hold still qualifies with vision at 10 frames a
  second.
- *Importance:* insurance. The stick sender is a separate program and never slows down.

**Tests (new):** `test_stick_curve`, `test_bf_hold`, `test_bf_route`, `test_frozen`, `test_scale`,
`test_thermal`. See section 3.

### Changed files

**`config.py`**
- New settings, each explained where it is set: Betaflight rates (`BF_RATES_TYPE`, `BF_RC_RATE`, `BF_SRATE`,
  `BF_EXPO`, `BF_DEADBAND_US`), `BF_KI_DEG_PER_M_S`, `BF_ENGAGE_OFF_PWM`, `BF_AUX_US`, `BF_ALT_KI_US_PER_M_S`,
  `BF_CLIMB_SMOOTH_S`, `BF_HEADING_FROM_VISION`, `BF_RAW_IMU`, `BF_ACC_1G`, `BF_GYRO_YAW_SIGN`,
  `CORNERS_PER_CELL`, `FROZEN_*`, `REHOME_INLIER_FRAC`, `VIO_HEIGHT`, `THERMAL_*`, `RECOVERY_RATIO`.
- `BF_MAX_TILT_DEG` changed from 8 to 15. Holding against wind needs about atan(0.28 x wind / 9.81) of lean:
  8 degrees holds about 5 m/s, 15 degrees about 9 m/s.

**`fusion/position_controller.py`** (the controller)
- Leans now go through `link/stick_curve.py`.
- New `HoldController` (PD + I). The I term starts at the lean the pilot was holding at the engage moment. That
  lean is exactly what holds the aircraft against the wind, so engaging is smooth and there is no wind offset.
  The I term has anti-windup and is frozen while disarmed (a clean bench C response).
- New `AltHold`: the height hold with a slow I term for battery sag.
- `sticks()` and `throttle()` are kept, so nothing that used them breaks.

**`link/msp_sender.py`** (the always-on stick sender)
- The lean cap is judged in **degrees** through Betaflight's curve (`cap_stick`).
- Engage hysteresis (on 1800, off 1700).
- Also asks for `MSP_RAW_IMU` every other tick. These are raw accelerometer and gyro values; Betaflight never
  corrects them with GPS. They are passed to the vision program.
- Height hold uses `AltHold`; the climb rate is smoothed over 0.5 s.
- Tells systemd "still alive" every tick (watchdog).
- `--monitor` also prints |acc| (must be about 1.00 g at rest) and the raw gyro.

**`link/msp.py`**
- AUX1-4 sent as `BF_AUX_US` (1000) instead of 1500.
- `MSP_RAW_IMU` (102) allowed, with `parse_raw_imu()`. GPS requests are still impossible to build.

**`link/betaflight_link.py`**
- Uses `HoldController`. At the engage edge in flight (armed), its I term is set from Betaflight's roll/pitch at
  that moment. On the bench (disarmed) it starts at 0. Found in self-review: a judge's 3-4 degree hand tilt at
  engage would otherwise offset the step 5 answer, and could make a 0.5 m move read as "centred". Test:
  `test_bf_hold` Test 5.
- Raw IMU lists are copied in one step before reading (they are filled by another thread; reading them while they
  grow can crash Python with "deque mutated during iteration", the same reason the team used `LockedDeque`).
- Betaflight's heading is **not** used (`BF_HEADING_FROM_VISION`). The yaw kept in the attitude history is our own
  integrated raw gyro (once `BF_GYRO_YAW_SIGN` is checked), else 0.
- Keeps the raw IMU samples: `moving()` ("are we being carried?") and `integ` (for `fusion/scale.py`).

**`vision_hold.py`** (Betaflight route only; the ArduPilot route is unchanged)
- **Heading from the camera:** the rotation measured against the home picture, chained across re-homes and the
  origin return, exactly like the north/east offsets.
- The yaw cross-check (`fusion/imu_check.py`) runs only with a yaw that is not Betaflight's (the raw gyro, after
  `BF_GYRO_YAW_SIGN` is set).
- Bench (disarmed): frozen-point filter and the tracker's "second-best motion" check, from the raw accelerometer.
- Feeds the height-from-motion estimator every frame; prints `vio h=` on the status line; uses it only with
  `VIO_HEIGHT = True`.
- Heat: skips frames when hot.
- Re-homes also when the agreeing points fall below 40% of the most seen on that reference, but **never during an
  engaged hold**.

**`vision/tracker.py`** (Saumya's file: small, switchable additions)
- `CORNERS_PER_CELL`: after the usual search, any of the 4x4 cells that got less than a third of its share is
  searched again on its own strength scale. The usual search takes the strongest corners of the whole picture
  first. On a test scene, a strongly textured area took **all 144** corners and the plain floor got **none**; now
  144 of 288 are on the floor. Cells that were fine are untouched. (A first version searched every cell this way
  and the climb test's re-home drift doubled, 15 to 31 cm, from weak, noisy corners. The top-up version keeps
  15 cm.)
- `exclude`: points the frozen filter dropped are skipped before RANSAC.
- `reject_static`: while carried, a "nothing moved" answer is checked against the second-best motion, and is never
  kept as the prediction for the next frame.
- With these off, the tracker behaves exactly as before.

**`vision/rehome.py`** (Divyanshi's): an extra trigger, `inlier_drop` (bench / not holding).

**`recovery/recovery.py`** (Divyanshi's): Lowe ratio test (`RECOVERY_RATIO = 0.8`, 0 = the old cross-check).

**`deploy/msp_sender.service`**: `WatchdogSec=2`, `NotifyAccess=main`, `CPUAffinity=0` (its own core).
**`deploy/vision_hold.service`**: `CPUAffinity=1 2 3`.

**`tests/test_betaflight.py`**: four assertions updated to the new intended behaviour: AUX = 1000, the cap judged in
degrees through the curve, and MSP_RAW_IMU in the list of commands sent. Everything else is unchanged.

### Looked at and deliberately NOT changed
- **A Kalman filter for velocity:** in the wind simulation it made the team's gains *worse* (75 m: 100% to 8%).
  The team's 0.7/0.3 smoothing stays.
- **Tracking on the image without the contrast boost:** the team measured that bare carpet needs CLAHE (93% vs 4
  corners). Kept.
- **Tilt correction from the gyro since engage** (better above 35 m): needs a clean high-rate gyro stream. The
  APB's IIO gyro read about -0.2 rad/s at rest through sysfs, and Betaflight's raw gyro is whole degrees per
  second. Do it after the APB gyro is confirmed in buffered mode. Betaflight's roll/pitch are used meanwhile;
  their GPS correction acts only on heading, above 2 m/s.
- **ArduPilot route:** unchanged.

---

## 3. Test results (this copy, 21 Nov)

- **Team suite:** see the run log at the end of this file. New tests:
  - `test_stick_curve`: matches Betaflight's documented centre sensitivity (6.4 vs 6.3, 18.0 vs 18).
  - `test_bf_hold`: 30 s holds in 3 to 7 m/s wind with gusts, through the real controller, curve and sender cap.
    - Results: 6/6 qualifying at 10, 35 and 75 m (worst 1.5, 1.3 and 2.8 m).
    - The code before these changes: 0/4.
    - Engage in steady wind: 0.30 m in the first 5 s.
    - Bench step 5 with a 4 degree hand tilt at engage: roll 1407 for 0.5 m right (pushes left).
    - Height with a noisy barometer and battery sag: within 0.82 m (limit 2 m).
  - `test_bf_route`: the real `vision_hold.py --fc betaflight` with a pretend sender and video.
    - Centred sticks before engage. Re-home at engage.
    - 0.3 m to the right gives roll 1433 (pushes left) and pitch 1500.
    - This holds although the drone turned 15 degrees and Betaflight's heading said 123 degrees.
  - `test_frozen`: see above. `test_scale`: height within 4.5% for 0.7 to 1.3 m with scrambled accelerometer
    axes; "don't know" when held still. `test_thermal`: 1 / 2 / 3 frames at 62 / 80.5 / 88.5 C.

**75 m is the edge in simulation** (worst 2.8 m against the 3 m limit). Climb the ladder one level at a time and
run `tools/hold_judge.py` on every blackbox.

---

## 4. Check on the aircraft before trusting this (props off, about 30 to 60 minutes)

1. **Ask the crew** for the Betaflight CLI values (read only, no `save`): `angle_limit`, `rates_type`,
   `roll_rc_rate`, `roll_srate`, `roll_expo` (and pitch), `deadband`, `msp_override_channels_mask` (must be 15).
   Put them in `config.py` `BF_*`, then run `python -m link.stick_curve`.
2. **ARM authority:** with override on and our sender running, the radio's ARM switch must still arm and disarm.
3. **Dead-sender test:** `systemctl stop vision_hold` gives centred sticks within 0.3 s on the Receiver tab.
   `kill -STOP` the sender: systemd restarts it within about 2 s.
4. `python -m link.msp_sender --monitor`:
   - |acc| about 1.00 g at rest (else fix `BF_ACC_1G`);
   - turn the drone right: the sign of the 3rd gyro number goes into `BF_GYRO_YAW_SIGN`;
   - lift the nose: pitch goes + (`BF_ATT_PITCH_SIGN`).
5. **Bench step 5 rehearsal:** move 0.5 m left / right / forward / back. Roll and pitch must push against each move.
6. **Legs rehearsal:** a teammate carries the drone 1 m up over the judging floor with legs in view. The status line
   distance should match the tape measure; read `vio h=` against a tape-measured height before turning
   `VIO_HEIGHT` on.
7. **10 minutes at the bench:** status line FPS and temperature. The board should stay under 80 C.
8. **First development flight at 10 m:** blackbox, then `tools/hold_judge.py`. Start with the gains as they are;
   halve `BF_KP`/`BF_KD`/`BF_KI` if it oscillates.

---

## 5. Full test run on this copy (21 Nov 2026, Linux, Python 3.13)

```
PASS  test_altitude                  43.7 s  ALL ALTITUDE TESTS PASSED
PASS  test_bench                     71.3 s  ALL BENCH TESTS PASSED (simulated floor: repeat on the real 
PASS  test_betaflight                 1.2 s  ALL BETAFLIGHT TESTS PASSED (simulated flight controller: co
PASS  test_bf_hold                    3.8 s  ALL BETAFLIGHT HOLD TESTS PASSED (simulation: confirm gains 
PASS  test_bf_route                  10.2 s  ALL BETAFLIGHT ROUTE TESTS PASSED (pretend sender and camera
PASS  test_budget                     0.1 s  ALL BUDGET TESTS PASSED
PASS  test_calib_size                 0.5 s  ALL CALIBRATION SIZE TESTS PASSED
PASS  test_ceiling                   27.1 s  ALL CEILING TESTS PASSED
PASS  test_failsafe                   0.0 s  ALL FAILSAFE TESTS PASSED
PASS  test_feature_filter             1.2 s  ALL FEATURE FILTER TESTS PASSED
PASS  test_frozen                    11.6 s  ALL FROZEN-FILTER TESTS PASSED (window 0.4 s, still 1.0 px)
PASS  test_gnss_monitor               7.8 s  ALL GNSS MONITOR TESTS PASSED
PASS  test_handover                   0.0 s  ALL HANDOVER TESTS PASSED (16 tests)
PASS  test_health                     1.3 s  ALL HEALTH TESTS PASSED
PASS  test_heartbeat_filter           0.2 s  ALL HEARTBEAT FILTER TESTS PASSED
PASS  test_hold_score                 0.9 s  ALL HOLD-SCORE TESTS PASSED
PASS  test_level                      8.0 s  ALL LEVEL TESTS PASSED
PASS  test_link_threads               4.2 s  ALL LINK THREAD TESTS PASSED
PASS  test_params                     4.7 s  ALL PARAM TESTS PASSED
PASS  test_preflight                  0.2 s  ALL PREFLIGHT TESTS PASSED
PASS  test_process_scale              8.2 s  ALL PROCESS-SCALE TESTS PASSED
PASS  test_reanchor                  30.5 s  ALL REANCHOR TESTS PASSED
PASS  test_recovery                  22.4 s  ALL RECOVERY TESTS PASSED
PASS  test_rehome                    31.4 s  ALL RE-HOME TESTS PASSED
PASS  test_replay                    38.9 s  ALL REPLAY TESTS PASSED
PASS  test_rolling_shutter           18.6 s  ALL ROLLING SHUTTER TESTS PASSED
PASS  test_scale                      0.2 s  ALL SCALE TESTS PASSED
PASS  test_stick_curve                0.2 s  ALL STICK CURVE TESTS PASSED
PASS  test_thermal                    0.0 s  ALL THERMAL TESTS PASSED
PASS  test_tilt                       6.2 s  ALL TILT TESTS PASSED
PASS  test_tracker                    2.1 s  ALL TESTS PASSED
PASS  test_trial_mode                11.0 s  ALL TRIAL-MODE TESTS PASSED
PASS  test_truth_drift                0.1 s  ALL TRUTH-DRIFT TESTS PASSED
33 of 33 test files passed in 368 s
```

To see every line that changed: `git diff` in this folder, or `docs/betaflight_changes.patch`.
