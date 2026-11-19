---
title: "Safety layer (Ayush, tasks A1–A6) — team context"
owner: Ayush
reviewers_needed: [Saumya (vision_hold.py lines), Divyanshi (config.py + docs), Praise (merge with praiseChopdePartD)]
status: ready for review — 7 stacked branches, none merged yet
base: main @ 5cc6889 (19 Nov 2026, "Wire feature filter and calib size check into vision_hold")
last_updated: 2026-11-19
tests: 21/21 test files pass on the branch tip; 22/22 when merged with praiseChopdePartD
tags: [trial-mode, preflight, hold-score, ceiling-advisor, gnss-monitor, ci]
---

# Safety layer (Ayush, A1–A6) — team context

> **One-paragraph summary.** These six pieces sit *around* the vision position-hold. They never change
> what the drone steers by. A1 makes sure GPS can never take over the scored hold. A2 checks everything
> before take-off and says GO or NO-GO. A3 turns a flight log into PASS/FAIL against the hold limits.
> A4 predicts the height at which our position error becomes too big. A5 watches the GPS for jamming
> or spoofing. A6 runs every test automatically on GitHub. Every piece has an on/off switch in
> `config.py`, its own test, and a plain-words header at the top of the file.

---

## 1. Where it sits in the program

```
camera ─► vision/tracker.py ─► vision_hold.py main loop ─► link/drone_link.py ─► ArduPilot (EKF3)
            (Saumya)              (Saumya + others)            (Saumya)
                                        │
                                        ├─ A1 trial guard ── wraps the link: drops any "use GPS" command
                                        └─ A4 + A5 side monitors ── called after each ODOMETRY is sent;
                                             read-only, write logs/flight_<time>_safety.csv

before flight:  preflight.py (A2) ─► GO / NO-GO
after flight:   tools/analyze_log.py, tools/truth_drift.py ─► tools/hold_score.py (A3) ─► PASS / FAIL
on every push:  GitHub Actions (A6) ─► python -m tests.run_all
```

**Rule the whole layer follows:** if one of these pieces crashes, the hold keeps flying. The side
monitors run inside `try/except` *after* the ODOMETRY message has already gone out.

---

## 2. Component cards

### A1 — Trial mode (GPS can never take over)
| | |
|---|---|
| **File** | `safety/trial_mode.py` (+ 5 marked lines in `vision_hold.py`) |
| **Why** | The rules: GNSS may help during setup but must not aid the scored hold (Grand Final package Stage 1 Rev 1.3, p.2; p.7 Q14). Before this, one forgotten `--no-gps-fallback` flag meant GPS (possibly spoofed) could take over when vision was lost. |
| **How it works** | Trial mode is the default. It sets `--no-gps-fallback` by itself and wraps the link object, so these can't send "use GPS": `set_source`, `command`, raw COMMAND_LONG/INT, writes to `EK3_SRC*` / `RCx_OPTION` parameters, and RC override. It blocks *before* the link records a pending source change. |
| **Switch** | `config.TRIAL_MODE = True`. Practice only: `python vision_hold.py --practice` (prints a loud banner). |
| **Test** | `python -m tests.test_trial_mode` decodes the real radio bytes. Trial mode: **0** GPS requests (8 blocked). Practice mode: 8, which proves the test can see them. |
| **Touches others' code?** | `vision_hold.py`: additions only, marked `NEW (Ayush A1)`. Saumya's `FALLBACK_TO_GPS` line is unchanged. |

### A2 — `preflight.py` (one GO / NO-GO command)
| | |
|---|---|
| **Run** | `python preflight.py --connect /dev/ttymxc2 --camera 2 --strict` (on the bench without an autopilot: `--bench`) |
| **Checks** | trial mode on · board not busy · temperature (WARN 75 °C, NO-GO 85 °C; the board shuts down at 95 °C, APB p.11) · free SD space · calibration matches 1280×720 (uses Saumya's `calibration/check_calib.py`) · timing offset · camera size and fps (≥ 50) · autopilot link · autopilot safety parameters (NO-GO if any `RCx_OPTION` = 90; WARN if `FLTMODE_CH` = 5, because CH5 is arm/disarm on the event radio, MRM p.13) |
| **Output** | Table on screen + `logs/preflight_<time>.txt`. Exit code 0 = GO, 1 = NO-GO (scripts can use it). |
| **Switches** | `PREFLIGHT_*` in `config.py`. `--strict` turns every WARN into NO-GO: use it for the trial. |
| **Test** | `tests/test_preflight.py` (fake camera, fake parameters). |

### A3 — Hold limits → PASS / FAIL + climb-ladder row
| | |
|---|---|
| **Run** | `python -m tools.hold_score 00000042.BIN --flight logs/flight_<time>.csv --surface "dry grass"`. It also prints automatically at the end of `tools/analyze_log.py` and `tools/truth_drift.py`. |
| **Why** | Judges score the hold against limits. We want the same verdict ourselves, right after every practice flight. |
| **⚠ Placeholders** | Stage 2 has not published the limits (package p.2, p.3, p.8). Every number is printed as **PLACEHOLDER** until `HOLD_LIMITS_OFFICIAL = True`. |
| **When Stage 2 arrives** | Copy each number into `HOLD_*` in `config.py`, write the page into `HOLD_LIMITS_SOURCE`, set a limit to `None` if it isn't judged, then set `HOLD_LIMITS_OFFICIAL = True` and run `python -m tests.test_hold_score`. |
| **Test** | `tests/test_hold_score.py` |

### A4 — Altitude Ceiling Advisor
| | |
|---|---|
| **File** | `safety/ceiling.py` (called from `safety/side_monitors.py`) |
| **Idea** | Our position error grows with height. Predicted RMS error = h·√((σpx/f)² + 2·tan²σθ). Here h = height, σpx = pixel noise (from the tracker's quality, via `odometry_sigma_px`), f = focal length in pixels, σθ = leftover tilt error. Highest safe height = the h at which the predicted error × margin 1.5 reaches the hold limit. |
| **Output** | A warning before a climb step (10/20/40/60/80/100 m) that would fail, plus the `ceil_*` columns in the safety CSV. **Watch only**: it never changes what is sent. |
| **Limit** | σθ = 0.5° is a **SIMULATED** value (SITL); it must be measured in flight. It also uses the A3 limits, so its answers are placeholders until Stage 2. |
| **Switch / test** | `CEILING_ADVISOR`, `CEILING_*` · `tests/test_ceiling.py` |

### A5 — GNSS integrity monitor (watch only)
| | |
|---|---|
| **File** | `safety/gnss_monitor.py` |
| **Input** | MAVLink `GPS_RAW_INT` (it asks the autopilot for it 5 times a second) + our vision position |
| **States** | `OK` · `JAMMED` (fix < 3D, fewer than 6 satellites, HDOP > 2.5, or no report for 2 s) · `SPOOF SUSPECTED` (a jump > 20 m faster than 25 m/s, or GPS and vision disagree by more than max(10 m, 4× GPS accuracy) for 3 s) · `NO DATA` |
| **Why** | The challenge is a contested (jammed/spoofed) battlefield. Showing *that we noticed* the GPS was bad is evidence for the judges, and it is never used to navigate. |
| **Tuning note** | 5 m / 2 s gave 3 false alarms in 10 min of honest GPS, so it was changed to 10 m / 3 s (measured). The thresholds are our own choices (NOT IN PDFs). |
| **Switch / test** | `GNSS_MONITOR`, `GNSS_*` · `tests/test_gnss_monitor.py` (a slow drag is caught in about 23 s) |

### A6 — CI (automatic tests on GitHub)
| | |
|---|---|
| **Files** | `.github/workflows/tests.yml`, `tests/run_all.py` |
| **What** | On every push or PR, GitHub runs the whole test suite on Python 3.10 and 3.12. |
| **Locally** | `python -m tests.run_all` (everything, ~4 min) · `--fast` (~1 min) · `python -m tests.run_all trial ceiling` (only matching tests) |

---

## 3. Interfaces (what goes in, what comes out)

| Piece | Needs from teammates | Gives back |
|---|---|---|
| A1 trial guard | the `link` object (`link/drone_link.py`) with `set_source` / `command` / `mav` | the same object, with "use GPS" calls blocked; `trial.summary()` printed at shutdown |
| A4/A5 side monitors | each frame: `t, height, tracker.focal_px, quality, north, east, status, reset_counter` | `logs/flight_<time>_safety.csv` (separate file: the main flight CSV columns are unchanged) |
| A2 preflight | `calibration/check_calib.py`, `link.check_link`, `config.CALIB_FILE` | exit code 0/1 + `logs/preflight_*.txt` |
| A3 hold score | ArduPilot `.bin` log + our flight CSV | PASS/FAIL lines + one climb-ladder row for `docs/WORKLIST.md` 3.8 |

**Contract to keep:** if anyone renames `tracker.focal_px`, `odometry_sigma_px`, or the link's
`set_source` / `command`, tell Ayush. The tests will fail (and CI will show red), which is the point.

---

## 4. Merge order and who reviews what

The branches are stacked, so each PR shows only its own change. Merge them in this order:

1. `ayush/a1-trial-mode` — **Saumya** reviews the 5 marked lines in `vision_hold.py`
2. `ayush/a2-preflight`
3. `ayush/a3-hold-score` — **Divyanshi** reviews the 2 lines each in `tools/analyze_log.py` and `tools/truth_drift.py`
4. `ayush/a4-ceiling` — **Saumya** reviews the side-monitor lines in `vision_hold.py`
5. `ayush/a5-gnss-monitor`
6. `ayush/a6-ci`
7. `ayush/docs-evidence` — **Divyanshi** reviews the rows added to `docs/*.md`

Not part of the stack: `ayush/proposed-team-fixes` (C4, C5, C6 below). It needs the owners' OK.

**With Praise's `praiseChopdePartD`:** it merges cleanly except for one text conflict in
`docs/WORKLIST.md` (both added a section at the end): keep both. 22/22 tests pass on the merged code.
Praise's branch sets `FALLBACK_TO_GPS = False`. The A1 test forces it back to `True` inside the test,
so it still proves the guard.

---

## 5. Open items (what is NOT done, and who owns it)

| # | Item | Owner | Where |
|---|---|---|---|
| C2 | Flight modes on CH5 by default, but CH5 = arm/disarm on the event radio (MRM p.13) | Divyanshi (`.param`) + organiser question 3 | `docs/spec_delta.md` |
| C3 | `calib.npz` is from a 640×480 webcam: **recalibrate on the Justice D at 1280×720** | Saumya | preflight says NO-GO until it is done |
| C4 | `/dev/ttyAMA0` in "how to run" comments; the APB uses `/dev/ttymxc2` (APB p.14) | Saumya / Divyanshi | proposed fix on `ayush/proposed-team-fixes` |
| C5 | `#RC8_OPTION,90` example would put "EKF source = GPS" on the payload button | Divyanshi | proposed fix (a warning comment) |
| C6 | `cv2.destroyAllWindows()` raises with headless OpenCV at shutdown | Saumya | proposed fix: only call it with `--show` |
| — | Stage 2 hold limits | everyone, when published | `config.py` `HOLD_*` (see A3) |
| — | Measure the tilt error σθ in a real flight | Ayush | `CEILING_TILT_SIGMA_DEG` |
| — | Organiser questions 1–10 | team lead to send | `docs/hardware.md` §6 |

---

## 6. FAQ (for teammates and for judges)

**Does any of this change how the drone flies?** No. A1 only *blocks* commands. A2 and A3 run
before and after the flight. A4 and A5 only read and log.

**What if a safety file crashes in flight?** The side monitors are wrapped in `try/except` and run
after the ODOMETRY message is sent. The hold continues, and the error goes to the console.

**How do I fly a practice flight with GPS fallback?** `python vision_hold.py --practice`. Never
use it in the scored trial: preflight will say NO-GO.

**Why a separate safety CSV?** So the main flight CSV that `analyze_log.py` reads keeps exactly
the columns it had.

**Where do the numbers come from?** Each one is either a PDF page (cited in a comment), measured
(the test says so), SIMULATED (labelled), or our choice (labelled NOT IN PDFs).

**How do I switch a feature off?** Set its switch in `config.py` to `False`, or use
`--set NAME=VALUE` on `vision_hold.py`.

---

## 7. Glossary

| Term | Plain meaning |
|---|---|
| VIO | Visual-Inertial Odometry: working out how we moved from the camera + motion sensors |
| EKF3 | ArduPilot's Extended Kalman Filter: blends the sensors into one position estimate |
| Source set | Which sensors the EKF trusts: set 1 = GPS, set 2 = our vision |
| ODOMETRY | The MAVLink message we send to tell the autopilot our position and how sure we are |
| MAVLink | The message language between the onboard computer and the autopilot |
| GNSS / GPS | Satellite positioning (GPS is one GNSS system) |
| Jamming / spoofing | Jamming = drowning the GPS signal; spoofing = faking it so the drone believes a wrong position |
| HDOP | Horizontal Dilution of Precision: how good the satellite geometry is (lower is better) |
| RMS | Root-mean-square: a typical size of the error |
| CI | Continuous Integration: tests that run automatically on every push |
| SITL | Software-In-The-Loop: ArduPilot running as a simulator on a laptop |
| APB / MRM / JD | The organiser PDFs: APB manual, MRM manual, Justice D camera datasheet |

---

## 8. Change log
- **2026-11-19** — Rebased onto main 5cc6889 (feature filter + calib size check). No conflicts. 21/21 tests. This file added.
- **2026-11-18** — A1–A6 built on main 1c3ebce. CI rehearsed on clean Python 3.10/3.12.
