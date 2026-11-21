---
name: dronecode
description: Project knowledge and working rules for DroneCode, the team's Red Team Hackathon Grand Final entry - GNSS-denied vision position hold (VIO) on the Orqa M2-AI drone with the nadir Justice D rolling-shutter camera, the APB onboard computer and ArduCopter or Betaflight. Use this skill whenever the user mentions the drone, the hackathon, DroneCode, vision_hold, VIO, position hold, the APB, Justice D, rolling shutter, ArduPilot/ArduCopter, Betaflight/MSP, Mission Planner, SITL, MAVLink, EKF, bench tests, Stage 2, scoring, hold_judge, calibration, preflight, or the teammates Saumya, Ayush or Praise - and before reading, reviewing, explaining, changing or testing anything under myprojects/Drone, even when the request looks small. It carries the event rules and scoring, the architecture (ArduCopter and Betaflight/MSP routes), who owns which files (teammates' code is edited minimally and marked), the current gaps against Stage 2, and the milestone plan.
---

# DroneCode

The team's entry for the Red Team Hackathon Grand Final (Contested Battlefield Aerial Operations
Challenge): make an Orqa M2-AI quadcopter hold position **without GNSS**, using only its downward
Justice D camera (rolling shutter), its inertial sensors and the APB onboard computer.

- Repo: the checkout this skill lives in (`.claude/skills/dronecode/`); remote `github.com/Sj011106/DroneCode`, branch `main`
- Organisers' Stage 2 download: `github.com/Haxerus/redteam-grand-final` (rules, scoring, ArduCopter firmware, `tools/hold_judge.py`)
- Team: Saumya, Ayush, Praise, Divyanshi

The one-paragraph design: `vision_hold.py` runs on the APB's Linux computer. Every camera frame it
measures how far the drone has moved from a saved "home" picture, corrects that with the IMU (tilt,
rolling shutter), scales pixels to metres with the height, judges how far to trust the result, and
sends it on. On Betaflight (the team's choice, 21 Nov 2026) our own controller turns it into stick
values sent over MSP; on the ArduCopter fallback the position goes to the autopilot over MAVLink.

## First, look at the live state

This project changes daily (four people push, and the organisers publish new rules), so anything
written in this skill may be stale. Before answering a status question or changing code:

1. `git -C <repo> status -sb` and `git -C <repo> log --oneline -8`. Fetch only if the user asks or
   the question is "what's new".
2. Run the gap checker: `python .claude/skills/dronecode/scripts/stage2_check.py .`. It reports, from the
   files as they are now, which Stage 2 clashes are still open.
3. Treat `references/status-and-gaps.md` as a dated snapshot. If the checker or the repo disagrees
   with it, the repo wins; say what changed and update the snapshot.

## Which reference to read

| The task is about | Read |
|---|---|
| Rules, scoring, bench or flight test, bonus points, what's allowed, the hardware as issued | `references/stage2-rules.md` |
| How the code works, a file's job, thresholds, the per-frame pipeline, MAVLink, ArduPilot settings | `references/architecture.md` |
| "Where are we", limitations, what to build next, milestones, open decisions and questions | `references/status-and-gaps.md` |
| Running tests, the simulator (SITL / Mission Planner), tools, deploying to the APB, error messages | `references/commands.md` |

Read only what the task needs. For a full "explain the project" request, read all four.

## Ground rules

These come from the team. Each has a reason; follow the reason when a case isn't listed.

**Everyone's code may be edited, with care (since 21 Nov 2026).** Until Stage 2 the rule was
"never edit Saumya's files, only report". On 21 Nov the team lifted it: everyone works on everything now. Who wrote what still matters,
because the owner has to be told what changed: Saumya wrote `vision/tracker.py`, `health/health.py`,
`link/drone_link.py`, the `calibration/` scripts, `vision/feature_filter.py` and the unmarked lines
of `vision_hold.py`; Ayush wrote `safety/`, `preflight.py`, `tools/hold_score.py`; Praise wrote
`handover.py`. So: make the smallest change that fixes the problem, keep an algorithm that works,
mark changed lines with a comment such as `STAGE 2`, and list the teammates' files you touched in
your summary. This is permission to fix and adapt, not to rewrite someone's module for style.

**New features follow the house pattern**, because the judges score evidence and the team must be
able to show each fix on and off:
- a header at the top of the file: IN PLAIN WORDS, HOW IT WORKS (technical), WHERE IT PLUGS IN, REFERENCES (real papers or docs);
- an on/off switch in `config.py`;
- a test in `tests/` that prints a number for the slides and runs as `python -m tests.test_x`;
- additions inside `vision_hold.py` carry a marker comment.

**Don't commit or push unless the person you are working with asks.** Otherwise give the commands. Team rule: work on a branch, open a pull request, merge when CI is green.

**Facts discipline.** Label every result number "simulated" or "real"; almost everything so far is
simulated. Hardware and rule facts come from the organisers' documents: where Stage 2 and Stage 1
differ, Stage 2 applies; where a document and the code differ, the document wins. If a fact isn't
in the repo or the documents, say it isn't known. Guessed hardware facts have already cost this
team time (the camera turned out to be 1920x1080, not the datasheet's 1280x720).

**Stage 2 hard rules the code must never break** (details in `references/stage2-rules.md`):
no changes to APB GPIO lines, device tree, kernel or boot configuration; no GNSS use from engage
to disengage; unmodified flight-controller firmware; files live in `/data` on the APB; everything
is tested with props off before any flight is requested.

**Safety shape of the code.** The pilot always wins: when the engage switch is off, the code
commands nothing. Anything that only watches (monitors, advisors, logs) runs after the position has
been sent and inside try/except, so a bug there can't stop the hold. Keep new code inside that shape.

**How to explain things.** Most of the team are computer-science students, not hardware or
drone people. Explain in plain words, define each abbreviation the first time, give exact
commands for the shell in use (PowerShell on a Windows laptop; `venv/bin/python -m ...` on the
APB), one step at a time, and read pasted output line by line.

## Repository map

```
vision_hold.py   main loop: camera -> track -> health -> fuse -> send -> handover -> log (both routes)
handover.py      Praise: engage / disengage / failsafe decisions, auto re-anchor
config.py        every team setting and on/off switch (FC = "betaflight" is the default)
preflight.py     Ayush: one GO / NO-GO command before a flight
vision/          tracker (Saumya), rolling_shutter (+ levelling), timing, enhance, rehome, feature_filter (Saumya)
fusion/          tilt, imu_check, height, position_controller (Betaflight: position error -> sticks)
health/          health (Saumya): quality 1..100 and OK / DEGRADED / LOST
recovery/        recovery (ORB re-find home, origin return); failsafe.py is no longer used
link/            Betaflight: msp, msp_sender (always-on), betaflight_link, apb_sensors
                 ArduCopter fallback: drone_link (Saumya), check_link
ardupilot/       vision_hold.param + load_params.py (ArduCopter fallback only)
calibration/     lens (calibrate, auto_calibrate, check_calib), camera timing (calibrate_timing)
compute/         profiler, budget (stale-frame dropping), benchmark
safety/          Ayush: trial_mode, side_monitors, ceiling, gnss_monitor
tools/           replay, analyze_log, truth_drift, hold_score, degrade_video, discover_hardware, sitl_* checks
deploy/          install.sh, vision_hold.service, msp_sender.service
tests/           27 test files + run_all.py; .github/workflows/tests.yml runs them on every push
docs/            WORKLIST, EVIDENCE, hardware, presentation_notes; spec_delta and TEAM_CONTEXT_ayush are historical
```

Always run from the repo root with `python -m package.module`, otherwise imports and `calib.npz`
are not found.

## Keeping this skill useful

When something durable changes (a milestone is finished, a
Stage 2 clash is fixed, the organisers answer a question), update `references/status-and-gaps.md`
and say so, then commit it with the code change so the whole team gets it. Keep the dated-snapshot
line at its top current.
