# Team Worklist: from now to the Grand Final

Rewritten 21 Nov 2026 for Stage 2 (Rev 2.0). The older phase plan (written for "ArduPilot only,
1280 x 720") is in git history. Tick items here as they are done and write the real number next to them.

## 0. Where we are in two minutes

- The aircraft ships with **Betaflight** and **the team has decided to use it** (21 Nov 2026).
  Our code flies it directly (`--fc betaflight`, boot default). The ArduCopter code
  (`--fc ardupilot`) stays as an unused fallback.
- All Stage 2 rule clashes are fixed in code. The bench features and the wide-lens tilt fix are in.
- **Everything is simulated.** Nothing has run on the aircraft or the APB.
- Scoring: bench 70 points (items A to E), talk 10 (F), flight 20 (G), bonuses up to 20.
  **We may not fly until D and E are passed on the bench.** Three bench sessions of 10 minutes,
  two tries per item, passed items stay passed. Two scored flights.
- Rules that get a team in trouble: never `gpioset` / GPIO / device tree, never enable
  `orqa-digital-osd`, files in `/data`, no GNSS from engage to disengage, do not plug our own
  Betaflight Configurator into the aircraft.

## 1. Decide today (whole team)

- [x] **Route: Betaflight** (team decision, 21 Nov 2026), for the bench and the flight. We do not
  flash ArduCopter. The ArduCopter code stays in the repo untouched as a fallback, but nobody
  spends time on it. `config.FC = "betaflight"` is the default everywhere (main program,
  `preflight.py`, `calibrate_timing`); the fallback needs `--fc ardupilot` typed out.
- [ ] **Who does what at the bench**: one person at the laptop (ssh, status line), one watching
  the organiser's screen, one speaking to the judge.
- [ ] Review the uncommitted branch `stage2-fixes` (files touched per owner are listed at the end).

## 2. Before we get the aircraft (no hardware)

- [ ] Everyone runs `python -m tests.run_all --fast` on their own laptop.
- [ ] Download the aarch64 packages into `wheels/` (command in the README, section 5). `pyserial`
  is now in `requirements.txt`: the Betaflight sender needs it.
- [ ] Presentation: slides from `docs/presentation_notes.md`; rehearse the "what is simulated,
  what is real" answer.
- [ ] Questions for the organisers: Betaflight `angle_limit`; expected throttle at engage; hover
  throttle; is the nadir camera's top edge toward the nose; may we see the bench floor early.

## 3. First hour with the aircraft (props OFF, battery in, video transmitter low power)

Do these in order. Each has a setting to flip if the answer is "no" (README section 6).

- [ ] Copy to `/data/DroneCode`, run `sh deploy/install.sh`. Self-test passes on the board: ____
- [ ] `python -m link.apb_sensors`: barometer and gyro found, numbers move. File names: ____
- [ ] `python -m link.msp_sender --monitor`: replies arrive from the flight controller.
- [ ] Lift the nose: pitch positive? ____  Right side down: roll positive? ____
- [ ] Throttle stick changes channel number ____ (expected the 4th).
- [ ] TL2 flips `engaged`. CH5 arms.
- [ ] Camera opens at 1920 x 1080; `preflight.py` says GO.
- [ ] Vision running, engaged, organiser's Receiver tab: carried right, roll below 1500? ____
  Carried forward, pitch below 1500? ____
- [ ] Ask the crew: `angle_limit` = ____, hover throttle = ____. Put both in `config.py`.
- [ ] `python -m compute.benchmark`: ____ ms per frame on the APB.
- [ ] Power-cycle: seconds until `READY` appears: ____ (limit 60).
- [ ] `python -m calibration.calibrate_timing` (props off): `timing.npz` saved.

## 4. Bench rehearsal on the real floor

Run each exactly as the judge will, with `journalctl -u vision_hold -f` on the laptop.

| Step | Do | Pass | Our real result |
|---|---|---|---|
| 1 | show the status line | estimate and gyro both changing | |
| 2 | "zero here" (`systemctl kill -s USR1 vision_hold`), carry to a mark 0.5 to 1.5 m away | within 0.25 m | |
| 3 | carry around a 3 m loop, stop over the start | within 0.3 m of zero | |
| 4 | raise by 0.5 m | within 0.15 m | |
| 5 | unplug the laptop, engage, move 0.5 m each way | roll and pitch oppose | |
| 6 | engage on and off, ready and not ready | commands / pilot / 1500s | |
| 7 | cover the camera 3 s | not ready within 1 s | |
| 8 | power-cycle the APB | ready within 60 s | |

If the floor is plain or shiny and tracking is weak, note it here: the organisers do not allow
our own floor aids.

## 5. Before any flight on Betaflight

- [ ] D and E are banked.
- [ ] Start with weaker gains than the defaults (`BF_KP_DEG_PER_M`, `BF_KD_DEG_PER_MPS`) and
  `BF_MAX_TILT_DEG` at 5. Raise only when the hold is clearly too soft.
- [ ] First engage low (about 5 m), pilot's thumb on the switch, 5 seconds only. Watch for
  growing swings (gains too high or a sign wrong) and height running away (`BF_HEIGHT_HOLD = False`
  falls back to the pilot's throttle).
- [ ] Record every flight (`--record`), look at it with `tools.analyze_log` before the next one.
- [ ] Ladder 10, 20, 35, 50, 75 m: only climb a step after a clean 30 s hold at the one below.

## 6. Event day

- [ ] Book a bench session early; claim only the items that passed in rehearsal.
- [ ] Before each session: `systemctl status msp_sender vision_hold`, both active.
- [ ] Battery in, radio on, video transmitter on low power, props off for the bench.
- [ ] Plan B if the Betaflight signs cannot be made right in time: claim A, B1 to B3 and the
  recovery bonus (they do not depend on the flight controller), then fix and rebook.

## Files changed on `stage2-fixes` by owner (for review)

- Saumya: `link/drone_link.py` (baud 230400), `calibration/check_calib.py`, `tests/test_tracker.py`,
  parts of `vision_hold.py`, `ardupilot/vision_hold.param`
- Ayush: `config.py`, `preflight.py` (Betaflight link check added), `calibration/calibrate_timing.py`, `tests/test_preflight.py`, `tests/test_hold_score.py`, `tests/test_ceiling.py`
- Praise: `tests/test_replay.py` (one assertion)
- New files: `link/msp.py`, `link/msp_sender.py`, `link/betaflight_link.py`, `link/apb_sensors.py`,
  `fusion/position_controller.py`, `deploy/msp_sender.service`, `tests/test_bench.py`,
  `tests/test_level.py`, `tests/test_betaflight.py`

Changes in other people's files are marked with a `STAGE 2` comment.
