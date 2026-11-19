# Team Worklist: from now to the Grand Final

This is **the** to-do list for the team. Every task explains **why** it matters, **exactly what to do**,
and **how we know it's done**, so anyone can pick it up without asking.

> **How to use this page**
> - Tick a box (`[x]`) when the task is done, commit, push. GitHub shows the progress.
> - Put your name after "Owner" when you take a task, so nobody does it twice.
> - Every command runs **from the repo root** (`DroneCode/`) with the venv active:
>   `venv\Scripts\activate` (Windows) or `source venv/bin/activate` (Linux / APB).
> - Stuck for more than 30 minutes? Write what you tried in the team chat.

---

## 0. Context in 2 minutes (read this first)

**The challenge.** Hold the Orqa M2-AI drone still over one spot **without GPS**, using only a cheap
downward-facing camera (Justice D, *rolling shutter*), the drone's own motion sensors (IMU), and the onboard
computer (APB). **Whoever holds steadily at the highest altitude wins the flight trial.** We also give a
presentation on three things: (1) our sensor fusion and how we handle rolling shutter, (2) how we use the
onboard computer, (3) how we detect and recover when vision fails.

**What's already built** (details: `README.md`):

| Brief item | Code | Status |
|---|---|---|
| Sensor fusion + rolling shutter | `vision/`, `fusion/`, `calibration/calibrate_timing.py` | ✅ built + tested in simulation |
| Onboard compute | `compute/` (profiler, budget, benchmark) | ✅ built; **never run on the APB yet** |
| Detect + recover | `health/`, `fusion/imu_check.py`, `recovery/` | ✅ built + tested in simulation |
| ArduPilot settings | `ardupilot/vision_hold.param` + `load_params.py` | ✅ built; **never loaded on the real drone** |
| Hardware | `docs/hardware.md`, `tools/discover_hardware.py` | ✅ fact sheet + discovery tool |

**Who flies the drone?** ArduPilot (the flight controller) does the controlling; the **event pilots (Ted and
Sujan)** do takeoff, landing and safety. Nobody on our team pilots. We **don't** write a flight controller. Our
code measures "how far are we from home" and sends it to ArduPilot, which blends it with its own sensors and flies.

> **Team decision: flight controller firmware = ArduPilot (≥ 4.5).** Not pursuing PX4, INAV or the optional
> Betaflight bonus route.

**Judging** (Stage 1 package): areas A–G scored on **evidence**, see [`docs/EVIDENCE.md`](EVIDENCE.md).
**Hardware facts** (APB login, ports, camera): [`docs/hardware.md`](hardware.md).
(README §8.)

**The big unknowns** (answered by the organisers' spec 72 h before, or on hardware day): how the camera is
connected, which serial port talks to the flight controller, the APB's speed and Python version, the
ArduPilot version, and the exact drift thresholds.

**Our biggest risks, in order:**
1. The APB is too slow (4 small ARM cores): could miss the 33 ms per frame budget → task 3.2
2. The camera won't open with OpenCV on the APB → task 3.1
3. Not enough ground texture at high altitude → tasks 1.5, 3.8
4. Untested on real hardware → Phase 3
5. Team doesn't know the flight procedure on the day → tasks 1.4, 4.1

**Team rules**
- **Don't rewrite Saumya's logic** (`vision/tracker.py`, `health/health.py`, `link/drone_link.py`, calibration).
  Improve *around* it. Additions in `vision_hold.py` are marked `# NEW`.
- Every new feature gets an on/off switch in `config.py`, a test in `tests/`, and a plain-words header.
- **Don't push half-finished work to `main`.** Work on a branch, open a pull request, merge when tests pass.
- After the code freeze (task 2.4) only **settings** change, no new features.
- **On the APB: `systemctl stop vision_hold` before running anything by hand** (if the boot service is enabled);
  two programs on one camera or serial port break each other. Run everything as `venv/bin/python -m ...`.

---

## Phase 1: Now, no hardware needed

### 1.1 Finish gaps 1–4 (re-homing, record/replay, ground truth, altitude sweep)
- [x] **Owner:** Claude → **DONE.** New: `vision/rehome.py`, `vision/enhance.py`, `tools/replay.py`
  (`vision_hold.py --record DIR`, `--replay DIR --set USE_TILT=0`), `tools/truth_drift.py`; 4 new tests.
- **Why:** (1) The health check (`health.py`) rejects the picture if the ground looks more than ~33% bigger
  or smaller than at home, so **climbing after saving home makes vision LOST**. That blocks the altitude
  trial. Re-homing saves a new reference picture while climbing without losing position.
  (2) Record + replay lets us tune everything on the ground from ONE flight. (3) Ground truth tells us our
  REAL drift (compared to GPS during practice). (4) The altitude sweep tells us how high the tracker can work.
- **Steps:** in progress. When merged, the README will list the new commands (`--record`, replay, truth tool).
- **Done when:** all tests pass, Saumya's code still unchanged, changes pushed.
- **Time:** in progress.

### 1.2 Send our questions to the organisers
- [ ] **Owner:** ______
- **Why:** Most of our risks disappear if we know the hardware details early instead of 72 h before.
- **Steps:**
  1. Copy the questions from `docs/hardware.md` §6 into an email or message to the organisers.
  2. When answers arrive, paste them into `docs/hardware.md` (replace the "likely" guesses) and commit.
- **Done when:** answers are written into `docs/hardware.md`.
- **Time:** 15 min to send.

### 1.3 Feature filter
- [ ] **Owner:** Saumya
- **Why:** Bad corners (the drone's own legs in view, sun glare, moving grass, shadows) can fool the tracker.
  Dropping them before RANSAC makes tracking cleaner, especially at low altitude where downwash moves grass.
  It's also a slide of its own.
- **Steps:**
  1. Read the header of `vision/feature_filter.py`: it has the suggested interface `keep_mask(gray, pts)`.
  2. Start with the simplest win: a fixed mask for parts of the drone visible in the camera.
  3. Then glare (pixels near 255) and points that stay outliers over several frames.
  4. Plug it in like `vision/rolling_shutter.py` does (a subclass or hook, not edits inside `tracker.py`),
     add `USE_FEATURE_FILTER` to `config.py`, and add `tests/test_feature_filter.py`.
- **Done when:** the test shows fewer outliers with the filter on, all other tests still pass.
- **Time:** 1–2 days.

### 1.4 Everyone practises the simulator procedure
- [ ] Owner 1: ______ - [ ] Owner 2: ______ - [ ] Owner 3: ______ - [ ] Owner 4: ______
- **Why:** On the day there's no time to learn. Every person must have done the full sequence at least once:
  connect → load settings → take off → switch to vision → hold → fail-safe.
- **Steps:** follow **README §9 "Simulator testing step by step"**:
  1. Start Mission Planner's simulator (Simulation tab → Multirotor).
  2. `python -m ardupilot.load_params --connect tcp:127.0.0.1:5762`, then restart the simulator.
  3. `python -m link.check_link --connect tcp:127.0.0.1:5762` → must say PASS.
  4. Arm, GUIDED, take off to 10 m, LOITER. Run `py -m tools.fake_vision`, press `v`.
  5. Disable GPS in the simulator (Full Parameter List → search `SIM_GPS`). The drone must keep holding.
  6. Add wind (`SIM_WIND_SPD 5`) and watch.
- **Done when:** each person has done steps 1–6 alone.
- **Time:** 1–2 h per person the first time.

### 1.5 Record real ground video and test the tracker on it
- [ ] **Owner:** ______
- **Why:** All our numbers so far come from **simulated** ground. Real grass, gravel and asphalt behave
  differently. This tells us which surfaces work and finds problems early, without the drone.
- **Steps:**
  1. With a phone or webcam pointing **straight down**, film 30–60 s of slow movement from a balcony,
     bridge or a long pole over grass, gravel, asphalt and a car park. Write down the height of each clip.
  2. For each clip: `python vision_hold.py --camera clip.mp4 --fixed-height <height in m> --trigger start --show`
     (needs the simulator running for the autopilot link, see 1.4).
  3. Then `python -m tools.analyze_log` and look at `docs/plots/*_health.png`: how much is OK vs LOST?
  4. Also try the clips made harder:
     `python -m tools.degrade_video clip.mp4 hard.mp4 --wobble 6 12 --blur 1.5`
  5. Write a short table into `docs/presentation_notes.md`: surface → % of time OK → notes.
- **Done when:** the table exists for at least 3 surfaces.
- **Time:** half a day.

### 1.6 Practise the failure drills in the simulator
- [ ] **Owner:** ______
- **Why:** The judges ask how we detect and recover. We must have SEEN it work, not just believe the tests.
- **Steps:**
  1. Make a video with blackouts: `python -m tools.degrade_video clip.mp4 drill.mp4 --blackout 10 2 --blackout 20 10`
  2. Simulator in LOITER, then: `python vision_hold.py --camera drill.mp4 --trigger mode --no-gps-fallback --show`
  3. Watch: 2 s blackout → LOST → back to OK. 10 s blackout → LOST → **FAILSAFE → LAND after 3 s**.
  4. `python -m tools.analyze_log` → save the health timeline plot for the slides.
- **Done when:** both behaviours seen, and the plot saved in `docs/plots/`.
- **Time:** 2 h.

### 1.7 Presentation draft
- [ ] **Owner (presenter):** ______
- **Why:** The presentation is part of the score, and it must answer the three brief items clearly.
- **Steps:**
  1. Start from `docs/presentation_notes.md` (slide → code → proof).
  2. Slides (suggested): (1) problem + our idea, (2) architecture (export the diagram in README §2 from
     github.com or mermaid.live as `docs/architecture.png`), (3) sensor fusion pipeline, (4) rolling shutter:
     what it is, how we measured and fixed it (92% of its error removed in tests), (5) onboard compute:
     ms per stage table, (6) detect + recover: health timeline with blackouts, (7) "physics we checked"
     (vibration, ground resolution, height accuracy), (8) results + what limits altitude.
  3. Comparison slide idea: NASA's Mars helicopter (Ingenuity) used the same basic idea (downward camera +
     rangefinder + flat ground), but with an expensive global-shutter camera. We do it with a cheap rolling-shutter one.
  4. Use simulated numbers now; replace with real ones after Phase 3.
- **Done when:** rehearsed once out loud, under the time limit.
- **Time:** 1 day.

### 1.8 Q&A preparation
- [ ] **Owner:** presenter + ______
- **Why:** Judges probe weak spots. A confident one-line answer beats a long explanation.
- **Steps:** write one or two sentences for each:
  - Why not use an existing VIO system like OpenVINS or VINS-Mono? *(integration risk on unknown hardware;
    ours is small, measurable, and each part is tested; we cite them as the research versions)*
  - How did you measure the rolling-shutter readout time? *(from video + IMU alone: `calibrate_timing.py`)*
  - What if the compass is wrong? *(`imu_check` flags disagreement; ArduPilot's EKF failsafe)*
  - Why is the tilt fix only first-order? *(the error is ~2 px at 5°; hover tilts are small; the upgrade path is documented)*
  - What limits your altitude? *(ground resolution, vibration, height accuracy; see the physics slide)*
  - What happens if vision never comes back? *(quality −1 → EKF coasts → after 3 s we LAND)*
  - Why not write your own flight controller? *(ArduPilot's is proven; the brief scores fusion/compute/recovery)*
- **Done when:** answers written at the end of `docs/presentation_notes.md`.
- **Time:** 2 h.

### 1.9 Automatic tests on every push (GitHub Actions)
- [x] **Owner:** Ayush (task A6) → **DONE, waiting for the first push.** `.github/workflows/tests.yml` runs
  `python -m tests.run_all` on Python 3.10 and 3.12 for every push and pull request. Rehearsed in clean
  environments: 21/21 test files pass on both. One command locally too: `python -m tests.run_all` (`--fast` ~1 min).
- **Why:** With 4 people pushing, something will break. GitHub runs all tests on every push and shows a
  green tick or red cross.
- **Done when:** a green tick appears on GitHub.
- **Time:** 30 min.

### 1.10 Agree on git rules
- [ ] **Owner:** whole team
- **Why:** Avoid overwriting each other's work, and never break `main` the day before the event.
- **Steps:** each task gets its own branch (`git checkout -b feature-filter`). Push the branch, open a pull
  request on GitHub, merge only when tests pass and someone else has looked at it.
- **Done when:** everyone has agreed in the chat.

### 1.11 Confirm with organisers that an ArduPilot ≥ 4.5 build will be issued for the APB FC
- [ ] **Owner:** ______
- **Why:** Team decision: ArduPilot only. The aircraft's stock firmware appears to be Betaflight (aircraft manual
  p.19), and before 4.5 ArduPilot ignores our "no data" flag (quality −1) and would fuse invalid positions.
  Stage 1 says the event builds are announced in Stage 2: make sure ours is on the list.
- **Steps:** email the organisers through the event thread: "Will an ArduPilot build, version 4.5 or newer, be
  issued for the DTK APB flight controller (STM32H743)? Which exact version? Which serial port / baud connects it
  to the i.MX (/dev/ttymxc2)?" Write the answer into `docs/hardware.md` §6.
- **Done when:** confirmed in writing, version noted.

---

## Phase 2: When the technical spec arrives (72 h before)

### 2.1 Fill in the unknowns
- [ ] **Owner:** ______
- **Why:** Some settings depend on the real hardware and are deliberately left blank or commented out.
- **Steps:**
  1. Read the spec. Update `docs/hardware.md` (facts replace guesses).
  2. `config.py`: frame rate (`TARGET_FPS`), failsafe action if the rules require one.
  3. `ardupilot/vision_hold.param` section 7: serial port, GPS off (`GPS_TYPE` or `GPS1_TYPE` depending on
     the ArduPilot version), rangefinder if fitted. Remove the `#` only for lines the spec confirms.
     **A wrong serial-port setting can disable the RC receiver.**
  4. `deploy/vision_hold.service`: check `--connect /dev/ttymxc2 --camera 2` and the `gpioset` lines against the spec.
  5. `python -m tests.test_params` must still pass.
- **Done when:** all files updated, tests pass, pushed.
- **Time:** 2–3 h.

### 2.2 Encode the official thresholds
- [ ] **Owner:** Ayush (task A3) → **machinery DONE, numbers WAITING for Stage 2.** No PDF has them (package p.2,
  p.3, p.8). `config.HOLD_*` hold clearly marked PLACEHOLDERS; `tools/analyze_log.py` (vision estimate, labelled)
  and `tools/truth_drift.py` (GPS referee, per altitude band) print PASS/FAIL. **When Stage 2 lands:** copy each
  number into `config.HOLD_*`, write the page into `HOLD_LIMITS_SOURCE`, set `HOLD_LIMITS_OFFICIAL = True`.
- **Why:** We need an instant PASS/FAIL after every practice flight, using the judges' own numbers.
- **Steps:** add the spec's drift and stability limits to `config.py` and a PASS/FAIL summary line to
  `tools/analyze_log.py` (ask Claude if needed).
- **Done when:** `analyze_log` prints PASS/FAIL against the official limits.

### 2.3 Offline install packages (if the APB has no internet)
- [ ] **Owner:** ______
- **Why:** Discovering at the field that `pip install` can't download anything would stop us completely.
- **Steps:** follow `docs/hardware.md` §7 with the APB's real Python version; put `wheels/` on a USB stick
  and in a shared drive.
- **Done when:** the install was tested on a Linux ARM machine, or confirmed unnecessary.

### 2.4 Code freeze
- [ ] **Owner:** ______
- **Why:** Most last-minute disasters come from last-minute changes.
- **Steps:**
  1. All tests pass: run every `python -m tests.test_*` (see README §10).
  2. `git tag v1.0-final` then `git push origin v1.0-final`.
  3. Copy the repo + `wheels/` + this checklist onto 2 USB sticks.
  4. From now on: only settings (`config.py`, `.param`, `deploy/`) change, each change written in the team chat.
- **Done when:** tag pushed, USB sticks ready.

---

## Phase 3: Hardware day (first time with the real drone)

> Order matters. Each step depends on the one before. **Props OFF** until step 3.6.
> **Before every manual step: `systemctl stop vision_hold`** (only matters once the service is enabled).

### 3.1 Discover the hardware
- [ ] **Owner:** ______
- **Why:** Finds the camera, the flight controller's serial port, the CPU, OS and Python version automatically.
- **Steps:**
  1. Laptop ↔ APB with the USB-C cable, then `ssh root@192.168.75.1` (APB manual p.10).
  2. Copy the repo over: on the laptop `rsync -avP DroneCode root@192.168.75.1:~` (or `git clone`).
  3. On the APB: `cd ~/DroneCode && sh deploy/install.sh` (needs Python; see 2.3 if there's no internet).
  4. `venv/bin/python -m tools.discover_hardware --route-fc-uart`. **Save the whole output** into `docs/hardware.md`.
     (`--route-fc-uart` runs the two `gpioset` commands that connect /dev/ttymxc2 to the flight controller.)
  5. Open `logs/hw_camera_*.jpg`: the picture showing the floor is the Justice D (expected: camera **2**).
- **Done when:** we know `--camera` and `--connect` values, and they're written in `docs/hardware.md`.

### 3.2 Install and benchmark ⚠ biggest risk
- [ ] **Owner:** ______
- **Why:** If the APB is too slow, positions arrive late and the hold oscillates.
- **Steps:**
  1. (Installed in 3.1.)
  2. `venv/bin/python -m compute.benchmark` → read the "total" line (frames are already shrunk to 640×360).
  3. **Under 33 ms:** great. **33–60 ms:** set `TARGET_FPS = 15` in `config.py` (ArduPilot copes with
     15 Hz). **Over 60 ms:** set `PROCESS_WIDTH = 480` in `config.py`, or fewer corners; ask Claude.
  4. Save the table (`logs/benchmark_*.md`) for the compute slide.
- **Done when:** the pipeline runs within budget, and the table is saved.

### 3.3 Connect to the flight controller and load our settings
- [ ] **Owner:** ______
- **Steps:**
  1. `venv/bin/python -m link.check_link --connect /dev/ttymxc2` (first run: shows what's wrong; it FAILS unless the
     flight controller runs ArduPilot ≥ 4.5).
  2. `venv/bin/python -m ardupilot.load_params --connect /dev/ttymxc2` (motors disarmed) → reboot the flight controller
     when it says so. It never overwrites an RC channel that already has a function: if it says SKIPPED, ask the
     event crew before using `--force`.
  3. `venv/bin/python -m link.check_link --connect /dev/ttymxc2` → must say **PASS**.
- **Done when:** PASS.

### 3.4 Calibrate the camera (props OFF)
- [ ] **Owner:** ______
- **Why:** The lens calibration turns pixels into angles; the timing calibration feeds the rolling-shutter fix.
- **Steps:**
  1. Print a checkerboard (8×5 inner corners; measure the square size and check `BOARD`/`SQUARE_M` in the script).
  2. `venv/bin/python -m calibration.auto_calibrate 2` → move the board around → `calib.npz` saved. The repo's current
     `calib.npz` is a webcam's: this replaces it. Note the reprojection error it prints: with a 114° lens, a high
     error means a wide-angle lens model is needed (tell Saumya).
  3. `venv/bin/python -m calibration.calibrate_timing --camera 2 --connect /dev/ttymxc2` → twist and rock the drone
     over a textured floor for 20 s → `timing.npz` saved. Write down the readout (must be < 16.7 ms at 60 fps)
     and the offset it prints.
  4. Set `VISO_DELAY_MS` in `ardupilot/vision_hold.param` ≈ −offset (ms) + benchmark total (ms) + 5, reload params.
- **Done when:** `calib.npz` and `timing.npz` exist, both committed.

### 3.5 Bench test (props OFF, drone in hand)
- [ ] **Owner:** ______
- **Why:** Catches sign or direction errors before they become a fly-away.
- **Steps:**
  1. `venv/bin/python vision_hold.py --connect /dev/ttymxc2 --camera 2 --trigger start --fixed-height 1.0 --no-gps-fallback`
     (watch the terminal; afterwards `venv/bin/python -m tools.analyze_log`).
  2. Hold the drone 1 m above a textured floor. Move it **forward** (nose direction): north/east must change
     in the direction the nose points. Move it **right**: it must change to the right. Rotate it: position stays ~0.
  3. Cover the camera → LOST. Uncover → OK again.
  4. Check which way is "up" in the camera picture: our code assumes **top of image = nose**. If not, STOP.
- **Done when:** all directions are correct. **If any direction is wrong, STOP and tell the team.**

### 3.6 First flight: 5–10 m, GPS on, recording on
- [ ] **Owner:** operator ______ + crew liaison ______ (the event pilots fly)
- **Why:** One recorded flight gives us real data: drift against GPS, and recordings to replay.
- **Steps:**
  1. Book the slot with the event crew; agree the plan: ALT_HOLD takeoff to 5–10 m over good texture, channel 7,
     then LOITER for 60 s. GPS stays on in practice so we have the truth to compare against.
  2. Operator on the APB:
     `venv/bin/python vision_hold.py --connect /dev/ttymxc2 --camera 2 --trigger rc --record`
     (trial mode is the default since task A1: GPS can't take over. GPS as a referee needs no flag, ArduPilot
     logs it anyway. Only `--practice` allows the old GPS fallback, with a loud banner. Never in the trial.)
  3. Pilot flips channel 7 → "HOME saved" → wait for status OK and source = vision → pilot switches to LOITER.
  4. Land. Copy `logs/`, the recording (`/sdcard/rec_*` or `logs/rec_*`) and the ArduPilot log (`.bin`) to the laptop.
  5. `python -m tools.analyze_log` and `python -m tools.truth_drift <log>.BIN` → our REAL drift vs GPS.
- **Done when:** a flight with a recording and its drift numbers.

### 3.7 Tune: autopilot side in SITL / flight, vision side with replay
- [ ] **Owner:** ______
- **Why:** Replay has no autopilot inside, so it can't show how ArduPilot reacts. `VISO_DELAY_MS` and the noise
  we send (`ODOM_PX_NOISE_*`) change how ArduPilot flies: tune them in the simulator (README §9) and confirm in
  flight, using the symptom table in README §8 (slow swaying → delay; twitching → noise).
- **Steps:**
  1. Autopilot side: SITL with `tools/fake_vision.py` (step `VISO_DELAY_MS` ±20 ms), then one flight to confirm.
  2. Vision side: replay the recording with `config.py` vision settings changed
     (`python vision_hold.py --replay <rec> --set REHOME_SCALE_MIN=0.8`, `--set CLAHE_CLIP=3.0`, ...) and compare
     `analyze_log` output.
- **Done when:** the hold looks steady in flight, and the chosen values are committed.

### 3.8 The climb ladder: find our ceiling
- [ ] **Owner:** operator + crew liaison (the event pilots fly)
- **Why:** We must arrive at the trial knowing our maximum safe altitude, not guessing it.
- **Steps:** 10 m → 20 m → 40 m → 60 m → higher, each a 60 s hold with `--record`. After each:
  `python -m tools.truth_drift <log>.BIN` for the **real** drift (GPS as referee; `analyze_log` only shows what
  vision believes) and `analyze_log` for % time OK. Write each result in the table below.
  **Stop at the first height that fails the official thresholds.**
  **Auto-fill (Ayush, A3):** `python -m tools.hold_score <log>.BIN --flight logs/flight_<time>.csv --surface "dry grass"`
  prints the PASS/FAIL and the finished row for this table. During the flight, the Ceiling Advisor (A4) prints
  "predicted max safe height here" and warns before a step it predicts will fail (`logs/flight_<time>_safety.csv`).

  | Height | Surface | % time OK | RMS drift | Max drift | PASS/FAIL | Notes |
  |---|---|---|---|---|---|---|
  | 10 m | | | | | | |
  | 20 m | | | | | | |
  | 40 m | | | | | | |
  | 60 m | | | | | | |

- **Done when:** the table is filled in, and the trial altitude = last PASS height minus a safety margin.

### 3.9 "With vs without" proof for the slides
- [ ] **Owner:** ______
- **Steps:** replay one good recording with each switch in `config.py` turned off in turn
  (`USE_ROLLING_SHUTTER`, `USE_TILT`, `USE_IMU_CHECK`, `USE_RECOVERY`, ...), then `analyze_log` each run.
- **Done when:** one plot or number per feature, pasted into the slides.

### 3.10 Real blackout test in flight
- [ ] **Owner:** operator + crew liaison (the event pilots fly; agree the test with them first)
- **Steps:** at a safe height, cover the camera for 2 s, then (separately) 10 s. Confirm recovery after 2 s,
  and LAND after 3 s for the 10 s case (or the pilot takes over).
- **Done when:** both seen and logged.

---

## Phase 4: Event day

### 4.1 Roles (decide before the day)
| Role | Person | Job |
|---|---|---|
| Pilot | **Ted / Sujan (event crew)** | Fly takeoff, the hold setup, landing; final say on safety. Not our team |
| Crew liaison | ______ | Agrees each test with the pilots beforehand (plan, altitude, abort calls) |
| Operator | ______ | Laptop/SSH; starts the program; watches status |
| Caller | ______ | Reads out loud: altitude, status (OK/LOST), time held |
| Logger | ______ | Saves all logs after every flight; notes the time of anything odd |

### 4.2 Trial sequence (print this)
1. On the APB: `systemctl stop vision_hold` (if it's enabled), then
   `venv/bin/python preflight.py --strict` → **VERDICT: GO** (it runs `link.check_link` too, plus trial mode,
   calibration, timing, camera, temperature, SD card, busy devices and the GPS-switch parameters). Any NO-GO: do not fly.
2. Start the program:
   `venv/bin/python vision_hold.py --connect /dev/ttymxc2 --camera 2 --trigger rc --no-gps-fallback --record`
   (or, if the boot service is enabled for the trial: `systemctl start vision_hold`, then
   `journalctl -u vision_hold -f` to watch it).
3. Pilot takes off in **ALT_HOLD** and climbs to the planned altitude **over the chosen spot** (GPS may help during
   setup only).
4. Pilot flips **channel 7** → "HOME saved" → wait for status **OK** and source = vision (vision is set up
   *before* the hold, so engaging is smooth).
5. Pilot engages the hold (**LOITER**, or the switch Stage 2 specifies). Caller reads status every 10 s.
6. Abort criteria (pilot switches to ALT_HOLD/LAND immediately): status LOST for more than 2 s, drone visibly
   drifting more than the threshold, any unexpected behaviour. Our failsafe LANDs by itself after 3 s without a
   valid position.
7. After landing: copy logs + recording + `.bin`, run `analyze_log` and `truth_drift`; note results in
   `docs/EVIDENCE.md` (area G).

### 4.3 Plan B
- [ ] Decide **before** the day: if the target altitude fails, which lower altitude do we retry at?
  (the last PASS from task 3.8). Never improvise a new altitude on the day.

### 4.4 Choose the hover spot
- [ ] On arrival: walk the field. Choose ground with **large, high-contrast features** (road edges, rocks,
  field boundaries). Avoid uniform grass, water and moving shadows (trees).

---

## Nice to have (only if Phases 1–3 are done)

- [ ] Send velocity to ArduPilot too (`EK3_SRC2_VELXY 6`) for a tighter hold. Test in the simulator first.
- [ ] A separate thread for reading the camera, if the APB is slow (task 3.2).
- [ ] A live video with the health overlay (`--show`) recorded for the presentation.

---

## Ayush's tasks (A1–A6, 18 Nov) and what they leave for others

| Task | Status | Files | Evidence |
|---|---|---|---|
| A1 Trial safety lock | ✅ done (sim) | `safety/trial_mode.py`, 5 lines in `vision_hold.py`, `config.TRIAL_MODE` | `tests/test_trial_mode.py`: 0 GPS requests on the radio in trial mode (8 blocked), vision_hold end to end through a blackout + shutdown: GPS requested 0 times (practice: 2) |
| A2 Pre-flight GO / NO-GO | ✅ done (fakes) / 🟡 run on the APB | `preflight.py`, `config.PREFLIGHT_*` | `tests/test_preflight.py`: 39 checks; the repo's webcam `calib.npz` is NO-GO |
| A3 Thresholds → PASS/FAIL | 🟡 placeholders until Stage 2 | `tools/hold_score.py`, 2 lines each in `analyze_log` / `truth_drift`, `config.HOLD_*` | `tests/test_hold_score.py` |
| A4 Altitude Ceiling Advisor | ✅ done (sim) | `safety/ceiling.py`, `safety/side_monitors.py`, `config.CEILING_*` | `tests/test_ceiling.py`: predicted vs measured error −12% to −20% at 5–80 m (tolerance ±25%); sent messages identical on/off; 0.1 ms/frame |
| A5 GNSS integrity monitor | ✅ done (sim) / 🟡 needs GPS_RAW_INT at the event | `safety/gnss_monitor.py`, `config.GNSS_*` | `tests/test_gnss_monitor.py`: 0 false alarms in 10 min of honest GPS; teleport caught at once, 0.5 m/s drag in 23 s; sent messages identical on/off |
| A6 CI | ✅ done, needs the first push | `.github/workflows/tests.yml`, `tests/run_all.py` | 21/21 on clean Python 3.10 and 3.12 |

Left for others (found while doing A1–A6):
- [ ] **Saumya:** `vision_hold.py` safe shutdown calls `cv2.destroyAllWindows()` even without `--show`. With an
  OpenCV that has no window support (headless builds, possibly the APB), it raises and the rest of the shutdown
  is skipped. Proposed one-line fix: only call it `if args.show`. Also the `/dev/ttyAMA0` comment on line 69 → `/dev/ttymxc2`.
- [ ] **Divyanshi:** `ardupilot/vision_hold.param`: decide `FLTMODE_CH` (9 on the event radio) and arming
  (`RC5_OPTION` 153) once question 3 in `docs/hardware.md` §6 is answered; delete `#RC8_OPTION,90` (payload
  button); `/dev/ttyAMA0` → `/dev/ttymxc2` in the `.param` header and `link/check_link.py` comments.
- [ ] **Whoever flies first:** measure the leftover tilt error (`config.CEILING_TILT_SIGMA_DEG`, now 0.5°
  SIMULATED) and the real pixel noise (`ODOM_PX_NOISE_BEST`: the advisor's simulated prediction is ~15% optimistic).
- [ ] **Everyone:** send `docs/hardware.md` §6 questions 1–10 (task 1.2).

## Where to find things

| Need | Look in |
|---|---|
| How everything works | `README.md` (architecture §2, every frame §3, structure §4) |
| Flight control + tuning symptoms | `README.md` §8 |
| Simulator steps | `README.md` §9 |
| Hardware facts, questions, offline install | `docs/hardware.md` |
| What the judges score (A–G) + our evidence | `docs/EVIDENCE.md` |
| Slide → code → proof | `docs/presentation_notes.md` |
| All settings and on/off switches | `config.py`, `ardupilot/vision_hold.param` |
| What a file does | the header at the top of every `.py` file |
| Pre-flight GO / NO-GO | `python preflight.py --strict` |
| Every test, one command | `python -m tests.run_all` |
| Our docs vs the organiser PDFs | `docs/spec_delta.md` |
