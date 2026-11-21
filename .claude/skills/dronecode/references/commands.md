# Commands: tests, simulator, tools, deployment

All commands run from the repo root (`DroneCode/`). On a Windows laptop the shell is
PowerShell and the interpreter is `python` (or `venv\Scripts\python.exe`). On the APB it is
`venv/bin/python`. Chain PowerShell commands with `;`, not `&&`.

## Tests
```
python -m tests.run_all              # every test file, one line each (about 4 min on the laptop)
python -m tests.run_all --fast       # skips the slow simulated flights (about 1 min)
python -m tests.run_all handover replay    # only files whose name contains these words
python -m tests.test_tracker         # any single file
```
Known Windows quirk: `tests.test_preflight` creates symbolic links and fails with WinError 1314
unless Developer Mode (or admin) is on. It passes on Linux and in CI.

## Simulator (ArduPilot SITL through Mission Planner)
Only relevant to the ArduCopter route. The organisers provide no simulator; this is the team's own.

1. Mission Planner -> Simulation tab -> Multirotor. Mission Planner uses port 5760; our tools use
   `tcp:127.0.0.1:5762` (or 5763).
2. Load and verify settings, then restart the simulator without wiping:
   ```
   python -m ardupilot.load_params --connect tcp:127.0.0.1:5762
   python -m link.check_link --connect tcp:127.0.0.1:5762        # want: PASS
   ```
   In the simulator `RC7_OPTION` is 7 by default and gets SKIPPED; add `--force` there (never on a
   real aircraft without asking the crew).
3. Checks, in order (drone on the ground, disarmed, GPS on):
   ```
   python -m tools.sitl_link_check --connect tcp:127.0.0.1:5762       # want RESULT: 4/4
   python -m tools.sitl_odometry_check --connect tcp:127.0.0.1:5762   # want PASS
   python -m tools.sitl_handover_check --connect tcp:127.0.0.1:5762   # want 6/6
   ```
4. Flying on fake vision (Windows only, keys read from the terminal):
   ```
   python -m tools.sitl_odometry_follow --connect tcp:127.0.0.1:5762
   ```
   Start it BEFORE arming (ArduPilot won't arm without vision messages). Then in Mission Planner:
   Actions tab -> Guided -> Arm -> immediately right-click the map -> Takeoff 10 -> Loiter. In the
   terminal: `v` = use vision, `g` = use GPS, `c` = camera covered, `x` = stop sending, `q` = quit.
   Turn GPS off with the `SIM_GPS*` parameter (the name differs between versions) and the drone
   must keep holding.
5. The whole program on a video: `python vision_hold.py --camera clip.mp4 --show`
   (make blackouts with `python -m tools.degrade_video in.mp4 out.mp4 --blackout 10 2`).

Messages worth recognising:
- `PreArm: VisOdom: not healthy`: no ODOMETRY arriving; start our sender first.
- `PreArm: VisOdom: out of memory`: `VISO_TYPE` changed without a reboot; restart the simulator.
- `EKF3 IMU0 is using external nav data`: the EKF is now on our vision.
- "simulator refused LOITER": no position estimate yet; wait for the EKF, then retry.
- Armed then disarmed after about 10 s: no takeoff was commanded in time.

Mission Planner places: Actions tab (modes, arm), EKF button on the HUD (bars should stay low; red
lines at 0.5 and 0.8), Messages tab, Config -> Full Parameter List, Ctrl+F -> MAVLink Inspector,
Ctrl+F -> Review a Log (graph `XKF1.PN/PE` against `VISP.PX/PY`).

## Main program
```
python vision_hold.py                                  # simulator link, camera 0, trial mode on
python vision_hold.py --camera 0 --fixed-height 0.5 --show    # webcam bench test over a table
python vision_hold.py --record                         # also save a replayable recording
python vision_hold.py --replay <dir> --log out.csv --set USE_TILT=False
python vision_hold.py --practice                       # GPS fallback allowed (never in a scored run)
```
Home is saved by itself on the first textured frame; again when the pilot enters LOITER/POSHOLD.

## After a run
```
python -m tools.analyze_log                    # newest logs/flight_*.csv -> plots + summary
python -m tools.truth_drift 00000042.BIN       # ArduPilot log: drift with GPS as referee
python <stage2 download>/tools/hold_judge.py 00000042.BIN    # what the judges will compute
```

## Calibration
```
python -m calibration.check_calib                         # is calib.npz usable?
python -m calibration.auto_calibrate <camera> --no-window --square <metres>
python -m calibration.calibrate_timing --camera <camera> --connect <link>    # props off
python -m vision.feature_filter build <recording>         # makes mask.png
python -m health.health stats <video or recording>        # brightness / sharpness for tuning
```
`check_calib --write-spec` writes a datasheet fallback, but at 1280 x 720 until its constants are
updated for the real 1920 x 1080 camera.

## On the APB
```
ssh root@192.168.75.1                      # USB-C, no password
```
- Put the repo in `/data` (not the home folder: the root filesystem is nearly full).
- Python 3.12 and numpy are there; no compiler. Bring wheels for aarch64 / cp312 and install with
  `pip install --no-index --find-links wheels/ ...` (opencv headless, pymavlink and its
  dependencies). Check on a laptop first which wheels exist.
- Camera: `/dev/video2`. FC link: `/dev/ttymxc2` (115200 MSP for Betaflight, 230400 MAVLink 2 for
  ArduCopter). Do not run `gpioset`. Do not enable `orqa-digital-osd`.
- `deploy/install.sh` and `deploy/vision_hold.service` exist but predate Stage 2 (they still
  contain the `gpioset` lines); fix before using. Stop the service before any manual run:
  `systemctl stop vision_hold`; watch it with `journalctl -u vision_hold -f`.
- `python -m compute.benchmark` gives the per-stage timing table for the board.
- `python preflight.py --bench` (no autopilot) or `--strict` before a flight.

## Git
Branch first, pull request, merge when CI is green. Commit or push only when asked.
`git fetch` is safe to run when asked what is new; `git pull` only when asked.

## Stage 2 rehearsals and the Betaflight route (added 21 Nov 2026)
```
python -m tests.test_bench        # the four bench moves through the real program
python -m tests.test_level        # lean 0..15 deg, old tilt fix against levelled points
python -m tests.test_betaflight   # pretend Betaflight: framing, signs, timeout, no GPS request (1 s)
python .claude/skills/dronecode/scripts/stage2_check.py .     # rule-by-rule status of the checkout
```
On the APB, Betaflight route (the stock firmware):
```
rsync -avP DroneCode root@192.168.75.1:/data/           # from the laptop
cd /data/DroneCode && sh deploy/install.sh              # uses wheels/ if present (no internet on the board)
venv/bin/python -m link.apb_sensors                     # which IIO sensors exist, do the numbers move?
systemctl stop msp_sender vision_hold                   # always before a manual run
venv/bin/python -m link.msp_sender --monitor            # tilt signs, channel order, CH10, what we send
systemctl enable --now msp_sender vision_hold           # event set-up
journalctl -u vision_hold -f                            # the live estimate for the judge (READY / NOT READY)
systemctl kill -s USR1 vision_hold                      # "zero here" at the start mark
```
Bench sign checks with `--monitor` (props off, battery in, video transmitter on low power):
lift the nose -> pitch must go positive (else `BF_ATT_PITCH_SIGN = 1`); right side down -> roll
positive; move the throttle stick -> the 4th channel number changes (else `BF_RC_THROTTLE_INDEX = 2`);
flip TL2 -> `engaged True`. Then with vision running and engaged: carry the aircraft right -> roll
below 1500; forward -> pitch below 1500 (else flip `BF_ROLL_SIGN` / `BF_PITCH_SIGN`; both wrong ->
`CAMERA_UPSIDE_DOWN = True`). Do not connect a Betaflight Configurator of our own: the drone manual
warns an outdated one resets the flight controller. Ask the crew for `angle_limit`.
