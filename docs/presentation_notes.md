# Presentation notes

Each slide points at the code that answers it and the proof we show.
Numbers below are from the simulated tests (`python -m tests.<name>`); replace with
flight / APB numbers once we have them. Fly practice flights with `--record` so every
"with vs without" comparison can be replayed from the same flight (`--replay DIR --set ...`).

| Brief item | What we say | Code | Proof |
|---|---|---|---|
| Sensor fusion pipeline | Camera tracks ground corners (Harris + LK + forward-backward + RANSAC). The IMU removes rotation effects: per-row rolling shutter, tilt, and a yaw cross-check. Filtered rangefinder/barometer height gives metres. ArduPilot EKF3 fuses our position with its IMU, weighted by our quality score and a noise that grows with height. | `vision/`, `fusion/`, `vision_hold.py`, `ardupilot/vision_hold.param` | `test_tilt`: fake drift from a 10 deg tilt **94 cm -> 9.8 cm**; `test_replay`: tilt fix off on the same recording **3.9 -> 57.6 cm** |
| Rolling shutter | Every row of the Justice D is captured at a different time. We measure readout time and camera-IMU delay from the video and IMU alone, then rotate each tracked point back to the frame's mid-row time using interpolated attitude. | `vision/rolling_shutter.py`, `vision/timing.py`, `calibration/calibrate_timing.py` | `test_rolling_shutter`: calibration recovers 30 ms readout as **29 ms** and -20 ms delay as **-19 ms**; fix removes **92%** of rolling-shutter error; `test_replay`: RS fix off -> trusted frames **114 -> 56** |
| Altitude (the trial) | "VIO gets harder with altitude": pixel accuracy stays ~0.6 px, so metre error grows with height; we (1) re-take the reference picture while climbing so the hold never breaks, (2) boost local contrast so flat light at height isn't mistaken for blur, (3) tell the EKF to trust us less as we climb. | `vision/rehome.py`, `vision/enhance.py`, `link/drone_link.py` | `test_rehome`: climb 10 -> 40 m, without: lost at **13.4 m**; with: **100%** trusted, 8 re-homes, error 7-8 cm mean. `test_altitude`: **0.7 cm at 5 m -> 12 cm at 80 m**; flat light 40-80 m: **0% -> 91%** trusted with the boost |
| Onboard compute | Everything runs on the APB, every stage is timed every frame, the camera queue is kept at one frame and stale frames are dropped. | `compute/` | `compute.benchmark` table (laptop: **6.9 ms/frame** incl. contrast boost, 33 ms budget; ORB recovery ~10-15 ms, only while LOST). Re-run on the APB. |
| Detect and recover | Health scores every frame; the IMU cross-check catches "healthy-looking" lies. LOST sends quality -1 so the EKF coasts on IMU + baro. ORB re-finds home anywhere in view and re-locks the tracker. If vision stays lost 3 s: LAND. | `health/health.py`, `fusion/imu_check.py`, `recovery/` | `test_recovery`: after **2 s and 10 s** blackouts with 1-2 m drift, trusted again **0.2 s** after the picture returns (without recovery: never). `analyze_log` health timeline. |
| Real drift (judging) | Practice flights keep GPS on as a referee only; the autopilot's own log gives real drift, vision error and EKF error per altitude band. | `tools/truth_drift.py` | table + plot per flight (`python -m tools.truth_drift <log>.BIN`) |

## Status

- Fusion: done (rolling shutter, tilt, IMU check, height filter). Tilt fix is first-order (residual ~2 px at 5 deg).
- Rolling shutter: done + calibration tool. Needs `calibrate_timing` on the real Justice D.
- Altitude: re-home + contrast boost done in simulation; check on real footage at height.
- Compute: profiler in every flight log, benchmark tool. Needs a run on the APB.
- Detect + recover: done. `vision/feature_filter.py` is Saumya's to write.

## Still needed before the event

1. Run `calibrate_timing` on the real camera (props off) and `compute.benchmark` on the APB.
2. Fly in SITL / the real drone with `--record`, then `tools.analyze_log` and `tools.truth_drift` for the real plots.
3. `docs/architecture.png` (the pipeline diagram).
4. Fill `config.py`, `ardupilot/vision_hold.param` and `deploy/` from the technical spec.
