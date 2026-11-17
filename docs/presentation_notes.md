# Presentation notes

Each slide points at the code that answers it and the proof we show.
Numbers below are from the simulated tests (`python -m tests.<name>`); replace with
flight / APB numbers once we have them.

| Brief item | What we say | Code | Proof |
|---|---|---|---|
| Sensor fusion pipeline | Camera tracks ground corners (Harris + LK + forward-backward + RANSAC). The IMU removes rotation effects: per-row rolling shutter, tilt, and a yaw cross-check. Filtered rangefinder/barometer height gives metres. ArduPilot EKF3 fuses our position with its IMU, weighted by our quality score. | `vision/`, `fusion/`, `vision_hold.py`, `ardupilot/vision_hold.param` | Pipeline diagram (header of `vision_hold.py`); `test_tilt`: fake drift from a 10 deg tilt **94 cm -> 9.8 cm** |
| Rolling shutter | Every row of the Justice D is captured at a different time. We measure readout time and camera-IMU delay from the video and IMU alone, then rotate each tracked point back to the frame's mid-row time using interpolated attitude. | `vision/rolling_shutter.py`, `vision/timing.py`, `calibration/calibrate_timing.py` | `test_rolling_shutter`: calibration recovers 30 ms readout as **29 ms** and -20 ms delay as **-19 ms**; fix removes **92%** of rolling-shutter error |
| Onboard compute | Everything runs on the APB, every stage is timed every frame, the camera queue is kept at one frame and stale frames are dropped. | `compute/` | `compute.benchmark` table (laptop: **6.4 ms/frame**, 33 ms budget; ORB recovery 9 ms, only while LOST). Re-run on the APB. |
| Detect and recover | Health scores every frame; the IMU cross-check catches "healthy-looking" lies. LOST sends quality -1 so the EKF coasts on IMU + baro. ORB re-finds home anywhere in view and re-locks the tracker. If vision stays lost 3 s: LAND. | `health/health.py`, `fusion/imu_check.py`, `recovery/` | `test_recovery`: after **2 s and 10 s** blackouts with 1-2 m drift, trusted again **0.2 s** after the picture returns (without recovery: never). `analyze_log` health timeline. |

## Status

- Fusion: done (rolling shutter, tilt, IMU check, height filter). Tilt fix is first-order (residual ~2 px at 5 deg).
- Rolling shutter: done + calibration tool. Needs `calibrate_timing` on the real Justice D.
- Compute: profiler in every flight log, benchmark tool. Needs a run on the APB.
- Detect + recover: done. `vision/feature_filter.py` is Saumya's to write.

## Still needed before the event

1. Run `calibrate_timing` on the real camera (props off) and `compute.benchmark` on the APB.
2. Fly in SITL / the real drone, then `tools.analyze_log` for the real plots.
3. `docs/architecture.png` (the pipeline diagram).
4. Fill `config.py`, `ardupilot/vision_hold.param` and `deploy/` from the technical spec.
