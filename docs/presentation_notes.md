# Presentation notes

Rewritten 21 Nov 2026 for Stage 2. The panel gives 10 points (area F) and judges the bonus for
timing and rolling shutter. Each row points at the code and the proof. Full map: [`EVIDENCE.md`](EVIDENCE.md).

**Route: Betaflight** (team decision, 21 Nov 2026).

**Say it first, before anyone asks:** every number below is from our simulated tests. Replace each
with the real one as soon as we have it, and say which is which.

| Topic | What we say | Code | Proof (simulated) |
|---|---|---|---|
| The pipeline | The camera tracks ground corners from a home picture. The motion sensors remove what leaning and the rolling shutter do to the picture. Height turns pixels into metres. Each frame is graded before anything is sent. | `vision/`, `fusion/`, `health/`, `vision_hold.py` | `test_replay`: 2.2 cm mean error; tilt fix off 50.2 cm |
| Rolling shutter | Every row is captured at a different time. We rotate each tracked point using the tilt at the moment its own row was scanned. Timing is measured from video and gyro alone. | `vision/rolling_shutter.py`, `vision/timing.py`, `calibration/calibrate_timing.py` | 92% of the error removed; 30 ms readout measured as 29 ms |
| What we found with the real lens | The camera is 114 degrees wide. A leaning drone stretches one side of the picture, and our first tilt fix lost trust at 8 degrees. We now rotate points to a straight-down view before matching. | `vision/rolling_shutter.py` (levelling) | `test_level`: trusted to 10 degrees, at most 0.5 cm; old fix up to 20.6 cm off |
| Flying Betaflight | Betaflight has no position hold and no safety timeout. We wrote the controller and a separate always-on sender that centres the sticks whenever vision is silent for 0.3 s. A GPS request cannot even be built. | `link/msp_sender.py`, `link/msp.py`, `fusion/position_controller.py` | `test_betaflight`: commands oppose all four moves; first message safe |
| Staying accurate over distance and height | New reference picture when the view has moved or zoomed too far; snap back to the first home when it is in view again. | `vision/rehome.py`, `recovery/recovery.py` | bench loop ends 0.0 cm from start; climb 10 to 40 m 98.8% trusted |
| Onboard compute | Runs on the APB's four A53 cores in Python. 1920 x 1080 is shrunk to 640 x 360, every stage is timed every frame, stale frames are dropped. | `compute/` | laptop about 8 to 10 ms per frame; **APB number needed** |
| Detect and recover | Dark, blurred, frozen or disagreeing frames mean "not ready": sticks centred. Home is searched for again by fingerprint. | `health/health.py`, `recovery/` | trusted 0.2 s after a blackout ends |

## What we are honest about

- Simulated until measured. The simulated floor has good texture; the real one may not.
- The Betaflight controller gains are first guesses and there is no simulator for that route.
- The lens numbers are from the datasheet, not a calibration.
- Leaning beyond about 15 degrees still loses tracking.

## Still needed for the slides

1. Real numbers: `compute.benchmark` on the APB, start-up time, bench results on the real floor.
2. `calibrate_timing` on the real camera, then one replay with the rolling-shutter fix on and off
   (the timing bonus).
3. One picture of the two-program Betaflight design (the diagram in the README, section 2).

## Comparison slide idea

NASA's Mars helicopter Ingenuity used the same basic idea: a downward camera, a flat-ground
assumption and motion sensors. It had a global-shutter camera and a laser rangefinder. We have a
rolling-shutter FPV camera and a barometer, and correct the rolling shutter in software.
