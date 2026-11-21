# Spec delta: our docs and code vs the six organiser PDFs   (Ayush, 18 Nov 2026)

> **Historical (written before Stage 2, 20 Nov 2026).** Kept for the record. Stage 2 changed the firmware
> (Betaflight shipped, ArduCopter optional), the camera size (1920 x 1080) and several rules.
> Current facts: `README.md`, `docs/hardware.md`, `docs/WORKLIST.md`.

Every fact below was checked against the PDFs; page numbers are the **printed** page numbers (as in
`docs/hardware.md`). If a PDF and our code disagree, the PDF wins. "NOT IN PDFs" = nobody may guess it.

Package = Grand Final Participant Technical Package, Stage 1, **Rev 1.3** (the same revision `hardware.md` was
written from: there is no newer package yet). APB = DTK APB User Manual v1.7.2. JD / JA = Justice D / Justice A
datasheets v1.0. M2 = M2-AI Datasheet v1.0. MRM = MRM2-10AI User Manual v1.0.

## Contradictions found (code or settings vs PDFs)

| # | What | Where | PDF | Severity | Status |
|---|---|---|---|---|---|
| C1 | GPS could take over the scored hold (`FALLBACK_TO_GPS = True`; only `--no-gps-fallback` stopped it) | `vision_hold.py` l.80, LOST branch, shutdown | Package p.2, p.7 Q14: no GNSS aiding in the scored hold | CRITICAL | **Fixed (A1):** trial mode is the default (`safety/trial_mode.py`); Saumya's line untouched |
| C2 | ArduPilot reads flight modes from CH5 by default (`FLTMODE_CH` not set in the `.param`) | `ardupilot/vision_hold.param` | MRM p.13: CH5 = Arm/Disarm, CH9 = flight mode (TL1) | CRITICAL if the event build keeps defaults | Open: question 3 below. `preflight.py` warns if `FLTMODE_CH` = 5 |
| C3 | `calib.npz` is a 640×480 webcam file; on the Justice D the centre lands at (161, 118) instead of ~(320, 180) | `calib.npz` | JD; APB p.12: 1280×720 | CRITICAL until recalibrated | Caught: Saumya's `calibration/check_calib.py` + `preflight.py` say NO-GO |
| C4 | `/dev/ttyAMA0` in three "how to run" comments | `vision_hold.py` l.69, `.param` l.15, `link/check_link.py` l.20 | APB p.14: `/dev/ttymxc2` | HIGH | Open: comment fixes for Saumya / Divyanshi |
| C5 | `#RC8_OPTION,90` would put "EKF source set" (set 1 = GPS) on the payload button | `.param` l.111 | MRM p.13: CH8 = payload activation | HIGH if uncommented | Caught: `preflight.py` NO-GO if any `RCx_OPTION` = 90; trial mode blocks the command |
| C6 | With OpenCV built without window support, `cv2.destroyAllWindows()` at shutdown raises | `vision_hold.py` safe shutdown | APB p.5/p.16 (no display assumed); package p.5 Q1 (software not guaranteed) | MEDIUM: shutdown messages are already sent, the rest of the shutdown is skipped | Open: one-line fix proposed to Saumya (only call it with `--show`). Found by the CI rehearsal (A6) |

## Facts `hardware.md` was missing (now in this file)

| Fact | PDF | Why it matters |
|---|---|---|
| Autonomous takeoff, climb, landing, waypoints are outside the task | Package p.2 | Our failsafe LAND is a safety action, not a task: confirm it is welcome (question 8) |
| Flight testing is planned outdoors | Package p.6 Q7 | Wind and sun: see wind and temperature rows |
| Justice D: WDR "ACE, 88dB", AI-DNR, white balance auto, day/night auto colour/B&W, min. 0.003 lux | JD | An automatic switch to black and white or noise smoothing could change the ground texture mid-hold |
| Justice D: OSD (voltage, name, timer) | JD | If burned into the frames, it never moves and pulls the measured shift toward zero (question 6) |
| Camera angle adjustable 0–30° by a screw (we may not change it) | MRM p.11; Package p.6 Q8 | Must be MEASURED: our maths assumes straight down |
| Board operating range 0–70 °C | APB p.3 | `preflight.py` temperature check |
| FC also on CAN 2 | APB p.14 | Info |
| SD card may be FAT32 (4 GB per file ≈ 16 min of lossless recording) | APB p.11 | `preflight.py` warns on FAT32 |
| Event radio channel map (CH5 arm, CH7 left button with no function, CH8 payload, CH9 mode) | MRM p.13 | C2, C5; CH7 (our home trigger) is a button, not a switch |
| Take-off allowed in gusts under 46.3 km/h | M2 p.3 | `ANGLE_MAX` 20° may be too little (expected 15–21° needed at 13 m/s) |
| 10-inch props "generate substantial vibrations" | MRM p.22 | Supports the rolling-shutter work |
| VTx not more than 60 s at max power on the ground | MRM p.14 | Bench tests (WORKLIST 3.4, 3.5) |
| 29 min flight time with the recommended battery | M2 p.2 | Climb-ladder planning |
| The "second Justice datasheet" is the **Justice A** (forward, analog CVBS, PAL/NTSC, 1200 TVL, shutter 1/50–1/10000 s); same sensor, lens and FOV as the Justice D | JA; Package p.6 Q9 | Not used by our solution (Package p.2: nadir Justice D) |

## Correction to `hardware.md`

- "rolling shutter: datasheet, package Q3" → the Justice D datasheet does **not** state the shutter type; only
  the package does (p.5 Q3: "The challenge uses it as a rolling-shutter camera").

## NOT IN PDFs (must come from Stage 2 or be measured)

Hold thresholds and how they are measured (package p.2, p.3, p.8 defer them) · which switch engages the hold ·
issued firmware builds (ArduPilot ≥ 4.5 not confirmed, p.5 Q2) · delivered sensor rates and timestamps (p.5 Q4) ·
FC baud and FC-side serial port · Justice D focal length, readout time, exposure · whether GPS reports reach the
APB during the scored hold.
