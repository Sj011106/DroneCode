# Stage 2 rules, scoring and event configuration

Source: Grand Final Participant Technical Package, Stage 2, Revision 2.0 (20 November 2026), in
`github.com/Haxerus/redteam-grand-final` (`docs/Red-Team-Grand-Final-Stage-2.md` and `.pdf`; if the
two differ the PDF applies). Where Stage 2 is more specific than Stage 1, Stage 2 applies. Re-read
the original before quoting a number to a judge; the organisers can publish a new revision.

## Contents
1. The task in one paragraph
2. Platform as issued
3. Flight controller: two routes
4. GNSS rule
5. Scoring table
6. Bench test protocol (70 points)
7. Flight test protocol (20 points)
8. Bonus routes (20 points)
9. Things the organisers answered
10. The judges' scoring tool

## 1. The task
Each team gets one aircraft for the whole event. The pilot (event crew) takes off, climbs to a
level, holds steady and flips the **engage switch**. From that moment the team's solution must hold
position with no GNSS until the pilot switches off. The nadir Justice D must contribute materially;
the forward Justice A may also be used. No physical modifications, no extra sensors. Stage 1's
judging areas A to G still apply.

## 2. Platform as issued
| Item | Value |
|---|---|
| Access | USB-C to the APB (USB Ethernet), `ssh root@192.168.75.1`, no password |
| Nadir camera | Justice D on `/dev/video2`: **1920 x 1080, 60 fps, YUYV**, rolling shutter, no exposure control. The sensor format is set at boot. (The datasheet said 1280 x 720.) |
| Forward camera | Justice A on `/dev/video3` through an ADV7282 decoder: 720 x 576, about 50 fps, PAL; 1 to 3 % of frames are partial |
| APB sensors | ICM-42605 IMU (up to 4 kHz) and DPS310 barometer, through Linux IIO |
| FC sensors | a second IMU and barometer on the flight controller, through MSP or MAVLink |
| Not present | **rangefinder, magnetometer** |
| Timestamps | cameras and IMU on CLOCK_MONOTONIC; camera timestamps mark the **end** of the frame; no hardware sync |
| Computer | i.MX8M Plus, 4 x Cortex-A53, about 3.9 GB RAM, Orqa Yocto Linux, **Python 3.12 with numpy**, no compiler; pip installs aarch64 wheels from local files; ROS 2 not supported |
| Storage | keep files in **`/data`**; the root filesystem is almost full and `/tmp` is lost at reboot |
| Pilot video | analog passthrough of the Justice A with the Betaflight OSD; the APB is not in that path |
| Practice | no simulator and no dataset from the organisers; practise on the team's own aircraft |

Platform rules:
- Do not change the APB device tree, kernel or boot configuration.
- **Do not change the APB GPIO lines.** They route the FC link and the pilot video.
- Do not enable the `orqa-digital-osd` service: it takes the cameras and the FC link.

## 3. Flight controller: two routes

### Betaflight (installed on every aircraft)
| Item | Value |
|---|---|
| Firmware | Orqa Betaflight 2025.12.5, board `APB` |
| Link | APB `/dev/ttymxc2`, **115200 baud, MSP v1** |
| Command | `MSP_SET_RAW_RC`: 8 x uint16 microseconds in the order **roll, pitch, throttle, yaw**, AUX1..AUX4. The FC uses only the first four. 50 Hz recommended |
| Engage | **CH10 (TL2) high** selects MSP OVERRIDE and ANGLE together. Low returns control to the pilot at once |
| Log | blackbox to FC flash, downloaded by the crew after each flight |

Safety rule: **the FC has no command timeout.**
- If the process stops sending while the switch is on, the FC keeps the last command. Only the pilot's switch-off recovers.
- Before the first command after an FC power-up, the four channels read minimum while the switch is on: throttle cut and full tilt.
- So send continuously from start-up, ready or not. When not ready: roll, pitch, yaw 1500 and a throttle that holds height.

On this route the team's code is the position controller and the height controller.

### ArduCopter 4.7.2 (optional)
Files in the Stage 2 download under `firmware/ardupilot/`. The team flashes it (or asks the
organisers): back up 2 MB from `0x08000000` in DFU first, enter DFU by typing `bl` in the
Betaflight CLI, mass erase, write bootloader at `0x08000000` and application at `0x08060000`,
verify, then power-cycle. Link to the APB is then **MAVLink 2 on `/dev/ttymxc2` at 230400 baud**.

Status as stated by the organisers: boot, USB, MAVLink to the APB and IMU orientation were checked
on an event aircraft. Receiver, motor order and direction, flight modes and tune are **not done:
"this is your work"**; ArduCopter starts at default parameters and was **not flown** by the
organisers. The flight crew must approve the aircraft before it flies. A failed flash needs the
FC boot button, which is inside the aircraft. Going back to Betaflight needs the organisers and the
backup file. Modified images void the flight (the crew checks the firmware identity).

## 4. GNSS rule
GPS stays connected because the organisers measure drift with it. GNSS may be used before engage
(for example to initialise). **From the moment the engage switch goes on until it goes off, the
solution must not use GNSS**: Betaflight `MSP_RAW_GPS`, ArduCopter GPS messages or GPS-aided state.
At the first bench session the team shows the judge where the solution gets its inputs and how it
stops using GNSS at engage. Judges may inspect code and logs at any time. Use of GNSS during a hold
voids that flight.

## 5. Scoring
Every item is pass or fail, run and read by an organiser from organiser equipment.

| Area | Item | Where | Points |
|---|---|---|---:|
| A Sensor data | live camera and IMU data on the APB | bench step 1 | 5 |
| B Estimation | measured displacement and return | bench steps 2-4 | 25 |
| C Onboard execution | correct commands with the laptop unplugged | bench step 5 | 10 |
| D FC integration and handover | engage, disengage, not-ready behaviour | bench step 6 | 15 |
| E Reliability | camera loss and APB restart | bench steps 7-8 | 15 |
| F Engineering and explanation | judges' panel | presentations | 10 |
| G Live position hold | measured hold, altitude ladder | flight | 20 |
| | | **Base total** | **100** |

Process:
- Scoring is open from the start of the event. Book a 10-minute judged bench session at the scoring desk; **up to 3 sessions** per team.
- At the start of a session, declare the items claimed; only those are tested; **2 tries per item**.
- Passed items are **banked**; a later failure doesn't remove them.
- **Flying is allowed only when D and E are banked.** Each team gets **2 scored flights**.
- Test everything with props off before asking for outdoor flight time.

Terms categories: Algorithm 30 (A + B), Technical Implementation 45 (C + D + G), Feasibility and
Scalability 15 (E), Entrepreneurial Drive and Storytelling 10 (F).

## 6. Bench test protocol
Equipment: the judging-area floor as it is, masking-tape start and distance marks, a tape measure,
two tape marks on a wall or pole 0.5 m apart, an organiser laptop on the FC USB (Betaflight Receiver
tab, or Mission Planner for ArduCopter). The floor can be seen before the session. Team-supplied
aids are not used. Set-up: props off, battery in, radio on; the judge holds the aircraft level
about 1 m above the floor; **the team's laptop shows the live estimate**.

| Step | Item | Test | Pass condition | Pts |
|---|---|---|---|---:|
| 1 | A | show the process on the APB reading the Justice D and the IMU | both streams live at the same time, in the team's process | 5 |
| 2 | B1 | judge carries the aircraft from the start mark to a chosen mark 0.5 to 1.5 m away, any direction | displayed horizontal displacement within **0.25 m** of the taped distance | 10 |
| 3 | B2 | judge carries it around a loop of about 3 m and stops over the start mark | displayed position within **0.3 m** of zero | 10 |
| 4 | B3 | over the start mark, judge raises it from the low mark to the high mark | displayed height change within **0.15 m** of 0.5 m | 5 |
| 5 | C | laptop unplugged from the APB; engage on; judge moves it 0.5 m left, right, forward, back | on the organiser laptop the command responds to each move with the correct sign. Betaflight: roll and pitch oppose the move. ArduCopter: the position follows the move | 10 |
| 6 | D | judge turns the engage switch on and off, once with the solution ready and once not ready | ready + on: commands appear. Off: pilot control at once. Not ready: Betaflight receives roll, pitch, yaw 1500, never minimum values; **ArduCopter receives no position** | 15 |
| 7 | E1 | judge covers the Justice D for 3 s | within 1 s the solution shows not ready. Betaflight: 1500s while covered. ArduCopter: no position while covered. Normal output only after the camera is uncovered | 8 |
| 8 | E2 | judge power-cycles the APB | the solution restarts by itself and shows ready within **60 s**. Betaflight: safe values from its first command | 7 |

## 7. Flight test protocol
Before take-off the crew checks the firmware identity and FC log space, and the team confirms the
full solution was tested props off and is running and sending now.

In the air: the pilot climbs to level 1, reads altitude from the OSD (relative to take-off), holds
within 2 m of the level and switches the engage switch on. The hold runs until the pilot switches
off: after 30 s, or earlier for safety. After a 30 s hold the pilot climbs to the next level and
engages again. The flight continues until a hold fails, the battery is low, or the top level is done.

| Level | Height | A full hold scores |
|---|---:|---:|
| 1 | 10 m | 12 |
| 2 | 20 m | 14 |
| 3 | 35 m | 16 |
| 4 | 50 m | 18 |
| 5 | 75 m | 20 |

Measurement, from the FC log: **hold start** = engage switch going on; **engage point** = GPS
position and barometric altitude at hold start; horizontal error = GPS distance from the engage
point; vertical error = barometric altitude change. **A hold qualifies when, for 30 s from hold
start, horizontal error stays within 3 m and vertical error within 2 m.**

Without a full hold, the best result scores: under 5 s within limits 0; 5 to under 10 s 2; 10 to
under 30 s 5. Only the best result counts. Time within limits ends at the first sample outside a
limit or at switch-off. The flight lead can lower the top levels for every team equally, decides on
conditions, and gives an equal reflight if they stop a flight or the FC log is missing.

Winners: overall = highest base + bonus (max 120); ties by higher G, then who banked D and E first.
Flight portion = highest level with a qualifying hold; ties by time within limits at the next level.

## 8. Bonus routes (each optional, earned once)
| Route | How it is measured | Points |
|---|---|---:|
| Betaflight integration | a qualifying hold flown on Betaflight | +5 |
| GNSS-free start | a qualifying hold in a flight where the solution never used GNSS from power-up to landing; declare before take-off, show code and logs on request | +5 |
| Timing and rolling shutter | judges' panel at the presentations: treatment of camera timing and rolling shutter, with measured evidence | up to +5 |
| Onboard recovery | in bench step 3 the judge covers the Justice D for 3 s mid-loop; at the start mark the displayed position is within 0.5 m of zero | +5 |

## 9. Other answers
- No PX4 or INAV build exists for the event board.
- Site and last flight time are announced at check-in; the aircraft is received at check-in.
- Development flights only after full props-off testing; scored flights have priority.

## 10. The judges' tool
`tools/hold_judge.py` in the Stage 2 download scores every engaged hold from an FC log:
`python hold_judge.py 00000012.BIN` (ArduCopter dataflash, needs pymavlink) or a Betaflight
blackbox CSV. It reads CH10 >= 1800 as hold start (ArduCopter: `RCIN.C10`), GPS (receiver 0,
3D fix), BARO, applies 3 m / 2 m / 30 s and the level table (engage altitude relative to take-off
within 2 m of 10, 20, 35, 50 or 75 m). Run it on the team's own logs to see what the judges see.
