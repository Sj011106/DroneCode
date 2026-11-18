# Hardware: what we have (from the organisers' documents)

Nobody on the team is a hardware person, and we don't need one: **the organisers provide and fit all
hardware, and no physical changes are allowed.** This page collects the facts from the Stage 1 technical
package (Rev 1.3) and the OEM manuals in its folder, so nobody has to dig through PDFs.
Page numbers refer to the printed pages of each manual.

> **Team decision: flight controller firmware = ArduPilot (≥ 4.5).** We are not pursuing PX4, INAV or the
> optional Betaflight bonus route. `link/check_link.py` fails if the flight controller isn't ArduPilot ≥ 4.5.

## 1. The rules that shape our design (Stage 1 package)

| Rule | Consequence for us |
|---|---|
| Use the supplied M2-AI as is: **no physical modifications** (camera position, angle, mount) | We can't fix vibration mechanically: software only (rolling-shutter fix, health checks) |
| **No extra cameras or sensors** | No rangefinder unless the organisers supply one: height comes from the barometer |
| Solution runs **entirely on the APB**, using the **nadir Justice D** + onboard inertial sensing | No laptop in the loop; `compute/benchmark.py` must be run on the APB |
| **GNSS may be used during setup**, but must **not aid the scored hold** | `--no-gps-fallback` always in the trial; source set 2 (vision) before the hold is engaged |
| **Event pilots (Ted and Sujan) fly**: takeoff, landing, safety. The pilot holds the aircraft steady, then **a pilot switch engages our hold at that spot** | Our team doesn't pilot. Hold engagement must be smooth (judging area D) |
| Fiducial markers, tripods etc. allowed **for development only** | Never part of the scored solution |
| Stage 2 brings: event firmware builds, access + deployment details, sensor rates/timestamps, thresholds | Fill the open items in section 6 then |

## 2. Camera: Justice D (datasheet v1.0 + APB manual)

| Fact | Value | Source |
|---|---|---|
| Sensor | Sony 1/2.8" CMOS, **rolling shutter** (as used by the challenge) | datasheet, package Q3 |
| Resolution / rate | **1280 × 720 @ 60 fps**, 16:9 | datasheet |
| Field of view | **114° horizontal, 86° vertical**, 121.5° diagonal (very wide lens) | datasheet |
| Output | MIPI CSI, digital | datasheet |
| Linux device | CSI-1 = **/dev/video2** (CSI-2 = /dev/video3), fixed 1280×720@60 | APB manual p.12 |
| Controls | **none**: "without any external controls" (no exposure / gain setting) | APB manual p.12 |

What this means:
- Use **`--camera 2`** (same as `--camera /dev/video2`).
- **`calib.npz` in the repo is from a 640×480 webcam, not the Justice D.** Recalibrate the real camera at 1280×720
  (`calibration/auto_calibrate.py`). With a 114° lens, check the reprojection error: if it's high, a wide-angle
  lens model (OpenCV's rational model or `cv2.fisheye`) is needed. The calibration scripts are Saumya's, so tell Saumya.
- **Rolling-shutter readout must be < 16.7 ms** (one frame at 60 fps). `config.READOUT_S` defaults to 15 ms until
  `calibration/calibrate_timing.py` measures it.
- We **process at 640×360** (`config.PROCESS_WIDTH`) and 30 fps: a quarter of the pixels, same metres
  (`tests/test_process_scale.py`). Stale frames are dropped by age (`compute/budget.py`).
- No exposure control: in very bright or dark light, our contrast boost (`vision/enhance.py`) and the health
  checks are all we have.

## 3. The APB (DTK APB User Manual v1.7.2)

| Fact | Value | Source |
|---|---|---|
| Onboard computer | NXP **i.MX8M Plus, 4× Cortex-A53**, 4 GB LPDDR4, **4 GB eMMC**, micro-SD up to 512 GB, NPU 2.25 TOPS | p.3, package Q1 |
| Flight controller (same board) | STM32H743 | p.3 |
| OS | **Orqa OS (Yocto Linux BSP)**. Python / pip / OpenCV are **not** stated as included | p.3, package Q1 |
| Login | USB-C to the laptop = USB-Ethernet gadget, board at **192.168.75.1**: `ssh root@192.168.75.1` | p.10 |
| Copy files | `rsync -avP DroneCode root@192.168.75.1:~` | p.10 |
| Ethernet adapter | DHCP client + static 192.168.1.75 | p.10 |
| Clock | wrong after boot over USB only (needs NTP): log file names may have odd dates | p.10 |
| Storage | micro-SD mounts at **/sdcard**; `/data` = left-over eMMC space | p.11, p.18 |
| **Flight-controller link** | **/dev/ttymxc2 (UART3)**, behind an internal switch. Route it to the FC first: `gpioset -t0 -c gpiochip3 13=1` then `gpioset -t0 -c gpiochip3 0=0`. Baud rate not stated | p.14 |
| Extra sensors on the Linux side | **ICM42605 IMU + DPS310 barometer** connected to the i.MX (not only the FC) | p.5 |
| Temperature | fan holds ~62 °C; **automatic shutdown at 95 °C**; read `/sys/class/thermal/thermal_zone0/temp` | p.11 |
| GStreamer | present (`gst-launch-1.0 ...` example) | p.16 |

What this means:
- **The 4 GB eMMC is small**: `--record` saves 640×360 grey frames and goes to `/sdcard` when a card is in; it stops
  recording (not flying) below `config.RECORD_MIN_FREE_MB`. Measured worst case (noisy texture, 640×360,
  30 fps): FFV1 lossless ≈ **250 MB per minute**, `config.RECORD_CODEC = "MJPG"` ≈ **90 MB per minute** (lossy).
  Without an SD card, ~2 GB free eMMC = ~8 min of FFV1: put a card in for practice flights.
- **The FC link needs the switch.** `tools/discover_hardware.py --route-fc-uart` and the boot service
  (`ExecStartPre`) run the two `gpioset` commands from p.14.
- **Possible upgrade later:** the Linux-side ICM42605 could give a high-rate IMU directly for the rolling-shutter
  fix (the 50 Hz MAVLink attitude can't follow vibration above ~25 Hz). `discover_hardware.py` lists it if Linux sees it.

## 4. Aircraft (M2-AI datasheet v1.0, MRM2-10AI manual v1.0)

- 1359 g, 10" props, 2814 motors, Orqa ESC3030 4-in-1 70 A, GPS/GLONASS + magnetometer + barometer module.
- **The stock flight-controller firmware appears to be Betaflight** (the aircraft manual's troubleshooting refers
  to the Betaflight Configurator, MRM p.19). Stock modes on switch TL1 (CH9): up = Acro, middle = Position Hold
  (GPS), down = Return to Home (MRM p.18). **So an ArduPilot build must be flashed or issued for the event**
  (WORKLIST task 1.11).
- The manual describes no nadir camera and gives no image orientation: **check on day 1 which way the top of the
  Justice D image points relative to the nose** (our code assumes top of image = nose; WORKLIST 3.5 bench test).
- Without GPS, stock heading = whatever the drone faced at power-up (MRM p.21): our setup uses ArduPilot's compass.
- Safety: props off during USB-C configuration; never power on without antennas (MRM p.3).

## 5. Day-1 checklist (no hardware knowledge needed)

> **Before running anything by hand: `systemctl stop vision_hold`** (if the boot service was enabled). Two
> programs on one camera or serial port break each other.

1. Laptop ↔ APB with USB-C. `ssh root@192.168.75.1`.
2. Copy the repo: from the laptop, `rsync -avP DroneCode root@192.168.75.1:~`.
3. On the APB: `cd ~/DroneCode && sh deploy/install.sh` (or the offline steps in section 7).
4. `venv/bin/python -m tools.discover_hardware --route-fc-uart` → **save the whole output** into this file.
   Look at `logs/hw_camera_*.jpg`: the one showing the floor is the Justice D (expected: camera 2).
5. `venv/bin/python -m compute.benchmark` → total under 33 ms? (WORKLIST 3.2)
6. `venv/bin/python -m link.check_link --connect /dev/ttymxc2` → firmware must be ArduPilot ≥ 4.5.
7. `venv/bin/python -m ardupilot.load_params --connect /dev/ttymxc2`, reboot the FC, check_link again → **PASS**.
8. Props OFF: `venv/bin/python -m calibration.auto_calibrate 2`, then
   `venv/bin/python -m calibration.calibrate_timing --camera 2 --connect /dev/ttymxc2`.

## 6. Still unknown (Stage 2 / ask the organisers)

- Is Python 3 + pip available on the event image, and may we install packages (numpy, OpenCV, pymavlink)?
- Will an **ArduPilot ≥ 4.5 build** be issued for the APB flight controller? Which version exactly?
- FC serial settings on the link: which SERIALx, baud, MAVLink 2? Does the gpioset switch keep its state over reboots?
- Camera pixel format on /dev/video2, and does OpenCV's V4L2 backend open it (or only GStreamer)?
- Image orientation of the nadir camera relative to the nose; camera mount damping.
- IMU / attitude rates delivered, timestamp conventions, camera–IMU timing information (package Q4).
- Rangefinder supplied? Drift and stability thresholds, altitude levels, how altitude is measured.
- What does the pilot's "engage" switch do exactly (which channel, which mode)?

## 7. Offline install (if the APB has no internet)

On a laptop with internet, once you know the APB's Python version (step 4 above prints it, e.g. 3.10):

```bash
pip download -r requirements.txt --only-binary=:all: \
    --platform manylinux2014_aarch64 --python-version 3.10 -d wheels/
```

Copy `wheels/` to the APB, then on the APB:

```bash
python3 -m venv venv
venv/bin/pip install --no-index --find-links wheels/ -r requirements.txt
```

`requirements.txt` is the single list (numpy, opencv-python, pymavlink, matplotlib), so the offline wheels match
what `install.sh` installs. If a package has no ARM wheel, pip says so: test this **before** the event.
matplotlib is only needed for plots; skip it on the APB if it has no wheel and make the plots on a laptop.

Sources: Stage 1 Participant Technical Package Rev 1.3; Justice D Datasheet v1.0; M2 AI Datasheet v1.0;
DTK APB User Manual v1.7.2; MRM2-10AI User Manual v1.0 (OEM folder of the package).
