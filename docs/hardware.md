# Hardware: what we have (from the organisers' documents)

Rewritten 21 Nov 2026. Sources: Grand Final Technical Package **Stage 2, Rev 2.0** (20 Nov 2026) and
the OEM manuals (DTK APB User Manual v1.7.2, MRM2-10AI User Manual v1.0, M2-AI and Justice D / A
datasheets v1.0). **Where they differ, Stage 2 wins.** Nobody on the team is a hardware person and
no physical changes are allowed: the organisers provide and fit everything.

Nothing on this page has been measured by us yet.

## 1. Rules that shape the design

| Rule | Consequence for us |
|---|---|
| Do not change APB GPIO lines, device tree, kernel or boot config | **Never run `gpioset`.** The APB manual's p.14 commands are forbidden at the event. If the flight controller link is silent, ask the organisers |
| Do not enable `orqa-digital-osd` | It takes the cameras and the flight controller link |
| Keep files in `/data` | The root filesystem is nearly full and `/tmp` is lost at reboot. Recordings can go to `/sdcard` |
| No GNSS from engage to disengage | Betaflight route never requests it; ArduCopter route runs in trial mode |
| No extra sensors, no modifications | Height from the barometer; no rangefinder, **no magnetometer** |
| Do not connect an outdated Betaflight Configurator | The drone manual warns it can reset the flight controller to stock, and the aircraft then flips. We never plug ours in; the organiser laptop shows the Receiver tab |

## 2. Cameras

| | Justice D (nadir, the one we use) | Justice A (forward) |
|---|---|---|
| Device | `/dev/video2` | `/dev/video3` (through an ADV7282 decoder) |
| Picture | **1920 x 1080, 60 fps, YUYV** (the datasheet said 1280 x 720) | 720 x 576, about 50 fps, PAL; 1 to 3% partial frames |
| Shutter | rolling; no exposure control | |
| Field of view | 114 deg horizontal, 86 deg vertical (datasheet) | same lens |
| Focal length we use | 623 px at 1920 wide (208 px at our 640 working size), computed from the field of view | |
| Time stamps | CLOCK_MONOTONIC, marks the **end** of the frame; no hardware sync | |

- `calib.npz` holds the datasheet numbers with no distortion (`python -m calibration.check_calib
  --write-spec`). A real calibration on the real camera would be better.
- The lens is wide: leaning stretches one side of the picture. That is why points are levelled
  before matching (`config.USE_LEVEL_POINTS`).
- The camera's angle is adjustable 0 to 30 degrees by a screw (MRM p.11). We may not change it;
  our maths assumes straight down. Look at it and ask if it is not.
- The datasheet lists auto white balance, WDR and noise reduction, with no way to switch them off.

## 3. The APB (onboard computer)

| Item | Value |
|---|---|
| Processor | i.MX8M Plus, 4 x Cortex-A53, about 3.9 GB RAM; an NPU we do not use |
| Software | Orqa Yocto Linux, **Python 3.12 with numpy, no compiler, no internet**. Packages are installed from aarch64 wheel files we bring (`wheels/`) |
| Access | USB-C to a laptop = USB Ethernet; `ssh root@192.168.75.1`, no password |
| Own sensors | ICM-42605 motion sensor (up to 4 kHz) and DPS310 barometer, through Linux IIO files (`link/apb_sensors.py`) |
| Link to the flight controller | `/dev/ttymxc2` |
| Temperature | works 0 to 70 C; shuts down at 95 C (`preflight.py` warns at 75) |
| Clock | the wall clock is wrong until network time arrives, and then jumps |

## 4. The aircraft (Orqa M2-AI / MRM2-10AI)

- 1359 g, 10 inch props, 2814 880 kV motors. It hovers at low throttle, so small throttle changes
  have a large effect: our height hold may move the throttle by at most ±50 (`BF_THROTTLE_AUTHORITY`).
- **Flight controller firmware as issued: Betaflight 2025.12.5.** MSP v1 at 115200 on
  `/dev/ttymxc2`. With the engage switch on it flies the stick values we send
  (`MSP_SET_RAW_RC`: roll, pitch, throttle, yaw). **It has no command timeout**, and before our
  first message it reads minimum values. Recommended rate 50 Hz.
- **Optional: ArduCopter 4.7.2**, flashed by the team, described by the organisers as untuned and
  unflown. MAVLink 2 at 230400.
- Radio channels (MRM p.13 and Stage 2): CH1 roll, CH2 pitch, CH3 throttle, CH4 yaw, CH5 arm,
  CH9 (TL1) flight mode, **CH10 (TL2) = engage switch**, CH11 video transmitter.
- The video transmitter overheats after 60 s on the ground at full power: keep it low or off on
  the bench (MRM p.14).
- The bench needs the battery in: USB-C alone does not power the flight controller.
- **Props off whenever the USB-C cable is in.**
- 10 inch props "generate substantial vibrations" (MRM p.22): a reason for the rolling-shutter work.

## 5. What we still have to find out on the aircraft

| Question | Where the answer goes |
|---|---|
| Sign of pitch and roll in Betaflight's attitude reply | `BF_ATT_PITCH_SIGN`, `BF_ATT_ROLL_SIGN` |
| Position of the throttle in Betaflight's channel reply | `BF_RC_THROTTLE_INDEX` |
| Do our roll and pitch commands push against the move | `BF_ROLL_SIGN`, `BF_PITCH_SIGN`, `CAMERA_UPSIDE_DOWN` |
| Betaflight `angle_limit` and the hover throttle (ask the crew) | `BF_ANGLE_LIMIT_DEG`, `BF_HOVER_THROTTLE` |
| Names of the IIO sensor files | `python -m link.apb_sensors` prints them |
| Milliseconds per frame and start-up time on the APB | `python -m compute.benchmark`; time a power-cycle |
| Rolling-shutter readout time and camera-to-sensor delay | `python -m calibration.calibrate_timing` writes `timing.npz` |
| Is the nadir camera really straight down, top edge toward the nose | look, and ask |

The step-by-step list is in [`WORKLIST.md`](WORKLIST.md) section 3.

## 6. Installing with no internet on the board

On a laptop with internet:
```
pip download -r requirements.txt -d wheels --only-binary=:all: --platform manylinux2014_aarch64 --python-version 3.12
```
Copy the whole folder to `/data/DroneCode` and run `sh deploy/install.sh`. It reuses the board's
numpy and installs the rest from `wheels/`.
