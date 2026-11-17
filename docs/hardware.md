# Hardware: what we know, what we guess, what we must find out

Nobody on the team is a hardware person, and we don't need one. **The organisers provide and fit
all hardware.** Our job is to know what we're given and configure the software for it. This page
separates facts from guesses so nobody builds on a guess.

## 1. What the challenge brief says (facts)

| Part | Brief says | Means for us |
|---|---|---|
| Drone | Orqa **M2-AI** | Provided. ArduPilot flies it (our param file configures it) |
| Camera | "additional **fixed, nadir-facing Justice D**", fitted by the organisers, low-cost FPV, **rolling shutter** | We don't mount it. We calibrate it (lens + timing) |
| Computer | "compute provided by the **APB**", everything **onboard** | Our Python runs there; no laptop in the loop |
| GNSS | "degraded or disabled entirely" | Never trust GPS in the trial: `--no-gps-fallback` |
| Details | technical spec **72 h before** | Fill in the unknowns below then |

## 2. What public information suggests (likely, NOT confirmed)

- **Flight controller:** Orqa's MRM2-10 drone (the closest public sibling) uses the **Orqa H7 QuadCore**:
  STM32H743 processor, ICM-42688 IMU, ArduPilot supported. Its stock camera is the "Justice **Analog**"
  (1200 TVL), so "Justice **D**" is probably a digital variant. That's a guess.
- **APB:** Orqa's support site described the "DTK APB" as **one board = H7 flight controller + NXP
  i.MX 8M Plus onboard computer** (Linux). From NXP's public spec, the i.MX 8M Plus has **4× Arm Cortex-A53
  (up to 1.8 GHz)**, a 2.3 TOPS NPU (AI accelerator), an image signal processor and MIPI-CSI camera inputs.

## 3. What that means for our code

| # | Consequence | Risk | What we do |
|---|---|---|---|
| 1 | Cortex-A53 cores are **several times slower** than a laptop core. Our 6–10 ms/frame on a laptop could become 30–60 ms | 🔴 could miss the 33 ms budget | Run `compute.benchmark` on the APB **first thing**. Knobs if slow: lower resolution, fewer corners, run at 15–20 fps (EKF3 copes) |
| 2 | The NPU only speeds up neural networks; we use classic computer vision | 🟢 | Slide: "runs on 4 ARM cores, NPU left free" |
| 3 | Flight controller and computer are on **one board**, linked by an internal serial port. On i.MX boards these are usually named `/dev/ttymxcN`, **not** `/dev/ttyAMA0` (that's Raspberry Pi naming, used in our examples) | 🟠 | `tools/discover_hardware.py` finds the right port; then fix `deploy/vision_hold.service` |
| 4 | Camera may arrive through the i.MX image processor (MIPI) or an analog capture. pip's OpenCV has **no GStreamer** | 🟠 camera might not open | `discover_hardware.py` tests every `/dev/video*`. If none gives a picture, use the board vendor's OpenCV/GStreamer |
| 5 | The board may have **no internet** to pip install | 🟠 | Prepare offline packages (section 5) |
| 6 | We probably **can't change the camera mount** (vibration) | 🟠 | Software side: ArduPilot harmonic notch + check vibration in logs (`VIBE`); ask organisers how the camera is damped |
| 7 | Rolling-shutter timing isn't on any datasheet | 🟢 | `calibrate_timing.py` measures it from video + IMU |

## 4. Day-1 checklist when we get the drone (no hardware knowledge needed)

1. Get a terminal on the APB (the spec should say how: SSH over Wi-Fi/Ethernet, or a USB cable).
2. Copy the repo over (`git clone`, or `scp`/USB stick).
3. `python3 -m tools.discover_hardware` → **save the whole output** and share it with the team.
4. Look at `logs/hw_camera_*.jpg`: the one showing the ground is the Justice D.
5. Install: `sh deploy/install.sh` (online) or the offline steps below.
6. `python3 -m compute.benchmark` → is the total under 33 ms?
7. `python3 -m link.check_link --connect <port from step 3>`, then
   `python3 -m ardupilot.load_params --connect <port>` and reboot the flight controller.
8. Props OFF: `python3 -m calibration.auto_calibrate <camera>`, then `python3 -m calibration.calibrate_timing --camera <camera> --connect <port>`.
9. Fix the port and camera in `deploy/vision_hold.service`.

## 5. Offline install (if the APB has no internet)

On a laptop with internet, once we know the APB's Python version (step 3 above, e.g. 3.10):

```bash
pip download numpy opencv-python-headless pymavlink --only-binary=:all: \
    --platform manylinux2014_aarch64 --python-version 3.10 -d wheels/
```

Copy `wheels/` to the APB, then: `python3 -m pip install --no-index --find-links wheels/ numpy opencv-python-headless pymavlink`.
Use `opencv-python-headless` on the board (no screen libraries needed). If a package has no ARM wheel,
pip will say so. Test this **before** the event, not at the field.

## 6. Questions for the organisers / to find in the spec

- How do we log in to the APB (SSH address, user, password)? Does it have internet?
- Which OS and Python version is on the APB? Can we install Python packages?
- How is the Justice D connected (MIPI-CSI, USB, analog capture)? Which `/dev/video*`? Resolution, frame rate?
  Can we set exposure?
- Which serial port and baud link the APB computer to the flight controller? Is it already set to MAVLink 2?
- Which ArduPilot version is on the flight controller? Are we allowed to change parameters? Which ones?
- Is there a downward rangefinder? Which type and range?
- How is the camera mounted (vibration damping)? Are we allowed to change anything physical?
- What exactly are the drift and stability thresholds, and how is altitude measured in the trial?
- Is a safety pilot provided? Which flight modes must be on the switches?

Sources: [Orqa MRM2-10 product page](https://orqafpv.com/products/mrm2-10),
[NXP i.MX 8M Plus](https://www.nxp.com/products/i.MX8MPLUS),
[ArduPilot Orqa F405 page](https://ardupilot.org/copter/docs/common-OrqaF405.html).
