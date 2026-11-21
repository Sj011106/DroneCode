# Cameras: front camera, blur, lens centre (STAGE 2, 21 Nov 2026)

What changed and why, and the exact tests to run on the APB. Every new behaviour is a
`config.py` setting; the ones that have not been flown yet are **off** by default.

## 1. Front camera (Justice A, `/dev/video3`)

> **WARNING before anything in section 1 (added at integration, 21 Nov 2026).** The APB manual (section 4.5)
> says opening `/dev/video3` can freeze the forward camera's decoder, and that picture is the PILOT's view.
> The team rule from the live-view work is "never open `/dev/video3`". So: do **not** run `tools/front_cam.py`
> and do **not** enable `front_view` on the aircraft until the organisers say it is safe. `install.sh` only
> installs the service; it stays disabled. Nothing in the hold opens the front camera.

**Why it did not work.** Nothing in the code opened `/dev/video3`, and the down-camera
code can't just be pointed at it:

| Down camera (Justice D) | Front camera (Justice A) |
|---|---|
| digital, 1920 x 1080 YUYV, 60 fps | analogue PAL through the ADV7282 decoder: 720 x 576 |
| grey = **first** byte of each pair (YUYV) | grey = **second** byte (UYVY): the old code reads colour |
| progressive | interlaced: two half-pictures 20 ms apart, "combed" when moving |
| | 1-3 % partial frames (Stage 2 section 02); flat screen when no signal |

**New:** `vision/front_camera.py` (reader on its own thread, right byte, one field,
partial/no-signal pictures dropped, driver timestamps), `tools/front_cam.py` (check +
browser view on port 8089), `deploy/front_view.service` (installed, never enabled
automatically; lowest CPU priority). It is optional in Stage 2 and **never** used by the hold.

**Step 1: probe (read-only, 2 minutes)**
```bash
v4l2-ctl -d /dev/video3 --list-formats-ext     # expect UYVY 720x576 (or 720x288 fields)
v4l2-ctl -d /dev/video3 --query-std            # PAL = video seen; nothing = no signal
v4l2-ctl -d /dev/video3 --log-status | grep -i -E "signal|lock|standard"
fuser -v /dev/video3                           # anything listed = device busy
venv/bin/python -m tools.front_cam             # 10 s check, 4 test pictures in /data
```

| Result | Meaning | Do |
|---|---|---|
| `VERDICT: working`, ~25 good/s (~50 if fields) | fine | look at `/data/front_*_field.png`; optional: `systemctl enable --now front_view` |
| all "no-signal" | no video reaches the decoder | hardware: ask the organisers (aircraft #5 is taped "BLACK VID NO OSD"). Stop after 30 min |
| many "bad" | other byte order | `python -m tools.front_cam --fmt YUYV`; if that works set `FRONT_CAMERA_FORMAT = "YUYV"` |
| "cannot open" | busy or wrong node | `fuser -v /dev/video3`; check `ls /dev/video*` |

**Before enabling `front_view` for a flight:** note `vision_hold`'s pictures per second with
it off and on (journal / live view). If it drops by more than 10 %, leave it off while holding.

## 2. "Blurry at the top": what it is

f = 874 px at 1920 wide; exposure up to ~16 ms indoors (no exposure control):

| Cause | Size of the smear | Worse higher up? |
|---|---|---|
| turning / vibration | turn rate x f x exposure: 30 deg/s ~ 5 px, 100 deg/s ~ 15 px (full res) | same at every height; more throttle, wind and corrections up there |
| moving sideways | speed x f x exposure / height: 1 m/s ~ 9 px at 1 m, ~ 1 px at 10 m | no, better |
| **detail lost by our 640 shrink** | one 640-px pixel = height / 291: 3.4 cm at 10 m, 12 cm at 35 m, 26 cm at 75 m | **yes** |
| focus (fixed lens) | soft at 1 m, sharp far away | no, better |

The live view and the recordings show the 640-wide picture the tracker uses, so above
~20 m gravel and grass disappear in it. That is resolution, not blur. The mentors'
"play with the latency": high up the ground crosses the picture slowly (1 m/s at 50 m ~
17 px/s at full resolution), so we can spend more time per picture on a sharper one.

**Step 2: find out which (read-only; stop vision_hold first; same floor each time)**
```bash
venv/bin/python -m tools.blur_check --tag still              # on a stand, props off
venv/bin/python -m tools.blur_check --tag wobble             # rock it by hand, slow then fast
venv/bin/python -m tools.blur_check --tag buf1 --buffers 1   # queue of 1 ...
venv/bin/python -m tools.blur_check --tag buf4 --buffers 4   # ... vs 4
```
It prints pictures per second, the **camera latency** (driver time -> our hands), and
sharpness per turn-rate band for the full-resolution centre and the 640 copy, and saves
lossless PNG pairs in `/data/blur_<tag>/`.

| You see | It is | Setting |
|---|---|---|
| sharpness falls in the faster rows | motion blur | keep `BF_MAX_TILT_RATE_DEG_S`; consider `BLUR_SKIP_RATIO = 0.4` |
| hover worse than hand-wobble at the same rate, wavy lines in PNGs | vibration | props / mount: organisers. Software keeps the sharpest pictures |
| full PNGs sharp, 640 ones soft | detail, not blur | `HIGH_ALT_CROP_FROM_M = 15.0` (after a test flight) |
| soft at 1 m but sharp out of a window | focus | leave the lens alone; bench only |
| low contrast after coming in from the cold | fog on the lens | wait ~10 min |
| buf4 ~60/s, buf1 ~30/s | 1-buffer queue drops frames | keep `CAMERA_BUFFERS_WITH_THREAD = 4` |

## 3. What changed in the program

| Setting (config.py) | Default | What it does |
|---|---|---|
| `CAMERA_BUFFERS_WITH_THREAD` | 4 | queue length with the grab thread (was forced to 1) |
| `CAMERA_DRIVER_TIMESTAMP` | True | picture time = driver's time minus READOUT_S/2 (middle row), not "when we got it". Recalibrate `calibrate_timing` after this change |
| `HOME_SHARPEST` / `HOME_SHARP_RATIO` / `HOME_SHARP_MAX_WAIT_FRAMES` | True / 0.6 / 8 | home waits up to 8 pictures for a sharp one |
| `BLUR_SKIP_RATIO` / `BLUR_SKIP_MAX_IN_A_ROW` | None / 2 | optional: skip much-blurrier pictures, at most 2 in a row |
| `BF_MAX_TILT_RATE_DEG_S` | 40 | our lean request changes at most 40 deg/s (0 -> 8 deg in 0.2 s). Not ready is still centred at once |
| `HIGH_ALT_CROP_FROM_M` / `HIGH_ALT_CROP_SIZE` | None / (960, 960) | engaged at or above this height: track a full-resolution crop of the lens centre. Chosen once per engage; needs `calib.npz` |

Files: `vision/sharpness.py`, `vision/process_mode.py`, `vision/front_camera.py`,
`compute/budget.py` (LatestFrame times, queue length), `fusion/position_controller.py`
(SlewLimiter), `link/betaflight_link.py`, `vision_hold.py`, `tools/blur_check.py`,
`tools/front_cam.py`, `tools/centre_mask.py`, `deploy/front_view.service`,
`tests/test_cameras.py` (simulated 30 m drift: crop worst error 0.05 cm vs 0.26 cm for 640).

## 4. Lens: use the part we measured

**Check first:** the `calib.npz` in the repo is still the **datasheet** one (623 px);
`vision_hold.py` now warns about it at start-up. Copy the measured file (874 px) from the
APB into the repo folder.

874 px at 1920 wide = a 95 deg pinhole-equivalent view; the datasheet's 114 deg counts the
squeezed fisheye edges. With edge correction off, corners near the edges seem to move less
than they do, so:

1. **Trusted zone (5 min):** checkerboard at one distance; measure the square size in pixels
   in the middle, half way out and near the edge. Where it is more than 2 % off the middle,
   the trusted zone ends.
2. `venv/bin/python -m tools.centre_mask 0.35` (your radius as a fraction of width) keeps
   landmarks inside it (works with the prop/OSD `mask.png`; `--undo` to go back).
3. Next time on the aircraft: 20 checkerboard shots including the corners, fisheye model,
   then `USE_LEVEL_POINTS = True` again.

## 5. Integration notes (21 Nov 2026, combined with the Betaflight changes)

These camera changes were written against `main`; they were combined with the Betaflight zip
(`docs/BETAFLIGHT_CHANGES.md`: stick curve, PD + I hold, 15 deg lean cap). What had to change:

- **Smooth leans work in degrees now.** The Betaflight changes replaced the straight-line stick mapping with
  Betaflight's own rates curve, so `SlewLimiter` converts through `link/stick_curve.py`: at most
  `BF_MAX_TILT_RATE_DEG_S` degrees of lean per second (0 -> 15 deg takes ~0.4 s). It runs after `HoldController`.
- **Armed or not** comes from the flight controller's own flag (MSP_STATUS) first, the arm channel only as a
  fallback (measured: radio off, the channel reads 1775 = "armed").
- `tests/test_cameras.py` test 4 checks the limit in degrees.
- **Sharpness is 10x cheaper**: 16-bit Laplacian + `meanStdDev`, the same number (laptop: 1.3 ms -> 0.12 ms a picture).
- **Replay fix (Betaflight changes):** the "re-home when landmarks drop" rule depends on the pilot's switch, which a
  recording does not store, so a replay could re-home where the live run did not (`tests/test_replay.py` failed
  7 runs of 9). It is now off in replay. Limit: a bench recording that re-homed this way replays differently
  until `engaged` is stored in `frames.csv`.

**Tested (laptop, simulated):** all 35 test files pass except `test_preflight` on Windows only (it makes a
symlink; Windows refuses without admin rights, same before these changes). `python -m tests.test_cameras` passes (30 m drift: crop 0.05 cm vs 0.26 cm
for the 640 picture). **Not tested on the aircraft:** everything in this file. First things to measure:
`tools/blur_check.py` (latency, buf1 vs buf4), then one bench run to see that pictures per second did not drop.
