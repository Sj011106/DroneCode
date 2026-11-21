# Live view: the laptop screen for the judges

Stage 2 bench rule: **"Your laptop shows your live estimate."** This is that screen.

- It runs **on the drone**, inside `vision_hold.py`, as a tiny web server.
- The laptop only needs a **browser**. Nothing to install on it.
- The drone does **not** need it. With the laptop unplugged it does no work at all. It can't crash the vision loop, it never sends anything to the flight controller, and if it can't start, `vision_hold` simply carries on.

## What you see

| Part | Meaning | Judge step |
|---|---|---|
| Big badge **READY** / **NOT READY** / **NO DATA** | NOT READY = our sticks are centred. NO DATA = no update for 1 s | D, E1 (cover the camera: must turn red within 1 s) |
| Camera picture with a yellow cross and **"nose"** | What the tracker sees. The cross is straight below the drone; "nose" marks where the code thinks the front is | First-hour check: put a hand under the nose; it must appear at the top |
| **Distance from home** | Metres from home | B1 (within 0.25 m of the tape) |
| **Forward / Right** | Same, split along the nose direction at home | B1 direction, C |
| **Height change** | Metres since home (from the picture's zoom on the bench) | B3 (0.5 m ± 0.15) |
| **Track since home** | Top view of the path, 1 m grid, grey circle = 0.3 m | B2 loop (must end inside the circle) |
| Chips | engage switch, tracker status + quality, frames per second | D, A |
| Table | our stick request, height, re-locks, re-homes, data age | C, debugging |

"Clear trail" only clears the drawing in your browser. Nothing on the page changes the estimate.

---

## Step 1: where the code is

- Branch **`praise-live-view`**, made from `main` at `753ffff`. Main is not touched.
- New files: `tools/live_view.py`, `tests/test_live_view.py` and this page.
- Small additions, all marked `# STAGE 2`:
  - 4 calls in `vision_hold.py`
  - the switch `LIVE_VIEW` and its settings in `config.py`
  - one line in the README

Try it on a laptop (no drone needed):

```
git fetch origin
git checkout praise-live-view
python -m tests.test_live_view
python -m tools.live_view --demo
```

Open http://localhost:8088. You'll see a fake floor going round a loop, with a 2 s "covered" moment every lap. Ctrl+C to stop.

Getting it into `main` is a team decision: open a Pull Request `praise-live-view` -> `main` on GitHub.

## Step 2: on the drone

Copy the repo to the drone as usual (`/data/DroneCode`), then restart the program:

```
ssh root@192.168.75.1
systemctl restart vision_hold
journalctl -u vision_hold -n 20
```

The journal should show: `Live view: open http://192.168.75.1:8088 on the laptop`.

On the laptop, open **http://192.168.75.1:8088** in Chrome or Edge.

**Picture for evidence:** http://192.168.75.1:8088/snapshot.jpg (save it in the evidence folder).

## If it doesn't show

| Symptom | Fix |
|---|---|
| Browser can't connect | `ping 192.168.75.1` from the laptop. If that fails, it's the USB-C cable or USB Ethernet, not the page |
| Journal says "Live view off (... Address already in use)" | Another program uses port 8088. Set `LIVE_VIEW_PORT = 8089` in `config.py` and use that number in the browser |
| Page loads but stays NO DATA | `vision_hold` is not running or crashed: `systemctl status vision_hold` |
| Picture says "no camera picture" | The camera isn't delivering. Check `journalctl -u vision_hold` |
| Switch it off completely | `LIVE_VIEW = False` in `config.py` |

## Cost on the drone

- **No browser open:** two tiny function calls per frame, no copies, no JPEG.
- **One browser open:** one 640×360 JPEG about 8 times a second (a few ms each, in a separate thread).

At most 3 browsers can watch the video at once.
