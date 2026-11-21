#!/usr/bin/env python3
"""DroneCode vs the Stage 2 package: which known clashes are still open in this checkout?

  python stage2_check.py [path to the DroneCode repo, default: current folder]

Read-only. Standard library only. Prints one line per check: FAIL (breaks a Stage 2 rule or a
bench/flight requirement), WARN (needs a decision or depends on the chosen flight controller),
ok. Exit code 1 if anything is FAIL. The checks are plain text searches on purpose: they report
what the files say now, so this stays true as teammates fix things.
"""
import os
import re
import subprocess
import sys
import zipfile

repo = os.path.abspath(sys.argv[1] if len(sys.argv) > 1 else ".")
rows = []


def read(rel):
    try:
        with open(os.path.join(repo, rel), encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return None


def code_lines(text, comment="#"):
    """Lines with their comments removed (a rule only counts if it is live code)."""
    return [ln.split(comment, 1)[0] for ln in (text or "").splitlines()]


def add(level, name, detail):
    rows.append((level, name, detail))


def setting(text, name):
    m = re.search(rf"^{name}\s*=\s*(.+?)\s*(?:#.*)?$", text or "", re.M)
    return m.group(1).strip() if m else None


if not os.path.exists(os.path.join(repo, "vision_hold.py")):
    sys.exit(f"{repo} does not look like the DroneCode repo (no vision_hold.py)")

# 1. GPIO
service = read("deploy/vision_hold.service")
live = [ln for ln in code_lines(service) if "gpioset" in ln]
add("FAIL" if live else "ok", "no gpioset in the boot service",
    f"{len(live)} live gpioset line(s) in deploy/vision_hold.service" if live else "none")
disc = read("tools/discover_hardware.py") or ""
has = any(re.search(r"\[\s*[\"']gpioset[\"']", ln) for ln in code_lines(disc))
add("WARN" if has else "ok", "no gpioset in tools",
    "tools/discover_hardware.py can still run gpioset (--route-fc-uart)" if has else "none")

# 2. camera size
cfg = read("config.py")
size = setting(cfg, "PREFLIGHT_CAMERA_SIZE")
add("ok" if size and "1920" in size and "1080" in size else "FAIL", "preflight expects 1920x1080",
    f"config.PREFLIGHT_CAMERA_SIZE = {size}")
cc = read("calibration/check_calib.py") or ""
spec = re.search(r"^SPEC_W,\s*SPEC_H\s*=\s*(\d+),\s*(\d+)", cc, re.M)
spec = f"{spec.group(1)}x{spec.group(2)}" if spec else "?"
add("ok" if spec == "1920x1080" else "WARN", "datasheet calibration size", f"check_calib SPEC = {spec} (camera is 1920x1080)")

# 3. calibration, timing, mask
calib = os.path.join(repo, "calib.npz")
if not os.path.exists(calib):
    add("FAIL", "lens calibration", "no calib.npz")
else:
    try:
        names = zipfile.ZipFile(calib).namelist()
    except zipfile.BadZipFile:
        names = []
    add("ok" if "size.npy" in names else "FAIL", "lens calibration",
        "calib.npz records its image size" if "size.npy" in names
        else "calib.npz has no saved size: the laptop-webcam file, wrong for the Justice D")
add("ok" if os.path.exists(os.path.join(repo, "timing.npz")) else "WARN", "camera timing",
    "timing.npz present" if os.path.exists(os.path.join(repo, "timing.npz")) else "no timing.npz: rolling-shutter fix uses guessed numbers")
add("ok" if os.path.exists(os.path.join(repo, "mask.png")) else "WARN", "feature-filter mask",
    "mask.png present" if os.path.exists(os.path.join(repo, "mask.png")) else "no mask.png: feature filter is off")

# 4. flight-controller route
py = []
for root, dirs, files in os.walk(repo):
    dirs[:] = [d for d in dirs if d not in ("venv", ".git", "__pycache__", "logs")]
    py += [os.path.join(root, f) for f in files if f.endswith(".py")]
msp = [os.path.relpath(p, repo) for p in py
       if re.search(r"MSP_SET_RAW_RC|msp_set_raw_rc", open(p, encoding="utf-8", errors="replace").read())]
add("ok" if msp else "WARN", "Betaflight (MSP) output",
    ", ".join(msp[:4]) if msp else "none in the repo: only the ArduPilot/MAVLink route exists")
# the GNSS rule on the Betaflight route: no code may name or build a GPS request
gps_req = [os.path.relpath(p, repo) for p in py if os.sep + "tests" + os.sep not in p and ".claude" not in p and os.path.basename(p) != "msp.py"
           and re.search(r"MSP_RAW_GPS|MSP_COMP_GPS|MSP_GPSSVINFO", open(p, encoding="utf-8", errors="replace").read())]
allowed = re.search(r"ALLOWED\s*=\s*frozenset\(\{([^}]*)\}", read("link/msp.py") or "")
if msp:
    add("FAIL" if gps_req else "ok", "Betaflight: no GPS request anywhere", ", ".join(gps_req) if gps_req
        else "only " + (allowed.group(1).strip() if allowed else "?") + " can be sent")
    sender_unit = read("deploy/msp_sender.service")
    add("ok" if sender_unit and "Restart=always" in sender_unit else "FAIL", "Betaflight: always-on sender service",
        "deploy/msp_sender.service restarts by itself" if sender_unit else "no deploy/msp_sender.service")
    unit_fc = re.search(r"^ExecStart=.*--fc (\w+)", service or "", re.M)
    add("ok" if unit_fc else "WARN", "boot service names its flight controller",
        f"vision_hold.service starts with --fc {unit_fc.group(1)}" if unit_fc else "no --fc in ExecStart")
baud = setting(read("link/drone_link.py"), "BAUD")
add("ok" if baud == "230400" else "WARN", "ArduCopter serial baud", f"link/drone_link.py BAUD = {baud} (Stage 2: 230400)")
param = read("ardupilot/vision_hold.param") or ""
yaw = re.findall(r"^(EK3_SRC\d_YAW),\s*(\d+)", param, re.M)
compass = [n for n, v in yaw if v == "1"]
add("WARN" if compass else "ok", "ArduCopter route only: yaw source without a magnetometer",
    f"{', '.join(compass)} = 1 (compass), but the aircraft has no magnetometer" if compass else "no compass yaw configured")
ch10 = re.search(r"^(FLTMODE_CH|RC10_OPTION),", param, re.M)
add("ok" if ch10 else "WARN", "ArduCopter route only: engage switch CH10 mapped",
    ch10.group(0).rstrip(",") + " is set" if ch10 else "no FLTMODE_CH / RC10_OPTION in vision_hold.param")

# 5. GNSS during the hold, limits
gm = setting(cfg, "GNSS_MONITOR")
add("WARN" if gm == "True" else "ok", "no GNSS reads during a hold",
    "config.GNSS_MONITOR = True: reads GPS_RAW_INT while engaged" if gm == "True" else f"GNSS_MONITOR = {gm}")
official, max_m = setting(cfg, "HOLD_LIMITS_OFFICIAL"), setting(cfg, "HOLD_MAX_DRIFT_M")
add("ok" if official == "True" else "WARN", "official hold limits",
    f"HOLD_LIMITS_OFFICIAL = {official}, HOLD_MAX_DRIFT_M = {max_m} (Stage 2: 30 s within 3 m horizontal, 2 m vertical)")

# 6. bench needs
vh = read("vision_hold.py") or ""
rh = read("vision/rehome.py") or ""
due_body = "\n".join(code_lines(rh)).split("def due", 1)[-1].split("def ", 1)[0]   # the body of Rehome.due()
uses_shift = "shift" in due_body
add("ok" if uses_shift else "WARN", "re-home on sideways movement",
    "rehome.due() considers shift" if uses_shift
    else "rehome.due() looks at scale only: home leaves the view on the 1 m bench")
# "no data" packets: an explicit quality -1 send, or zeros substituted for an untrusted position
sends_no_data = setting(cfg, "ARDUPILOT_SILENT_WHEN_NOT_READY") != "True" and bool(
    re.search(r"send_odometry\(.*-1,\s*reset_counter", vh) or re.search(r"else\s*\(0\.0,\s*0\.0\)", vh))
add("WARN" if sends_no_data else "ok", "silent when not ready",
    "vision_hold still sends ODOMETRY (zeros, quality -1) when not ready (bench D/E1 on ArduCopter: 'no position')"
    if sends_no_data else "ArduCopter: nothing is sent when not ready; Betaflight: centred sticks")
lvl = setting(cfg, "USE_LEVEL_POINTS")
add("ok" if lvl == "True" else "WARN", "wide-lens tilt fix (level points)",
    f"config.USE_LEVEL_POINTS = {lvl}: simulation says it is needed from about 3 deg of lean, but on the real camera (21 Nov) it made tracking worse until the lens edges are calibrated")
missing = [t for t in ("test_bench", "test_level", "test_betaflight") if not os.path.exists(os.path.join(repo, "tests", t + ".py"))]
add("WARN" if missing else "ok", "Stage 2 rehearsal tests present", ", ".join(missing) + " missing" if missing else "test_bench, test_level, test_betaflight")

# 7. git
def git(*args):
    try:
        return subprocess.run(["git", "-C", repo, *args], capture_output=True, text=True, timeout=20).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return ""


print(f"DroneCode Stage 2 check: {repo}")
print(f"  branch {git('rev-parse', '--abbrev-ref', 'HEAD') or '?'}  commit {git('log', '-1', '--format=%h %ad %s', '--date=short') or '?'}")
dirty = git("status", "--porcelain")
print(f"  working tree: {'clean' if not dirty else str(len(dirty.splitlines())) + ' changed/untracked file(s)'}\n")
width = max(len(n) for _, n, _ in rows)
for level, name, detail in rows:
    print(f"  {level:4s}  {name:{width}s}  {detail}")
fails = sum(level == "FAIL" for level, _, _ in rows)
warns = sum(level == "WARN" for level, _, _ in rows)
print(f"\n{fails} FAIL, {warns} WARN, {len(rows) - fails - warns} ok")
sys.exit(1 if fails else 0)
