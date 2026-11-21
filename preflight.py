# =====================================================================================
# preflight.py  -  ONE command before every flight: GO or NO-GO, with a reason per line
#                                                                     (OWNER: Ayush, task A2)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   A pilot's checklist, run by the computer. Each line says GO, WARN or NO-GO and WHY:
#     trial mode      GPS can't take over the scored hold (safety/trial_mode.py)
#     busy devices    nothing else (e.g. the vision_hold service) holds the camera / serial port
#     temperature     the APB is not too hot (fan holds ~62 C, it shuts itself down at 95 C)
#     storage         the SD card has room for the recording (evidence for the judges)
#     calibration     calib.npz is the Justice D's, made at the camera's size (Saumya's check_calib)
#     timing          timing.npz exists and its numbers are possible for a 60 fps camera
#     camera          /dev/video2 opens at 1920x1080 and really delivers ~60 fps
#     flight controller  BETAFLIGHT (our route, STAGE 2): the always-on sender (link/msp_sender.py) is running
#                     and the flight controller answers it (tilt and radio channels arriving).
#                     ARDUCOPTER fallback (--fc ardupilot): link/check_link.py passes (ArduPilot >= 4.5,
#                     attitude rate, height, params)
#     autopilot safety  no switch can select the GPS source set, GPS velocity is not fused,
#                     flight-mode channel is not the arm switch
#   Any NO-GO = do not fly. WARN = fly only if you understand it (or use --strict).
#
# HOW IT WORKS (technical)
#   Every check is a small function that returns Check(name, level, reason). Everything it
#   touches (files, /proc, the camera, the subprocess) is passed in, so tests run with fakes.
#   check_link.py is run as a subprocess (it has no importable function and blocks while it
#   waits for a heartbeat) and judged by its exit code. The autopilot-safety parameters
#   are read read-only with pymavlink AFTER check_link has released the serial port.
#
# HOW TO RUN (on the APB, from the repo folder, with the boot service stopped)
#   systemctl stop vision_hold
#   python preflight.py                                   # camera 2, /dev/ttymxc2 (APB manual p.12, p.14)
#   python preflight.py --bench                           # no autopilot connected: skip the link checks
#   python preflight.py --strict                          # WARN counts as NO-GO (the scored trial)
#   Exit code 0 = GO, 1 = NO-GO. The report is also saved to logs/preflight_<time>.txt.
#
# WHERE IT PLUGS IN
#   Uses calibration/check_calib.py (Saumya), vision/timing.py, link/check_link.py,
#   safety/trial_mode.py. Changes nothing on the drone: it only reads.
#   Settings: the PREFLIGHT_* block in config.py.
#
# REFERENCES
#   - DTK APB User Manual v1.7.2: p.3 (0-70 C board range), p.11 (thermal_zone0, fan ~62 C,
#     shutdown 95 C, /sdcard FAT32/exFAT/NTFS), p.12 (/dev/video2 fixed 1280x720 @ 60), p.14
#     (/dev/ttymxc2 to the flight controller).
#   - Justice D datasheet v1.0: 1280x720 @ 60 fps. MRM2-10AI manual p.13: CH5 = Arm/Disarm.
#   - ArduPilot: FLTMODE_CH default 5; RCx_OPTION 90 = EKF source set; EK3_SRC_OPTIONS bit 0
#     = fuse all velocities. https://ardupilot.org/copter/docs/common-ekf-sources.html
# =====================================================================================

import argparse
import contextlib
import io
import os
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass

import config

GO, WARN, NOGO = "GO", "WARN", "NO-GO"
SERVICE_FILES = ("deploy/vision_hold.service", "/etc/systemd/system/vision_hold.service")


@dataclass
class Check:
    name: str
    level: str
    reason: str


# ---------------- 1. trial mode ----------------
def check_trial_mode(service_files=SERVICE_FILES, trial_on=None):
    trial_on = config.TRIAL_MODE if trial_on is None else trial_on
    if not trial_on:
        return Check("trial mode", NOGO, "config.TRIAL_MODE is False: GPS could take over the scored hold")
    for path in service_files:
        if os.path.exists(path):
            with open(path) as f:
                start = [ln for ln in f if ln.strip().startswith("ExecStart=")]
            if any("--practice" in ln for ln in start):
                return Check("trial mode", NOGO, f"{path} starts vision_hold with --practice (GPS fallback allowed)")
    return Check("trial mode", GO, "ON: GPS fallback impossible, GPS commands blocked")


# ---------------- 2. busy devices ----------------
def check_busy(devices, proc_root="/proc", systemctl=None, my_pid=None):
    """Is another process (e.g. the vision_hold service) holding the camera or serial port?"""
    my_pid = os.getpid() if my_pid is None else my_pid
    devices = {os.path.realpath(d) for d in devices if d and d.startswith("/dev/")}
    holders = []
    if os.path.isdir(proc_root):
        for pid in filter(str.isdigit, os.listdir(proc_root)):
            if int(pid) == my_pid:
                continue
            fd_dir = os.path.join(proc_root, pid, "fd")
            try:
                fds = os.listdir(fd_dir)
            except OSError:
                continue
            for fd in fds:
                try:
                    target = os.readlink(os.path.join(fd_dir, fd))
                except OSError:
                    continue
                if target in devices:
                    try:
                        with open(os.path.join(proc_root, pid, "cmdline"), "rb") as f:
                            cmd = f.read().replace(b"\0", b" ").decode(errors="ignore").strip()
                    except OSError:
                        cmd = "?"
                    holders.append(f"pid {pid} ({cmd[:60]}) holds {target}")
    state = systemctl() if systemctl else _service_state()
    if state == "active":
        holders.insert(0, "the vision_hold service is running (systemctl stop vision_hold)")
    if holders:
        return Check("busy devices", NOGO, "; ".join(holders))
    return Check("busy devices", GO, f"camera and serial port free ({', '.join(sorted(devices)) or 'none given'})")


def _service_state():
    if not shutil.which("systemctl"):
        return "unknown"
    try:
        return subprocess.run(["systemctl", "is-active", "vision_hold"], capture_output=True,
                              text=True, timeout=5).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return "unknown"


# ---------------- 3. temperature ----------------
def check_temperature(path="/sys/class/thermal/thermal_zone0/temp"):
    try:
        with open(path) as f:
            c = int(f.read().strip()) / 1000.0
    except (OSError, ValueError):
        return Check("CPU temperature", WARN, f"can't read {path} (not on the APB?)")
    if c >= config.PREFLIGHT_TEMP_NOGO_C:
        return Check("CPU temperature", NOGO, f"{c:.0f} C >= {config.PREFLIGHT_TEMP_NOGO_C} C (board shuts down at 95 C): "
                                              f"let it cool, check the fan, shade it")
    if c >= config.PREFLIGHT_TEMP_WARN_C:
        return Check("CPU temperature", WARN, f"{c:.0f} C >= {config.PREFLIGHT_TEMP_WARN_C} C (fan normally holds ~62 C)")
    return Check("CPU temperature", GO, f"{c:.0f} C")


# ---------------- 4. storage ----------------
def _fs_type(path, mounts_file):
    try:
        with open(mounts_file) as f:
            rows = [ln.split() for ln in f if len(ln.split()) >= 3]
    except OSError:
        return None
    best = None
    for dev, mnt, fs, *_ in rows:
        if path == mnt or path.startswith(mnt.rstrip("/") + "/"):
            if best is None or len(mnt) > len(best[0]):
                best = (mnt, fs)
    return best[1] if best else None


def check_storage(path=None, mounts_file="/proc/mounts", usage=shutil.disk_usage):
    path = path or config.PREFLIGHT_SD_PATH
    if not os.path.isdir(path):
        return Check("storage", WARN, f"no {path}: recordings go to logs/ on the 4 GB eMMC (insert an SD card)")
    free_mb = usage(path).free / 1e6
    fs = _fs_type(path, mounts_file)
    minutes = free_mb / 250.0                       # lossless FFV1 worst case ~250 MB/min (config.py)
    if free_mb < config.RECORD_MIN_FREE_MB:
        return Check("storage", NOGO, f"{path}: {free_mb:.0f} MB free < {config.RECORD_MIN_FREE_MB} MB: no recording = no evidence")
    if free_mb < config.PREFLIGHT_SD_WARN_MB:
        return Check("storage", WARN, f"{path}: {free_mb:.0f} MB free (~{minutes:.0f} min of recording)")
    if fs == "vfat":
        return Check("storage", WARN, f"{path} is FAT32: one file stops at 4 GB (~16 min of recording); use exFAT")
    return Check("storage", GO, f"{path}: {free_mb / 1000:.1f} GB free (~{minutes:.0f} min of recording), {fs or 'fs ?'}")


# ---------------- 5. calibration ----------------
def check_calibration(path=None, camera_size=None):
    """Saumya's check_calib (size saved, focal plausible for 114 deg, centre) + size == camera."""
    from calibration.check_calib import check
    from vision.tracker import calibration_size
    path = path or config.CALIB_FILE
    camera_size = tuple(camera_size or config.PREFLIGHT_CAMERA_SIZE)
    out = io.StringIO()
    with contextlib.redirect_stdout(out):
        ok = check(path)
    lines = [ln.strip() for ln in out.getvalue().splitlines() if ln.strip()]
    fails = [ln for ln in lines if "FAIL" in ln]
    if not ok:
        why = fails[0] if fails else (lines[-1] if lines else "check_calib failed")
        return Check("calibration", NOGO, f"{path}: {why}")
    size = calibration_size(path)
    if size is not None and tuple(size) != camera_size:
        return Check("calibration", NOGO, f"{path} made at {size[0]}x{size[1]}, camera gives "
                                          f"{camera_size[0]}x{camera_size[1]}: recalibrate at the camera's size")
    return Check("calibration", GO, f"{path}: {size[0]}x{size[1]}, plausible for the Justice D")


# ---------------- 6. timing ----------------
def check_timing(path=None, fps=60.0):
    from vision.timing import load_timing
    path = path or config.TIMING_FILE
    if not os.path.exists(path):
        return Check("timing", NOGO, f"no {path}: rolling-shutter fix would use GUESSED numbers "
                                     f"(run python -m calibration.calibrate_timing)")
    t = load_timing(path)
    if not 0.0 < t.readout_s <= 1.0 / fps:
        return Check("timing", NOGO, f"readout {t.readout_s * 1000:.1f} ms impossible at {fps:.0f} fps "
                                     f"(must be 0..{1000 / fps:.1f} ms): redo the timing calibration")
    if abs(t.offset_s) > 0.1:
        return Check("timing", WARN, f"camera-IMU offset {t.offset_s * 1000:+.0f} ms is unusually large")
    return Check("timing", GO, f"readout {t.readout_s * 1000:.1f} ms, offset {t.offset_s * 1000:+.1f} ms")


# ---------------- 7. camera ----------------
def check_camera(source="2", opener=None, n_frames=30, clock=time.perf_counter):
    import cv2
    opener = opener or (lambda s: cv2.VideoCapture(int(s) if str(s).isdigit() else s))
    want_w, want_h = config.PREFLIGHT_CAMERA_SIZE
    cap = opener(source)
    try:
        if not cap.isOpened():
            return Check("camera", NOGO, f"can't open camera {source} (busy, unplugged, or wrong number)"), None
        ok, frame = cap.read()                      # first frame (often slow): not timed
        if not ok or frame is None:
            return Check("camera", NOGO, f"camera {source} opens but gives no picture"), None
        t0, got = clock(), 0
        for _ in range(n_frames):
            ok, frame = cap.read()
            got += bool(ok)
        dt = clock() - t0
    finally:
        cap.release()
    w, h = frame.shape[1], frame.shape[0]
    fps = got / dt if dt > 0 else 0.0
    if (w, h) != (want_w, want_h):
        return Check("camera", NOGO, f"camera {source} gives {w}x{h}, expected {want_w}x{want_h} (Justice D)"), (w, h)
    if fps < config.PREFLIGHT_MIN_FPS:
        return Check("camera", WARN, f"{w}x{h} but only {fps:.0f} fps (expected ~60)"), (w, h)
    return Check("camera", GO, f"{w}x{h} at {fps:.0f} fps"), (w, h)


# ---------------- 8. autopilot link (wraps link/check_link.py) ----------------
def check_link(connect, runner=subprocess.run, timeout=60):
    cmd = [sys.executable, "-m", "link.check_link", "--connect", connect]
    try:
        p = runner(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return Check("autopilot link", NOGO, f"no answer from {connect} within {timeout} s (no heartbeat?)")
    except OSError as e:
        return Check("autopilot link", NOGO, f"could not run check_link: {e}")
    text = (p.stdout or "") + (p.stderr or "")
    if p.returncode == 0:
        return Check("autopilot link", GO, "check_link PASS (firmware, attitude rate, height, parameters)")
    fixes = re.findall(r"^\s+- (.+)$", text.split("FIX BEFORE FLIGHT:")[-1], re.M) if "FIX BEFORE FLIGHT" in text else []
    why = "; ".join(fixes) if fixes else (text.strip().splitlines()[-1] if text.strip() else f"exit code {p.returncode}")
    return Check("autopilot link", NOGO, f"check_link: {why}")


# ---------------- 8b. Betaflight link (STAGE 2) ----------------
def check_betaflight(link=None, wait_s=3.0, sleep=time.sleep):
    """Is link/msp_sender.py running, and does the flight controller answer it? Listens to the sender's
    telemetry for wait_s seconds. Read-only: nothing is sent to the aircraft."""
    if link is None:
        from link.betaflight_link import BetaflightLink
        link = BetaflightLink()
        try:
            link.start()
        except OSError as e:
            return Check("Betaflight link", NOGO, f"cannot listen for the sender ({e}): is vision_hold still running?")
    sleep(wait_s)
    link.running = False
    if link.mode_time is None:
        return Check("Betaflight link", NOGO, "nothing from link/msp_sender.py (systemctl start msp_sender)")
    if not link.attitudes or not link.rc:
        return Check("Betaflight link", NOGO, "the sender runs but the flight controller does not answer "
                     "(battery in? /dev/ttymxc2 at 115200? ask the organisers, never gpioset)")
    hz = len(link.attitudes) / wait_s if wait_s else 0.0
    if wait_s and hz < 20:
        return Check("Betaflight link", WARN, f"tilt arrives at only {hz:.0f} Hz (want about 50)")
    if len(link.rc) < config.BF_ENGAGE_CHANNEL:
        return Check("Betaflight link", NOGO, f"only {len(link.rc)} radio channels reported: no engage switch (CH{config.BF_ENGAGE_CHANNEL})")
    return Check("Betaflight link", GO, f"sender running, tilt at {hz:.0f} Hz, {len(link.rc)} radio channels, "
                 f"engage switch {'ON' if link.engaged else 'off'}; no GNSS is requested on this route")


# ---------------- 9. autopilot safety parameters ----------------
SAFETY_PARAMS = ["FLTMODE_CH", "EK3_SRC_OPTIONS"] + [f"RC{i}_OPTION" for i in range(5, 17)]


def check_autopilot_safety(params):
    """params: {name: value} read from the autopilot. Missing ones are reported, not guessed."""
    out = []
    switches = [n for n in params if n.endswith("_OPTION") and int(params[n]) == 90]
    if switches:
        out.append(Check("GPS on a switch", NOGO, f"{', '.join(switches)} = 90 (EKF source set): a pilot switch "
                                                  f"could select source set 1 = GPS during the scored hold"))
    else:
        out.append(Check("GPS on a switch", GO, "no RC channel selects the EKF source set"))
    if "EK3_SRC_OPTIONS" not in params:
        out.append(Check("GPS velocity", WARN, "could not read EK3_SRC_OPTIONS"))
    elif int(params["EK3_SRC_OPTIONS"]) & 1:
        out.append(Check("GPS velocity", NOGO, "EK3_SRC_OPTIONS bit 0 set: the EKF fuses GPS velocity even "
                                               "on the vision source set = GNSS aiding"))
    else:
        out.append(Check("GPS velocity", GO, "EK3_SRC_OPTIONS = 0: only the active source set is fused"))
    ch = params.get("FLTMODE_CH")
    if ch is None:
        out.append(Check("flight-mode channel", WARN, "could not read FLTMODE_CH"))
    elif int(ch) == 5:
        out.append(Check("flight-mode channel", WARN, "FLTMODE_CH = 5, but the event radio has Arm/Disarm on CH5 and "
                                                      "the mode switch on CH9 (MRM2-10AI manual p.13): confirm"))
    else:
        out.append(Check("flight-mode channel", GO, f"FLTMODE_CH = {int(ch)}"))
    return out


def read_params(connect, names, timeout=10.0):
    """Read-only: ask the autopilot for these parameters. Returns {name: value}."""
    from link.drone_link import BAUD      # noqa: F401  (also switches pymavlink to MAVLink 2)
    from pymavlink import mavutil
    conn = mavutil.mavlink_connection(connect, baud=BAUD)
    try:
        if conn.wait_heartbeat(timeout=timeout) is None:
            return {}
        for n in names:
            conn.mav.param_request_read_send(conn.target_system, conn.target_component, n.encode(), -1)
        got, end = {}, time.time() + 3.0
        while time.time() < end and len(got) < len(names):
            m = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if m is not None and m.param_id in names:
                got[m.param_id] = m.param_value
        return got
    finally:
        conn.close()


# ---------------- running everything ----------------
def verdict(checks, strict=False):
    bad = [c for c in checks if c.level == NOGO or (strict and c.level == WARN)]
    return NOGO if bad else GO


def report(checks, strict=False):
    width = max(len(c.name) for c in checks)
    lines = [f"PRE-FLIGHT  {time.strftime('%Y-%m-%d %H:%M:%S')}  (board clock: may be wrong after a USB-only boot)"]
    lines += [f"  {c.level:5s}  {c.name:{width}s}  {c.reason}" for c in checks]
    v = verdict(checks, strict)
    lines.append(f"\nVERDICT: {v}" + ("  (strict: WARN counts as NO-GO)" if strict else ""))
    if v == NOGO:
        lines.append("Do not fly. Fix every NO-GO line above, then run preflight again.")
    return "\n".join(lines)


def run_checks(args, runner=subprocess.run, reader=read_params):
    checks = [check_trial_mode()]
    dev_cam = f"/dev/video{args.camera}" if str(args.camera).isdigit() else args.camera
    betaflight = getattr(args, "fc", config.FC) == "betaflight"      # STAGE 2
    # Betaflight: the serial port is held by the msp_sender service ON PURPOSE, so only the camera must be free
    checks.append(check_busy([dev_cam] if betaflight else [dev_cam, args.connect]))
    checks.append(check_temperature())
    checks.append(check_storage())
    cam_check, cam_size = check_camera(args.camera)
    checks.append(check_calibration(camera_size=cam_size))
    checks.append(check_timing())
    checks.append(cam_check)
    if betaflight:
        if not args.bench:
            checks.append(check_betaflight())
    elif not args.bench:
        checks.append(check_link(args.connect, runner=runner))
        checks += check_autopilot_safety(reader(args.connect, SAFETY_PARAMS))
    return checks


def main(argv=None):
    ap = argparse.ArgumentParser(description="Pre-flight GO / NO-GO")
    ap.add_argument("--connect", default="/dev/ttymxc2", help="autopilot link (APB manual p.14)")
    ap.add_argument("--camera", default="2", help="camera (2 = /dev/video2, APB manual p.12)")
    ap.add_argument("--bench", action="store_true", help="no flight controller connected: skip the link checks")
    ap.add_argument("--fc", default=config.FC, choices=["betaflight", "ardupilot"],   # STAGE 2
                    help="flight controller route (default: config.FC)")
    ap.add_argument("--strict", action="store_true", default=config.PREFLIGHT_STRICT,
                    help="WARN counts as NO-GO (use for the scored trial)")
    args = ap.parse_args(argv)
    checks = run_checks(args)
    text = report(checks, args.strict)
    print(text)
    os.makedirs("logs", exist_ok=True)
    path = time.strftime("logs/preflight_%Y%m%d_%H%M%S.txt")
    with open(path, "w") as f:
        f.write(text + "\n")
    print(f"Saved: {path}")
    return 0 if verdict(checks, args.strict) == GO else 1


if __name__ == "__main__":
    sys.exit(main())
