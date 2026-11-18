# =====================================================================================
# tools/discover_hardware.py  -  "what hardware do we actually have?"  (run on the APB)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   No hardware knowledge needed. Copy the repo onto the drone's computer (the APB) and
#   run this once. It looks around and prints, in plain words:
#     - what computer it is (CPU, cores, memory, operating system, Python version),
#     - whether our libraries are installed and whether OpenCV can talk to cameras,
#     - every camera it can find: resolution, frame rate, and a sample picture saved
#       to logs/ so you can SEE which one points at the ground,
#     - every serial port, and which one has the flight controller on it (it listens
#       for the autopilot's "heartbeat" at the usual speeds) + the ArduPilot version.
#     - the APB's own sensors (IMU + barometer on the Linux side), board temperature and
#       whether the micro-SD card (/sdcard, for recordings) is there.
#   At the end it prints the exact --camera and --connect values to use with
#   vision_hold.py, check_link.py and load_params.py. Send the output to the team.
#   It only LISTENS on serial ports: it never sends commands that change anything.
#   STOP THE SERVICE FIRST (sudo systemctl stop vision_hold): two programs on one serial
#   port corrupt each other's messages. The tool refuses to scan while vision_hold runs.
#
# APB FACTS (DTK APB User Manual v1.7.2)
#   - Flight-controller link = /dev/ttymxc2 (UART3), behind an internal switch that must be
#     set first (p.14):  gpioset -t0 -c gpiochip3 13=1 ; gpioset -t0 -c gpiochip3 0=0
#     --route-fc-uart runs exactly those two commands (never done by default).
#   - Justice D on CSI-1 = /dev/video2 (CSI-2 = /dev/video3), fixed 1280x720@60, no exposure
#     or gain control (p.12). vision_hold.py: --camera 2 (same as --camera /dev/video2).
#   - ICM42605 IMU + DPS310 barometer sit on the i.MX side (p.5); thermal shutdown at 95 C (p.11).
#
# HOW IT WORKS (technical)
#   Linux /proc and /sys for CPU/RAM, /etc/os-release, glob /dev/video* and opens each
#   with cv2.CAP_V4L2, glob /dev/tty{mxc,S,AMA,USB,ACM}* and tries pymavlink
#   wait_heartbeat at 921600/115200/57600/1500000 baud. ArduPilot version from the
#   AUTOPILOT_VERSION message (requested with MAV_CMD_REQUEST_MESSAGE, read-only).
#   On Windows/macOS it runs the parts that make sense (for testing on a laptop).
#
# HOW TO RUN (from the repo root, on the APB)
#   venv/bin/python -m tools.discover_hardware                  # everything (~1-2 min)
#   venv/bin/python -m tools.discover_hardware --route-fc-uart  # first set the FC UART switch
#   venv/bin/python -m tools.discover_hardware --no-serial      # skip the serial-port scan
#
# REFERENCES
#   - Linux V4L2 video devices: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html
#   - MAVLink HEARTBEAT / AUTOPILOT_VERSION: https://mavlink.io/en/messages/common.html
#   - NXP i.MX 8M Plus (the APB's onboard computer): https://www.nxp.com/products/i.MX8MPLUS
#   - Linux IIO sensors: https://www.kernel.org/doc/html/latest/driver-api/iio/index.html
#   - Orqa DTK APB User Manual v1.7.2 (OEM folder of the event technical package)
# =====================================================================================

import argparse
import glob
import os
import platform
import shutil
import subprocess
import sys
import time

BAUDS = (921600, 115200, 57600, 1500000)
SERIAL_PATTERNS = ("/dev/ttymxc2", "/dev/ttymxc*", "/dev/ttyS*", "/dev/ttyAMA*", "/dev/ttyUSB*",
                   "/dev/ttyACM*", "/dev/serial/by-id/*")      # APB FC link (ttymxc2) first
FC_UART_ROUTE = (["gpioset", "-t0", "-c", "gpiochip3", "13=1"],   # APB manual p.14: SW1 = 1
                 ["gpioset", "-t0", "-c", "gpiochip3", "0=0"])    #                  SW0 = 0


def section(title):
    print(f"\n=== {title} " + "=" * max(0, 70 - len(title)))


def read(path):
    try:
        with open(path) as f:
            return f.read()
    except OSError:
        return ""


def computer():
    section("Computer")
    print(f"System   : {platform.system()} {platform.release()} ({platform.machine()})")
    os_name = [l.split("=", 1)[1].strip('"') for l in read("/etc/os-release").splitlines()
               if l.startswith("PRETTY_NAME=")]
    if os_name:
        print(f"OS       : {os_name[0]}")
    cpu = read("/proc/cpuinfo")
    model = [l.split(":", 1)[1].strip() for l in cpu.splitlines()
             if l.lower().startswith(("model name", "hardware", "cpu part"))]
    soc = read("/sys/devices/soc0/soc_id").strip() or read("/proc/device-tree/model").strip("\x00\n")
    print(f"CPU      : {os.cpu_count()} cores  {model[0] if model else platform.processor()}")
    if soc:
        print(f"Board/SoC: {soc}")
    mhz = read("/sys/devices/system/cpu/cpu0/cpufreq/cpuinfo_max_freq").strip()
    if mhz:
        print(f"Max clock: {int(mhz) / 1000:.0f} MHz")
    mem = [l for l in read("/proc/meminfo").splitlines() if l.startswith("MemTotal")]
    if mem:
        print(f"Memory   : {int(mem[0].split()[1]) / 1024:.0f} MB")
    total, _, free = shutil.disk_usage(".")
    print(f"Disk     : {free / 1e9:.1f} GB free of {total / 1e9:.1f} GB")
    print(f"Python   : {sys.version.split()[0]}  ({sys.executable})")
    if glob.glob("/dev/galcore") or glob.glob("/dev/neutron*") or glob.glob("/dev/vipnano*"):
        print("NPU      : AI accelerator device found (not needed by our code)")


def libraries():
    section("Python libraries")
    ok = True
    for name in ("numpy", "cv2", "pymavlink", "matplotlib"):
        try:
            mod = __import__(name)
            print(f"  {name:10s} {getattr(mod, '__version__', 'installed')}")
        except ImportError:
            print(f"  {name:10s} MISSING" + ("  (only needed for plots)" if name == "matplotlib" else ""))
            ok = ok and name == "matplotlib"
    try:
        import cv2
        info = cv2.getBuildInformation()
        for key in ("V4L/V4L2", "GStreamer", "FFMPEG"):
            line = next((l.strip() for l in info.splitlines() if l.strip().startswith(key)), f"{key}: ?")
            print(f"  OpenCV video backend  {line}")
        print("  (pip's opencv has NO GStreamer: if the camera only works through GStreamer,")
        print("   install the board vendor's OpenCV instead)")
    except ImportError:
        pass
    if not ok:
        print("  -> pip install -r requirements.txt  (offline: see docs/hardware.md)")


def cameras(save_dir="logs"):
    section("Cameras")
    try:
        import cv2
    except ImportError:
        print("  OpenCV missing - can't test cameras")
        return []
    if shutil.which("v4l2-ctl"):
        out = subprocess.run(["v4l2-ctl", "--list-devices"], capture_output=True, text=True).stdout
        print("  v4l2-ctl --list-devices:\n    " + out.strip().replace("\n", "\n    "))
    devices = sorted(glob.glob("/dev/video*"), key=lambda d: int(d.replace("/dev/video", "") or 0))
    if not devices:                                     # Windows / macOS: try indexes
        devices = list(range(4))
    os.makedirs(save_dir, exist_ok=True)
    found = []
    for dev in devices:
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2) if isinstance(dev, str) else cv2.VideoCapture(dev)
        if not cap.isOpened():
            cap.release()
            continue
        frame = None
        for _ in range(5):                              # first frames are often black
            ok, f = cap.read()
            if ok:
                frame = f
        if frame is None:
            print(f"  {dev}: opens but gives no picture (metadata node or needs GStreamer)")
            cap.release()
            continue
        h, w = frame.shape[:2]
        t0, n = time.time(), 0
        while time.time() - t0 < 1.0:                   # measure real frame rate for 1 s
            n += cap.read()[0]
        index = int(str(dev).replace("/dev/video", ""))   # the number to pass as --camera
        path = os.path.join(save_dir, f"hw_camera_{index}.jpg")
        cv2.imwrite(path, frame)
        print(f"  {dev}: {w}x{h}, ~{n} fps measured (driver says {cap.get(cv2.CAP_PROP_FPS):.0f}), "
              f"brightness {frame.mean():.0f}/255 -> sample saved {path}  (use --camera {index})")
        found.append(index)
        cap.release()
    if not found:
        print("  No camera could be opened with OpenCV.")
    return found


def vision_hold_running():
    """True if the boot service or a manual vision_hold.py is using the serial port."""
    if shutil.which("systemctl"):
        r = subprocess.run(["systemctl", "is-active", "vision_hold"], capture_output=True, text=True)
        if r.stdout.strip() == "active":
            return True
    if shutil.which("pgrep"):
        return subprocess.run(["pgrep", "-f", "vision_hold.py"], capture_output=True).returncode == 0
    return False


def route_fc_uart():
    """APB manual p.14: switch UART3 (/dev/ttymxc2) to the flight controller."""
    section("Routing /dev/ttymxc2 to the flight controller (APB manual p.14)")
    for cmd in FC_UART_ROUTE:
        print("  $ " + " ".join(cmd))
        if not shutil.which(cmd[0]):
            print("  gpioset not found - is this the APB?")
            return
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode:
            print(f"  failed: {r.stderr.strip()}")


def serial_ports(timeout_s=2.5):
    section("Serial ports / flight controller")
    try:
        os.environ["MAVLINK20"] = "1"
        from pymavlink import mavutil
    except ImportError:
        print("  pymavlink missing - can't look for the flight controller")
        return []
    if vision_hold_running():
        print("  vision_hold is running (boot service or by hand): NOT scanning, two programs on one")
        print("  serial port corrupt each other. Stop it first: sudo systemctl stop vision_hold")
        return []
    ports, seen = [], set()
    for pat in SERIAL_PATTERNS:                          # keep priority order, skip duplicates
        for p in sorted(glob.glob(pat)):
            real = os.path.realpath(p)                   # /dev/serial/by-id/* are links
            if real not in seen:
                seen.add(real)
                ports.append(p)
    if not ports:
        print("  No serial ports found (normal on Windows: use COMx with check_link).")
        return []
    hits = []
    for port in ports:
        for baud in BAUDS:
            try:
                m = mavutil.mavlink_connection(port, baud=baud)
                hb = m.wait_heartbeat(timeout=timeout_s)
            except Exception as e:                      # busy, no permission, not a tty, I/O error
                print(f"  {port}: can't use ({e.__class__.__name__}: {e})")
                break
            if hb is None:
                m.close()
                continue
            autopilot = mavutil.mavlink.enums["MAV_AUTOPILOT"][hb.autopilot].name
            version = "?"
            try:
                m.mav.command_long_send(m.target_system, m.target_component,
                                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 148, 0, 0, 0, 0, 0, 0)
                av = m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=2)
                if av:
                    v = av.flight_sw_version
                    version = f"{v >> 24 & 255}.{v >> 16 & 255}.{v >> 8 & 255}"
            except Exception:
                pass
            print(f"  {port} @ {baud}: FLIGHT CONTROLLER FOUND  {autopilot}, firmware {version}, "
                  f"mode {m.flightmode}")
            hits.append((port, baud, version, autopilot))
            m.close()
            break
        else:
            print(f"  {port}: no MAVLink heartbeat at {', '.join(map(str, BAUDS))}")
    if not hits and "/dev/ttymxc2" in ports:
        print("  Hint: on the APB the FC link is /dev/ttymxc2 behind a switch: rerun with --route-fc-uart")
    return hits


def board():
    section("APB sensors, temperature, storage")
    names = [read(os.path.join(d, "name")).strip() for d in sorted(glob.glob("/sys/bus/iio/devices/iio:device*"))]
    if names:
        print(f"  Linux-side sensors (IIO): {', '.join(n or '?' for n in names)}")
        print("  (APB manual p.5: ICM42605 IMU + DPS310 barometer on the i.MX side - a possible")
        print("   high-rate IMU for the rolling-shutter fix later; not used by our code yet)")
    else:
        print("  No Linux-side IIO sensors listed")
    temp = read("/sys/class/thermal/thermal_zone0/temp").strip()
    if temp:
        print(f"  Board temperature: {int(temp) / 1000:.1f} C (fan holds ~62 C; shutdown at 95 C)")
    if os.path.ismount("/sdcard"):
        total, _, free = shutil.disk_usage("/sdcard")
        print(f"  /sdcard: {free / 1e9:.1f} GB free of {total / 1e9:.1f} GB -> recordings go here")
    else:
        print("  /sdcard: no micro-SD card mounted -> recordings go to logs/ on the 4 GB eMMC")


def main():
    ap = argparse.ArgumentParser(description="Find out what hardware this computer has")
    ap.add_argument("--no-serial", action="store_true")
    ap.add_argument("--no-camera", action="store_true")
    ap.add_argument("--route-fc-uart", action="store_true",
                    help="APB only: run the two gpioset commands that connect /dev/ttymxc2 to the FC")
    args = ap.parse_args()

    computer()
    libraries()
    board()
    if args.route_fc_uart:
        route_fc_uart()
    cams = [] if args.no_camera else cameras()
    fcs = [] if args.no_serial else serial_ports()

    section("What to use")
    if cams:
        print(f"  Cameras found: {', '.join(map(str, cams))}. Open the saved logs/hw_camera_*.jpg:")
        print("  the one showing the GROUND is the Justice D (manual: /dev/video2) -> --camera <its NUMBER>")
    else:
        print("  Camera: none found. Check the spec for how the Justice D is connected.")
    if fcs:
        port, baud, version, autopilot = fcs[0]
        print(f"  Flight controller: --connect {port}  (baud {baud})")
        if "ARDUPILOT" not in autopilot:
            print(f"  WARNING: it runs {autopilot}. Team decision: ArduPilot >= 4.5 only -> ask the")
            print("  organisers for the event's ArduPilot build for the APB flight controller.")
        if baud != 921600:
            print(f"  NOTE: our link opens serial ports at 921600 baud but the autopilot answered at"
                  f" {baud}: set that serial port's SERIALx_BAUD = 921 on the autopilot.")
        print(f"  Firmware {version}: GPS param name is GPS_TYPE up to 4.5, GPS1_TYPE from 4.6"
              " (ardupilot/vision_hold.param section 7).")
        print(f"  Next: venv/bin/python -m link.check_link --connect {port}")
    elif not args.no_serial:
        print("  Flight controller: not found. On the APB: rerun with --route-fc-uart (FC link is")
        print("  /dev/ttymxc2 behind a switch), and check the autopilot's SERIALx_PROTOCOL = 2 (MAVLink 2).")
    print("  Benchmark this computer: venv/bin/python -m compute.benchmark")


if __name__ == "__main__":
    main()
