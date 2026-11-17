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
#   At the end it prints the exact --camera and --connect values to use with
#   vision_hold.py, check_link.py and load_params.py. Send the output to the team.
#   It only LISTENS on serial ports: it never sends commands that change anything.
#
# HOW IT WORKS (technical)
#   Linux /proc and /sys for CPU/RAM, /etc/os-release, glob /dev/video* and opens each
#   with cv2.CAP_V4L2, glob /dev/tty{mxc,S,AMA,USB,ACM}* and tries pymavlink
#   wait_heartbeat at 921600/115200/57600/1500000 baud. ArduPilot version from the
#   AUTOPILOT_VERSION message (requested with MAV_CMD_REQUEST_MESSAGE, read-only).
#   On Windows/macOS it runs the parts that make sense (for testing on a laptop).
#
# HOW TO RUN (from the repo root, on the APB)
#   python3 -m tools.discover_hardware              # everything (~1-2 min)
#   python3 -m tools.discover_hardware --no-serial  # skip the serial-port scan
#
# REFERENCES
#   - Linux V4L2 video devices: https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html
#   - MAVLink HEARTBEAT / AUTOPILOT_VERSION: https://mavlink.io/en/messages/common.html
#   - NXP i.MX 8M Plus (the APB's likely onboard computer): https://www.nxp.com/products/i.MX8MPLUS
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
SERIAL_PATTERNS = ("/dev/ttymxc*", "/dev/ttyS*", "/dev/ttyAMA*", "/dev/ttyUSB*", "/dev/ttyACM*",
                   "/dev/serial/by-id/*")


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
        name = str(dev).replace("/dev/", "")
        path = os.path.join(save_dir, f"hw_camera_{name}.jpg")
        cv2.imwrite(path, frame)
        print(f"  {dev}: {w}x{h}, ~{n} fps measured (driver says {cap.get(cv2.CAP_PROP_FPS):.0f}), "
              f"brightness {frame.mean():.0f}/255 -> sample saved {path}")
        found.append(dev)
        cap.release()
    if not found:
        print("  No camera could be opened with OpenCV.")
    return found


def serial_ports(timeout_s=2.5):
    section("Serial ports / flight controller")
    try:
        os.environ["MAVLINK20"] = "1"
        from pymavlink import mavutil
    except ImportError:
        print("  pymavlink missing - can't look for the flight controller")
        return []
    ports = sorted({p for pat in SERIAL_PATTERNS for p in glob.glob(pat)})
    if not ports:
        print("  No serial ports found (normal on Windows: use COMx with check_link).")
        return []
    hits = []
    for port in ports:
        for baud in BAUDS:
            try:
                m = mavutil.mavlink_connection(port, baud=baud)
            except Exception as e:                      # busy, no permission, not a tty...
                print(f"  {port}: can't open ({e.__class__.__name__}: {e})")
                break
            hb = m.wait_heartbeat(timeout=timeout_s)
            if hb is None:
                m.close()
                continue
            autopilot = mavutil.mavlink.enums["MAV_AUTOPILOT"][hb.autopilot].name
            version = "?"
            m.mav.command_long_send(m.target_system, m.target_component,
                                    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 148, 0, 0, 0, 0, 0, 0)
            av = m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=2)
            if av:
                v = av.flight_sw_version
                version = f"{v >> 24 & 255}.{v >> 16 & 255}.{v >> 8 & 255}"
            print(f"  {port} @ {baud}: FLIGHT CONTROLLER FOUND  {autopilot}, firmware {version}, "
                  f"mode {m.flightmode}")
            hits.append((port, baud, version))
            m.close()
            break
        else:
            print(f"  {port}: no MAVLink heartbeat at {', '.join(map(str, BAUDS))}")
    return hits


def main():
    ap = argparse.ArgumentParser(description="Find out what hardware this computer has")
    ap.add_argument("--no-serial", action="store_true")
    ap.add_argument("--no-camera", action="store_true")
    args = ap.parse_args()

    computer()
    libraries()
    cams = [] if args.no_camera else cameras()
    fcs = [] if args.no_serial else serial_ports()

    section("What to use")
    if cams:
        print(f"  Cameras found: {', '.join(map(str, cams))}. Open the saved logs/hw_camera_*.jpg:")
        print("  the one showing the GROUND is the Justice D -> vision_hold.py --camera <that one>")
    else:
        print("  Camera: none found. Check the spec for how the Justice D is connected.")
    if fcs:
        port, baud, version = fcs[0]
        print(f"  Flight controller: --connect {port}  (baud {baud})")
        if baud != 921600:
            print(f"  NOTE: link/drone_link.py BAUD is 921600 but the autopilot answered at {baud}:"
                  f" set BAUD = {baud} there (or SERIALx_BAUD on the autopilot).")
        print(f"  ArduPilot {version}: GPS param name is GPS_TYPE up to 4.5, GPS1_TYPE from 4.6"
              " (ardupilot/vision_hold.param section 7).")
        print(f"  Next: python3 -m link.check_link --connect {port}")
    elif not args.no_serial:
        print("  Flight controller: not found on any serial port. Check the spec for the APB's")
        print("  internal UART name, and that the autopilot's SERIALx_PROTOCOL = 2 (MAVLink 2).")
    print("  Benchmark this computer: python3 -m compute.benchmark")


if __name__ == "__main__":
    main()
