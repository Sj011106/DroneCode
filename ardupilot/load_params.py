# =====================================================================================
# ardupilot/load_params.py  -  write vision_hold.param into ArduPilot, and check it
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Loads our ArduPilot settings file straight from the APB over the same cable the
#   vision program uses, so nobody needs a laptop with Mission Planner at the field.
#   It sets each value, reads it back to make sure the autopilot really took it, and
#   prints a table: OK / CHANGED / FAILED / NOT FOUND (= this firmware doesn't have it).
#   --verify only reads and compares, it never changes anything (safe any time).
#
# HOW IT WORKS (technical)
#   Parses NAME,VALUE lines ('#' = comment). PARAM_SET (MAVLink param protocol), then
#   waits for the PARAM_VALUE echo that link/drone_link.py's reader stores in
#   link.params. Values are sent as REAL32; ArduPilot converts to the param's real type.
#   No reply within the timeout = parameter not present in this firmware version. Newer
#   ArduPilot renamed a few (ANGLE_MAX -> ATC_ANGLE_MAX in degrees, LOIT_SPEED -> LOIT_SPEED_MS
#   in m/s, LOIT_ACC_MAX -> LOIT_ACC_MAX_M in m/s^2): when the old name is missing, the new
#   one is used with the value converted. Anything still NOT FOUND is a FAILURE (exit 1):
#   a missing safety/limit setting must never pass silently.
#   Safety: refuses to write while the motors are ARMED, prints every old value, and will
#   not overwrite an RC channel that already has a function (RCx_OPTION != 0, e.g. an
#   emergency stop) unless --force.
#
# HOW TO RUN (from the repo root, drone DISARMED)
#   python -m ardupilot.load_params --dry-run                     # just show what's in the file
#   python -m ardupilot.load_params --connect /dev/ttymxc2        # write + verify
#   python -m ardupilot.load_params --connect /dev/ttymxc2 --verify
#   Then reboot the flight controller if it says so.
#
# REFERENCES
#   - MAVLink parameter protocol: https://mavlink.io/en/services/parameter.html
#   - ArduPilot parameters: https://ardupilot.org/copter/docs/parameters.html
# =====================================================================================

import argparse
import os
import re
import sys
import time

import config

DEFAULT_FILE = os.path.join(os.path.dirname(__file__), "vision_hold.param")
NEEDS_REBOOT = ("AHRS_EKF_TYPE", "EK2_ENABLE", "EK3_ENABLE", "VISO_TYPE", "SERIAL", "GPS", "RNGFND")
RENAMED = {"ANGLE_MAX": ("ATC_ANGLE_MAX", 0.01),      # centidegrees -> degrees
           "LOIT_SPEED": ("LOIT_SPEED_MS", 0.01),     # cm/s -> m/s
           "LOIT_ACC_MAX": ("LOIT_ACC_MAX_M", 0.01)}  # cm/s^2 -> m/s^2


def parse_param_file(path=DEFAULT_FILE):
    """[(NAME, value), ...] in file order. Accepts 'NAME,VALUE' or 'NAME VALUE'."""
    params = []
    with open(path) as f:
        for n, line in enumerate(f, 1):
            line = line.split("#", 1)[0].strip()
            if not line:
                continue
            parts = line.replace(",", " ").split()
            if len(parts) != 2:
                raise ValueError(f"{path}:{n}: expected NAME,VALUE, got {line!r}")
            params.append((parts[0].upper(), float(parts[1])))
    return params


def _wait_for(link, name, timeout):
    t_end = time.time() + timeout
    while time.time() < t_end:
        if name in link.params:
            return link.params[name]
        time.sleep(0.02)
    return None


def read_param(link, name, timeout=1.5):
    link.params.pop(name, None)
    with link.send_lock:
        link.conn.mav.param_request_read_send(link.conn.target_system, link.conn.target_component,
                                              name.encode(), -1)
    return _wait_for(link, name, timeout)


def write_param(link, name, value, timeout=1.5):
    from pymavlink import mavutil
    link.params.pop(name, None)
    with link.send_lock:
        link.conn.mav.param_set_send(link.conn.target_system, link.conn.target_component,
                                     name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    return _wait_for(link, name, timeout)


def same(a, b):
    return a is not None and abs(a - b) < 1e-3 * max(1.0, abs(b))


def apply(link, params, verify_only=False, force=False):
    """Returns [(name, wanted, before, after, status)]. status: OK, CHANGED, WRONG (verify),
    FAILED, NOT FOUND, SKIPPED (would overwrite an RC channel function; use force)."""
    rows = []
    for name, want in params:
        before = read_param(link, name)
        if before is None and name in RENAMED:                 # newer firmware: renamed + new units
            new_name, k = RENAMED[name]
            before = read_param(link, new_name)
            if before is not None:
                name, want = new_name, want * k
        if before is None:
            rows.append((name, want, None, None, "NOT FOUND"))
        elif same(before, want):
            rows.append((name, want, before, before, "OK"))
        elif verify_only:
            rows.append((name, want, before, before, "WRONG"))
        elif re.fullmatch(r"RC\d+_OPTION", name) and before != 0 and not force:
            rows.append((name, want, before, before, "SKIPPED"))   # channel already has a job
        else:
            after = write_param(link, name, want)
            rows.append((name, want, before, after, "CHANGED" if same(after, want) else "FAILED"))
    return rows


def firmware(link, timeout=3.0):
    """(autopilot name, (major, minor, patch) or None), read-only: asks for AUTOPILOT_VERSION."""
    from pymavlink import mavutil
    hb = link.conn.messages.get("HEARTBEAT")
    name = mavutil.mavlink.enums["MAV_AUTOPILOT"][hb.autopilot].name if hb else "unknown"
    link.command(mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 148)   # 148 = AUTOPILOT_VERSION
    t_end = time.time() + timeout
    while time.time() < t_end:
        av = link.conn.messages.get("AUTOPILOT_VERSION")
        if av is not None:
            v = av.flight_sw_version
            return name, (v >> 24 & 255, v >> 16 & 255, v >> 8 & 255)
        time.sleep(0.05)
    return name, None


def print_table(rows):
    fmt = lambda v: "-" if v is None else f"{v:g}"
    print(f"{'PARAMETER':18s} {'WANT':>8s} {'WAS':>8s} {'NOW':>8s}  STATUS")
    for name, want, before, after, status in rows:
        print(f"{name:18s} {fmt(want):>8s} {fmt(before):>8s} {fmt(after):>8s}  {status}")


def main():
    ap = argparse.ArgumentParser(description="Load / verify ardupilot/vision_hold.param")
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    ap.add_argument("--file", default=DEFAULT_FILE)
    ap.add_argument("--verify", action="store_true", help="only compare, never write")
    ap.add_argument("--dry-run", action="store_true", help="only print the file, no connection")
    ap.add_argument("--force", action="store_true", help="also overwrite RC channels that have a function")
    args = ap.parse_args()

    params = parse_param_file(args.file)
    if args.dry_run:
        for name, value in params:
            print(f"{name:18s} {value:g}")
        print(f"{len(params)} parameters")
        return

    from link.drone_link import DroneLink
    link = DroneLink(args.connect)
    link.start()
    if not args.verify and link.conn.motors_armed():
        link.running = False
        raise SystemExit("Motors are ARMED: refusing to change parameters. Disarm first.")
    rows = apply(link, params, verify_only=args.verify, force=args.force)
    link.running = False
    print_table(rows)

    changed = [r[0] for r in rows if r[4] == "CHANGED"]
    bad = [r for r in rows if r[4] in ("FAILED", "WRONG", "SKIPPED")]
    missing = [r[0] for r in rows if r[4] == "NOT FOUND"]
    for r in rows:
        if r[4] == "SKIPPED":
            print(f"\n{r[0]} is {r[2]:g} (that channel already has a function): NOT changed. "
                  f"Check with the event crew, then rerun with --force if it is really free.")
    if missing:
        print(f"\nNOT FOUND on this firmware: {', '.join(missing)}. We need ArduPilot >= "
              f"{'.'.join(map(str, config.MIN_ARDUPILOT_VERSION))} (python -m link.check_link shows the version).")
    if any(c.startswith(NEEDS_REBOOT) for c in changed):
        print("\nREBOOT the flight controller: some changed settings only apply after a reboot.")
    if bad or missing:
        print(f"\n{len(bad) + len(missing)} parameter(s) not as wanted"
              + (" - run without --verify to fix" if args.verify and bad else ""))
        sys.exit(1)
    print("\nAll parameters as wanted.")


if __name__ == "__main__":
    main()
