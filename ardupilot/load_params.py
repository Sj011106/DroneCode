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
#   No reply within the timeout = parameter not present in this firmware version.
#
# HOW TO RUN (from the repo root, drone DISARMED)
#   python -m ardupilot.load_params --dry-run                     # just show what's in the file
#   python -m ardupilot.load_params --connect /dev/ttyAMA0        # write + verify
#   python -m ardupilot.load_params --connect /dev/ttyAMA0 --verify
#   Then reboot the flight controller if it says so.
#
# REFERENCES
#   - MAVLink parameter protocol: https://mavlink.io/en/services/parameter.html
#   - ArduPilot parameters: https://ardupilot.org/copter/docs/parameters.html
# =====================================================================================

import argparse
import os
import sys
import time

DEFAULT_FILE = os.path.join(os.path.dirname(__file__), "vision_hold.param")
NEEDS_REBOOT = ("AHRS_EKF_TYPE", "EK2_ENABLE", "EK3_ENABLE", "VISO_TYPE", "SERIAL", "GPS", "RNGFND")


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


def apply(link, params, verify_only=False):
    """Returns [(name, wanted, before, after, status)]."""
    rows = []
    for name, want in params:
        before = read_param(link, name)
        if before is None:
            rows.append((name, want, None, None, "NOT FOUND"))
        elif same(before, want):
            rows.append((name, want, before, before, "OK"))
        elif verify_only:
            rows.append((name, want, before, before, "WRONG"))
        else:
            after = write_param(link, name, want)
            rows.append((name, want, before, after, "CHANGED" if same(after, want) else "FAILED"))
    return rows


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
    rows = apply(link, params, verify_only=args.verify)
    link.running = False
    print_table(rows)

    changed = [r[0] for r in rows if r[4] == "CHANGED"]
    bad = [r for r in rows if r[4] in ("FAILED", "WRONG")]
    missing = [r[0] for r in rows if r[4] == "NOT FOUND"]
    if missing:
        print(f"\nNot on this firmware (check the name/version): {', '.join(missing)}")
    if any(c.startswith(NEEDS_REBOOT) for c in changed):
        print("\nREBOOT the flight controller: some changed settings only apply after a reboot.")
    if bad:
        print(f"\n{len(bad)} parameter(s) not as wanted" + (" - run without --verify to fix" if args.verify else ""))
        sys.exit(1)
    print("\nAll parameters as wanted." if not missing else "")


if __name__ == "__main__":
    main()
