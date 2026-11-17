# =====================================================================================
# link/check_link.py  -  pre-flight check of the autopilot connection
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Run this before every flight. It connects to the autopilot exactly like the real
#   program does and checks that everything vision_hold.py needs is actually arriving:
#   attitude (fast enough for the rolling-shutter fix), height, the remote-control
#   switch, the flight mode, and that EVERY setting in ardupilot/vision_hold.param is
#   what the autopilot really has. Ends with PASS or a list of what to fix. ~10 seconds.
#
# HOW IT WORKS (technical)
#   Uses link/drone_link.DroneLink (the same background reader). Rates are measured by
#   counting ATTITUDE samples for 3 s. Rolling-shutter correction interpolates attitude
#   between samples, so it wants >= 40 Hz (requested: 50 Hz via MAV_CMD_SET_MESSAGE_INTERVAL).
#
# HOW TO RUN (from the repo root)
#   python -m link.check_link                          # simulator
#   python -m link.check_link --connect /dev/ttyAMA0   # real drone (serial)
#
# REFERENCES
#   - MAVLink message interval: https://mavlink.io/en/mavgen_python/howto_requestmessages.html
#   - ArduPilot non-GPS setup: https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html
# =====================================================================================

import argparse
import math
import sys
import time

from ardupilot.load_params import apply, parse_param_file, print_table
from link.drone_link import DroneLink

MIN_ATTITUDE_HZ = 40


def main():
    ap = argparse.ArgumentParser(description="Pre-flight check of the MAVLink connection")
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    args = ap.parse_args()

    link = DroneLink(args.connect)
    link.start()
    link.check_params()                            # prints its own warnings
    n0, t0 = len(link.attitudes), time.time()
    time.sleep(3.0)
    samples = [a for a in list(link.attitudes) if a[0] >= t0]
    rate = len(samples) / (time.time() - t0)
    now = time.time()
    problems = []

    print(f"\nAttitude : {rate:5.1f} Hz", end="")
    if samples:
        _, r, p, y = samples[-1]
        print(f"   roll {math.degrees(r):+.1f}  pitch {math.degrees(p):+.1f}  yaw {math.degrees(y):+.1f} deg")
    else:
        print()
    if rate < MIN_ATTITUDE_HZ:
        problems.append(f"attitude only {rate:.0f} Hz (need >= {MIN_ATTITUDE_HZ} for rolling-shutter fix)")

    h = link.height(now)
    src = "rangefinder" if link.rangefinder and now - link.rangefinder[0] < 0.5 else "barometer"
    print(f"Height   : {h:.2f} m ({src})" if h is not None else "Height   : NONE")
    if h is None:
        problems.append("no height (no rangefinder and no LOCAL_POSITION_NED) - vision can't scale")

    print(f"Mode     : {link.conn.flightmode}")
    print(f"RC ch7   : {link.rc.get(7, 'none')}")
    print(f"Origin   : {'set' if link.origin_set else 'not set yet (vision_hold.py sets it)'}")
    print("\nArduPilot parameters vs ardupilot/vision_hold.param:")
    rows = apply(link, parse_param_file(), verify_only=True)
    print_table(rows)
    wrong = [r[0] for r in rows if r[4] == "WRONG"]
    if wrong:
        problems.append(f"parameters not as in vision_hold.param: {', '.join(wrong)} "
                        f"(fix: python -m ardupilot.load_params, then reboot)")

    link.running = False
    if problems:
        print("\nFIX BEFORE FLIGHT:\n  - " + "\n  - ".join(problems))
        sys.exit(1)
    print("\nPASS")


if __name__ == "__main__":
    main()
