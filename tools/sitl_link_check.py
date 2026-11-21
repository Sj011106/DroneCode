"""Link + handover check against a running ArduPilot simulator (no cv2).

Uses the real DroneLink and the real handover.py, with the same handover block as
vision_hold.py. The script plays the pilot (mode changes) and the camera (quality 80/0).

    $env:PYTHONPATH = "."
    python tools/sitl_link_check.py [--connect tcp:127.0.0.1:5762]
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import config
from handover import Handover, REANCHOR
from link import drone_link
from link.drone_link import DroneLink

LOG = os.path.join("logs", "sitl_link_check.log")
logf = None
results = []


def say(msg):
    line = f"{time.strftime('%H:%M:%S')}  {msg}"
    print(line, flush=True)
    if logf:
        logf.write(line + "\n")
        logf.flush()


def verdict(n, title, ok, detail=""):
    results.append(ok)
    say(f"{n}. {'PASS' if ok else 'FAIL'} - {title} {detail}")


def make_handover():
    return Handover(failsafe_after_s=getattr(config, "FAILSAFE_AFTER_S", 3.0),
                    failsafe_mode=getattr(config, "FAILSAFE_ACTION", "LAND"))


def pilot_set(link, mode):
    with link.send_lock:
        link.conn.set_mode(mode)


def tick(link, handover, quality, sends, stats):
    """One loop iteration = the handover block of vision_hold.py. Records its duration."""
    t0 = time.monotonic()
    now = t0
    for action in handover.update(now, link.conn.flightmode, quality,
                                  mode_fresh=link.mode_age() < 1.5):
        if action == REANCHOR:
            say("  REANCHOR")
        elif action[0] == "GAVE_UP":
            say("  GAVE_UP")
        else:
            say(f"  SET_MODE {action[1]} (flightmode={link.conn.flightmode})")
            sends.append(action[1])
            with link.send_lock:
                link.conn.set_mode(action[1])
    stats["max_iter"] = max(stats["max_iter"], time.monotonic() - t0)


def run_loop(link, handover, seconds, quality, sends, stats, at=None, stop_when=None):
    """30 Hz loop. `at` = list of (seconds_from_start, callable) one-shot events."""
    at = sorted(at or [], key=lambda e: e[0])
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        el = time.monotonic() - start
        while at and at[0][0] <= el:
            at.pop(0)[1]()
        tick(link, handover, quality, sends, stats)
        if stop_when and stop_when():
            break
        time.sleep(1 / 30)


def to_mode(link, mode, wait=4.0):
    end = time.monotonic() + wait
    while time.monotonic() < end:
        if link.conn.flightmode == mode:
            return True
        pilot_set(link, mode)
        time.sleep(0.5)
    return link.conn.flightmode == mode


def main():
    global logf
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    args = ap.parse_args()
    os.makedirs("logs", exist_ok=True)
    logf = open(LOG, "a", encoding="utf-8")
    say(f"=== sitl_link_check start {args.connect} ===")

    link = DroneLink(args.connect)
    link.start()
    time.sleep(1.0)
    say(f"connected, flightmode={link.conn.flightmode}, mode_age={link.mode_age():.2f}")

    # ---- 1. mode_age stays fresh while modes change for 15 s
    say("1. switching STABILIZE/LOITER/ALT_HOLD for 15 s")
    ages, order, start, i = [], ["STABILIZE", "LOITER", "ALT_HOLD"], time.monotonic(), 0
    while time.monotonic() - start < 15:
        pilot_set(link, order[i % 3])
        i += 1
        for _ in range(15):                      # ~1.5 s, sampling every 0.1 s
            ages.append(link.mode_age())
            time.sleep(0.1)
    mx = max(ages)
    verdict(1, "mode_age < 1.5 s and never inf", mx < 1.5 and mx != float("inf"), f"(max {mx:.2f} s)")

    # ---- 2. HEARTBEAT filter: feed made-up HEARTBEAT messages through the REAL filter (DroneLink.is_autopilot_heartbeat)
    from pymavlink import mavutil
    M = mavutil.mavlink
    sysid = link.conn.target_system

    class _Hb:
        def __init__(self, s, t, a):
            self._s, self.type, self.autopilot = s, t, a

        def get_srcSystem(self):
            return self._s

    cases = [(_Hb(sysid, M.MAV_TYPE_QUADROTOR, M.MAV_AUTOPILOT_ARDUPILOTMEGA), True),       # the flight controller
             (_Hb(sysid, M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID), False),                   # a ground station
             (_Hb(sysid, M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID), False),    # our own heartbeat
             (_Hb(sysid + 1, M.MAV_TYPE_QUADROTOR, M.MAV_AUTOPILOT_ARDUPILOTMEGA), False)]   # another vehicle
    got = [link.is_autopilot_heartbeat(m) for m, _ in cases]
    verdict(2, "only the autopilot's heartbeat can refresh mode_age", got == [e for _, e in cases], f"(got {got})")

    # ---- 3. LOITER + quality 0 -> one LAND, mode LAND, no loop freeze
    say("3. LOITER, camera covered (quality 0) up to 12 s")
    if not to_mode(link, "STABILIZE") or not to_mode(link, "LOITER"):
        verdict(3, "enter LOITER", False, "(simulator refused LOITER - no position yet?)")
    else:
        h, sends, stats = make_handover(), [], {"max_iter": 0.0}
        run_loop(link, h, 12, 0, sends, stats, stop_when=lambda: link.conn.flightmode == "LAND")
        run_loop(link, h, 3, 0, sends, stats)    # nothing more may be sent after LAND
        frozen = stats["max_iter"] > 1.0
        ok = sends == ["LAND"] and link.conn.flightmode == "LAND" and not frozen
        verdict(3, "exactly one LAND, mode is LAND, no FREEZE", ok,
                f"(sends={sends}, mode={link.conn.flightmode}, slowest iteration "
                f"{stats['max_iter'] * 1000:.1f} ms{', FREEZE' if frozen else ''})")

    # ---- 4. pilot switches to ALT_HOLD 1 s after loss -> no LAND
    say("4. LOITER, camera covered, pilot -> ALT_HOLD after 1 s")
    if not to_mode(link, "STABILIZE") or not to_mode(link, "LOITER"):
        verdict(4, "enter LOITER", False, "(simulator refused LOITER)")
    else:
        h, sends, stats = make_handover(), [], {"max_iter": 0.0}
        run_loop(link, h, 8, 0, sends, stats, at=[(1.0, lambda: pilot_set(link, "ALT_HOLD"))])
        ok = sends == [] and link.conn.flightmode == "ALT_HOLD"
        verdict(4, "no LAND after pilot takeover", ok, f"(sends={sends}, mode={link.conn.flightmode})")

    to_mode(link, "STABILIZE")
    link.running = False
    say(f"RESULT: {sum(results)}/4")
    logf.close()
    sys.exit(0 if len(results) == 4 and all(results) else 1)


if __name__ == "__main__":
    main()
