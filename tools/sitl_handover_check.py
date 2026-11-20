"""Camera-free check of handover.py against a running ArduPilot simulator.

The script plays BOTH roles: the "pilot" (it changes flight modes) and the "camera"
(it feeds quality 50 = tracking, 0 = covered). Handover decisions come from the real
handover.py; SET_MODE actions are really sent to the simulator. No OpenCV needed.

Run from the repo root (simulator running, disarmed):
    $env:PYTHONPATH = "."
    python tools/sitl_handover_check.py            # default tcp:127.0.0.1:5762
    python tools/sitl_handover_check.py --connect tcp:127.0.0.1:5763

Writes logs/sitl_handover_check.log (keep it as evidence).
"""
import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pymavlink import mavutil

from handover import Handover, REANCHOR

LOG_PATH = os.path.join("logs", "sitl_handover_check.log")
results = []
logf = None


def say(msg):
    line = f"{time.strftime('%H:%M:%S')}  {msg}"
    print(line, flush=True)
    if logf:
        logf.write(line + "\n")
        logf.flush()


class Sim:
    def __init__(self, conn):
        self.m = mavutil.mavlink_connection(conn)
        self.m.wait_heartbeat(timeout=15)
        self.modes = self.m.mode_mapping()
        self.mode = None
        self.mode_t = 0.0
        self.h = Handover()
        self.quality = 50
        self.actions = []          # every action handover returned, in order
        self.pump(0.5)

    def pump(self, dur):
        """Read autopilot heartbeats for dur seconds."""
        end = time.monotonic() + dur
        while time.monotonic() < end:
            msg = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=0.1)
            if msg and msg.type != mavutil.mavlink.MAV_TYPE_GCS and msg.autopilot != 8:
                name = {v: k for k, v in self.modes.items()}.get(msg.custom_mode)
                self.mode, self.mode_t = name, time.monotonic()

    def pilot_sets(self, name):
        self.m.set_mode(self.modes[name])
        say(f"  pilot -> {name}")

    def run(self, seconds, quality=None, until_mode=None):
        """Run the handover loop at 10 Hz. Returns actions produced in this window."""
        if quality is not None:
            self.quality = quality
        got, end = [], time.monotonic() + seconds
        while time.monotonic() < end:
            self.pump(0.1)
            now = time.monotonic()
            fresh = (now - self.mode_t) < 2.0
            for a in self.h.update(now, self.mode, self.quality, mode_fresh=fresh):
                got.append(a)
                self.actions.append(a)
                say(f"  handover action: {a}   (mode={self.mode}, q={self.quality})")
                if isinstance(a, tuple) and a[0] == "SET_MODE":
                    self.m.set_mode(self.modes[a[1]])
            if until_mode and self.mode == until_mode:
                break
        return got

    def get_into(self, name, tries=4):
        for _ in range(tries):
            self.pilot_sets(name)
            self.pump(1.0)
            if self.mode == name:
                return True
        return False


def check(n, title, ok, detail=""):
    results.append(ok)
    say(f"CHECK {n}: {'PASS' if ok else 'FAIL'} - {title}  {detail}")


def main():
    global logf
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762")
    args = ap.parse_args()
    os.makedirs("logs", exist_ok=True)
    logf = open(LOG_PATH, "a", encoding="utf-8")
    say(f"=== sitl_handover_check start, connecting {args.connect} ===")
    s = Sim(args.connect)
    say(f"connected, mode now {s.mode}")

    # Pick a hold mode the simulator will accept (LOITER needs a position estimate).
    if not s.get_into("STABILIZE"):
        say("ABORT: simulator will not enter STABILIZE")
        sys.exit(2)
    hold = None
    for cand in ("LOITER", "POSHOLD"):
        if s.get_into(cand, tries=3):
            hold = cand
            break
    if hold is None:
        say("ABORT: simulator refuses LOITER and POSHOLD (no position yet?). "
            "Wait a minute for GPS/EKF in Mission Planner and re-run.")
        sys.exit(2)
    say(f"hold mode that works here: {hold}")
    s.get_into("STABILIZE")
    s.h = Handover()  # fresh state
    s.actions.clear()

    # 1. Camera covered while the pilot is NOT in a hold mode: nothing happens.
    say("1. STABILIZE, camera covered 5 s")
    a = s.run(5, quality=0)
    check(1, "no actions outside a hold mode", a == [] and s.mode == "STABILIZE", f"actions={a} mode={s.mode}")

    # 2. Enter hold with a good camera: exactly one REANCHOR.
    say(f"2. pilot -> {hold}, camera good 3 s")
    s.pilot_sets(hold)
    a = s.run(3, quality=50)
    check(2, "REANCHOR on entering hold", a == [REANCHOR] and s.mode == hold, f"actions={a} mode={s.mode}")

    # 6. (done early, while holding) short loss recovers: no LAND.
    say("6. camera covered 1.5 s then uncovered")
    a = s.run(1.5, quality=0) + s.run(3, quality=50)
    check(6, "short loss does not LAND", ("SET_MODE", "LAND") not in a and s.mode == hold, f"actions={a} mode={s.mode}")

    # 3. Camera covered in hold: LAND sent exactly once, and the sim ends up in LAND.
    say("3. camera covered ~8 s in hold")
    a = s.run(8, quality=0, until_mode="LAND")
    a += s.run(3)  # make sure no repeat sends after LAND took effect
    lands = a.count(("SET_MODE", "LAND"))
    check(3, "LAND sent exactly once and mode is LAND", lands == 1 and s.mode == "LAND", f"LAND sends={lands} mode={s.mode}")

    # 5. Re-enter hold: REANCHOR again.
    say(f"5. pilot -> STABILIZE then {hold}, camera good")
    s.pilot_sets("STABILIZE")
    s.run(1.5, quality=50)
    s.pilot_sets(hold)
    a = s.run(3, quality=50)
    check(5, "REANCHOR again on re-entering hold", a == [REANCHOR] and s.mode == hold, f"actions={a} mode={s.mode}")

    # 4. Vision lost, then pilot takes over with ALT_HOLD before the failsafe: no LAND.
    say("4. camera covered 2 s, pilot switches to ALT_HOLD, camera stays covered 8 s")
    a = s.run(2, quality=0)
    s.pilot_sets("ALT_HOLD")
    a += s.run(8)
    check(4, "pilot takeover (ALT_HOLD) causes no failsafe", ("SET_MODE", "LAND") not in a and s.mode == "ALT_HOLD",
          f"actions={a} mode={s.mode}")

    s.get_into("STABILIZE")
    say(f"RESULT: {sum(results)}/{len(results)} checks passed")
    say("NOTE: this proves handover.py logic against the simulator only; vision_hold.py wiring "
        "(home save, camera) still needs a machine where OpenCV loads.")
    logf.close()
    sys.exit(0 if all(results) and len(results) == 6 else 1)


if __name__ == "__main__":
    main()
