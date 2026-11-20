# tools/sitl_odometry_check.py
# SITL check: does the autopilot accept our ODOMETRY (zero position, quality 80) from the
# real DroneLink without drifting or "stopped aiding"? Drone must be DISARMED.
#   python -m tools.sitl_odometry_check [--connect tcp:127.0.0.1:5763]   (run from the repo root)
# Appends everything it prints to logs/sitl_odometry_check.log. No cv2.

import argparse
import os
import sys
import threading
import time

import config  # noqa: F401  (odometry_sigma_px reads config)
from link.drone_link import DroneLink, SRC_GPS, SRC_VISION

SEND_HZ = 20
BASELINE_S, SEND_S = 10, 40
SWITCH_AT_S, MONITOR_S, RECOVER_S = 5, 25, 10
HEIGHT_M, FOCAL_PX = 30.0, 416.0
QUALITY = 80
DRIFT_LIMIT_M = 0.5
LOG_PATH = os.path.join("logs", "sitl_odometry_check.log")


class Tee:
    def __init__(self, path):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        self.f = open(path, "a", encoding="utf-8")
        self.f.write(f"\n===== run {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
        self.lock = threading.Lock()

    def __call__(self, text=""):
        with self.lock:
            print(text, flush=True)
            self.f.write(text + "\n")
            self.f.flush()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5763")
    args = ap.parse_args()
    out = Tee(LOG_PATH)

    # ---- 1. link + origin (same as vision_hold.py: heartbeat 1 Hz, set_origin every 2 s until confirmed)
    out(f"[1] connecting to {args.connect}")
    link = DroneLink(args.connect)
    link.start()

    # DroneLink's reader thread consumes every message and only keeps local z, so tap the
    # connection's recv_match to also see local N/E and STATUSTEXT (DroneLink itself unchanged).
    state = {"ne": None, "texts": []}
    real_recv = link.conn.recv_match

    def tapped(*a, **kw):
        msg = real_recv(*a, **kw)
        if msg is not None:
            kind = msg.get_type()
            if kind == "LOCAL_POSITION_NED":
                state["ne"] = (msg.x, msg.y)
            elif kind == "STATUSTEXT":
                state["texts"].append((time.time(), msg.text))
                out(f"    STATUSTEXT: {msg.text}")
        return msg
    link.conn.recv_match = tapped

    last_hb = last_origin = 0.0

    def housekeeping():
        nonlocal last_hb, last_origin
        now = time.time()
        if now - last_hb > 1.0:
            link.send_heartbeat()
            last_hb = now
        if not link.origin_set and now - last_origin > 2.0:
            link.set_origin()
            last_origin = now

    # ---- 2. baseline
    out(f"[2] baseline {BASELINE_S} s (origin set at start; local N/E once per second)")
    t0 = time.time()
    for s in range(BASELINE_S):
        while time.time() < t0 + s:
            housekeeping()
            time.sleep(0.02)
        out(f"    t={s:2d}s  local N/E = {state['ne']}  origin_set={link.origin_set}")
    base_ne = state["ne"] or (0.0, 0.0)

    # ---- 3/4/5. send ODOMETRY at 20 Hz, switch to vision, monitor, switch back
    out(f"[3] sending ODOMETRY {SEND_HZ} Hz for {SEND_S} s (north=east=down=0, quality {QUALITY}, "
        f"height {HEIGHT_M} m, focal {FOCAL_PX} px, reset_counter=1)")
    reset_counter = 1
    t_start = time.time()
    next_send, next_print = t_start, t_start + SWITCH_AT_S
    switched = back = False
    acked_at = None
    t_vis = t_gps = None
    ne_at_switch = None
    max_n = max_e = 0.0
    n_sent = 0
    recover_ok = None
    first_text_after_switch = None
    stopped_aiding = None

    while True:
        now = time.time()
        elapsed = now - t_start
        if elapsed >= SEND_S:
            break
        housekeeping()
        if now >= next_send:
            att = link.attitude_at(now) or (0.0, 0.0, 0.0)
            link.send_odometry(now, 0.0, 0.0, 0.0, att, (0, 0, 0), QUALITY, reset_counter,
                               metres_per_px=HEIGHT_M / FOCAL_PX)
            n_sent += 1
            next_send += 1.0 / SEND_HZ
        if not switched and elapsed >= SWITCH_AT_S:
            link.set_source(SRC_VISION)
            switched, t_vis = True, now
            ne_at_switch = state["ne"] or base_ne
            nt = len(state["texts"])
            out(f"[4] waiting for ack, then monitoring {MONITOR_S} s (N/E at switch = {ne_at_switch})")
        if switched and acked_at is None and link.acked_source == SRC_VISION:
            acked_at = now
            out(f"    acked_source == VISION after {now - t_vis:.2f} s")
        if switched and not back and t_vis and now - t_vis >= MONITOR_S:
            link.set_source(SRC_GPS)
            back, t_gps = True, now
            out(f"[5] back to GPS; watching {RECOVER_S} s for recovery")
        if switched and now >= next_print:
            ne = state["ne"]
            if ne:
                max_n = max(max_n, abs(ne[0] - ne_at_switch[0]))
                max_e = max(max_e, abs(ne[1] - ne_at_switch[1]))
            new = [t for ts, t in state["texts"] if ts >= t_vis]
            if new and first_text_after_switch is None:
                first_text_after_switch = new[0]
            if any("stopped aiding" in t.lower() for t in new) and stopped_aiding is None:
                stopped_aiding = next(t for t in new if "stopped aiding" in t.lower())
            phase = "GPS-recover" if back else "VISION"
            out(f"    t={elapsed:4.1f}s [{phase}] N/E = {ne}  max|dN|,|dE| so far = {max_n:.3f}, {max_e:.3f} m"
                f"  sent={n_sent}")
            next_print += 1.0
        time.sleep(0.002)

    # recovery verdict: estimate back near where it started (pre-switch) after GPS source restored
    ne = state["ne"]
    if ne and ne_at_switch:
        err = max(abs(ne[0] - ne_at_switch[0]), abs(ne[1] - ne_at_switch[1]))
        recover_ok = err < 1.0 and acked_at is not None
        out(f"[5] recovery: final N/E = {ne}, offset from pre-switch = {err:.3f} m -> "
            f"{'RECOVERED' if recover_ok else 'NOT recovered'}; acked_source now = {link.acked_source}")
    else:
        out("[5] recovery: no LOCAL_POSITION_NED seen -> cannot tell")

    link.running = False
    drift = max(max_n, max_e)
    ok = drift < DRIFT_LIMIT_M and stopped_aiding is None and acked_at is not None
    out("")
    if ok:
        out(f"PASS  (max drift {drift:.3f} m, no 'stopped aiding')")
    else:
        reasons = []
        if acked_at is None:
            reasons.append("vision source never acked")
        if drift >= DRIFT_LIMIT_M:
            reasons.append(f"drift {drift:.3f} m >= {DRIFT_LIMIT_M} m")
        if stopped_aiding:
            reasons.append(f"'stopped aiding': {stopped_aiding}")
        out(f"FAIL  ({'; '.join(reasons)})")
        out(f"first STATUSTEXT after switch: {first_text_after_switch!r}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
