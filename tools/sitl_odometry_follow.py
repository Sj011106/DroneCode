# tools/sitl_odometry_follow.py  (Windows-only: uses msvcrt for keys)
# Like fake_vision.py (SIMSTATE true position -> N/E metres from the first SIMSTATE, 2 cm noise,
# 20 Hz) but sends it through the real DroneLink.send_odometry, exactly as sitl_odometry_check.py.
#
#   python -m tools.sitl_odometry_follow [--connect tcp:127.0.0.1:5763]   (run from the repo root)
#
# Keys (click the terminal first):
#   v = EKF source VISION     g = EKF source GPS
#   c = toggle "camera covered" (zeros, quality -1, no metres_per_px: what vision_hold sends at quality <= 0)
#   x = toggle "stop sending" (no odometry at all)     q = quit

import argparse
import math
import msvcrt
import random
import time

import config  # noqa: F401  (odometry_sigma_px reads config)
from link.drone_link import DroneLink, SRC_GPS, SRC_VISION
from pymavlink import mavutil

NOISE_M = 0.02
SEND_HZ = 20
QUALITY = 80
HEIGHT_M, FOCAL_PX = 30.0, 416.0
RESET_COUNTER = 1
BLOCKED_QUALITY = -1     # vision_hold.py sets quality = -1 for its "can't use this frame" cases


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--connect", default="tcp:127.0.0.1:5763")
    args = ap.parse_args()

    link = DroneLink(args.connect)
    link.start()

    # DroneLink's reader consumes every message; tap recv_match to also see SIMSTATE,
    # local N/E and STATUSTEXT (DroneLink itself is unchanged).
    st = {"sim": None, "ne": None, "texts": []}
    real_recv = link.conn.recv_match

    def tapped(*a, **kw):
        msg = real_recv(*a, **kw)
        if msg is not None:
            kind = msg.get_type()
            if kind == "SIMSTATE":
                st["sim"] = msg
            elif kind == "LOCAL_POSITION_NED":
                st["ne"] = (msg.x, msg.y)
            elif kind == "STATUSTEXT":
                st["texts"].append(msg.text)
        return msg
    link.conn.recv_match = tapped

    with link.send_lock:     # same request as fake_vision.py (SIMSTATE comes with the full stream set)
        link.conn.mav.request_data_stream_send(link.conn.target_system, link.conn.target_component,
                                               mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

    print("Waiting for SIMSTATE ...")
    t_wait = time.time()
    while st["sim"] is None:
        if time.time() - t_wait > 10:
            print("No SIMSTATE received. Is the simulator running?")
            link.running = False
            return
        time.sleep(0.05)
    lat0, lon0 = st["sim"].lat / 1e7, st["sim"].lng / 1e7
    m_lat = 111320.0
    m_lon = 111320.0 * math.cos(math.radians(lat0))
    print(f"Origin at lat {lat0:.7f}, lon {lon0:.7f}")
    print("Keys: v = vision, g = GPS, c = cover camera, x = stop sending, q = quit\n")

    covered = stopped = False
    max_err_vision = 0.0
    shown_texts = 0
    sent_n = sent_e = 0.0
    last_hb = last_origin = last_send = last_print = 0.0

    try:
        while True:
            now = time.time()
            if now - last_hb > 1.0:
                link.send_heartbeat()
                last_hb = now
            if not link.origin_set and now - last_origin > 2.0:
                link.set_origin()
                last_origin = now

            if now - last_send >= 1.0 / SEND_HZ:
                last_send = now
                sim = st["sim"]
                true_n = (sim.lat / 1e7 - lat0) * m_lat
                true_e = (sim.lng / 1e7 - lon0) * m_lon
                if covered:
                    sent_n, sent_e = 0.0, 0.0
                    quality, mpp = BLOCKED_QUALITY, None
                else:
                    sent_n = true_n + random.gauss(0, NOISE_M)
                    sent_e = true_e + random.gauss(0, NOISE_M)
                    quality, mpp = QUALITY, HEIGHT_M / FOCAL_PX
                if not stopped:
                    att = link.attitude_at(now) or (0.0, 0.0, 0.0)
                    link.send_odometry(now, sent_n, sent_e, 0.0, att, (0, 0, 0), quality,
                                       RESET_COUNTER, metres_per_px=mpp)

            if now - last_print >= 1.0:
                last_print = now
                ne = st["ne"]
                if ne:
                    err = math.hypot(sent_n - ne[0], sent_e - ne[1])
                    if link.acked_source == SRC_VISION and not covered and not stopped:
                        max_err_vision = max(max_err_vision, err)
                    drone = f"drone N={ne[0]:7.2f} E={ne[1]:7.2f}  err={err:5.2f} m"
                else:
                    drone = "drone: (no LOCAL_POSITION_NED yet)"
                flags = ("COVERED " if covered else "") + ("NOT-SENDING" if stopped else "")
                print(f"sent N={sent_n:7.2f} E={sent_e:7.2f} | {drone} | mode={link.conn.flightmode} "
                      f"| src={link.acked_source} {flags}")
            while shown_texts < len(st["texts"]):
                print(f"[drone] {st['texts'][shown_texts]}")
                shown_texts += 1

            if msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key == "v":
                    link.set_source(SRC_VISION)
                elif key == "g":
                    link.set_source(SRC_GPS)
                elif key == "c":
                    covered = not covered
                    print(f">>> camera covered: {covered}")
                elif key == "x":
                    stopped = not stopped
                    print(f">>> sending stopped: {stopped}")
                elif key == "q":
                    break
            time.sleep(0.005)
    finally:
        link.running = False
        print(f"\nMax error while source was VISION (camera uncovered, sending): {max_err_vision:.3f} m")


if __name__ == "__main__":
    main()
