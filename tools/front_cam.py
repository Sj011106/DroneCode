# =====================================================================================
# tools/front_cam.py  -  check (and optionally show) the forward camera, Justice A   STAGE 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Run this on the APB before anything else with the front camera. It opens /dev/video3
#   the right way (vision/front_camera.py), counts good / partial / no-signal pictures
#   for a few seconds, and saves 4 test pictures in /data:
#       front_<i>_frame.png   the full picture (may look "combed" when things move)
#       front_<i>_field.png   one half-picture, stretched to square pixels: use this one
#   It changes nothing on the board.
#   WARNING: the APB manual (section 4.5) says opening /dev/video3 can freeze the forward camera's decoder
#   (the PILOT's picture). Ask the organisers before running this on the aircraft. See docs/CAMERAS.md.
#
#   --serve shows it in a browser on port 8089, 2 pictures a second (the down camera's
#   live view stays on 8088). For the bench / operator view only.
#
# HOW TO RUN (from the repo root, on the APB)
#   python -m tools.front_cam                       # 10 s check of /dev/video3 (UYVY)
#   python -m tools.front_cam --fmt YUYV            # if the first run counts many "bad" pictures
#   python -m tools.front_cam --serve               # operator view: http://192.168.75.1:8089
#
# WHAT THE RESULT MEANS (docs/CAMERAS.md has the full table)
#   good ~25/s (~50/s if the driver gives single fields)  -> it works
#   everything "no-signal"                                -> no video reaches the decoder:
#                                                            hardware, ask the organisers
#   many "bad"                                            -> try --fmt YUYV
#   "cannot open"                                         -> fuser -v /dev/video3 (busy?)
# =====================================================================================
import argparse
import os
import time

import cv2

import config
from vision.front_camera import FrontCamera, comb_score


def check(cam, seconds, out_dir):
    t0, last, saved = time.monotonic(), 0, 0
    while time.monotonic() - t0 < seconds:
        item = cam.latest()
        if item and item[3] != last:
            field, frame, t, last = item
            if last % 25 == 0 and saved < 4:
                cv2.imwrite(os.path.join(out_dir, f"front_{saved}_frame.png"), frame)
                cv2.imwrite(os.path.join(out_dir, f"front_{saved}_field.png"),
                            cv2.resize(field, (768, 576), interpolation=cv2.INTER_LINEAR))
                age = "?" if cam.age_s is None else f"{1000 * cam.age_s:.0f} ms"
                print(f"  #{last}: mean {frame.mean():.0f}  std {frame.std():.0f}  "
                      f"comb {comb_score(frame):.2f}  age when received {age}")
                saved += 1
        time.sleep(0.005)
    el = time.monotonic() - t0
    print(f"good {cam.good} ({cam.good / el:.1f}/s)   bad/partial {cam.bad}   no-signal {cam.no_signal}")
    if cam.good == 0 and cam.no_signal > 0:
        print("VERDICT: the decoder sees no video. Camera, cable or power: ask the organisers.")
    elif cam.good == 0:
        print("VERDICT: no usable picture. Try --fmt YUYV, then check 'v4l2-ctl -d /dev/video3 --list-formats-ext'.")
    else:
        print(f"VERDICT: working. Pictures saved in {out_dir} (look at the *_field.png ones).")


def serve(cam, port, fps):
    from tools.live_view import LiveView
    live = LiveView(port=port, enabled=True)
    print(f"Front camera view on port {port} ({fps} pictures/s). Ctrl+C to stop.")
    last = 0
    try:
        while True:
            item = cam.latest()
            if item and item[3] != last:
                field, _, _, last = item
                live.frame(cv2.resize(field, (384, 288)))
                live.state(ready=cam.good > 0, status="FRONT CAMERA",
                           reason=f"good {cam.good}  bad {cam.bad}  no-signal {cam.no_signal}")
            time.sleep(1.0 / fps)
    except KeyboardInterrupt:
        pass
    finally:
        live.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description="Check / show the forward camera (Justice A)")
    ap.add_argument("--device", default=config.FRONT_CAMERA_DEVICE)
    ap.add_argument("--fmt", default=config.FRONT_CAMERA_FORMAT, help="UYVY (ADV7282 default) or YUYV")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--out", default="/data" if os.path.isdir("/data") else ".")
    ap.add_argument("--serve", action="store_true", help="show it in a browser instead of a check")
    ap.add_argument("--port", type=int, default=config.FRONT_VIEW_PORT)
    ap.add_argument("--fps", type=float, default=2.0)
    args = ap.parse_args(argv)
    cam = FrontCamera(args.device, args.fmt)
    print(f"{args.device}: {cam.fmt} {cam.w}x{cam.h}   grey = byte {cam.y_at} of each pair   "
          f"single fields from the driver: {cam.fields_only}")
    try:
        if args.serve:
            serve(cam, args.port, args.fps)
        else:
            check(cam, args.seconds, args.out)
    finally:
        cam.close()


if __name__ == "__main__":
    main()
