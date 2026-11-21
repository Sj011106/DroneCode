# =====================================================================================
# tools/blur_check.py  -  WHY is the down picture blurry? (Justice D, /dev/video2)   STAGE 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   "Blurry" can be four different things, each with a different fix:
#     motion     the drone turned while the picture was taken (worse with faster turns)
#     vibration  props shake the camera (worse with props on, at the same turn rate)
#     focus      fixed-focus lens: soft at 1 m on the bench, sharp at height
#     detail     NOT blur: we shrink 1920 -> 640 wide. Above ~20 m the ground's texture
#                is smaller than one of our pixels and the picture LOOKS blurry
#   This tool records the camera at FULL resolution for a few seconds, reads the APB's
#   gyro at the same time, and prints a sharpness number per turn-rate band, for the
#   full-resolution centre AND for the 640-wide copy the tracker uses. It also saves
#   12 full-resolution PNGs (lossless) and the matching 640 copies, so you can SEE it.
#   It also prints the camera latency: how old a picture is when we receive it.
#   It changes nothing on the board. Stop vision_hold first (it holds the camera).
#
# HOW TO RUN (from the repo root, on the APB; same floor every time)
#   python -m tools.blur_check --tag still              # on a stand, props off, still
#   python -m tools.blur_check --tag wobble             # rock it by hand, slow then fast
#   python -m tools.blur_check --tag buf1 --buffers 1   # same as "still" with a 1-picture queue
#   python -m tools.blur_check --tag buf4 --buffers 4   # ... and with 4 (CAMERA_BUFFERS_WITH_THREAD)
#   Results: /data/blur_<tag>/  (frames.csv + PNGs). docs/CAMERAS.md says how to read them.
#
# REFERENCES
#   - Pertuz, Puig, Garcia, "Analysis of focus measure operators for shape-from-focus",
#     Pattern Recognition 2013 (variance of the Laplacian as a sharpness measure).
# =====================================================================================
import argparse
import glob
import os
import threading
import time

import cv2
import numpy as np

from vision.sharpness import sharpness

IIO_ROOT = "/sys/bus/iio/devices"


def find_iio(name, root=IIO_ROOT):
    for d in sorted(glob.glob(os.path.join(root, "iio:device*"))):
        try:
            with open(os.path.join(d, "name")) as f:
                if name in f.read():
                    return d
        except OSError:
            pass
    return None


class GyroLog:
    """Turn rate magnitude (deg/s) on a side thread, ~150 samples a second."""

    def __init__(self):
        self.dir, self.samples, self.alive = find_iio("icm426"), [], True
        if self.dir:
            threading.Thread(target=self._loop, daemon=True).start()

    def _read(self, name):
        with open(os.path.join(self.dir, name)) as f:
            return float(f.read())

    def _loop(self):
        scale = self._read("in_anglvel_scale")
        while self.alive:
            try:
                r = [self._read(f"in_anglvel_{a}_raw") for a in "xyz"]
                self.samples.append((time.monotonic(), float(np.degrees(np.linalg.norm(r) * scale))))
            except (OSError, ValueError):
                pass
            time.sleep(0.005)

    def peak_since(self, t):
        rates = [r for ts, r in self.samples[-60:] if ts >= t]
        return max(rates) if rates else float("nan")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Motion / vibration / focus / detail: what blurs the down camera?")
    ap.add_argument("--device", default="/dev/video2")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--tag", default="test")
    ap.add_argument("--buffers", type=int, default=4, help="driver queue length to test (1 vs 4)")
    ap.add_argument("--out", default="/data" if os.path.isdir("/data") else ".")
    args = ap.parse_args(argv)
    W, H = 1920, 1080
    out = os.path.join(args.out, f"blur_{args.tag}")
    os.makedirs(out, exist_ok=True)

    gyro = GyroLog()
    if not gyro.dir:
        print("No gyro found: sharpness only.")
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, args.buffers)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open {args.device} (is vision_hold running? systemctl stop vision_hold)")
    t_end = time.monotonic() + 1.5
    while time.monotonic() < t_end:                     # the first pictures are dark (config.CAMERA_WARMUP_S)
        cap.read()

    rows, saved, n = [], 0, 0
    t_prev = time.monotonic()
    t_end = t_prev + args.seconds
    while time.monotonic() < t_end:
        ok, f = cap.read()
        t_app = time.monotonic()
        if not ok or f is None:
            continue
        t_drv = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
        buf = f.reshape(-1)
        if buf.size != W * H * 2:
            print(f"Unexpected picture {f.shape}: not 1920x1080 YUYV raw. Stopping.")
            break
        g = np.ascontiguousarray(buf.reshape(H, W * 2)[:, 0::2])          # the Y of YUYV = grey
        crop = np.ascontiguousarray(g[H // 2 - 320:H // 2 + 320, W // 2 - 320:W // 2 + 320])
        small = cv2.resize(g, (640, 360), interpolation=cv2.INTER_AREA)   # what the tracker sees
        rate = gyro.peak_since(t_app - 0.03) if gyro.dir else float("nan")
        rows.append((t_app, t_drv, sharpness(crop), sharpness(small), rate, float(g.mean())))
        n += 1
        if n % 20 == 0 and saved < 12:
            cv2.imwrite(os.path.join(out, f"full_{n:04d}.png"), g, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            cv2.imwrite(os.path.join(out, f"small_{n:04d}.png"), small)
            saved += 1
    gyro.alive = False
    cap.release()
    if len(rows) < 3:
        raise SystemExit("Too few pictures.")

    a = np.array(rows)
    print(f"pictures {len(a)}   our rate {1 / np.median(np.diff(a[:, 0])):.1f}/s   "
          f"driver rate {1 / max(1e-6, np.median(np.diff(a[:, 1]))):.1f}/s   queue {args.buffers}")
    age = a[:, 0] - a[:, 1]
    if np.all((age >= 0) & (age < 0.5)):
        print(f"latency (driver stamp -> our hands): median {1000 * np.median(age):.0f} ms, "
              f"max {1000 * age.max():.0f} ms   <- this IS the camera latency")
    else:
        print("driver timestamps are not on the monotonic clock: latency not measurable this way")
    print(f"brightness {a[:, 5].mean():.0f}/255   ALL: sharpness full-res centre {np.median(a[:, 2]):.0f}   "
          f"640 copy {np.median(a[:, 3]):.0f}")
    for lo, hi in ((0, 10), (10, 30), (30, 100), (100, 1e9)):
        m = (a[:, 4] >= lo) & (a[:, 4] < hi)
        if m.any():
            print(f"  turning {lo:>3}-{'up' if hi >= 1e9 else int(hi):>3} deg/s: {int(m.sum()):4d} pictures   "
                  f"full-res centre {np.median(a[m, 2]):8.0f}   640 copy {np.median(a[m, 3]):8.0f}")
    np.savetxt(os.path.join(out, "frames.csv"), a, delimiter=",", comments="",
               header="t_app,t_driver,sharp_full_centre,sharp_640,gyro_dps,brightness")
    print(f"Saved {saved} full-res + 640 PNG pairs and frames.csv in {out}")


if __name__ == "__main__":
    main()
