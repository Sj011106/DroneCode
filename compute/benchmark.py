# =====================================================================================
# compute/benchmark.py  -  the numbers for the "onboard compute" slide   BRIEF ITEM 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Run this ON THE APB. It pushes video through the complete vision pipeline (the same
#   code that flies) as fast as it can, and prints how many milliseconds each step
#   takes, the frame rate, CPU and memory. No drone or autopilot needed. Without a
#   video it makes its own (simulated hovering drone with vibration), so it always runs.
#   It also times one recovery search (ORB), which only runs while vision is LOST.
#
# HOW TO RUN (from the repo root)
#   python -m compute.benchmark                      # simulated video, 300 frames
#   python -m compute.benchmark flight.mp4           # a recorded Justice D video
#   python -m compute.benchmark flight.mp4 --frames 900
#   Results: printed as a Markdown table and saved to logs/benchmark_<time>.md
#
# HOW IT WORKS (technical)
#   Stages timed per frame: grab (decode + grey), track (Harris/LK/FB/RANSAC + rolling
#   shutter correction, fed a synthetic 50 Hz attitude stream so its cost is counted),
#   health (health.py), fusion (tilt + IMU check). Uses compute/profiler.py.
#
# REFERENCES
#   - See compute/profiler.py. Platform info from Python's platform module.
# =====================================================================================

import argparse
import math
import os
import platform
import time

import cv2
import numpy as np

from compute.profiler import Profiler
from fusion.imu_check import ImuCheck
from fusion.tilt import cancel_tilt
from health.health import HealthChecker
from recovery.recovery import Recovery
from tools.degrade_video import make_ground, render
from vision.rolling_shutter import RollingShutterTracker
from vision.tracker import load_calibration


def synthetic_frames(n, K, size=(640, 480)):
    ground = make_ground()
    rng = np.random.default_rng(0)
    pose = lambda t: (0.05 * t, 0.0, 4.16, math.radians(2) * math.sin(25 * t),
                      math.radians(2) * math.sin(19 * t), 0.1 * t)
    for i in range(n):
        yield i / 30, cv2.cvtColor(render(ground, K, size, pose, i / 30, readout_s=0.025,
                                          noise=2, rng=rng), cv2.COLOR_GRAY2BGR)


def video_frames(path, n):
    cap = cv2.VideoCapture(path)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    for i in range(n):
        ok, frame = cap.read()
        if not ok:
            break
        yield i / fps, frame
    cap.release()


def main():
    ap = argparse.ArgumentParser(description="Time every pipeline stage on this computer")
    ap.add_argument("video", nargs="?", help="recorded video (default: simulated)")
    ap.add_argument("--frames", type=int, default=300)
    args = ap.parse_args()

    K, dist = load_calibration()
    frames = list(video_frames(args.video, args.frames) if args.video
                  else synthetic_frames(args.frames, K))       # pre-load: don't time disk reads
    if len(frames) < 10:
        print("Not enough frames.")
        return
    t_end = frames[-1][0] + 1
    history = [(t, 0.01 * math.sin(25 * t), 0.01 * math.sin(19 * t), 0.0)
               for t in np.arange(-1, t_end, 0.02)]             # 50 Hz attitude stream
    tracker = RollingShutterTracker(K, None if not args.video else dist, attitude_history=history)
    health, imu = HealthChecker(), ImuCheck()
    imu.set_home(0.0)
    prof = Profiler(budget_ms=1000 / 30)

    first = cv2.cvtColor(frames[0][1], cv2.COLOR_BGR2GRAY)
    tracker.set_reference(first, frames[0][0])
    for t, frame in frames[1:]:
        prof.start_frame()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        prof.lap("grab")
        res = tracker.track(gray, t)
        prof.lap("track")
        health.check(gray, res)
        prof.lap("health")
        if res.ok:
            cancel_tilt(res.shift_px, (0.01, 0.01, 0.0), (0.0, 0.0, 0.0), tracker.focal_px, res.rotation_deg)
            imu.check(res.rotation_deg, 0.0)
        prof.lap("fusion")
        prof.end_frame()

    rec = Recovery(tracker)
    rec.set_home(first)
    t0 = time.perf_counter()
    for _, frame in frames[1:11]:
        rec.find_home(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    orb_ms = (time.perf_counter() - t0) * 100            # mean of 10, in ms

    h, w = first.shape
    report = (f"# Pipeline benchmark\n\n"
              f"Machine: {platform.machine()} | {platform.processor() or platform.system()} | "
              f"{os.cpu_count()} cores | Python {platform.python_version()} | OpenCV {cv2.__version__}\n"
              f"Input: {args.video or 'simulated rolling-shutter video'} | {w}x{h} | {len(frames) - 1} frames\n"
              f"Note: frames are processed back to back, so FPS here = the maximum this computer can do.\n\n"
              f"{prof.table()}\n\n"
              f"Recovery search (ORB, runs only while LOST): {orb_ms:.1f} ms per attempt\n")
    print(report)
    os.makedirs("logs", exist_ok=True)
    path = time.strftime("logs/benchmark_%Y%m%d_%H%M%S.md")
    with open(path, "w") as f:
        f.write(report)
    print(f"Saved {path}")


if __name__ == "__main__":
    main()
