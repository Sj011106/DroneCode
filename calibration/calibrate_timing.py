# =====================================================================================
# calibration/calibrate_timing.py  -  measure the camera's timing   BRIEF ITEM 1
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   The rolling-shutter fix needs two numbers we can't read off a datasheet:
#     offset  : how old a picture is when our program gets it (the camera, cable and
#               driver all add delay), relative to the IMU's clock
#     readout : how long the camera takes to scan from the top row to the bottom row
#   This program measures both. You hold the drone (PROPS OFF) over a textured floor
#   and twist / rock it in all directions for ~20 s. The camera sees the floor turn,
#   the IMU feels the drone turn. We slide the IMU's recording back and forth in time
#   until both agree best (= offset), then try readout values until the rolling-shutter
#   correction makes consecutive frames fit a clean motion best (= readout).
#   The result is saved to timing.npz, which vision/timing.py loads automatically.
#
# HOW IT WORKS (technical)
#   1. Frame pairs: Shi-Tomasi corners + pyramidal LK between consecutive frames.
#   2. Offset: per pair, the measured image motion (centre-point flow + in-plane rotation,
#      from a robust similarity fit) is compared with the motion predicted from the IMU
#      attitude at (t + offset). Grid search over offset minimises the squared difference
#      (the scale is known from the focal length, so no correlation normalisation needed).
#   3. Readout: for each candidate readout, both frames' points are rolling-shutter
#      corrected (vision/rolling_shutter.correct_points) and a similarity is fitted
#      (LMedS). Rows captured at different times break the similarity model, so the
#      median residual is lowest at the true readout.
#
# HOW TO RUN (from the repo root, drone connected, props OFF)
#   python -m calibration.calibrate_timing --camera 1
#   python -m calibration.calibrate_timing --camera 1 --connect /dev/ttymxc2 --seconds 30
#
# REFERENCES
#   - Karpenko, Jacobs, Baek, Levoy, "Digital Video Stabilization and Rolling Shutter
#     Correction using Gyroscopes", Stanford CSTR 2011-03 (camera-gyro sync + readout
#     estimated from video, the same idea as here).
#   - Furgale, Rehder, Siegwart, "Unified Temporal and Spatial Calibration for
#     Multi-Sensor Systems", IROS 2013 (Kalibr: the heavyweight alternative).
#   - Rousseeuw, "Least Median of Squares Regression", JASA 1984 (LMedS).
# =====================================================================================

import argparse
import math
import time

import cv2
import numpy as np

import config
from fusion.tilt import nadir_offset_px
from vision.rolling_shutter import attitude_at, correct_points
from vision.timing import CameraTiming, save_timing

LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))


def track_pair(prev_gray, gray, max_corners=300):
    """Corners in prev_gray and where they went in gray (raw pixels), LK-tracked."""
    pts = cv2.goodFeaturesToTrack(prev_gray, max_corners, 0.01, 10)
    if pts is None:
        return None
    nxt, st, _ = cv2.calcOpticalFlowPyrLK(prev_gray, gray, pts, None, **LK)
    ok = st.ravel() == 1
    if ok.sum() < 20:
        return None
    return pts[ok].reshape(-1, 2), nxt[ok].reshape(-1, 2)


def _undistort(pts, K, dist):
    if dist is None:
        return pts.astype(np.float32)
    return cv2.undistortPoints(pts.reshape(-1, 1, 2).astype(np.float32), K, dist, P=K).reshape(-1, 2)


def estimate_offset(pairs, history, K, search_s=0.15, step_s=0.001):
    """pairs: list of (t0, pts0_raw, t1, pts1_raw). Returns the offset (seconds) that best
    lines up image motion with IMU motion, and the RMS mismatch in pixels."""
    f = (K[0, 0] + K[1, 1]) / 2
    centre = np.array([K[0, 2], K[1, 2]])
    meas, times = [], []
    for t0, p0, t1, p1 in pairs:
        M, _ = cv2.estimateAffinePartial2D(p0, p1, method=cv2.LMEDS)
        if M is None:
            continue
        flow = M[:, :2] @ centre + M[:, 2] - centre          # where the centre point moved
        rot = math.atan2(M[1, 0], M[0, 0]) * f                 # in-plane rotation, in "pixels"
        meas.append([flow[0], flow[1], rot])
        times.append([t0, t1])
    meas, times = np.array(meas), np.array(times)
    best = (None, float("inf"))
    for off in np.arange(-search_s, search_s + step_s / 2, step_s):
        a0 = attitude_at(history, times[:, 0] + off)
        a1 = attitude_at(history, times[:, 1] + off)
        if a0 is None:
            break
        pred = np.array([np.r_[nadir_offset_px(r1, p1, f) - nadir_offset_px(r0, p0, f), -(y1 - y0) * f]
                         for (r0, p0, y0), (r1, p1, y1) in zip(a0, a1)])
        err = float(np.sqrt(np.mean((meas - pred) ** 2)))
        if err < best[1]:
            best = (float(off), err)
    return best


def estimate_readout(pairs, history, K, dist, rows, offset_s, max_readout_s=0.04, step_s=0.0005):
    """Returns the readout time (seconds) that makes consecutive frames fit a similarity
    best after rolling-shutter correction, and the residual (px) at that readout and at 0."""
    und = [(t0, p0, _undistort(p0, K, dist), t1, p1, _undistort(p1, K, dist)) for t0, p0, t1, p1 in pairs]
    results = []
    for readout in np.arange(0.0, max_readout_s + step_s / 2, step_s):
        timing = CameraTiming(readout, offset_s)
        res = []
        for t0, p0, u0, t1, p1, u1 in und:
            c0 = correct_points(p0, u0, t0, rows, history, timing, K)
            c1 = correct_points(p1, u1, t1, rows, history, timing, K)
            M, _ = cv2.estimateAffinePartial2D(c0, c1, method=cv2.LMEDS)
            if M is not None:
                res.append(np.median(np.linalg.norm(c0 @ M[:, :2].T + M[:, 2] - c1, axis=1)))
        results.append((float(readout), float(np.mean(res))))
    best = min(results, key=lambda r: r[1])
    return best[0], best[1], results[0][1]


# ---------------- Live calibration on the drone ----------------
def main():
    from compute.budget import make_low_latency
    from link.drone_link import DroneLink
    from vision.enhance import downscale, scale_camera_matrix
    from vision.tracker import load_calibration

    ap = argparse.ArgumentParser(description="Measure camera readout time and camera-IMU offset")
    ap.add_argument("--connect", default="tcp:127.0.0.1:5762", help="ArduCopter route only")
    ap.add_argument("--fc", default=config.FC, choices=["betaflight", "ardupilot"],   # STAGE 2
                    help="betaflight: tilt comes from link/msp_sender.py, which must be running")
    ap.add_argument("--camera", default="0")
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    K, dist = load_calibration()
    if K is None:
        print("Run calibration/auto_calibrate.py first (needs calib.npz).")
        return
    if args.fc == "betaflight":                            # STAGE 2: same tilt source as in flight
        from link.betaflight_link import BetaflightLink
        link = BetaflightLink()
    else:
        link = DroneLink(args.connect)
    link.start()
    cap = cv2.VideoCapture(int(args.camera) if args.camera.isdigit() else args.camera)
    make_low_latency(cap)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30

    # Record first, analyse afterwards: doing optical flow inside the loop would slow it down,
    # frames would queue up and their delay would bias the measured offset (vs. flight).
    print(f"PROPS OFF. Twist and rock the drone over a textured floor for {args.seconds:.0f} s...")
    history, frames = [], []
    t_end = time.time() + args.seconds
    while time.time() < t_end:
        ok, frame = cap.read()
        t = time.time()
        if not ok:
            continue
        for s in list(link.attitudes):                    # keep our own full IMU recording
            if not history or s[0] > history[-1][0]:
                history.append(s)
        small = downscale(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))   # same size as in flight
        if not frames:
            K = scale_camera_matrix(K, small.shape[1] / frame.shape[1])
        frames.append((t, small))
    link.running = False
    cap.release()
    rows = frames[0][1].shape[0] if frames else None
    pairs = []
    for (t0, g0), (t1, g1) in zip(frames, frames[1:]):
        pr = track_pair(g0, g1)
        if pr:
            pairs.append((t0, pr[0], t1, pr[1]))
    print(f"Recorded {len(frames)} frames ({len(pairs)} usable pairs) and {len(history)} IMU samples.")
    if len(pairs) < 50 or len(history) < 50:
        print("Not enough data - check the camera and the flight controller link "
              "(Betaflight: is msp_sender running?), then retry.")
        return

    offset, err = estimate_offset(pairs, history, K)
    print(f"Offset  = {offset * 1000:+.1f} ms   (image vs IMU motion mismatch {err:.2f} px)")
    readout, res, res0 = estimate_readout(pairs, history, K, dist, rows, offset, max_readout_s=1.0 / fps)
    print(f"Readout = {readout * 1000:.1f} ms   (fit residual {res0:.3f} px -> {res:.3f} px)")
    save_timing(readout, offset)
    print("Saved timing.npz - commit it, and set VISO_DELAY_MS (WORKLIST 3.4)")


if __name__ == "__main__":
    main()
