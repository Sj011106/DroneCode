# =====================================================================================
# tools/truth_drift.py  -  how far did the drone REALLY drift?   (live trial: drift thresholds)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Our own log only knows what VISION believes. The judges measure what the drone
#   really did. In PRACTICE flights we keep the GPS receiver on as a referee (the
#   autopilot is flying on vision, GPS is only recorded). This tool reads the
#   autopilot's own log file (.bin) after the flight and answers, per altitude band:
#     - real drift: how far the drone wandered from the hold point (GPS),
#     - vision error: how far our vision position was from the truth,
#     - EKF error: how far the autopilot's blended estimate was from the truth.
#   Everything comes from ONE clock (the flight controller's), so no time syncing.
#
# HOW IT WORKS (technical)
#   - Reads GPS (Lat, Lng; first receiver I = 0, Status >= 3 = 3D fix only), XKF1 core 0
#     (PN, PE, PD), VISP (PX, PY: the vision positions ArduPilot received; rows with Ign = 1,
#     i.e. below VISO_QUAL_MIN, are skipped), BARO (Alt, I = 0) and MODE via pymavlink's
#     DFReader. Missing message types are reported and skipped, never fatal.
#   - VISP TimeUS is when ArduPilot RECEIVED the position (CTimeMS = corrected capture
#     time): so "vision error" includes ~VISO_DELAY_MS x speed of lag. At a hover that's small.
#   - VISP Rst (reset counter) changes when vision_hold saves a new home: positions restart
#     at 0. Each Rst segment is aligned to the truth at its first sample, so a re-trigger
#     doesn't look like a huge vision error.
#   - GPS lat/lon -> metres North/East (local flat-earth approximation, fine over < 1 km).
#   - Hold point: the first switch to LOITER/POSHOLD after vision starts (the pilot engages
#     the hold), else the first VISP, or --start seconds.
#     All three tracks are compared as DISPLACEMENTS from that moment, so their
#     different origins don't matter.
#   - Vision/EKF are interpolated onto GPS times. GPS itself has ~0.1-0.2 s lag and
#     ~0.5-2 m absolute error, but its SHORT-TERM displacement is good to a few dm;
#     use an RTK GPS for cm-level truth.
#
# HOW TO RUN (from the repo root)
#   python -m tools.truth_drift 00000042.BIN
#   python -m tools.truth_drift 00000042.BIN --start 120 --end 300 --out docs/plots
#
# REFERENCES
#   - ArduPilot onboard log messages (GPS, XKF1, VISP, BARO):
#     https://ardupilot.org/copter/docs/logmessages.html
#   - pymavlink DFReader: https://github.com/ArduPilot/pymavlink/blob/master/DFReader.py
#   - Local tangent-plane approximation: Groves, "Principles of GNSS, Inertial, and
#     Multisensor Integrated Navigation Systems", 2nd ed., 2013, ch. 2.
# =====================================================================================

import argparse
import math
import os

import numpy as np

EARTH_R = 6378137.0
BANDS = ((0, 5), (5, 10), (10, 20), (20, 40), (40, 80), (80, 1000))
HOLD_MODE_NUMS = {5, 16}                          # ArduCopter LOITER = 5, POSHOLD = 16


def latlon_to_ne(lat, lon, lat0, lon0):
    """Degrees -> metres North/East of (lat0, lon0)."""
    n = np.radians(np.asarray(lat) - lat0) * EARTH_R
    e = np.radians(np.asarray(lon) - lon0) * EARTH_R * math.cos(math.radians(lat0))
    return n, e


def _rel(track, t0):
    """(t, n, e) arrays -> displacement from the value at t0 (interpolated)."""
    t, n, e = (np.asarray(x, float) for x in track)
    return t, n - np.interp(t0, t, n), e - np.interp(t0, t, e)


def drift_report(truth, alt, vision=None, ekf=None, t0=None, t_end=None, bands=BANDS):
    """Pure comparison. truth/vision/ekf = (t, north, east) in metres (any origin);
    alt = (t, height_m). Returns one dict per altitude band with data."""
    t0 = t0 if t0 is not None else (vision[0][0] if vision is not None else truth[0][0])
    tt, tn, te = _rel(truth, t0)
    keep = (tt >= t0) & (tt <= (t_end if t_end is not None else np.inf))
    tt, tn, te = tt[keep], tn[keep], te[keep]
    h = np.interp(tt, *alt)
    out = []
    others = {"vision": vision, "ekf": ekf}
    errs = {}
    for name, track in others.items():
        if track is not None and len(track[0]) > 1:
            ot, on, oe = _rel(track, t0)
            errs[name] = np.hypot(np.interp(tt, ot, on) - tn, np.interp(tt, ot, oe) - te)
    dist = np.hypot(tn, te)
    dt = np.median(np.diff(tt)) if len(tt) > 1 else 0.0
    for lo, hi in bands:
        m = (h >= lo) & (h < hi)
        if not m.any():
            continue
        row = dict(band=f"{lo}-{hi} m", seconds=float(m.sum() * dt),
                   drift_rms=float(np.sqrt(np.mean(dist[m] ** 2))), drift_max=float(dist[m].max()))
        for name, e in errs.items():
            row[f"{name}_err_rms"] = float(np.sqrt(np.mean(e[m] ** 2)))
        out.append(row)
    return out, dict(t=tt, dist=dist, h=h, **errs)


def _messages(path):
    from pymavlink import mavutil
    log = mavutil.mavlink_connection(path)
    while True:
        m = log.recv_match(type=["GPS", "XKF1", "VISP", "BARO", "MODE"], blocking=False)
        if m is None:
            return
        yield m


def align_segments(t, n, e, rst, truth):
    """Vision restarts at 0 whenever its reset counter (Rst) changes (a new home). Shift each
    segment so its first sample equals the truth position at that moment."""
    t, n, e, rst = (np.asarray(x, float) for x in (t, n, e, rst))
    n, e = n.copy(), e.copy()
    starts = np.r_[0, np.nonzero(np.diff(rst))[0] + 1]
    ends = np.r_[starts[1:], len(t)]
    for a, b in zip(starts, ends):
        n[a:b] += np.interp(t[a], truth[0], truth[1]) - n[a]
        e[a:b] += np.interp(t[a], truth[0], truth[2]) - e[a]
    return t, n, e


def load_bin(path, messages=None):
    """Read the tracks from an ArduPilot .bin log (or any iterable of DF messages, for tests).
    Missing types -> reported. Returns dict(truth, alt, [vision], [ekf], hold_start)."""
    gps, ekf, vis, baro, modes = [], [], [], [], []
    for m in (messages if messages is not None else _messages(path)):
        d, kind = m.to_dict(), m.get_type()
        t = d.get("TimeUS", 0) / 1e6
        if kind == "GPS" and d.get("I", 0) == 0 and d.get("Status", 0) >= 3:
            gps.append((t, d["Lat"], d["Lng"]))
        elif kind == "XKF1" and d.get("C", 0) == 0:
            ekf.append((t, d["PN"], d["PE"], -d["PD"]))
        elif kind == "VISP" and not d.get("Ign", 0):
            vis.append((t, d.get("PX", math.nan), d.get("PY", math.nan), d.get("Rst", 0)))
        elif kind == "BARO" and d.get("I", 0) == 0:
            baro.append((t, d["Alt"]))
        elif kind == "MODE":
            modes.append((t, d.get("ModeNum", d.get("Mode"))))
    for name, rows in (("GPS (3D fix)", gps), ("XKF1", ekf), ("VISP", vis), ("BARO", baro), ("MODE", modes)):
        print(f"  {name:13s}: {len(rows)} samples" + ("  <- missing" if not rows else ""))
    if not gps:
        raise SystemExit("No GPS with a 3D fix in this log: no truth to compare against.")
    g = np.array(gps)
    n, e = latlon_to_ne(g[:, 1], g[:, 2], g[0, 1], g[0, 2])
    tracks = dict(truth=(g[:, 0], n, e))
    if ekf:
        a = np.array(ekf)
        tracks["ekf"] = (a[:, 0], a[:, 1], a[:, 2])
    if vis:
        a = np.array(vis)
        tracks["vision"] = align_segments(a[:, 0], a[:, 1], a[:, 2], a[:, 3], tracks["truth"])
    if baro:
        a = np.array(baro)
        tracks["alt"] = (a[:, 0], a[:, 1])
    elif ekf:
        a = np.array(ekf)
        tracks["alt"] = (a[:, 0], a[:, 3])
    else:
        tracks["alt"] = (g[:, 0], np.zeros(len(g)))
    first_vis = vis[0][0] if vis else None
    engaged = [t for t, mode in modes if mode in HOLD_MODE_NUMS and (first_vis is None or t >= first_vis)]
    tracks["hold_start"] = engaged[0] if engaged else first_vis
    return tracks


def print_report(rows):
    print(f"\n{'altitude':>10s} {'time':>7s} {'drift RMS':>10s} {'drift max':>10s} "
          f"{'vision err':>11s} {'EKF err':>9s}   (metres, vs GPS truth)")
    for r in rows:
        v = f"{r['vision_err_rms']:.2f}" if "vision_err_rms" in r else "-"
        k = f"{r['ekf_err_rms']:.2f}" if "ekf_err_rms" in r else "-"
        print(f"{r['band']:>10s} {r['seconds']:6.0f}s {r['drift_rms']:10.2f} {r['drift_max']:10.2f} "
              f"{v:>11s} {k:>9s}")


def main():
    ap = argparse.ArgumentParser(description="Real drift vs GPS truth, from an ArduPilot .bin log")
    ap.add_argument("log")
    ap.add_argument("--start", type=float,
                    help="hold start, seconds of log time (default: first LOITER/POSHOLD after vision starts)")
    ap.add_argument("--end", type=float)
    ap.add_argument("--out", default="docs/plots")
    args = ap.parse_args()

    print(f"Reading {args.log}")
    tr = load_bin(args.log)
    t0 = args.start if args.start is not None else tr["hold_start"]
    print(f"Hold start: {t0:.1f} s" if t0 is not None else "Hold start: first GPS sample")
    rows, series = drift_report(tr["truth"], tr["alt"], tr.get("vision"), tr.get("ekf"), t0, args.end)
    print_report(rows)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(args.out, exist_ok=True)
    name = os.path.splitext(os.path.basename(args.log))[0]
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    t = series["t"] - series["t"][0]
    ax[0].plot(t, series["dist"], "k", label="real drift (GPS)")
    for key, colour in (("vision", "tab:green"), ("ekf", "tab:blue")):
        if key in series:
            ax[0].plot(t, series[key], colour, alpha=.7, label=f"{key} error")
    ax[0].set_ylabel("metres"); ax[0].legend(); ax[0].grid(alpha=.3)
    ax[1].plot(t, series["h"], "tab:orange"); ax[1].set_ylabel("height (m)")
    ax[1].set_xlabel("time since hold start (s)"); ax[1].grid(alpha=.3)
    fig.suptitle("Real drift vs GPS truth")
    path = f"{args.out}/{name}_truth_drift.png"
    fig.savefig(path, dpi=120, bbox_inches="tight")
    print(f"\nPlot saved: {path}")


if __name__ == "__main__":
    main()
