# =====================================================================================
# tools/analyze_log.py  -  turn a flight log into slide plots and numbers
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Every flight writes logs/flight_<date>.csv (one line per camera frame). This reads
#   one and makes three pictures for the presentation, plus a short summary:
#     1. position.png : where vision thinks the drone is vs home, over time
#     2. health.png   : trust timeline (OK / DEGRADED / LOST) and quality score:
#                       shows blackouts, recovery and how fast trust came back
#     3. compute.png  : milliseconds per pipeline stage per frame (onboard compute)
#   Note: positions are the VISION estimate. To measure true drift, compare against an
#   external reference (GPS log, tape marks on the ground, overhead video).
#
# HOW TO RUN (from the repo root)
#   python -m tools.analyze_log                         # newest log in logs/
#   python -m tools.analyze_log logs/flight_20261117_173046.csv --out docs/plots
#
# REFERENCES
#   - Matplotlib: https://matplotlib.org/stable/  (Hunter, "Matplotlib: A 2D Graphics
#     Environment", Computing in Science & Engineering 2007)
# =====================================================================================

import argparse
import csv
import glob
import math
import os

import matplotlib
matplotlib.use("Agg")                    # no screen needed (works on the APB too)
import matplotlib.pyplot as plt
import numpy as np

STATUS_LEVEL = {"OK": 2, "DEGRADED": 1, "LOST": 0}
STAGES = ("ms_grab", "ms_track", "ms_health", "ms_fusion", "ms_send")


def num(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return math.nan


def load(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path} is empty")
    cols = {k: [r.get(k) for r in rows] for k in rows[0]}
    t = np.array([num(x) for x in cols["t_capture"]])
    return cols, t - t[0]


def segments(t, mask):
    """[(start, end)] time spans where mask is True."""
    spans, start = [], None
    for ti, m in zip(t, mask):
        if m and start is None:
            start = ti
        elif not m and start is not None:
            spans.append((start, ti))
            start = None
    if start is not None:
        spans.append((start, t[-1]))
    return spans


def main():
    ap = argparse.ArgumentParser(description="Plots + summary from a vision_hold flight log")
    ap.add_argument("log", nargs="?")
    ap.add_argument("--out", default="docs/plots")
    args = ap.parse_args()
    path = args.log or max(glob.glob("logs/flight_*.csv"), key=os.path.getmtime)
    cols, t = load(path)
    os.makedirs(args.out, exist_ok=True)
    name = os.path.splitext(os.path.basename(path))[0]

    status = cols["status"]
    ok = np.array([s == "OK" for s in status])
    north = np.array([num(x) for x in cols["north_m"]])
    east = np.array([num(x) for x in cols["east_m"]])
    dist = np.hypot(north, east)
    quality = np.array([num(x) for x in cols["quality"]])

    # 1. position
    fig, ax = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax[0].plot(t, north, label="north"); ax[0].plot(t, east, label="east")
    ax[0].set_ylabel("metres from home"); ax[0].legend(); ax[0].grid(alpha=.3)
    ax[1].plot(t, dist * 100, color="k"); ax[1].set_ylabel("distance from home (cm)")
    ax[1].set_xlabel("time (s)"); ax[1].grid(alpha=.3)
    fig.suptitle(f"Vision position estimate - {name}")
    fig.savefig(f"{args.out}/{name}_position.png", dpi=120, bbox_inches="tight"); plt.close(fig)

    # 2. health timeline
    fig, ax = plt.subplots(2, 1, figsize=(10, 5), sharex=True)
    ax[0].step(t, [STATUS_LEVEL.get(s, 0) for s in status], where="post", color="tab:blue")
    ax[0].set_yticks([0, 1, 2], ["LOST", "DEGRADED", "OK"]); ax[0].grid(alpha=.3)
    for a, b in segments(t, ~ok):
        ax[0].axvspan(a, b, color="tab:red", alpha=.12)
    ax[1].plot(t, np.where(quality > 0, quality, np.nan), color="tab:green")
    ax[1].set_ylim(0, 105); ax[1].set_ylabel("quality"); ax[1].set_xlabel("time (s)"); ax[1].grid(alpha=.3)
    fig.suptitle("Vision trust: health status and quality (red = not trusted)")
    fig.savefig(f"{args.out}/{name}_health.png", dpi=120, bbox_inches="tight"); plt.close(fig)

    # 3. compute
    have = [s for s in STAGES if s in cols]
    if have:
        ms = np.array([[num(x) for x in cols[s]] for s in have])
        fig, ax = plt.subplots(figsize=(10, 4))
        ax.stackplot(t, np.nan_to_num(ms), labels=[s[3:] for s in have])
        fps = 30
        ax.axhline(1000 / fps, color="r", ls="--", label=f"budget ({fps} fps)")
        ax.set_ylabel("ms per frame"); ax.set_xlabel("time (s)"); ax.legend(loc="upper right"); ax.grid(alpha=.3)
        fig.suptitle("Onboard compute per frame")
        fig.savefig(f"{args.out}/{name}_compute.png", dpi=120, bbox_inches="tight"); plt.close(fig)

    # summary
    lost_spans = segments(t, ~ok)
    print(f"Log: {path}  ({t[-1]:.1f} s, {len(t)} frames)")
    print(f"Trusted (OK): {100 * ok.mean():.1f}% of frames")
    if ok.any():
        d = dist[ok & ~np.isnan(dist)]
        print(f"Distance from home while OK: RMS {100 * np.sqrt(np.mean(d ** 2)):.1f} cm, "
              f"max {100 * d.max():.1f} cm (vision estimate)")
    if lost_spans:
        longest = max(b - a for a, b in lost_spans)
        print(f"Untrusted periods: {len(lost_spans)}, longest {longest:.2f} s")
    if "relocks" in cols:
        print(f"Re-locks (recovery): {int(num(cols['relocks'][-1]))}")
    if "rehomes" in cols:
        print(f"Re-homes (climb/descent): {int(num(cols['rehomes'][-1]))}")
    if "ms_total" in cols:
        tot = np.array([num(x) for x in cols["ms_total"]])
        print(f"Per frame: mean {np.nanmean(tot):.1f} ms, 95% {np.nanpercentile(tot, 95):.1f} ms")
    print(f"Plots saved in {args.out}/")


if __name__ == "__main__":
    main()
