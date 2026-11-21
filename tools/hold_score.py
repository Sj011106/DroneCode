# =====================================================================================
# tools/hold_score.py  -  PASS / FAIL against the hold limits, and one climb-ladder row
#                                                                     (OWNER: Ayush, task A3)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   After every practice hold we want an instant answer: would the judges call this a
#   PASS? This file holds that judgement in ONE place. It takes "how far from the hold
#   point" over time and checks it against the limits in config.py:
#     hold time        the hold lasted at least HOLD_MIN_SECONDS
#     max drift        the drone never got further than HOLD_MAX_DRIFT_M from the point
#     RMS drift        the typical distance (root of the mean of the squares) <= HOLD_RMS_DRIFT_M
#     time inside      at least HOLD_MIN_INSIDE_PCT % of the time within HOLD_RADIUS_M
#     spread           ("stability") the wobble around its own average <= HOLD_MAX_SPREAD_M
#   A limit set to None is not judged. IMPORTANT: the official numbers DO NOT EXIST YET.
#   The Grand Final package (Stage 1 Rev 1.3) says Stage 2 will define "the approved
#   altitude levels, qualifying hold and measurement rules" (p.2; also p.3, p.8). Until
#   someone copies them into config.py and sets HOLD_LIMITS_OFFICIAL = True, every line
#   this prints says PLACEHOLDER.
#
#   Which drift? Two very different things:
#     - tools/truth_drift.py: the REAL drift, GPS kept on as a referee in practice. This is
#       what the judges measure (with their own setup).
#     - tools/analyze_log.py: what VISION BELIEVES. Useful, but a vision that is wrong
#       can believe it is perfect. Its PASS/FAIL is labelled "vision estimate".
#
# HOW IT WORKS (technical)
#   Distances are horizontal, from the engage point (the hold start), as truth_drift
#   already computes them. Samples with no position (NaN) are left out of the distance
#   numbers; "hold time" is the span of valid samples. RMS = sqrt(mean(d^2)).
#   Spread = RMS distance from the track's own mean position (needs north/east).
#
# HOW TO RUN (from the repo root)
#   python -m tools.hold_score 00000042.BIN --flight logs/flight_20261120_101500.csv \
#          --surface "dry grass" [--start 120 --end 180]
#     -> PASS/FAIL from the GPS referee + % time OK from our log + the climb-ladder row
#        (docs/WORKLIST.md task 3.8), ready to paste.
#   tools/truth_drift.py and tools/analyze_log.py also print a verdict at the end.
#
# WHERE IT PLUGS IN
#   config.py: HOLD_* limits, HOLD_LIMITS_OFFICIAL, USE_HOLD_SCORE (switch).
#   Read-only: never touches the drone or vision_hold.py.
#
# REFERENCES
#   - Grand Final Participant Technical Package, Stage 1 Rev 1.3: p.2, p.3, p.8 (limits
#     come in Stage 2). docs/WORKLIST.md tasks 2.2 and 3.8.
# =====================================================================================

import argparse
import csv
import math

import numpy as np

import config


def limits():
    """The limits from config.py, and whether they are official."""
    return dict(official=bool(config.HOLD_LIMITS_OFFICIAL), source=config.HOLD_LIMITS_SOURCE,
                min_s=config.HOLD_MIN_SECONDS, max_m=config.HOLD_MAX_DRIFT_M, rms_m=config.HOLD_RMS_DRIFT_M,
                radius_m=config.HOLD_RADIUS_M, inside_pct=config.HOLD_MIN_INSIDE_PCT,
                spread_m=config.HOLD_MAX_SPREAD_M)


def score(t, dist, ok=None, north=None, east=None, lim=None):
    """t (s), dist (m from the hold point; NaN = no position), ok (trusted frames, optional),
    north/east (for the spread). Returns a dict with the numbers and each check."""
    lim = lim or limits()
    t, dist = np.asarray(t, float), np.asarray(dist, float)
    valid = ~np.isnan(dist)
    d = dist[valid]
    out = dict(limits=lim, samples=int(valid.sum()))
    if d.size == 0:
        out.update(seconds=0.0, rms=math.nan, max=math.nan, inside_pct=math.nan, spread=math.nan,
                   pct_ok=math.nan, checks=[("data", "no position samples", False)], passed=False)
        return out
    tv = t[valid]
    out["seconds"] = float(tv[-1] - tv[0]) if tv.size > 1 else 0.0
    out["rms"] = float(np.sqrt(np.mean(d ** 2)))
    out["max"] = float(d.max())
    out["inside_pct"] = float(100 * np.mean(d <= lim["radius_m"])) if lim["radius_m"] is not None else math.nan
    out["pct_ok"] = float(100 * np.mean(np.asarray(ok, bool))) if ok is not None and len(ok) else math.nan
    out["spread"] = math.nan
    if north is not None and east is not None:
        n, e = np.asarray(north, float)[valid], np.asarray(east, float)[valid]
        out["spread"] = float(np.sqrt(np.mean((n - n.mean()) ** 2 + (e - e.mean()) ** 2)))
    checks = []
    if lim["min_s"] is not None:
        checks.append((f"hold time {out['seconds']:.0f} s >= {lim['min_s']:g} s", out["seconds"], out["seconds"] >= lim["min_s"]))
    if lim["max_m"] is not None:
        checks.append((f"max drift {out['max']:.2f} m <= {lim['max_m']:g} m", out["max"], out["max"] <= lim["max_m"]))
    if lim["rms_m"] is not None:
        checks.append((f"RMS drift {out['rms']:.2f} m <= {lim['rms_m']:g} m", out["rms"], out["rms"] <= lim["rms_m"]))
    if lim["inside_pct"] is not None and lim["radius_m"] is not None:
        checks.append((f"time inside {lim['radius_m']:g} m {out['inside_pct']:.1f}% >= {lim['inside_pct']:g}%",
                       out["inside_pct"], out["inside_pct"] >= lim["inside_pct"]))
    if lim["spread_m"] is not None and not math.isnan(out["spread"]):
        checks.append((f"spread {out['spread']:.2f} m <= {lim['spread_m']:g} m", out["spread"], out["spread"] <= lim["spread_m"]))
    out["checks"] = checks
    out["passed"] = bool(checks) and all(c[2] for c in checks)
    return out


def verdict_word(sc):
    word = "PASS" if sc["passed"] else "FAIL"
    return word if sc["limits"]["official"] else f"{word} (placeholder limits)"


def format_verdict(sc, source):
    """Text block. source = 'GPS referee' or 'vision estimate (NOT the judges' measurement)'."""
    lim = sc["limits"]
    lines = [f"HOLD VERDICT [{source}]: {verdict_word(sc)}"]
    if not lim["official"]:
        lines.append(f"  !! PLACEHOLDER limits ({lim['source']}): Stage 2 defines the real ones "
                     f"(Grand Final package p.2). Do not quote this verdict as official.")
    for text, _, ok in sc["checks"]:
        lines.append(f"  {'ok  ' if ok else 'FAIL'}  {text}")
    if not math.isnan(sc.get("pct_ok", math.nan)):
        lines.append(f"  info  vision trusted (OK) {sc['pct_ok']:.1f}% of frames")
    return "\n".join(lines)


def ladder_row(height_m, surface, pct_ok, rms, maxd, sc, notes=""):
    """One row of the climb-ladder table in docs/WORKLIST.md task 3.8:
    | Height | Surface | % time OK | RMS drift | Max drift | PASS/FAIL | Notes |"""
    f = lambda x, unit: "-" if x is None or (isinstance(x, float) and math.isnan(x)) else f"{x:.{0 if unit == '%' else 2}f}{unit}"
    h = "-" if height_m is None or math.isnan(height_m) else f"{height_m:.0f} m"
    note = notes or ("" if sc["limits"]["official"] else "placeholder limits")
    return f"| {h} | {surface or '-'} | {f(pct_ok, '%')} | {f(rms, ' m')} | {f(maxd, ' m')} | {verdict_word(sc)} | {note} |"


# ---------------- hooks used by truth_drift.py and analyze_log.py ----------------
def print_band_verdicts(series, bands=None):
    """truth_drift: one GPS-referee verdict per altitude band that has data."""
    if not config.USE_HOLD_SCORE:
        return []
    from tools.truth_drift import BANDS
    out = []
    t, dist, h = series["t"], series["dist"], series["h"]
    for lo, hi in bands or BANDS:
        m = (h >= lo) & (h < hi)
        if m.sum() < 2:
            continue
        sc = score(t[m], dist[m])
        out.append((f"{lo}-{hi} m", sc))
        print(f"\n{lo}-{hi} m: " + format_verdict(sc, "GPS referee"))
    return out


def print_vision_verdict(t, north, east, ok, height=None):
    """analyze_log: verdict on what VISION believed, plus a ladder row labelled as such."""
    if not config.USE_HOLD_SCORE:
        return None
    north, east = np.asarray(north, float), np.asarray(east, float)
    ok = np.asarray(ok, bool)
    dist = np.where(ok, np.hypot(north, east), np.nan)
    sc = score(t, dist, ok=ok, north=np.where(ok, north, np.nan), east=np.where(ok, east, np.nan))
    print("\n" + format_verdict(sc, "vision estimate - NOT the judges' measurement; use tools.truth_drift"))
    h = float(np.nanmedian(height)) if height is not None and np.isfinite(height).any() else None
    print("Ladder row (vision estimate only):")
    print("  " + ladder_row(h, None, sc["pct_ok"], sc["rms"], sc["max"], sc, "vision estimate, not truth"))
    return sc


# ---------------- command: GPS referee + our log -> the ladder row ----------------
def read_flight(path):
    """t (s, our clock), ok, height from a vision_hold flight log."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    num = lambda x: float(x) if x not in (None, "", "nan") else math.nan
    t = np.array([num(r["t_capture"]) for r in rows])
    ok = np.array([r["status"] == "OK" for r in rows])
    h = np.array([num(r.get("height_m")) for r in rows])
    return t, ok, h


def ladder_from(tracks, flight_csv=None, surface=None, start=None, end=None):
    """tracks = tools.truth_drift.load_bin(...) output. Returns (score, ladder row text)."""
    from tools.truth_drift import drift_report
    t0 = start if start is not None else tracks["hold_start"]
    _, series = drift_report(tracks["truth"], tracks["alt"], tracks.get("vision"), tracks.get("ekf"), t0, end)
    pct_ok = math.nan
    if flight_csv:
        _, ok, _ = read_flight(flight_csv)
        pct_ok = float(100 * ok.mean()) if ok.size else math.nan
    sc = score(series["t"], series["dist"])
    sc["pct_ok"] = pct_ok
    height = float(np.median(series["h"])) if len(series["h"]) else None
    return sc, ladder_row(height, surface, pct_ok, sc["rms"], sc["max"], sc)


def main(argv=None):
    ap = argparse.ArgumentParser(description="PASS/FAIL + climb-ladder row from a practice hold")
    ap.add_argument("bin", help="ArduPilot .bin log (GPS kept on as referee)")
    ap.add_argument("--flight", help="our logs/flight_*.csv for '% time OK'")
    ap.add_argument("--surface", help="what the ground looked like, e.g. 'dry grass'")
    ap.add_argument("--start", type=float, help="hold start (log seconds); default: pilot engaged LOITER/POSHOLD")
    ap.add_argument("--end", type=float)
    args = ap.parse_args(argv)
    from tools.truth_drift import load_bin
    tracks = load_bin(args.bin)
    sc, row = ladder_from(tracks, args.flight, args.surface, args.start, args.end)
    print(format_verdict(sc, "GPS referee"))
    print("\nClimb-ladder row (docs/WORKLIST.md task 3.8):")
    print("| Height | Surface | % time OK | RMS drift | Max drift | PASS/FAIL | Notes |")
    print(row)
    return 0 if sc["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
