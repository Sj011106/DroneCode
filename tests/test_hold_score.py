# =====================================================================================
# tests/test_hold_score.py  -  PASS / FAIL and the climb-ladder row   (OWNER: Ayush, task A3)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We invent holds with KNOWN answers and check the verdict:
#     a 60 s hold wobbling 0.4 m around the point        -> PASS
#     the same with one 4 m excursion                    -> FAIL (max drift, limit 3 m)
#     a 20 s hold                                        -> FAIL (hold time)
#     no position at all                                 -> FAIL, no crash
#   The limits are the official Stage 2 ones (30 s, 3 m horizontal). A verdict must still say
#   PLACEHOLDER whenever config.HOLD_LIMITS_OFFICIAL is False.
#   Then the two hooks: truth_drift's per-altitude-band verdict (GPS referee) on a known
#   climb, and the real tools/analyze_log.py on a small flight log (vision estimate,
#   labelled as such). Finally the ladder row for docs/WORKLIST.md task 3.8 from a
#   pretend .bin + flight log.
#
# Run (from the repo root):  python -m tests.test_hold_score      (~3 s)
# REFERENCES: tools/hold_score.py
# =====================================================================================

import contextlib
import csv
import io
import math
import os
import sys
import tempfile

import numpy as np

import config
from tools import hold_score as hs
from tools.truth_drift import drift_report

tmp = tempfile.mkdtemp(prefix="dronecode_holdscore_")
t = np.arange(0, 60, 0.2)                                     # 60 s at 5 Hz (GPS rate)

print("Test 1: limits are the official Stage 2 ones")
lim = hs.limits()
print(f"  official={lim['official']}, source='{lim['source']}', max {lim['max_m']} m, RMS {lim['rms_m']} m, "
      f"{lim['inside_pct']}% inside {lim['radius_m']} m, >= {lim['min_s']} s")
assert lim["official"] is True and lim["max_m"] == 3.0 and lim["min_s"] == 30.0, "Stage 2 Rev 2.0 section 06"

print("\nTest 2: holds with known answers")
n, e = 0.4 * np.cos(0.5 * t), 0.4 * np.sin(0.5 * t)
good = hs.score(t, np.hypot(n, e), north=n, east=e)
print(hs.format_verdict(good, "GPS referee"))
assert good["passed"] and abs(good["rms"] - 0.4) < 1e-6 and abs(good["max"] - 0.4) < 1e-6
assert abs(good["seconds"] - 59.8) < 1e-6
assert abs(good["spread"] - 0.4) < 0.01, "spread of a 0.4 m circle around its centre = 0.4 m"
assert "PLACEHOLDER" not in hs.format_verdict(good, "x") and hs.verdict_word(good) == "PASS"

d_spike = np.hypot(n, e)
d_spike[150:155] = 4.0                                        # 1 s excursion to 4 m (limit 3 m)
spike = hs.score(t, d_spike)
print(hs.format_verdict(spike, "GPS referee"))
failed = [c[0] for c in spike["checks"] if not c[2]]
assert not spike["passed"] and any("max drift" in c for c in failed)

short = hs.score(t[:100], np.hypot(n, e)[:100])
assert not short["passed"] and any("hold time" in c[0] and not c[2] for c in short["checks"])
print(f"  20 s hold -> {hs.verdict_word(short)} ({short['checks'][0][0]})")

empty = hs.score(t, np.full_like(t, np.nan))
assert not empty["passed"] and empty["samples"] == 0
print(f"  no position at all -> {hs.verdict_word(empty)}, no crash")

saved = config.HOLD_LIMITS_OFFICIAL
config.HOLD_LIMITS_OFFICIAL = False                           # unofficial numbers must be labelled
assert hs.verdict_word(hs.score(t, np.hypot(n, e))) == "PASS (placeholder limits)"
assert "PLACEHOLDER" in hs.format_verdict(hs.score(t, np.hypot(n, e)), "x")
config.HOLD_LIMITS_OFFICIAL = saved

print("\nTest 3: truth_drift hook - one GPS-referee verdict per altitude band")
tg = np.arange(0, 120, 0.2)
wn, we = 0.3 * np.cos(0.3 * tg), 0.3 * np.sin(0.3 * tg)
wn = np.where(tg > 60, wn + 4.5 * (tg - 60) / 60, wn)        # drifts away above ~20 m (past the 3 m limit)
alt = (tg, np.where(tg < 60, 15.0, 30.0))                    # 60 s at 15 m, then 60 s at 30 m
_, series = drift_report((tg, 500 + wn, 20 + we), alt)
out = io.StringIO()
with contextlib.redirect_stdout(out):
    bands = dict(hs.print_band_verdicts(series))
print(out.getvalue().rstrip())
assert bands["10-20 m"]["passed"] and not bands["20-40 m"]["passed"]
assert "GPS referee" in out.getvalue()

print("\nTest 4: the real tools/analyze_log.py prints a labelled vision verdict")
log = os.path.join(tmp, "flight_test.csv")
with open(log, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_capture", "status", "quality", "north_m", "east_m", "height_m", "relocks", "rehomes", "ms_total"])
    for i, ti in enumerate(np.arange(0, 45, 1 / 30)):
        lost = 600 <= i < 630                                 # 1 s LOST
        w.writerow([f"{1000 + ti:.3f}", "LOST" if lost else "OK", -1 if lost else 90,
                    "nan" if lost else f"{0.2 * math.sin(ti):.3f}", "nan" if lost else f"{0.1 * math.cos(ti):.3f}",
                    20.0, 0, 0, 9.5])
from tools import analyze_log
argv, sys.argv = sys.argv, ["analyze_log", log, "--out", os.path.join(tmp, "plots")]
out = io.StringIO()
with contextlib.redirect_stdout(out):
    analyze_log.main()
sys.argv = argv
text = out.getvalue()
print("\n".join(text.splitlines()[-9:]))
assert "vision estimate - NOT the judges' measurement" in text
assert "| 20 m | - | 98% |" in text, "ladder row from the vision log (98% trusted, 20 m)"

print("\nTest 5: hold_score command logic - GPS referee + our log -> climb-ladder row")
tracks = dict(truth=(tg, 500 + 0.3 * np.cos(0.3 * tg), 20 + 0.3 * np.sin(0.3 * tg)),
              alt=(tg, np.full_like(tg, 20.0)), hold_start=10.0)
sc, row = hs.ladder_from(tracks, log, "dry grass")
print("  | Height | Surface | % time OK | RMS drift | Max drift | PASS/FAIL | Notes |")
print("  " + row)
cells = [c.strip() for c in row.strip("|").split("|")]
assert len(cells) == 7, "same 7 columns as the table in docs/WORKLIST.md task 3.8"
assert cells[0] == "20 m" and cells[1] == "dry grass" and cells[2] == "98%" and cells[5].startswith("PASS")
assert 0.5 < sc["max"] <= 0.6 + 1e-6, "a 0.3 m circle is at most 0.6 m (its diameter) from its start"

print("\nTest 6: switch off -> the tools print exactly what they printed before")
config.USE_HOLD_SCORE = False
assert hs.print_band_verdicts(series) == [] and hs.print_vision_verdict(t, n, e, np.ones_like(t, bool)) is None
config.USE_HOLD_SCORE = True

print("\nALL HOLD-SCORE TESTS PASSED")
