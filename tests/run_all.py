# =====================================================================================
# tests/run_all.py  -  run EVERY test file, one line each, exit code 1 if any fails
#                                                                     (OWNER: Ayush, task A6)
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   One command instead of 20: finds every tests/test_*.py, runs each as
#   `python -m tests.test_<name>` (exactly how the team runs them by hand), and prints
#   PASS/FAIL, the time it took and its last line. GitHub Actions runs this same command
#   on every push (.github/workflows/tests.yml), so a green tick on GitHub and a green
#   run on your laptop mean the same thing.
#
# HOW TO RUN (from the repo root)
#   python -m tests.run_all              # everything (~4 min)
#   python -m tests.run_all --fast       # skip the slow simulated-flight tests (~1 min)
#   python -m tests.run_all trial ceiling   # only tests whose name contains these words
#   A failing test's full output is printed at the end.
# =====================================================================================

import argparse
import glob
import os
import subprocess
import sys
import time

SLOW = ("test_altitude", "test_rehome", "test_replay", "test_recovery", "test_ceiling", "test_rolling_shutter")


def main(argv=None):
    ap = argparse.ArgumentParser(description="Run every test file")
    ap.add_argument("only", nargs="*", help="only tests whose name contains one of these words")
    ap.add_argument("--fast", action="store_true", help="skip the slow simulated-flight tests")
    args = ap.parse_args(argv)
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    names = sorted(os.path.splitext(os.path.basename(p))[0] for p in glob.glob(os.path.join(root, "tests", "test_*.py")))
    if args.only:
        names = [n for n in names if any(w in n for w in args.only)]
    if args.fast:
        names = [n for n in names if n not in SLOW]
    failed, start = [], time.time()
    for n in names:
        t0 = time.time()
        p = subprocess.run([sys.executable, "-m", f"tests.{n}"], cwd=root, capture_output=True, text=True,
                           env={**os.environ, "MPLBACKEND": "Agg"})
        out = (p.stdout + p.stderr).strip()
        last = out.splitlines()[-1][:60] if out else ""
        ok = p.returncode == 0
        print(f"{'PASS' if ok else 'FAIL'}  {n:28s} {time.time() - t0:6.1f} s  {last}", flush=True)
        if not ok:
            failed.append((n, out))
    print(f"\n{len(names) - len(failed)} of {len(names)} test files passed in {time.time() - start:.0f} s")
    for n, out in failed:
        print(f"\n===== {n} (last 40 lines) =====\n" + "\n".join(out.splitlines()[-40:]))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
