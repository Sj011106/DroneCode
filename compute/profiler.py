# =====================================================================================
# compute/profiler.py  -  how much of the onboard computer do we use?   BRIEF ITEM 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   A stopwatch for every step of every frame: grabbing the picture, tracking, health
#   checks, recovery, sending to the autopilot. At 30 frames per second each frame has
#   33 ms. If we use more, pictures pile up, positions arrive late, and the drone
#   wobbles. This records the times so we can PROVE on a slide that everything runs on
#   the APB in real time, and see which step to speed up if it doesn't.
#
# HOW IT WORKS (technical)
#   - time.perf_counter() laps per stage (monotonic, highest-resolution clock).
#   - FPS from the time between frame ends (rolling window of the last 300 frames).
#   - CPU % = process CPU time (os.times user + system) / wall time; >100% = several cores.
#   - Peak memory from resource.getrusage (Linux/macOS; not available on Windows).
#   No extra libraries: everything is in the Python standard library.
#
# WHERE IT PLUGS IN
#   vision_hold.py: prof.start_frame() at the top of the loop, prof.lap("track") etc.
#   after each step, prof.end_frame() before logging. Per-frame ms go into the CSV log;
#   prof.table() prints the summary at shutdown. compute/benchmark.py uses the same class.
#
# REFERENCES
#   - Python time.perf_counter: https://docs.python.org/3/library/time.html#time.perf_counter
#   - Python os.times / resource.getrusage: https://docs.python.org/3/library/os.html#os.times
# =====================================================================================

import os
import time
from collections import deque

import numpy as np

try:
    import resource                       # Linux / macOS only
except ImportError:
    resource = None

WINDOW = 300                              # frames kept for the statistics (~10 s at 30 fps)


class Profiler:
    def __init__(self, budget_ms=None):
        self.budget_ms = budget_ms        # time allowed per frame (e.g. 33.3 at 30 fps)
        self.samples = {}                 # stage -> deque of ms
        self.frame_ends = deque(maxlen=WINDOW)
        self.current = {}                 # this frame's laps, for the CSV log
        self.frames = self.over_budget = 0
        self._t0 = self._last = None
        self._cpu0, self._wall0 = sum(os.times()[:2]), time.perf_counter()

    def start_frame(self):
        self._t0 = self._last = time.perf_counter()
        self.current = {}

    def lap(self, stage):
        """Time since the previous lap (or frame start) is charged to `stage`."""
        if self._last is None:
            return
        now = time.perf_counter()
        ms = (now - self._last) * 1000
        self.current[stage] = self.current.get(stage, 0.0) + ms
        self._last = now

    def end_frame(self):
        """Close the frame. Returns this frame's total ms."""
        now = time.perf_counter()
        total = (now - self._t0) * 1000
        self.current["total"] = total
        for stage, ms in self.current.items():
            self.samples.setdefault(stage, deque(maxlen=WINDOW)).append(ms)
        self.frame_ends.append(now)
        self.frames += 1
        if self.budget_ms and total > self.budget_ms:
            self.over_budget += 1
        return total

    def fps(self):
        if len(self.frame_ends) < 2:
            return 0.0
        return (len(self.frame_ends) - 1) / (self.frame_ends[-1] - self.frame_ends[0])

    def cpu_percent(self):
        wall = time.perf_counter() - self._wall0
        return 100.0 * (sum(os.times()[:2]) - self._cpu0) / wall if wall > 0 else 0.0

    @staticmethod
    def peak_memory_mb():
        if resource is None:
            return None
        kb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        return kb / 1024 / (1024 if os.uname().sysname == "Darwin" else 1)   # macOS reports bytes

    def summary(self):
        """{stage: (mean ms, 95th percentile ms)} plus overall numbers."""
        stages = {s: (float(np.mean(v)), float(np.percentile(v, 95))) for s, v in self.samples.items()}
        return dict(stages=stages, fps=self.fps(), cpu=self.cpu_percent(),
                    mem_mb=self.peak_memory_mb(), frames=self.frames,
                    over_budget=self.over_budget / max(self.frames, 1))

    def table(self):
        """Markdown table, ready to paste on a slide."""
        s = self.summary()
        lines = ["| Stage | mean ms | 95% ms |", "|---|---|---|"]
        for stage, (mean, p95) in sorted(s["stages"].items(), key=lambda kv: kv[0] == "total"):
            lines.append(f"| {stage} | {mean:.1f} | {p95:.1f} |")
        mem = f"{s['mem_mb']:.0f} MB" if s["mem_mb"] else "n/a"
        lines.append(f"\nFPS {s['fps']:.1f} | CPU {s['cpu']:.0f}% | peak memory {mem} | "
                     f"frames over budget {100 * s['over_budget']:.1f}% ({s['frames']} frames)")
        return "\n".join(lines)
