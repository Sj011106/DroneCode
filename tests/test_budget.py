# =====================================================================================
# tests/test_budget.py  -  stale-frame dropping keeps the frame rate AND the freshness   BRIEF 2
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   A pretend camera delivers a picture every 1/60 s into a small driver queue. Our loop
#   takes X ms per picture. We compare three rules for throwing away old pictures:
#     never drop, the OLD rule (based on loop time; a reviewer found it halves the frame
#     rate) and the NEW rule (based on the picture's age, compute/budget.py).
#   The new rule must be at least as fast as the old one, never make the data older, drop
#   nothing with a 1-frame queue, and with a long driver queue keep the data much fresher
#   (old position data makes the autopilot correct for where the drone WAS).
#
# Run (from the repo root):  python -m tests.test_budget
# =====================================================================================

import math

from compute.budget import FrameBudget

P = 1 / 60                                          # Justice D frame period


def simulate(rule, work_s, queue_len, seconds=10.0):
    """Returns (processed frames per second, mean age of a frame's data when its result is ready)."""
    t, last, n, ages = 0.0, -1, 0, []

    def read():                                     # oldest frame still in the driver queue
        nonlocal t, last
        newest = math.floor(t / P + 1e-9)
        idx = max(last + 1, newest - queue_len + 1)
        if idx > newest:
            t = idx * P                             # wait for it to arrive
        last = idx
        return idx

    budget = FrameBudget(camera_fps=1 / P, queue_len=queue_len)
    while t < seconds:
        start = t
        idx = read()
        t_capture = t
        read_wait = t - start
        t += work_s                                 # process the frame
        n += 1
        ages.append(t - idx * P)
        if rule == "old":
            drops = min(int((t - start) / P), 3)
        elif rule == "new":
            drops = budget.frames_to_drop(t - t_capture, read_wait)
        else:
            drops = 0
        for _ in range(drops):
            read()                                  # grab()
    return n / seconds, sum(ages) / len(ages)


print(f"{'queue':>5} {'work':>6} | {'never drop':>17} | {'OLD rule':>17} | {'NEW rule':>17}")
print(f"{'':>5} {'ms':>6} | {'fps    age ms':>17} | {'fps    age ms':>17} | {'fps    age ms':>17}")
for queue_len in (1, 4):
    for work_ms in (10, 20, 30, 45):
        res = {r: simulate(r, work_ms / 1000, queue_len) for r in ("never", "old", "new")}
        print(f"{queue_len:5d} {work_ms:6d} | " + " | ".join(
            f"{fps:6.1f} {age * 1000:9.1f} " for fps, age in res.values()))
        assert res["new"][0] >= res["old"][0] - 0.2, "new rule must be at least as fast as the old one"
        assert res["new"][1] <= res["never"][1] + 1e-3, "new rule must never make data older"
        if queue_len == 1:
            assert res["new"] == res["never"], "1-frame queue: nothing should be dropped"

fps_old, _ = simulate("old", 0.020, 1)
fps_new, _ = simulate("new", 0.020, 1)
print(f"\n20 ms per frame, 1-frame queue: OLD rule {fps_old:.1f} fps -> NEW rule {fps_new:.1f} fps")
assert fps_new > 1.3 * fps_old
_, age_never = simulate("never", 0.020, 4)
_, age_new = simulate("new", 0.020, 4)
print(f"20 ms per frame, 4-frame queue: data age {age_never * 1000:.0f} ms (never drop) -> "
      f"{age_new * 1000:.0f} ms (NEW rule)")
assert age_new < 0.6 * age_never
print("\nALL BUDGET TESTS PASSED")
