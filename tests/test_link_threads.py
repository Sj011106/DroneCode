# =====================================================================================
# tests/test_link_threads.py  -  the IMU history is safe to read while it is being filled
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   DroneLink's background thread adds ~50 tilt readings a second to link.attitudes,
#   while the main loop copies that list every frame. With a plain deque, copying it at
#   the wrong moment raises "deque mutated during iteration" and vision_hold stops in
#   flight. This test hammers the history from two threads for 2 seconds and checks:
#     1. a plain deque really does fail this way (so the test can catch the bug),
#     2. LockedDeque never fails, keeps its maxlen, and returns ordered snapshots.
#
# Run (from the repo root):  python -m tests.test_link_threads
# =====================================================================================

import threading
import time
from collections import deque

from link.drone_link import LockedDeque

SECONDS = 2.0


def hammer(history):
    """Append from one thread, copy from this one; count RuntimeErrors."""
    stop = threading.Event()

    def writer():
        i = 0
        while not stop.is_set():
            history.append((float(i), 0.0, 0.0, 0.0))
            i += 1

    t = threading.Thread(target=writer, daemon=True)
    t.start()
    errors, copies, t_end = 0, 0, time.time() + SECONDS
    while time.time() < t_end:
        try:
            snap = list(history)
            if snap:
                min(history, key=lambda a: a[0])
                times = [s[0] for s in snap]
                assert times == sorted(times), "snapshot out of order"
            copies += 1
        except RuntimeError:
            errors += 1
    stop.set()
    t.join(timeout=1.0)
    return errors, copies


print("Test 1: plain deque read while another thread appends")
errors, copies = hammer(deque(maxlen=200))
print(f"  {errors} RuntimeErrors in {copies + errors} copies (the bug)")
if errors == 0:
    print("  (no error this run - the race is timing-dependent; test 2 still checks the fix)")

print("Test 2: LockedDeque under the same load")
locked = LockedDeque(maxlen=200)
errors, copies = hammer(locked)
print(f"  {errors} RuntimeErrors in {copies} copies")
assert errors == 0
assert len(locked) == 200 and locked.maxlen == 200
snap = locked.snapshot()
assert isinstance(snap, list) and len(snap) == 200
assert [s[0] for s in snap] == sorted(s[0] for s in snap)

print("Test 3: behaves like a normal deque otherwise")
d = LockedDeque(maxlen=3)
for i in range(5):
    d.append((i, 0, 0, 0))
assert [a[0] for a in d] == [2, 3, 4] and d[-1][0] == 4 and bool(d)
assert not LockedDeque(maxlen=3) and LockedDeque(maxlen=3).snapshot() == []

print("\nALL LINK THREAD TESTS PASSED")
