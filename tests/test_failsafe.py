# =====================================================================================
# tests/test_failsafe.py  -  vision never comes back -> act, once, on time   BRIEF ITEM 3
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   We feed the failsafe a made-up story: vision OK, then a short glitch (must NOT
#   trigger), then lost for good (must trigger exactly once, after FAILSAFE_AFTER_S),
#   then vision returns (re-arms), then lost again (triggers again).
#
# Run (from the repo root):  python -m tests.test_failsafe
# =====================================================================================

from recovery.failsafe import Failsafe

fs = Failsafe(after_s=3.0, action="LAND")
dt = 1 / 30
fired = []


def feed(lost, seconds, t0):
    t = t0
    while t < t0 + seconds - 1e-9:
        action = fs.update(lost, t)
        if action:
            fired.append((round(t, 2), action))
        t += dt
    return t0 + seconds


t = feed(False, 2.0, 0.0)      # flying on vision
t = feed(True, 1.5, t)         # short glitch: less than 3 s
t = feed(False, 1.0, t)        # back
print(f"after a 1.5 s glitch: fired {fired} (should be none)")
assert fired == []

lost_at = t
t = feed(True, 6.0, t)         # lost for good
print(f"lost at {lost_at:.2f} s for 6 s: fired {fired} (should be ONE 'LAND' about 3 s later)")
assert len(fired) == 1 and fired[0][1] == "LAND"
assert abs(fired[0][0] - (lost_at + 3.0)) <= 2 * dt

t = feed(False, 1.0, t)        # vision back: re-arm
t = feed(True, 3.5, t)
print(f"after vision returned and was lost again: fired {len(fired)} times (should be 2)")
assert len(fired) == 2

none = Failsafe(after_s=3.0, action="NONE")
assert all(none.update(True, x * dt) is None for x in range(300))
print("action NONE never fires: OK")

# step(): the mode change must STICK. The first 2 commands get lost; the autopilot only
# switches on the 3rd. Then it must stop resending. A deaf autopilot: give up after N tries.
import threading
from types import SimpleNamespace

import config


class Conn:
    def __init__(self, obey_on):
        self.flightmode, self.sent, self.obey_on = "LOITER", [], obey_on

    def set_mode(self, mode):
        self.sent.append(mode)
        if len(self.sent) == self.obey_on:
            self.flightmode = mode


for obey_on, expect_sent in ((3, 3), (99, config.FAILSAFE_MODE_RETRIES)):
    link = SimpleNamespace(conn=Conn(obey_on), send_lock=threading.Lock())
    fs = Failsafe(after_s=3.0, action="LAND")
    for i in range(int(15 / dt)):
        fs.step(True, i * dt, link)
    print(f"autopilot obeys on try {obey_on}: sent {len(link.conn.sent)} x LAND, now in {link.conn.flightmode}")
    assert len(link.conn.sent) == expect_sent
assert link.conn.flightmode == "LOITER" and fs.pending is None      # gave up, told the pilot
print("\nALL FAILSAFE TESTS PASSED")
