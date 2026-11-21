# =====================================================================================
# tests/test_thermal.py  -  hot board -> fewer frames (BETAFLIGHT CHANGE, compute/thermal.py)
# RUN:  python -m tests.test_thermal   (< 1 s)
# =====================================================================================
import os
import tempfile

import config
from compute.thermal import Thermal

d = tempfile.mkdtemp()
path = os.path.join(d, "temp")
now = [0.0]
th = Thermal(path, clock=lambda: now[0])
print("Test 1: no temperature file (laptop) -> full rate")
assert Thermal(os.path.join(d, "missing")).frame_every() == 1
for milli, every, orb in ((62000, 1, True), (config.THERMAL_SLOW_C * 1000 + 500, 2, True),
                          (config.THERMAL_HOT_C * 1000 + 500, 3, False), (70000, 1, True)):
    with open(path, "w") as f:
        f.write(f"{milli}\n")
    now[0] += 1.1
    got = th.frame_every()
    print(f"  {milli / 1000:5.1f} C -> every {got} frame(s), ORB {'on' if th.allow_orb() else 'off'}")
    assert got == every and th.allow_orb() == orb
with open(path, "w") as f:
    f.write("99000\n")
now[0] += 0.3                                             # read at most once a second
assert th.frame_every() == 1
print("\nALL THERMAL TESTS PASSED")
