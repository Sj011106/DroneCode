# =====================================================================================
# tests/test_params.py  -  the ArduPilot settings file and loader
# -------------------------------------------------------------------------------------
# IN PLAIN WORDS
#   Checks that ardupilot/vision_hold.param says what our code depends on (vision on,
#   source set 2 = vision, height/heading never from vision, channel 7 free...), and
#   that load_params.py really writes, reads back and reports, using a pretend
#   autopilot (no drone needed).
#
# Run (from the repo root):  python -m tests.test_params
# =====================================================================================

import threading

from ardupilot.load_params import apply, parse_param_file
from link.drone_link import SRC_GPS, SRC_VISION

p = dict(parse_param_file())
print(f"Test 1: vision_hold.param has {len(p)} active parameters")
assert len(p) == len(parse_param_file()), "duplicate parameter names"
must = {"VISO_TYPE": 1, "EK3_SRC2_POSXY": 6, "EK3_SRC2_YAW": 1, "EK3_SRC_OPTIONS": 0,
        "AHRS_EKF_TYPE": 3, "EK3_SRC1_POSXY": 3, "RC7_OPTION": 0, "FS_EKF_ACTION": 1}
for name, value in must.items():
    assert p[name] == value, f"{name} should be {value}, file says {p[name]}"
assert p["EK3_SRC2_POSZ"] in (1, 2), "height must come from baro (1) or rangefinder (2), never vision"
assert (SRC_GPS, SRC_VISION) == (1, 2), "drone_link source numbers must match EK3_SRC1 / EK3_SRC2"
assert not any(n.startswith(("SERIAL", "GPS", "RNGFND")) for n in p), \
    "hardware-specific params must stay commented until the spec is known"
print("  vision on, set 2 = vision, height/heading never from vision, RC7 free, EKF failsafe LAND: OK")


class FakeAutopilot:
    """Pretends to be ArduPilot: stores params, echoes PARAM_VALUE, knows only some names."""
    def __init__(self, known):
        self.store, self.params, self.send_lock = dict(known), {}, threading.Lock()
        self.conn = self
        self.mav = self
        self.target_system = self.target_component = 1

    def param_request_read_send(self, sys_id, comp, name, index):
        name = name.decode()
        if name in self.store:
            self.params[name] = self.store[name]

    def param_set_send(self, sys_id, comp, name, value, ptype):
        name = name.decode()
        if name in self.store:
            self.store[name] = value
            self.params[name] = value


print("\nTest 2: loader against a pretend autopilot")
fake = FakeAutopilot({"VISO_TYPE": 0.0, "EK3_SRC2_POSXY": 6.0})
wanted = [("VISO_TYPE", 1.0), ("EK3_SRC2_POSXY", 6.0), ("NOT_A_PARAM", 1.0)]
rows = apply(fake, wanted, verify_only=True)
status = {r[0]: r[4] for r in rows}
print(f"  --verify : {status}")
assert status == {"VISO_TYPE": "WRONG", "EK3_SRC2_POSXY": "OK", "NOT_A_PARAM": "NOT FOUND"}
assert fake.store["VISO_TYPE"] == 0.0, "--verify must never write"
rows = apply(fake, wanted)
status = {r[0]: r[4] for r in rows}
print(f"  write    : {status}")
assert status["VISO_TYPE"] == "CHANGED" and fake.store["VISO_TYPE"] == 1.0

# ---- 3. The confidence we send to ArduPilot scales with height ----
print("\nTest 3: position noise sent in ODOMETRY vs height (quality 50, focal 416 px)")
import math
from link.drone_link import DroneLink


class Capture:
    def odometry_send(self, *args):
        self.cov = args[13]                     # pose_covariance (after 3 angular rates)


link = object.__new__(DroneLink)            # no real connection needed
link.send_lock, link.conn = threading.Lock(), type("C", (), {"mav": Capture()})()


def sent_noise(height):
    link.send_odometry(0.0, 0, 0, 0, (0, 0, 0), (0, 0, 0), 50, 0,
                       metres_per_px=None if height is None else height / 416)
    c = link.conn.mav.cov
    return math.sqrt(c[0] + c[6] + c[11])       # what ArduPilot computes from it


old = sent_noise(None)
n2, n40, n60, n100 = sent_noise(2), sent_noise(40), sent_noise(60), sent_noise(100)
print(f"  without height (Saumya's formula): {old:.2f} m")
print(f"  2 m: {n2:.2f} m   40 m: {n40:.2f} m   60 m: {n60:.2f} m   100 m: {n100:.2f} m")
print("  (Saumya's formula stays the minimum; above ~40 m the pixel-based noise takes over)")
assert abs(n2 - old) < 1e-9 and abs(n40 - old) < 1e-9, "low down, Saumya's formula must still apply"
assert old < n60 < n100, "noise must grow with height up high"
assert n100 > 2 * old
print("\nALL PARAM TESTS PASSED")
