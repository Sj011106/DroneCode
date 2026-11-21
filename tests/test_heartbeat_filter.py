# tests/test_heartbeat_filter.py  -  only the autopilot's own heartbeat may refresh DroneLink.mode_age()
#
# IN PLAIN WORDS
#   The pilot-handover failsafe only sends a mode change when the autopilot's heartbeat is fresh. If a ground
#   station's heartbeat, or our own onboard-computer heartbeat, could refresh it, a dead autopilot would look
#   alive. This feeds made-up HEARTBEAT messages through the real filter (DroneLink.is_autopilot_heartbeat).
#
# Run (from the repo root):  python -m tests.test_heartbeat_filter

from link.drone_link import DroneLink
from pymavlink import mavutil

M = mavutil.mavlink


class Msg:
    def __init__(self, sysid, mtype, autopilot):
        self._s, self.type, self.autopilot = sysid, mtype, autopilot

    def get_srcSystem(self):
        return self._s


link = object.__new__(DroneLink)                     # no radio: only the filter is tested
link.conn = type("C", (), {"target_system": 1})()

cases = [
    ("the flight controller (quad, ArduPilot)", Msg(1, M.MAV_TYPE_QUADROTOR, M.MAV_AUTOPILOT_ARDUPILOTMEGA), True),
    ("a ground station on the same system id", Msg(1, M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_INVALID), False),
    ("a ground station that reports an autopilot", Msg(1, M.MAV_TYPE_GCS, M.MAV_AUTOPILOT_ARDUPILOTMEGA), False),
    ("our own onboard-computer heartbeat", Msg(1, M.MAV_TYPE_ONBOARD_CONTROLLER, M.MAV_AUTOPILOT_INVALID), False),
    ("another vehicle (system 2)", Msg(2, M.MAV_TYPE_QUADROTOR, M.MAV_AUTOPILOT_ARDUPILOTMEGA), False),
]
for name, msg, expect in cases:
    got = link.is_autopilot_heartbeat(msg)
    print(f"  {'ok ' if got == expect else 'BAD'} {name}: {got}")
    assert got == expect, name
print("\nALL HEARTBEAT FILTER TESTS PASSED")
