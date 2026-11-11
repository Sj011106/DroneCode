# fake_vision.py
# Pretends to be a camera: sends the drone its position over MAVLink,
# so the drone can hold position WITHOUT GPS.
#
# Where does the "fake camera" position come from?
#   The simulator knows the drone's TRUE position (SIMSTATE message).
#   We take that, add a little random noise (real cameras are noisy),
#   and send it back as if our vision system measured it.
#   Later, we replace this with the real output of ground_lock.py.
#
# Run Mission Planner's simulator FIRST, with the drone still ON THE GROUND.
# Then:  py fake_vision.py
#
# Keys (click the terminal first):
#   v = switch the drone to use OUR vision position (EKF source set 2)
#   g = switch back to GPS (EKF source set 1)
#   q = quit (vision stops -> drone will lose position!)

import math
import random
import time
import msvcrt                      # Windows-only: lets us read key presses
from pymavlink import mavutil

NOISE_M = 0.02      # 2 cm of random noise, like a decent camera
SEND_HZ = 20        # send 20 position updates per second

# ---------- 1. Connect ----------
print("Connecting...")
drone = mavutil.mavlink_connection("tcp:127.0.0.1:5762")
drone.wait_heartbeat()
print("Connected!")

# Ask for all data at 20 Hz (we need SIMSTATE and LOCAL_POSITION_NED)
drone.mav.request_data_stream_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 20, 1)

# Also ask specifically for LOCAL_POSITION_NED (message id 32) 10 times a second
drone.mav.command_long_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0, 32, 100000, 0, 0, 0, 0, 0)     # 100000 microseconds = 10 Hz


def set_source(n):
    """Tell the EKF which source set to use: 1 = GPS, 2 = our vision."""
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET,
        0, n, 0, 0, 0, 0, 0, 0)
    print(f"\n>>> Asked drone to use source set {n} ({'GPS' if n == 1 else 'VISION'})")


# ---------- 2. Remember the starting point (our "hold point" / origin) ----------
print("Waiting for the simulator's true position...")
first = drone.recv_match(type="SIMSTATE", blocking=True, timeout=10)
if first is None:
    print("No SIMSTATE received. Is the simulator running?")
    raise SystemExit
lat0 = first.lat / 1e7          # degrees
lon0 = first.lng / 1e7
print(f"Origin set at lat {lat0:.7f}, lon {lon0:.7f}")

METRES_PER_DEG_LAT = 111320.0                              # 1 degree north ~ 111 km
METRES_PER_DEG_LON = 111320.0 * math.cos(math.radians(lat0))  # shrinks away from equator

latest_sim = first     # newest true position from the simulator
latest_ekf = None      # newest position the DRONE believes (from its EKF)
last_send = 0.0

print("Sending vision positions. Keys: v = use vision, g = use GPS, q = quit\n")

# ---------- 3. Main loop ----------
while True:
    # Read every message waiting for us, keep the newest of each kind
    while True:
        msg = drone.recv_match(blocking=False)
        if msg is None:
            break
        if msg.get_type() == "SIMSTATE":
            latest_sim = msg
        elif msg.get_type() == "LOCAL_POSITION_NED":
            latest_ekf = msg
        elif msg.get_type() == "COMMAND_ACK" and msg.command == mavutil.mavlink.MAV_CMD_SET_EKF_SOURCE_SET:
            # The drone's reply to our "switch source" request
            result = mavutil.mavlink.enums["MAV_RESULT"][msg.result].name
            print(f"\n<<< Drone replied to source switch: {result}")
        elif msg.get_type() == "STATUSTEXT":
            # Show the drone's messages (same as the Messages tab) in our terminal
            print(f"\n[drone] {msg.text}")

    now = time.time()
    if now - last_send >= 1.0 / SEND_HZ:
        last_send = now

        # True position -> metres North/East of the origin
        north = (latest_sim.lat / 1e7 - lat0) * METRES_PER_DEG_LAT
        east = (latest_sim.lng / 1e7 - lon0) * METRES_PER_DEG_LON

        # Add camera-like noise
        north += random.gauss(0, NOISE_M)
        east += random.gauss(0, NOISE_M)

        # Send it as a "vision" position (NED frame, metres).
        # z = 0: height comes from the barometer, not from us.
        drone.mav.vision_position_estimate_send(
            int(now * 1e6),               # timestamp in microseconds
            north, east, 0.0,             # x = North, y = East, z (ignored)
            latest_sim.roll, latest_sim.pitch, latest_sim.yaw)

        # Show what we sent vs what the drone believes
        if latest_ekf is not None:
            drone_text = f"Drone thinks N={latest_ekf.x:6.2f} E={latest_ekf.y:6.2f} H={-latest_ekf.z:5.2f}"
        else:
            drone_text = "Drone thinks: (no position message yet)"
        print(f"\rWe sent N={north:6.2f} E={east:6.2f}   |   {drone_text}   ", end="")

    # Keyboard controls
    if msvcrt.kbhit():
        key = msvcrt.getwch().lower()
        if key == "v":
            set_source(2)
        elif key == "g":
            set_source(1)
        elif key == "q":
            print("\nStopping. The drone no longer gets vision data!")
            break

    time.sleep(0.005)   # tiny pause so we don't use 100% CPU