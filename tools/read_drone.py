# tools/read_drone.py
# Connects to the simulated drone and prints where it thinks it is.
# Run Mission Planner's simulator FIRST, then: py -m tools.read_drone

# -------------------------------------------------------------------------------------
# IN PLAIN WORDS (for non-technical readers)
#   The simplest possible connection test: connect to the (simulated) drone and print
#   where it thinks it is. If this works, the MAVLink link is fine.
# REFERENCES
#   - pymavlink: https://mavlink.io/en/mavgen_python/
# -------------------------------------------------------------------------------------

from pymavlink import mavutil

# The simulator opens extra ports for other programs. Mission Planner uses 5760,
# so we use 5762 (a second connection to the same virtual drone).
print("Connecting...")
drone = mavutil.mavlink_connection("tcp:127.0.0.1:5762")

# Wait until the drone says hello (a "heartbeat" message)
drone.wait_heartbeat()
print(f"Connected to drone (system {drone.target_system})")

# Ask the drone to send us all its data, 10 times per second
drone.mav.request_data_stream_send(
    drone.target_system, drone.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

while True:
    # Wait for the next position message
    msg = drone.recv_match(type="LOCAL_POSITION_NED", blocking=True)
    # NED = North, East, Down (in metres from the takeoff point).
    # Down is negative when flying, so height = -z.
    print(f"North: {msg.x:6.2f} m   East: {msg.y:6.2f} m   Height: {-msg.z:6.2f} m")