from pymavlink import mavutil

# Create a MAVLink connection
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=921600)

# Wait for the heartbeat message to start the connection
master.wait_heartbeat()

# Print all GCS messages received
while True:
    try:
        msg = master.recv_match(type=['THROTTLE_ARMED'], timeout=1)
        if not msg:
            continue
        print(msg.to_dict())
    except KeyboardInterrupt:
        break

