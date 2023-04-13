from pymavlink import mavutil

# Set the connection parameters
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)

# Wait for the heartbeat message to ensure that Pixhawk is connected
master.wait_heartbeat()

# Request the attitude information from Pixhawk
master.mav.request_data_stream_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_ALL,
    10,
    1
)

# Main loop to receive and process messages
while True:
    try:
        # Wait for the next message from Pixhawk
        msg = master.recv_match()

        # Check if the message is the attitude information
        if msg.get_type() == 'ATTITUDE':
            # Print the roll, pitch, and yaw values
            print('Roll:', msg.roll)
            print('Pitch:', msg.pitch)
            print('Yaw:', msg.yaw)

    except KeyboardInterrupt:
        # Stop the program if the user presses Ctrl-C
        break
