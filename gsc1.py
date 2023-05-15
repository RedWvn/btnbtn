from pymavlink import mavutil

# Create a MAVLink connection
master = mavutil.mavlink_connection('/dev/ttyUSB0', baud=921600)

# Wait for the heartbeat message to start the connection
master.wait_heartbeat()



def READ_THE_MESSAGE(master):

        # Wait for the heartbeat message to find the system ID
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.get_type() == 'HEARTBEAT':
            system_id = msg.get_srcSystem()
            break

    # Set the message rate to listen for STATUSTEXT messages
    master.mav.request_data_stream_send(system_id, 0, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # Listen for STATUSTEXT messages and print them to the console
    while True:
        msg = master.recv_match(type='STATUSTEXT', blocking=True)
        if msg is not None:
            print("Received message: " + msg.text.decode("utf-8"))


READ_THE_MESSAGE(master)

# # Print all GCS messages received
# while True:
#     try:
#         msg = master.recv_match(type=['THROTTLE_ARMED'], timeout=1)
#         if not msg:
#             continue
#         print(msg.to_dict())
#     except KeyboardInterrupt:
#         break

