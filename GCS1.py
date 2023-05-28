from pymavlink import mavutil
from rgb_btn import *
# Create the connection
# device = '/dev/ttyUSB0/'
# baudrate = 921600
device = 'tcp:127.0.0.1:5760'  #Connection to the hardware
#100.76.176.16
#device = 'tcp:100.76.176.16:5770'
print('Connecting to ' + device + '...')
vehicle = mavutil.mavlink_connection(device)

vehicle.wait_heartbeat()

print("Connected with TARGET SYSID = ", vehicle.target_system)

from cmd_msg import *
from waypoint import *

def recv_msg_gcs(master):

    # Set up a listener to receive GCS messages
    while True:
        # Check for incoming messages from autopilot
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=1.0)

        # If a STATUSTEXT message is received, print the message text
        if msg is not None:
            print(msg.text)

            if msg.text == "Land complete": break
            
        # Do other things here as needed
        time.sleep(0.1) # Delay to avoid consuming too much CPU time

def batt_status(master):

    # Set up a listener to receive GCS messages
    while True:
        # Check for incoming messages from autopilot
        msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=1.0)

        # If a STATUSTEXT message is received, print the message text
        if msg is not None:
            print(msg.current_battery)
           
        # Do other things here as needed
        time.sleep(0.1) # Delay to avoid consuming too much CPU time

def HDOP_RESET(master):

    while True:

        msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
        if msg is not None:
            print(msg.eph/100)

        return msg.eph/100

#recv_msg_gcs(vehicle)
print("Mission End!")
hdop_val = HDOP_RESET(vehicle)

if(hdop_val > 0):
        for i in range(0,50):
            neo_pixel_color(200,0,0)
            time.sleep(0.2)
            neo_pixel_color(0,50,10)
            time.sleep(0.2)