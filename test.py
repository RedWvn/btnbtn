"""ALL THE IMPORTS"""
"""PYMAVLINK IMPORT"""
from pymavlink import mavutil , mavwp

'''
HARDWARE LIBRARIES
'''

"""END OF IMPORTS """
'''
PYMAVLINK SKELETON START
'''
# Create the connection
# device = '/dev/ttyUSB0/'
# baudrate = 921600
#device = 'tcp:127.0.0.1:5760' 
#100.76.176.16
device = 'tcp:100.76.176.16:5760'
print('Connecting to ' + device + '...')
vehicle = mavutil.mavlink_connection(device)
print("Reached here")

while True:
    vehicle.wait_heartbeat()
    print("Connected with TARGET SYSID = ", vehicle.target_system)