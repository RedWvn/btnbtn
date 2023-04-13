#This is the main script where all the scripts pertaining to the button flow gets integrated. 
"""ALL THE IMPORTS"""
"""PYMAVLINK IMPORT"""
from pymavlink import mavutil , mavwp
import time
import waypoint, cmd_msg

'''
HARDWARE LIBRARIES
'''

"""END OF IMPORTS """
'''
PYMAVLINK SKELETON START
'''
# Create the connection
device = '/dev/ttyAMA0'
baudrate = 115200

print('Connecting to ' + device + '...')
vehicle = mavutil.mavlink_connection(device, baud=baudrate)

vehicle.wait_heartbeat()

wp = mavwp.MAVWPLoader()

master = vehicle    #copy the object


'''
PYMAVLINK SKELETON END
'''

"""
Flow in steps
Communication Part : FCU <--> CC
1. Get location 
    1.1 Compare the lat-long with the existing lat-long
    1.2 If within the threshold PROCEED 
    1.3 If not break and throw error 
2. Get Battery Status 
    2.1 If not used MAH greater than threshold PROCEED
    2.2 If greater then break and throw error

PROCEED:
3. Reset Mission 
4. Upload Mission 
5. Switch Mode to AUTO

Hardware Part : 
1. Start STROBE LED BLUE from STEP 1 to 5 
    1.1 If not PROCEED FLASH LED RED
2. AT the end of STEP 4 BUZZER ON For 5 SEC and OFF

AIRCRAFT Is Ready
--- AFTER PAYLOAD SWAP ---
3. Button HOLD 
 3.1 BUZZER ON and LED STROBE for 30sec

"""