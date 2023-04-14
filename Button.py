#This is the main script where all the scripts pertaining to the button flow gets integrated. 
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
device = '/dev/ttyAMA0'
baudrate = 115200

print('Connecting to ' + device + '...')
vehicle = mavutil.mavlink_connection(device, baud=baudrate)

vehicle.wait_heartbeat()

wp = mavwp.MAVWPLoader()

master = vehicle    #copy the obj

from cmd_msg import *
from waypoint import *

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
6. Flash the message to the GCS Hud

Hardware Part : 
1. Start STROBE LED BLUE from STEP 1 to 6 
    1.1 If not PROCEED FLASH LED RED
2. AT the end of STEP 4 BUZZER ON For 5 SEC and OFF

AIRCRAFT Is Ready i.e. after step 6
--- AFTER PAYLOAD SWAP ---
3. Button HOLD 
 3.1 BUZZER ON and LED STROBE for 30sec

"""

#Mission File 
file_name = "/"

#location vars
meas_lat, meas_long = 0, 0
lat_TH, long_TH = 100, 100
abs_lat, abs_long = 23, 23

#Booleans/count
PROCEED = 0

#Battery Status
used_mah = 0
max_mah = 7000

def main_flow():

    meas_lat, meas_long = GEO_LOCATION(master)
    
    if(abs(abs_lat - meas_lat) > lat_TH and abs(abs_long - meas_long) > long_TH ):

        print("Error")
        #Do something over here

    else: PROCEED = 1

    #Get Battery Status
    used_mah = BATTERY_STATUS(master)

    if(used_mah > 7000) : print("Aircraft won't be able to make it back to the Hub") #Do something about this
    else : PROCEED += 1

    if(PROCEED == 2) : 

        #Reset Mission 
        RESET_MISSION(master)

        #Upload Mission
        UPLOAD_MISSION(master, file_name)

        #Change Mode to AUTO
        CHANGE_MODE(master, mode="AUTO")

        #Flash the Message in the GCS
        FLASH_MSG_GCS(master, "CC has uploaded the return leg succesfully!")

        #This is where the btn code goes
        

















