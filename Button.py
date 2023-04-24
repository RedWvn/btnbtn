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
device = '/dev/ttyUSB0'
baudrate = 921600

print('Connecting to ' + device + '...')
vehicle = mavutil.mavlink_connection(device, baud=baudrate)

vehicle.wait_heartbeat()

wp = mavwp.MAVWPLoader()

master = vehicle    #copy the obj

from cmd_msg import *
from waypoint import *
from rgb_btn import *
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

#Mission Dir 
#file_name = "mission_upload.txt"
mission_dir = "/home/pi/ProjectBtn/MissionFiles"
#location vars
meas_lat, meas_long = 0, 0
lat_TH, long_TH = 0.01, 0.01
abs_lat, abs_long = 12.9785676, 77.64006309999999

#Booleans/count
PROCEED = 0

#Battery Status
used_mah = 0
max_mah = 7000

def main_flow():


    #Aircraft is Disarmed at this stage!
    DISARM_THE_FCU(master)

    #Toggle the safety switch 
    #TOGGLE_SAFETY_SWITCH(master, safety_switch_armed=False) #safeety_switch_armed = True, means the outputs are armed!

    #Button Logic with neo
    btn_main()

    print("Started the backend Sequence!!!!")

    #Gets the lat and long of the current GPS measurement
    meas_lat, meas_long = GEO_LOCATION(master)
    print("Measured Lat and Long: ", meas_lat, meas_long)

    #Gets the lat and long from the last waypoint of the mission file
    mission_lat, mission_long = GET_LAST_GEO_LOCATION(master)

    print("Waypoint Lat and Long: ", mission_lat, mission_long)


    
    if(abs(mission_lat - meas_lat) > lat_TH and abs(mission_long - meas_long) > long_TH ):

        print("Error")
        print("Difference:LAT:", mission_lat - meas_lat, "LONG:", mission_long - meas_long)
        #Do something over here
        #NeoPixel to Red

    else:

        PROCEED = 1
        print(PROCEED)
        #if(abs(abs_lat - meas_lat) > lat_TH and abs(abs_long - meas_long) > long_TH ):


   
        #Get Battery Status
        # used_mah = BATTERY_STATUS(master)

        # if(used_mah > 7000) : print("Aircraft won't be able to make it back to the Hub") #Do something about this
        # else : PROCEED += 1
        
        


        if(PROCEED == 1) : 

            #Reset Mission 
            RESET_MISSION(master)

            #Upload Mission
            mission_file = COMPARE_LAT_LON_WITH_MISSION_FILES(meas_lat, meas_long, mission_dir, 0.0001)
            if mission_file != None:
                print("Return Mission Found, Uploading the mission")
                #neopixel Yellow Color Here
                UPLOAD_MISSION(master, mission_file)
                #neopixel Green Color Here


                #Change Mode to AUTO
                CHANGE_MODE(master, mode="AUTO")

                #Auto Mode Indication - Rainbow Dance
                rainbow_cycle(0.01)

                #Flash the Message in the GCS
                FLASH_MSG_GCS(master, "CC has uploaded the return leg succesfully!")

                #Arming...
                time.sleep(2)
                flag = TOGGLE_SAFETY_SWITCH(master, safety_switch_armed=True)

                ARM_THE_FCU(master, flag)
                
                #Flash the Message in the GCS
                FLASH_MSG_GCS(master, "FCU ARMED!")

                #This is where the btn code goes

            else:
                print("Return Leg Mission not found!")
                #neopixel Red Color Here
                #Force


main_flow()














