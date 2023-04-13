from pymavlink import mavutil
import time

# Create the connection
device = '/dev/ttyAMA0'
baudrate = 115200

print('Connecting to ' + device + '...')
master = mavutil.mavlink_connection(device, baud=baudrate)

master.wait_heartbeat()


#Python wrapper for the pymavlink arm api   
def ARM_THE_FCU():

    '''
      mavlink cmd to send to the FCU to ARM
    '''
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)


#Python wrapper for the pymavlink disarm api   
def DISARM_THE_FCU():

    '''
      mavlink cmd to send to the FCU to DISARM
    '''
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)


# Arm
master.arducopter_arm()
