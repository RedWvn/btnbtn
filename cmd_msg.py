from pymavlink import mavutil

import time 
# Create the connection
#device = '/dev/ttyAMA0'
#baudrate = 115200

#print('Connecting to ' + device + '...')
#vehicle = mavutil.mavlink_connection(device, baud=baudrate)

#vehicle.wait_heartbeat()

#master = vehicle




##Added Cmd Ack 
##Added Getting the last waypoint from the waypoint file 


################################CMD_APIs########################################################### 

#Python wrapper for the pymavlink arm api   
def ARM_THE_FCU(master, flag):

    '''
      mavlink cmd to send to the FCU to ARM

      Additional safety switch to avoid false arming the vehicle.
      Simulates the GCS behaviour.
    '''
    if flag:
        master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1, 0, 0, 0, 0, 0, 0)

        #The below code waits for the acknowledgement 
        while True:
            msg = master.recv_match(type=['COMMAND_ACK'])
            if msg:
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == 0:
                    print("FCU is armed")
                    break
        return 1
    
    else:
        return 0


#Python wrapper for the pymavlink disarm api   
def DISARM_THE_FCU(master):

    '''
      mavlink cmd to send to the FCU to DISARM
    '''
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    0, 0, 0, 0, 0, 0, 0)

    while True:
        msg = master.recv_match(type=['COMMAND_ACK'])
        if msg:
            if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM and msg.result == 0:
                print("FCU is disarmed")
                break


def CHANGE_MODE(master, mode):

    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    while True:
        msg = master.recv_match(type=['COMMAND_ACK'])
        if msg:
            if msg.command == mavutil.mavlink.MAVLINK_MSG_ID_SET_MODE and msg.result == 0:
                print("Mode changed to,", mode)
                break



################################MESSAGE_APIs###########################################################          

def GEO_LOCATION(master):

    result = master.location()
    return result.lat, result.lng

def BATTERY_STATUS(master):

    master.mav.request_data_stream_send(master.target_system, master.target_component, mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 10000000, 1)

# Receive and parse messages from the FCU
    while True:
        msg = master.recv_match()
        if msg:
            if msg.get_type() == 'SYS_STATUS':
                # Extract battery status information
                #voltage = msg.voltage_battery / 1000.0 # Convert to volts
                #current = msg.current_battery / 100.0 # Convert to amps
                #remaining_capacity = msg.battery_remaining # Percentage of remaining battery capacity
                remaining_mah = msg.current_consumed
                #print("Voltage: ", voltage, "V")
                #print("Current: ", current, "A")
                print("Remaining MAH: ", remaining_mah, "%")
                break  # Break out of the loop after receiving battery status once

    return remaining_mah


def TOGGLE_SAFETY_SWITCH(master, safety_switch_armed):

    """
    Toggles the safety switch by sending a MAV_CMD_DO_SET_MODE command with the
    MAV_MODE_FLAG_SAFETY_ARMED flag set or unset, depending on the desired state
    of the safety switch.
    :param safety_switch_armed: True to arm the safety switch, False to disarm it.
    :param master: A pymavlink MAVLinkConnection object connected
                               to the drone's autopilot.
    """
    # Set the custom mode field to the desired value of the safety switch
    if safety_switch_armed:
        mode = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    else:
        mode = 0

    # Send a MAV_CMD_DO_SET_MODE command with the custom mode field set to the
    # desired value of the safety switch
    master.mav.command_long_send(
        master.target_system, # Target system ID (autopilot)
        master.target_component, # Target component ID (MAV_CMD_COMPONENT_ARM_DISARM)
        mavutil.mavlink.MAV_CMD_DO_SET_MODE, # Command ID
        0, # Confirmation
        mode, # desired value of the safety switch
        0, 
        0,
        0, 
        0, 0, 0)
    
    #The below code waits for the acknowledgement 
    while True:
        msg = master.recv_match(type=['COMMAND_ACK'])
        if msg:
            if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and msg.result == 0:
                if safety_switch_armed != True:
                    print("Toggled safety switch to OFF")
                    print("Vehicle Arming is Disabled")
                    print("It is safe to go near the vehicle!")
                    return 0
                
                else: 
                    print("Toggled safety switch to ON")
                    print("Vehicle Arming is enabled")
                    return 1 

                break


