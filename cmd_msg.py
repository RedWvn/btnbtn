from pymavlink import mavutil

# Create the connection
#device = '/dev/ttyAMA0'
#baudrate = 115200

#print('Connecting to ' + device + '...')
#vehicle = mavutil.mavlink_connection(device, baud=baudrate)

#vehicle.wait_heartbeat()

#master = vehicle

#Python wrapper for the pymavlink arm api   
def ARM_THE_FCU(master):

    '''
      mavlink cmd to send to the FCU to ARM
    '''
    master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1, 0, 0, 0, 0, 0, 0)

    print("ARMED_VEHICLE")


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

    print("DISARMED_VEHICLE")


def CHANGE_MODE(master, mode):

    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    print("Mode : ", mode)

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