import time
from dronekit import connect


connection_string = "tcp:127.0.0.1:5763"

vehicle = connect(connection_string, wait_ready = True)


#Create a message listener using the decorator.
@vehicle.on_message('HEARTBEAT')
def listener(self, name, message):    
    print (message)

#Create a message listener using the decorator.
@vehicle.on_message('BATTERY_STATUS')
def listener(self, name, message):    
    print (message)

while True:
    time.sleep(2)

vehicle.close()
