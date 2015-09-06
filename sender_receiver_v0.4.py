#
# This example shows how to use DroneKit-Python to get and set vehicle state, parameter and channel-override information. 
# It also demonstrates how to observe vehicle attribute (state) changes. 
# 
# Usage:
# * mavproxy.py
# * module load api
# * api start vehicle-state.py
#
# read some datas from pixhawk, and send it to
# the individual serial port;
# read the serial port to receive some datas
#
from droneapi.lib import VehicleMode
from pymavlink import mavutil
import time

import serial
import threading

# First get an instance of the API endpoint
api = local_connect()
# Get the connected vehicle (currently only one vehicle can be returned).
v = api.get_vehicles()[0]

# Get all vehicle attributes (state)
print "\nGet all vehicle attribute values:"
print " Location: %s" % v.location
print " Attitude: %s" % v.attitude
print " Velocity: %s" % v.velocity
print " GPS: %s" % v.gps_0
print " Groundspeed: %s" % v.groundspeed
print " Airspeed: %s" % v.airspeed
print " Mount status: %s" % v.mount_status
print " Battery: %s" % v.battery
print " Mode: %s" % v.mode.name    # settable
print " Armed: %s" % v.armed    # settable


# Set vehicle mode and armed attributes (the only settable attributes)
print "Set Vehicle.mode=GUIDED (currently: %s)" % v.mode.name 
v.mode = VehicleMode("GUIDED")
v.flush()  # Flush to guarantee that previous writes to the vehicle have taken place
while not v.mode.name=='GUIDED' and not api.exit:  #Wait until mode has changed
    print " Waiting for mode change ..."
    time.sleep(1)

print "Set Vehicle.armed=True (currently: %s)" % v.armed 
# v.armed = True
# v.flush()

# open the serial port between odroid and zibgee
myserial = serial.Serial('/dev/ttyUSB1', 115200, timeout=1.5)  # read timeout is 1.5s
print myserial.portstr
			
data=''
if myserial.isOpen():	
	# while not v.armed and not api.exit:
	while not api.exit:
		print " Waiting for arming..."
		
		# receive data
		data = myserial.readline()
		print 'read data: ' + data
		
		
		# send data
		myserial.write('#3')
		myserial.write('lat:'+str("%.3f" % v.location.lat))
		myserial.write('lon:'+str("%.3f" % v.location.lon))
		myserial.write('alt:'+str("%.3f" % v.location.alt))
		# the char of '\n' is sent by the zigbee
				
else:
    print 'serial port do not open!'
    self.closeCon()
	