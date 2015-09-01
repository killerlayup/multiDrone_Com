
import time
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil

# First get an instance of the API endpoint
api = local_connect()
# Get the connected vehicle (currently only one vehicle can be returned).
vehicle = api.get_vehicles()[0]

def arm_and_takeoff(aTargetAltitude):
	print "Basic pre-arm checks"
	if vehicle.mode.name == "INITIALISING":
		print "Waiting for vehicle to initialise"
		time.sleep(1)
	while vehicle.gps_0.fix_type < 2:
		print "Waiting for GPS........", vehicle.gps_0.fix_type
		time.sleep(1)

	print "Arming motors"

	vehicle.mode = VehicleMode("GUIDED")
	vehicle.armed = True
	vehicle.flush()

	while not vehicle.armed and not api.exit:
		print "Waiting for arming..."
		time.sleep(1)

	print "Take off!"
	Vehicle.commands.takeoff(aTargetAltitude)
	vehicle.flush()

	#Wait until the vehicle reaches a safe height
	while not api.exit:
		print "Altitude: ", vehicle.location.alt
		if vehicle.location.alt >= aTargetAltitude*0.95:
			print "Reached target altitude"
			break;
		time.sleep(1)

# Get Vehicle Home location ((0 index in Vehicle.commands)
print "Get home location" 
cmds = vehicle.commands
cmds.download()
cmds.wait_valid()
print " Home WP: %s" % cmds[0]

arm_and_takeoff(3)

#After the vehicle reaches a target height, do other things

print "Going to first point..."
point1 = Location(-35.361354, 149.165218, 3, is_relative=True)
vehicle.commands.goto(point1)
vehicle.flush()

# sleep so we can see the change in map
time.sleep(10)

print "Going to second point..."
point2 = Location(-35.363244, 149.168801, 3, is_relative=True)
vehicle.commands.goto(point2)
vehicle.flush()

# sleep so we can see the change in map
time.sleep(10)

print "Returning to Launch"
vehicle.mode    = VehicleMode("RTL")
vehicle.flush()