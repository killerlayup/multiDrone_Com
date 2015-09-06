#!/usr/bin/python
"""
Adaptive control based on relative position to keep two quadrotors mataining a constant distance. 
Use gps position of another quadrotors which is sent by zigbee.  
@author: Peng Cheng
@date: 2015/9/5 18:34 
		
"""
from droneapi.lib import VehicleMode, Location
from pymavlink import mavutil
import time
import serial

from leaderControl import leaderControl

# function: record currrent time(s)
current_milli_time = lambda: int(time.time() * 1000)

# First get an instance of the API endpoint
api = local_connect()
# Get the connected vehicle (currently only one vehicle can be returned).
vehicle = api.get_vehicles()[0]

# function: takeoff to the target height
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

# function: controlling vehicle movement using velocity
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)
    vehicle.flush()

# function: decode the received datas
def decode_position(rec_str):
	latStr = ""
	lonStr = ""
	i = 1
	if rec_str[0] == "a":
		while i < len(rec_str):
			if rec_str[i] == "o":
				break
			latStr += rec_str[i]
			i += 1
		i += 1
		while i < len(rec_str):
			if rec_str[i] == "\n":
				break
			lonStr += rec_str[i]
			i +=1

	lat = float(latStr)
	lon = float(lonStr)
	return lat, lon


# Get Vehicle Home location ((0 index in Vehicle.commands)
print "Get home location" 
cmds = vehicle.commands
cmds.download()
cmds.wait_valid()
print " Home WP: %s" % cmds[0]

arm_and_takeoff(3.5)

'''
After the vehicle reaches a target height, do other things
'''

# open the serial port between odroid and zibgee
myserial = serial.Serial('/dev/ttyUSB1', 115200, timeout=0.5)  # read timeout is 0.5s
print myserial.portstr

if myserial.isOpen():

    '''initialise datas'''
    receivedDatas = ""
	delt_T = 1.0  #1.0s
	lastRecord = current_milli_time()
	#leader object 
	leader = leaderControl(0.5, 0.5, vehicle.location.lat, vehicle.location.lon)  #vx0 = 0.5, vy0 = 0.5, x0 = vehicle.location.lat, y0 = vehicle.location.lon
	loop_cnt = 1

    '''control loop'''
    while not api.exit:
    	if v.mode.name != "GUIDED":
			print "User has changed fight mode, aborting loop!"
	        break
    	if current_milli_time() - lastRecord >= 1000:  #1000ms
    		lastRecord = current_milli_time()
    		print "[%s] current time is: %f" % (loop_cnt, lastRecord)
    		loop_cnt += 1

    		# read
    	    receivedDatas = myserial.readline()
    	    neighbourLat, neighbourLon = decode_position(receivedDatas)
    	    print "neighbourLat: " + neighbourLat
    	    print "neighbourLon: " + neighbourLon

    	    # write
    	    myserial.write('a'+str("%.8f" % vehicle.location.lat))
			myserial.write('o'+str("%.8f" % vehicle.location.lon))

			# control
			leader.x = vehicle.location.lat
			leader.y = vehicle.location.lon
			leader.controller(leader.x, neighbourLat, leader.y, neighbourLon, 0.00144)

			if leader.vx >= 4: #speed protection
				leader.vx = 4
			elif leader.vx <= -4:
				leader.vx = -4
			if leader.vy >= 4:
				leader.vy = 4
			elif leader.vy <= -4:
				leader.vy = -4

			send_ned_velocity(leader.vx, leader.vy, 0)  #vz = 0.0

    '''finished and landing'''
	vehicle.mode = VehicleMode("LAND") 
    	
else:
    print 'serial port do not open!'
    self.closeCon()
    time.sleep(2)
    vehicle.mode = VehicleMode("LAND")