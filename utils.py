from dronekit import connect, VehicleMode, LocationGlobalRelative
from math import pi, sin, cos, sqrt, atan2, radians
import time
import os
import json

##############################################################################
#Start SITL if no connection string specified
##############################################################################
def connect_SITL(fname):
    import dronekit_sitl

    #connection_string = sitl.connection_string()
    ardupath ="/home/uav/git/ardupilot"
    home = "41.714870,-86.240956,221,0"
    print 'home: {}'.format(home)
    printLog('home: {}'.format(home), fname)
    sitl_defaults = os.path.join(ardupath, 'Tools', 'autotest', 'default_params', 'copter.parm')
    sitl_args = ['-I{}'.format(0), '--home', home, '--model', '+', '--defaults', sitl_defaults]
    sitl = dronekit_sitl.SITL(path=os.path.join(ardupath, 'build', 'sitl', 'bin', 'arducopter'))
    sitl.launch(sitl_args, await_ready=True)

    tcp, ip, port = sitl.connection_string().split(':')
    port = str(int(port) + 0 * 10)
    connection_string = ':'.join([tcp, ip, port])
    return sitl, connection_string

def connect_Vehicle(connection_string, fname):
    # Connect to the Vehicle
    printLog('Connecting to vehicle on: %s' % connection_string, fname)
    vehicle = connect(connection_string, wait_ready=True)
    return vehicle


##############################################################################
# ARM and TAKEOFF
##############################################################################
def arm_and_takeoff(vehicle, aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print "Basic pre-arm checks"
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print " Waiting for vehicle to initialise..."
        time.sleep(1)

    print "home: " + str(vehicle.location.global_relative_frame.lat)

    print "Arming motors"
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print " Waiting for arming..."
        time.sleep(1)

    print "Taking off!"
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude


    while True:
        #Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*.95:
            print "Reached target altitude"
            break
        time.sleep(1)

##############################################################################
# GET DISTANCE BTWN 2 POINTS 
##############################################################################
def get_distance_meters(locationA, locationB):
    # approximate radius of earth in km
    R = 6373.0

    lat1 = radians(locationA.lat)
    lon1 = radians(locationA.lon)
    lat2 = radians(locationB.lat)
    lon2 = radians(locationB.lon)

    dlon = lon2 - lon1
    dlat = lat2 - lat1

    a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    distance = (R * c) * 1000

    return distance

##############################################################################
#Logging Utilities
##############################################################################
def printLog(msg, fname):
    fh = open(fname, "a")
    fh.write(msg+"\n")
    fh.close()

def initLog(fname):
    fh = open(fname, "w")
    fh.close()

##############################################################################
# ARM and TAKEOFF
##############################################################################
def load_json(path2file):
  d = None
  try:
    with open(path2file) as f:
	d = json.load(f)
  except Exception as e:
    exit("Invalid path or malformed json: {}".format(e))

  return d

##############################################################################
#Fly to a point
##############################################################################
def fly_to(dist, vehicle, targetLocation, fname):
  if targetLocation.lat > 41.715368 or targetLocation.lat < 41.714310:
    printLog("Latitude out of bounds", fname)
    return -1

  if targetLocation.lon > -86.239746 or targetLocation.lon < -86.243882:
    printLog("Longitude out of bounds", fname)
    return -1

  vehicle.groundspeed = 10
  vehicle.simple_goto(targetLocation)
  printLog("Flying to {}".format(targetLocation), fname)

  remDistance = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
  printLog("\t{} meters away from target -> {}".format(remDistance, vehicle.location.global_relative_frame), fname)

  while vehicle.mode.name == "GUIDED":
    if remDistance < dist:
	printLog("Arrived at waypoint\n", fname)
	break
    time.sleep(1)
    remDistance = get_distance_meters(targetLocation, vehicle.location.global_relative_frame)
    printLog("\t{} meters away from target -> {}".format(remDistance, vehicle.location.global_relative_frame), fname)


