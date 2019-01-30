#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Justin Garrard
#Homework1

"""
Modified from 3DR simple_goto.py
Then modified from SE_with_drones 02 goto.py
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
from math import pi, sin, cos, sqrt, atan2, radians, sqrt
import logging
import time
import os
import time
import datetime

from flight_plotter import Location, CoordinateLogger, GraphPlotter
from ned_utilities import ned_controller

################################################################################################
#Set up option parsing to get connection string
################################################################################################
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


################################################################################################
#Start SITL if no connection string specified
################################################################################################
if not connection_string:
    import dronekit_sitl

    #connection_string = sitl.connection_string()
    ardupath ="/home/uav/git/ardupilot"
    home = "41.714870,-86.240956,221,0"
    print 'home: {}'.format(home)
    sitl_defaults = os.path.join(ardupath, 'Tools', 'autotest', 'default_params', 'copter.parm')
    sitl_args = ['-I{}'.format(0), '--home', home, '--model', '+', '--defaults', sitl_defaults]
    sitl = dronekit_sitl.SITL(path=os.path.join(ardupath, 'build', 'sitl', 'bin', 'arducopter'))
    sitl.launch(sitl_args, await_ready=True)

    tcp, ip, port = sitl.connection_string().split(':')
    port = str(int(port) + 0 * 10)
    connection_string = ':'.join([tcp, ip, port])

    #vehicle = dronekit.connect(conn_string)
    #vehicle.wait_ready(timeout=120)

################################################################################################
# Connect to the Vehicle
################################################################################################
print 'Connecting to vehicle on: %s' % connection_string
vehicle = connect(connection_string, wait_ready=True)
print vehicle.location.global_relative_frame.lon



################################################################################################
# ARM and TAKEOFF
################################################################################################

# function:   	arm and takeoff
# parameters: 	target altitude (e.g., 10, 20)
# returns:	n/a

def arm_and_takeoff(aTargetAltitude):
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


################################################################################################
# function:    Get distance in meters
# parameters:  Two global relative locations
# returns:     Distance in meters
################################################################################################
def get_distance_meters(locationA, locationB, fname):
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

    print("Distance (meters): {}".format(distance))
    printLog("Distance (meters): {}\n".format(distance), fname)
    return distance

#################
#Logging Utility
#################
def printLog(msg, fname):
    fh = open(fname, "a")
    fh.write(msg)
    fh.close()

def initLog(fname):
    fh = open(fname, "w")
    fh.close()

###### Pt on Circle ######
def point_on_circle(radius, angle_deg, lat, lon):
  LON = (radius * cos(angle_deg * pi / 180)) + lon
  LAT = (radius * sin(angle_deg * pi / 180)) + lat
  return Location(LAT, LON)

###### Fly Circle ######
def fly_circle(radius, c_lat, c_lon, log, fname, step, dur):
  nedcontroller = ned_controller()
  angle = 200
  c = 0
  print "Flying Circle"
  printLog("Flying Circle\n", fname)
  while c <= (360/step):
	nextTarget = (point_on_circle(.00019, angle, c_lat, c_lon))
	currLoc = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
	dist_to_target = get_distance_meters(currLoc, nextTarget, fname)
	closestDist = dist_to_target
	ned = nedcontroller.setNed(currLoc, nextTarget)
	
	while dist_to_target > 1:
	  currLoc = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
	  dist_to_target = get_distance_meters(currLoc, nextTarget, fname)
	  if dist_to_target > closestDist:
	    break
	  else:
	    closestDistance = dist_to_target

	  #Needed to cleanup NED being off
	  currLoc = Location(vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
	  ned = nedcontroller.setNed(currLoc, nextTarget) 
	  #print "N: {} -- E:{} -- D:{}".format(ned.north, ned.east, ned.down)
	  nedcontroller.send_ned_velocity(ned.north, ned.east, ned.down, dur, vehicle)
	  time.sleep(1)
	  log.add_data(currLoc.lon, currLoc.lat)

	if(angle == 0):
	  angle = 360
	else:
	  angle = angle-step
	c+=1
	print angle


################################################################################################
# Fly to
################################################################################################
def fly_to(dist, vehicle, targetLocation, groundspeed, startTag, endTag, log, fname):
    text ="Flying from: " + str(vehicle.location.global_frame.lat) + "," + str(vehicle.location.global_frame.lon) + " to " + str(targetLocation.lat) + "," + str(targetLocation.lon)
    print text
    printLog(text, fname)

    vehicle.groundspeed = groundspeed
    currentTargetLocation = targetLocation
    vehicle.simple_goto(currentTargetLocation)
    remainingDistance=get_distance_meters(currentTargetLocation,vehicle.location.global_frame, fname)

    while vehicle.mode.name=="GUIDED":
        remainingDistance=get_distance_meters(currentTargetLocation,vehicle.location.global_frame, fname)
        if remainingDistance< dist:
            print "Arrived at waypoint."
            printLog("Arrived at waypoint.\n",fname)
            break;
        time.sleep(1)
	log.add_data(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat)

###### Test1 - Waypoints ######
def test1(log):
    initLog("Flight1")
    c = 1
    coords = [ [41.714530, -86.238980, 1], [41.714991, -86.238988, 1], [41.715144, -86.238984, 1], [41.715203, -86.238788, 1], [41.715144, -86.238634, 1], [41.715114, -86.238625, 1], [41.714921, -86.238647, 1], [41.714978, -86.239257, 1] ]

    start = time.time() 
    for coord in coords:
	print "Fly to waypoint {}".format(c)
	fly_to(coord[2], vehicle, LocationGlobalRelative(coord[0], coord[1], 20), 10, "GOTO-{}-START".format(c), "GOTO-{}-END".format(c), log, "Flight1")
	c+=1
    print "Fly home"
    fly_to(1,vehicle, LocationGlobalRelative(home[0], home[1], 20), 10, "GOTO-HOME", "GOTO-HOME", log, "Flight1")
    end = time.time()
    print "Flight took {} seconds".format(end-start)
    printLog("Flight took {} seconds\n".format(end-start), "Flight1")
 
###### Test2 - Waypoints/NEDCircle ######
def test2(log, fname, step, dur):
    initLog(fname)
    c = 1
    coords = [ [41.714620, -86.238841, 1], [41.714918, -86.238989, 1] ]
    start = time.time() 
    for coord in coords:
	print "Fly to waypoint {}".format(c)
	fly_to(coord[2], vehicle, LocationGlobalRelative(coord[0], coord[1], 20), 10, "GOTO-{}-START".format(c), "GOTO-{}-END".format(c), log, fname)
	c+=1
   
    fly_circle(get_distance_meters(vehicle.location.global_frame,LocationGlobalRelative(41.715002, -86.238809), fname), 41.715002, -86.238809, log, fname, step, dur)

    print "Fly home"
    fly_to(1,vehicle, LocationGlobalRelative(home[0], home[1], 20), 10, "GOTO-HOME", "GOTO-HOME", log, fname)
    end = time.time()
    print "Flight took {} seconds".format(end-start)
    printLog("Flight took {} seconds\n".format(end-start), "Flight2")
 


###### Main ######
if __name__ == '__main__':
    arm_and_takeoff(10)

    log1 = CoordinateLogger()
    log2 = CoordinateLogger()
    log3 = CoordinateLogger()
    log4 = CoordinateLogger()

    logNull = CoordinateLogger()

    home = [vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon]
    log1.add_data(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat)
    log2.add_data(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat)
    log3.add_data(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat)
    log4.add_data(vehicle.location.global_relative_frame.lon, vehicle.location.global_relative_frame.lat)
   
    #Simple waypoints, not covered in report
    test1(log1) 
    #NEDs
    test2(log2, "Flight5", 10, 1)
    test2(log3, "Flight3", 1, 1)
    test2(log4, "Flight6", 10, 4)

    print
    "Close vehicle object"
    vehicle.close()

       # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()
    
    plotter1 = GraphPlotter(log1.lat_array, log1.lon_array, logNull.lat_array, logNull.lon_array, "Longitude", "Latitude", "Flight1")
    plotter2 = GraphPlotter(log2.lat_array, log2.lon_array, logNull.lat_array, logNull.lon_array, "Longitude", "Latitude", "Flight2")
    plotter3 = GraphPlotter(log3.lat_array, log3.lon_array, logNull.lat_array, logNull.lon_array, "Longitude", "Latitude", "Flight3")
    plotter4 = GraphPlotter(log4.lat_array, log4.lon_array, logNull.lat_array, logNull.lon_array, "Longitude", "Latitude", "Flight4")
    
    plotter1.add_marker(-86.240956, 41.714870, "r")
    #Trees
    plotter1.add_marker(-86.239160, 41.714720, "g")
    plotter1.add_marker(-86.239374, 41.714794, "g")
    plotter1.add_marker(-86.238809, 41.715002, "g")
    plotter1.add_marker(-86.239362, 41.714470, "g")
    plotter1.add_marker(-86.238346, 41.714902, "g")
 
    plotter2.add_marker(-86.240956, 41.714870, "r")
    #Trees
    plotter2.add_marker(-86.239160, 41.714720, "g")
    plotter2.add_marker(-86.239374, 41.714794, "g")
    plotter2.add_marker(-86.238809, 41.715002, "g")
    plotter2.add_marker(-86.239362, 41.714470, "g")
    plotter2.add_marker(-86.238346, 41.714902, "g")
 
    plotter3.add_marker(-86.240956, 41.714870, "r")
    #Trees
    plotter3.add_marker(-86.239160, 41.714720, "g")
    plotter3.add_marker(-86.239374, 41.714794, "g")
    plotter3.add_marker(-86.238809, 41.715002, "g")
    plotter3.add_marker(-86.239362, 41.714470, "g")
    plotter3.add_marker(-86.238346, 41.714902, "g")
 
    plotter4.add_marker(-86.240956, 41.714870, "r")
    #Trees
    plotter4.add_marker(-86.239160, 41.714720, "g")
    plotter4.add_marker(-86.239374, 41.714794, "g")
    plotter4.add_marker(-86.238809, 41.715002, "g")
    plotter4.add_marker(-86.239362, 41.714470, "g")
    plotter4.add_marker(-86.238346, 41.714902, "g")
   
    plotter1.scatter_plot()
    plotter2.scatter_plot()
    plotter3.scatter_plot()
    plotter4.scatter_plot()
