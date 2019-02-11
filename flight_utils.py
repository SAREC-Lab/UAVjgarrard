#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
from math import sin, cos, sqrt, atan2, radians, sqrt
import logging
import os
import time
import datetime
import threading

########################################################################
#Logging & Config Setup Utilities
########################################################################
def load_json(path2file):
    d = None
    try:
        with open(path2file) as f:
            d = json.load(f)
    except Exception as e:
        exit('Invalid path or malformed json file! ({})'.format(e))

    return d

def printLog(msg, fname):
  fh = open(fname, 'a')
  fh.write(msg)
  fh.write('\n')
  fh.close()

def initLog(fname):
  fh = open(fname, 'w')
  fh.close()


########################################################################
#This section is utilities for managing the flight of a single drone
########################################################################

####Arm and Takeoff for Single Drone####
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

    #Wait until drone reaches target altitude
    while True:
        #Break and return from function just below target altitude.        
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*.95: 
            print "Reached target altitude"
            break
        time.sleep(1)

####Get the Coords of a Vehicle####
def get_location(vehicle):
    return vehicle.location.global_relative_frame

####Get Distance in Meters####
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

    if fname:
    	printLog("\tDistance (meters): {}".format(distance, locationA), fname)
    return distance

#######################################################################
#Functions for managing multiple drones sequentially
#######################################################################

#Launches multiple drones, could thread later?
def mult_arm_and_takeoff(drones, alts):
  for count, drone in enumerate(drones):
	arm_and_takeoff(drone, alts[count])
  return 1

	
#######################################################################
#Functions for managing multiple drones w/ threads
#######################################################################

####Launch all fly_to threads####
def threaded_launch(vehicles): 
  #Use a thread dictionary so they can be grabbed, and stopped
  threads = {}

  for i, vehicle in enumerate(vehicles):
  	name = "UAV-{}.log".format(i)
  	flight_utils.printLog("{}- starting {}".format(time.time(), name), log_name)
  	flight_utils.initLog(name)
	threads[name] = [flight_utils.thread_fly_to(args=(vehicle, routes[i], 10, 1, name, MOREARGS,)), q, i]
  	threads[name][0].start()
  return threads

####Threaded Fly to####
class thread_fly_to(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
	threading.Thread.__init__(self, group=group, target=target, name=name, verbose=verbose)
	self.vehicle = args[0]
	self.route = args[1]
	self.groundspeed = args[2]
	self.dist = args[3]
	self.fname = args[4]
	self.lowLat = args[5]
	self.highLat = args[6]
	self.lowLon = args[7]
	self.highLon = args[8]
    	self.kwargs = kwargs
	self.interrupt = 0
	self.crash = 0
	return

    # Fly to
    def fly_to(self, targetLocation):
        print("Trying to fly: checking geo-boundaries")
    if (targetLocation.lat < self.lowLat or targetLocation.lat > self.highLat):
        print("ERROR when assigning location! - Latitude outside range!")
        return
    if (targetLocation.lon < self.lowLon or targetLocation.lon > self.highLon):
        print("ERROR when assigning location! - Longitude outside range!")
        return

    	printLog("Flying to waypoint: {}".format(targetLocation), self.fname)
    	self.vehicle.groundspeed = self.groundspeed
    	currentTargetLocation = targetLocation
    	self.vehicle.simple_goto(currentTargetLocation)

    	while self.vehicle.mode.name=="GUIDED":
 	  if self.interrupt:
	    break

          remainingDistance=get_distance_meters(currentTargetLocation,self.vehicle.location.global_frame, self.fname)

          if remainingDistance< self.dist:
            printLog("Reached target", self.fname)
            break
          printLog("\tTime:{} \n\tCoords: {}".format(time.time(), self.vehicle.location.global_relative_frame), self.fname)

          time.sleep(1)

#	  if plotLog:
#	    plotLog.add_data(vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.lat)
    	printLog("\n", self.fname)

    def avoid(self, alt, other):
	self.interrupt = 1
	self.fly_to(self.vehicle.location.global_relative_frame)
	printLog("\nRunning collision avoidance w/ UAV-{}".format(other), self.fname)
	self.vehicle.groundspeed = self.groundspeed
	self.vehicle.simple_goto(LocationGlobalRelative(self.route[0][0], self.route[0][1], alt))
	time.sleep(3)
	printLog("Location: {}".format(self.vehicle.location.global_relative_frame), self.fname)

    def resume(self):
	if not self.crash:
	  printLog("Resuming execution", self.fname)
	  self.interrupt = 0
	  time.sleep(1)

    def go_down(self, other):
	printLog("Landing without grace - we crashed into UAV-{}".format(other), self.fname)
	self.interrupt = 1
	self.fly_to(self.vehicle.location.global_relative_frame)
	self.crash = 1

    def run(self):	
    	c = 1
	while len(self.route) > 0:
	  if self.crash:
	    break
	  elif self.interrupt:
	    continue
	  elif len(self.route) > 0:
	    self.fly_to(LocationGlobalRelative(self.route[0][0],self.route[0][1], self.route[0][2]))
	    if self.interrupt:
		continue
	    self.route.pop(0)
	    printLog("Shouldn't see on interrupt{}".format(self.route), self.fname)
    	printLog("Returning to Launch", self.fname)
    	self.vehicle.mode = VehicleMode("RTL")
    	# Close vehicle object before exiting script
    	printLog("Closing vehicle object", self.fname)
    	self.vehicle.close()
	return

####When central controller detects two drones are w/in 6 meters, monitor them####
def monitor(thread1, thread2, uav1, uav2):
  thread1[0].avoid(5, thread2[2])
  thread2[0].avoid(15, thread2[2])

  #Allow the uavs to avoid until a safe distance away
  while get_distance_meters(uav1.location.global_relative_frame, uav2.location.global_relative_frame, None) <= 4:
    time.sleep(1)
  print "resuming"
  thread1[0].resume()
  thread2[0].resume()
  return

####When central controller detects a drone has crashed, put it down####
def crash(thread, other):
  thread[0].go_down(other)
  return
