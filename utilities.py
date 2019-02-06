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

################################################################################################
#Logging Utilities
################################################################################################
def printLog(msg, fname):
  fh = open(fname, 'a')
  fh.write(msg)
  fh.write('\n')
  fh.close()

def initLog(fname):
  fh = open(fname, 'w')
  fh.close()



################################################################################################
# ARM and TAKEOFF
################################################################################################

# function:   	arm and takeoff
# parameters: 	target altitude (e.g., 10, 20)
# returns:	n/a

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

    if fname:
    	printLog("\tDistance (meters): {}".format(distance, locationA), fname)
    return distance


################################################################################################
# Threaded Fly to
################################################################################################
class thread_fly_to(threading.Thread):
    def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
	threading.Thread.__init__(self, group=group, target=target, name=name, verbose=verbose)
	self.vehicle = args[0]
	self.route = args[1]
	self.groundspeed = args[2]
	self.dist = args[3]
	self.fname = args[4]
	self.q = args[5]
    	self.kwargs = kwargs
	self.interrupt = 0
	self.crash = 0
	return

    # Fly to
    def fly_to(self, targetLocation):
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
#	return

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
	  #Somehow I'm getting here with an empty list
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

def crash(thread, other):
  thread[0].go_down(other)
  return
