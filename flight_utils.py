#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Modified from 3DR simple_goto.py
"""

from dronekit import connect, VehicleMode, LocationGlobalRelative
from math import sin, cos, sqrt, atan2, radians, sqrt, ceil
import logging
import os
import time
import datetime
import threading
import json

########################################################################
#Area search functions
########################################################################
#Might need to change these inputs to be full-fledged coordinates @ corner
def gen_search(lLat, hLat, lLon, hLon, numUAV, controlLog):
  eastDist = get_distance_meters(Location(lLat,lLon), Location(hLat,lLon), None)
  westDist = get_distance_meters(Location(lLat,hLon), Location(hLat,hLon), None)

  #These are the steps between each UAV sliver on the search area
  wstep = westDist/numUAV
  estep = eastDist/numUAV

  step5 = .000044952
  #The numPasses a UAV makes in its sliver will depend on the longer boundary length
  if wstep > estep:
	numPasses = wstep/5
	wminStep = step5 
	temp = estep/numPasses
	eminStep = temp/5 * step5
  else:
	numPasses = estep/5
	eminStep = step5
	temp = wstep/numPasses
	wminStep = temp/5 * step5

  numPasses = int(ceil(numPasses))
  wLoc = Location(hLat,lLon)
  eLoc = Location(hLat,hLon)
  routes = []
  for i in range(0,numUAV):
    route = []
    for j in range(0,numPasses):
	if j % 2 == 0:
	  route.append([wLoc.lat, wLoc.lon, 10])
	  route.append([eLoc.lat, eLoc.lon, 10])
	else:
	  route.append([eLoc.lat, eLoc.lon, 10])
	  route.append([wLoc.lat, wLoc.lon, 10])
	
	#By the time numPasses is through w/ one UAV, we should be at start coords for next UAV's search block
  	wLoc.lat = wLoc.lat - wminStep
	wLoc.lon = hLon
	eLoc.lat = eLoc.lat - eminStep
	eLoc.lon = lLon
    routes.append(route)

  return routes

class Location:
  def __init__(self, lat=0.0, lon=0.0):
	self.lat = lat
	self.lon = lon

#Return the index of the vehicle that is close to the box; else return none
def check_box(vehicles, boxLoc):
  drone = None
  for i, uav in enumerate(vehicles):
	dist = get_distance_meters(get_location(uav), boxLoc, None)
	if dist <= 3:
	  drone = i
	  break
  return drone

def drone_found(threads, vehicles, uav, controlLog):
  closest = None
  dist = 1000000
	  
  for i, vehicle in enumerate(vehicles):
    newD = get_distance_meters(get_location(vehicles[uav]), get_location(vehicle), None)
    if (newD < dist) and i != uav:
	dist = newD
	closest = i
    
  printLog("time-{}\n\tUAV-{} found the box at {}\n\tUAV-{} is closest".format(time.time(), uav, vehicles[uav].location.global_relative_frame, closest), controlLog)
  
  closeKey = "UAV-{}.log".format(closest)
  foundKey = "UAV-{}.log".format(uav)
 
  for key in threads.keys():
    if key != foundKey and key != closeKey:
	threads[key].endroute()	 
    elif key == foundKey:
  	threads[key].hover()
    elif key == closeKey:
  	threads[key].reroute([vehicles[uav].location.global_relative_frame.lat, vehicles[uav].location.global_relative_frame.lon, 10], 5)

  return closest, [vehicles[uav].location.global_relative_frame.lat, vehicles[uav].location.global_relative_frame.lon, 10]

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

def printRoute(routes):
  fh = open("routes.txt", "w")
  for i, route in enumerate(routes):
    msg = ''
    for coord in route:
	msg = msg + '{}\n\t'.format(coord)
    fh.write('\nUAV-{} route:\n\t{}'.format(i, msg))
  fh.close()
  return

def printLog(msg, fname):
  fh = open(fname, 'a')
  fh.write(msg)
  fh.write('\n')
  fh.close()

def initLog(fname):
  fh = open(fname, 'w')
  fh.close()

####Log the location of all drones in the ControlLog####
class log_locs(threading.Thread):
  def __init__(self, group=None, target=None, name=None, args=(), kwargs=None, verbose=None):
	threading.Thread.__init__(self, group=group, target=target, name=name, verbose=verbose)
	self.vehicles = args[0]
	self.log = args[1]
	self.box = args[2]
	self.interrupt = 0

  def run(self):
    while 1:
      if self.interrupt:
	break
      msg = ""
      for i, vehicle in enumerate(self.vehicles):
	msg = msg + "UAV-{}: {}\n\t".format(i, get_location(vehicle))
      printLog("time-{}\n\t{}".format(time.time(), msg), self.log)
      if(get_distance_meters(get_location(vehicle), self.box, None) < 6):
	shlep = 1
      else:
	shlep = 3
      time.sleep(shlep)
    return

  def stop(self):
    self.interrupt = 1	   
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
#Functions for managing multiple drones
#######################################################################

#Launches multiple drones, could thread later?
def mult_arm_and_takeoff(drones, alts):
  for count, drone in enumerate(drones):
	arm_and_takeoff(drone, alts[count])
  return 1

#Manage collision avoidance
def check_collisions(vehicles, threads, avoid, min_separation, log_name):
	crashedUAVs = []
	monitorThreads = {}

  	#check all vehicles for collisions
  	for uav in range(len(vehicles)):
	  #if this is the last drone, we've already checked it against everything
	  if uav == len(vehicles)-1:
	    break

	  #To make this more efficient we could remove vehicles when that vehicles thread finishes, but we get indexing issues into the vehicles list
	  for x in range(uav + 1, len(vehicles)):
	    dist = flight_utils.get_distance_meters(vehicles[uav].location.global_relative_frame, vehicles[x].location.global_relative_frame,None)
	  
	    #launch a monitor thread to see when two drones can resume their routes at regular altitude, or if we "crash" down the vehicles
	    #crash here
	    if dist <= 4:
		if uav not in crashedUAVs:
		  flight_utils.printLog("**CRASH**time-{}: UAV-{} and UAV-{} have crashed!".format(time.time(), uav, x), log_name)
		  key1 = "UAV-{}.log".format(uav)
	  	  print "UAV-{} has finished".format(uav)
		  t1 = threading.Thread(target=flight_utils.crash, args=(threads[key1], x,))
		  t1.start()
		  crashedUAVs.append(uav)
	  	  del threads[key1]
		if x not in crashedUAVs:
		  key2 = "UAV-{}.log".format(x)
	  	  print "UAV-{} has finished".format(x)
		  t2 = threading.Thread(target=flight_utils.crash, args=(threads[key2], uav,))
		  t2.start()
		  crashedUAVs.append(x)
		  del threads[key2]

	    elif dist <= min_separation and avoid:
		print "Avoid!"
		key = "{}{}".format(uav,x)
		key1 = "UAV-{}.log".format(uav)
		key2 = "UAV-{}.log".format(x)
		print "{}/{}".format(key1,key2)
		print "{}".format(threads.keys())
		if key in monitorThreads.keys():
		  if not monitorThreads[key].isAlive():
			del monitorThreads[key]
		  else:
		  	continue
		elif key1 in threads.keys() and key2 in threads.keys():
		  print("UAV-{} and UAV-{} are too close!".format(uav,x))
		  flight_utils.printLog("time-{}: UAV-{} and UAV-{} are about to crash!".format(time.time(),uav, x), log_name)
		  monitorThreads[key] = threading.Thread(target=flight_utils.monitor, args=(threads[key1], threads[key2], vehicles[uav], vehicles[x],))
		  monitorThreads[key].start()

####When central controller detects two drones are w/in 6 meters, monitor them####
def monitor(thread1, thread2, uav1, uav2):
  thread1[0].avoid(5, thread2[2])
  thread2[0].avoid(15, thread2[2])

  #Allow the uavs to avoid until a safe distance away
  while get_distance_meters(uav1.location.global_relative_frame, uav2.location.global_relative_frame, None) <= 4:
    time.sleep(1)
  thread1[0].resume()
  thread2[0].resume()
  return

####When central controller detects a drone has crashed, put it down####
def crash(thread, other):
  thread[0].go_down(other)
  return

#######################################################################
#Functions for managing multiple drones w/ threads
#######################################################################

####Launch all fly_to threads####
def threaded_launch(vehicles, routes, speed, dist, lLat, hLat, lLon, hLon, controlLog): 
  #Use a thread dictionary so they can be grabbed, and stopped
  threads = {}

  for i, vehicle in enumerate(vehicles):
  	name = "UAV-{}.log".format(i)
  	printLog("time-{}\n\tStarting {}".format(time.time(), name), controlLog)
  	initLog(name)
	threads[name] = thread_fly_to(args=(vehicle, routes[i], speed, dist, name, i, lLat, hLat, lLon, hLon,)) 
  	threads[name].start()
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
	self.number = args[5]
	self.lowLat = args[6]
	self.highLat = args[7]
	self.lowLon = args[8]
	self.highLon = args[9]
    	self.kwargs = kwargs
	self.interrupt = 0
	self.crash = 0
	self.wait = 0
	self.stop = 0
	self.rerouted = 0
	return

    # Fly to
    def fly_to(self, targetLocation):
   	c = 0
    	if (targetLocation.lat < self.lowLat or targetLocation.lat > self.highLat):
          printLog("ERROR when assigning location! - Latitude outside range!", self.fname)
          return
    	if (targetLocation.lon > self.lowLon or targetLocation.lon < self.highLon):
          printLog("ERROR when assigning location! - Longitude outside range!", self.fname)
          return

    	printLog("Flying to waypoint: {}".format(targetLocation), self.fname)
    	self.vehicle.groundspeed = self.groundspeed
    	currentTargetLocation = targetLocation
    	self.vehicle.simple_goto(currentTargetLocation)

    	while self.vehicle.mode.name=="GUIDED":
 	  if self.interrupt:
	    break
	  if self.stop:
	    break

          remainingDistance=get_distance_meters(currentTargetLocation,self.vehicle.location.global_frame, self.fname)
          if remainingDistance< self.dist:
            printLog("Reached target", self.fname)
            break
	  
	  c+=1
	  if c == 20:
	    c = 0
            printLog("Time:{}\n\tCoords: {}".format(time.time(), self.vehicle.location.global_relative_frame), self.fname)

          time.sleep(1)

#	  if plotLog:
#	    plotLog.add_data(vehicle.location.global_relative_frame.lon,vehicle.location.global_relative_frame.lat)
    	printLog("\n", self.fname)

    def reroute(self, target, dist):
	self.rerouted = 1
	self.stop = 1
	self.route = [target]
	self.dist = dist
	printLog("REROUTING to {}".format(target), self.fname)
	time.sleep(2)

    def hover(self):
	self.interrupt = 1
	self.wait = 1
	printLog("Hovering", self.fname)

    def endroute(self):
	self.interrupt = 1
	self.route = []
	printLog("Ending route, other drones found it", self.fname)

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
	  if self.wait:
	    time.sleep(25)
	    break
	  elif self.crash:
	    break
	  elif self.interrupt:
	    continue
	  elif len(self.route) > 0:
	    self.fly_to(LocationGlobalRelative(self.route[0][0],self.route[0][1], self.route[0][2]))
	    if self.interrupt:
		continue
	    self.route.pop(0)
	
	if self.rerouted:
	  printLog("Time:{}\n\tArrived at blackbox".format(time.time()), self.fname)
	  time.sleep(25)
            
    	printLog("Time:{}\n\tReturning to Launch".format(time.time()), self.fname)
    	self.vehicle.mode = VehicleMode("RTL")
    	# Close vehicle object before exiting script
    	printLog("Time:{}\n\tClosing vehicle object".format(time.time()), self.fname)
    	self.vehicle.close()
	return


