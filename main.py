#!/usr/bin/env python
#Justin Garrard
#Homework 4

import dronekit_sitl
import dronekit
import json
import os
import threading, Queue
import time
import sys
import flight_utils 
import classtest
from ground_control_station import SimpleGCS

#parse option for dronology
if len(sys.argv) != 2:
  print "Improper use!\n./main.py <1-6> <1-5> \n\tspecify test file and number of UAVs!"
  sys.exit() 

avoid = 0

testFile = "./homework_testcases/test{}.json".format(sys.argv[1])
config = flight_utils.load_json(testFile)
#Maybe later config file will be a list of areas to search but for now nah
config = config[0]

numUAV = len(config["starts"])
print "{} UAVs".format(numUAV)
log_name = "SearchBox.log"
flight_utils.initLog(log_name)
flight_utils.printLog("{} UAVs found in config file".format(numUAV), log_name)
#bounds = config["bounds"]
lLat = classtest.getsouth()
hLat = classtest.getnorth()
lLon = classtest.geteast()
hLon = classtest.getwest()

# These are the waypoints each drone must go to!
routes = flight_utils.gen_search(lLat, hLat, lLon, hLon, numUAV, log_name)
flight_utils.printRoute(routes)

#This should end the dronology thread
global DO_CONT
DO_CONT = 0

# A list of drones. (dronekit.Vehicle)
vehicles = []

ARDUPATH = "/home/uav/git/ardupilot"
gcs = SimpleGCS(ARDUPATH)
gcs.connect()

##Connect to dronology
# Start up all the drones specified in the json configuration file
for i, start in enumerate(config['starts']):
    	home = start 
   	name = "UAV-" + str(i)

	vehicle = gcs.registerDrone(home, name)
    	vehicles.append(vehicle)
    	vehicle_id = str("UAV-" + str(i))

#Have each vehicle launch to altitude before beginning to fly
flight_utils.mult_arm_and_takeoff(vehicles, [10] * len(vehicles))

#Begin flight for all vehicles
speed = 10
dist = 1
threads = flight_utils.threaded_launch(vehicles, routes, speed, dist, lLat, hLat, lLon, hLon, log_name)

min_separation = 6

box_lat, box_lon = classtest.getboxcoordinates()

#Thread to write vehicles locations to file
log_thread = flight_utils.log_locs(args=(vehicles, log_name,flight_utils.Location(box_lat, box_lon),))
log_thread.start()

nearUAV = -1
while 1:
	#Check if we've already gotten rid of all UAV threads
    	if len(threads.keys()) == 0:
		break

	#Monitor UAV threads
      	for k in threads.keys():
		#If a thread has finished running, remove the corresponding vehicle
		if not threads[k].isAlive():
	  	  #We should probably kill the vehicles from the list around here too
	  	  print "{} has finished".format(k)
	  	  del threads[k]
	
	nearUAV = flight_utils.check_box(vehicles, flight_utils.Location(box_lat, box_lon))
	if nearUAV > -1:
	  closest, target = flight_utils.drone_found(threads, vehicles, nearUAV, log_name)
	  #print "Closest: {}".format(closest) 
	  #print "Targe: {}".format(target) 
	  #while flight_utils.get_distance_meters(flight_utils.Location(target[0],target[1]), flight_utils.get_location(vehicles[closest]), None) > 10:
	  	#print flight_utils.get_distance_meters(flight_utils.Location(target[0],target[1]), flight_utils.get_location(vehicles[closest]), None)
	#	time.sleep(1)
	#  print "Cleaning up!"
	  flight_utils.printLog("time-{}\n\tUAV-{} arrived at box!".format(closest), log_name)
	  time.sleep(25)
	  flight_utils.printLog("time-{}\n\tUAV-{} and UAV-{} returning home".format(nearUAV, closest), log_name)
	  
	#If we have more than 1 UAV we check for collisions
	#if(len(vehicles) > 1):
	#  flight_utils.check_collisions(vehicles, threads, avoid, min_separation, log_name)

log_thread.stop()
print "Execution completed"
DO_CONT = 0

