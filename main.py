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

#parse option for dronology
if len(sys.argv) < 2 or len(sys.argv) > 3:
  print "Improper use!\n./main.py <1-5> -ca, where 1-5 is the test file and -ca enables avoidance"
  sys.exit() 

avoid = 0
testFile = "./test{}.json".format(sys.argv[1])
if len(sys.argv) == 3:
  avoid = 1

config = flight_utils.load_json(testFile)
  
# A list of sitl instances.
sitls = []

# A list of drones. (dronekit.Vehicle)
vehicles = []

# These are the waypoints each drone must go to!
routes = []

###Connect to dronology
# Start up all the drones specified in the json configuration file
for i, v_config in enumerate(config):
#    	copter = UAV_Copter()
    	home = v_config['start']
   	vehicle, sitl = copter.connect_vehicle(i, home)
    	sitls.append(sitl)
    	vehicles.append(vehicle)
    	routes.append(v_config['waypoints'])
    	vehicle_id = str("UAV-" + str(i))

log_name = "CA-{}Test-{}.log".format(avoid, sys.argv[1])
flight_utils.initLog(log_name)

#Have each vehicle launch to altitude before beginning to fly
flight_utils.mult_arm_and_takeoff(vehicles, [10] * len(vehicles))

#Begin flight for all vehicles
threads = flight_utils.threaded_launch(vehicles)

min_separation = 6
monitorThreads = {}
crashedUAVs = []

while 1:
	#Make functions out of these shits
	msg = ""
	count = 1
	for vehicle in vehicles:
	  msg = msg + "UAV-{}: {}\n\t".format(count, vehicle.location.global_relative_frame)
  	  count += 1
	flight_utils.printLog("time-{}\n\t{}".format(time.time(), msg), log_name)

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
			#change altitude

	#Check if we've already gotten rid of all UAV threads
    	if len(threads.keys()) == 0:
		break

	#Monitor UAV threads
      	for k in threads.keys():
		#If a thread has finished running, remove the corresponding vehicle
		if not threads[k][0].isAlive():
	  	  #We should probably kill the vehicles from the list around here too
	  	  print "{} has finished".format(k)
	  	  del threads[k]

for sitl in sitls:
  sitl.stop() 

 
