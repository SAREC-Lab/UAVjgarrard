#!/usr/bin/env python
#Justin Garrard
#Homework 3

#TODO
#Collision Avoidance
	#What happens if I send a vehicle.goto outside of the thread?
	#How to kill/restart threads, better enumeration mechanism

import dronekit_sitl
import dronekit
import json
import os
import threading, Queue
import time
import sys
import utilities
from copter import UAV_Copter

def load_json(path2file):
    d = None
    try:
        with open(path2file) as f:
            d = json.load(f)
    except Exception as e:
        exit('Invalid path or malformed json file! ({})'.format(e))

    return d

if len(sys.argv) < 2 or len(sys.argv) > 3:
  print "Improper use!\n./main.py <1-5> -ca, where 1-5 is the test file and -ca enables avoidance"
  sys.exit() 

avoid = 0
testFile = "./test{}.json".format(sys.argv[1])
if len(sys.argv) == 3:
  avoid = 1

config = load_json(testFile)
  

# A list of sitl instances.
sitls = []

# A list of drones. (dronekit.Vehicle)
vehicles = []

# A list of lists of lists (i.e., [ [ [lat0, lon0, alt0], ...] ...]
# These are the waypoints each drone must go to!
routes = []

# This is really temporary for this assignment so we can track IDs for each drone
# Next week we'll integrate with Dronology and replace it.
copters = []

# Start up all the drones specified in the json configuration file
for i, v_config in enumerate(config):
    	copter = UAV_Copter()
    	home = v_config['start']
   	vehicle, sitl = copter.connect_vehicle(i, home)
    	sitls.append(sitl)
    	vehicles.append(vehicle)
    	routes.append(v_config['waypoints'])
    	vehicle_id = str("UAV-" + str(i))
    	copter.setvalues(sitl, vehicle, v_config['waypoints'], vehicle_id)
    	copters.append(copter)

#have each vehicle launch to altitude before beginning to fly
#could be done w/ threads and joins but not as important rn 
for vehicle in vehicles: 
  	utilities.arm_and_takeoff(vehicle, 10)

#Use a thread dictionary so they can be grabbed, and stopped
threads = {}

log_name = "CA-{}Test-{}.log".format(avoid, sys.argv[1])
utilities.initLog(log_name)

for i, vehicle in enumerate(vehicles):
  	name = "UAV-{}.log".format(i)
  	utilities.printLog("{}- starting {}".format(time.time(), name), log_name)
  	q = Queue.Queue()
  	utilities.initLog(name)
	threads[name] = [utilities.thread_fly_to(args=(vehicle, routes[i], 10, 1, name, q,)), q, i]
  	threads[name][0].start()

min_separation = 6
monitorThreads = {}
crashedUAVs = []

while 1:
	msg = ""
	count = 1
	for vehicle in vehicles:
	  msg = msg + "UAV-{}: {}\n\t".format(count, vehicle.location.global_relative_frame)
  	  count += 1
	utilities.printLog("time-{}\n\t{}".format(time.time(), msg), log_name)

  	#check all vehicles for collisions
  	for uav in range(len(vehicles)):
	  #if this is the last drone, we've already checked it against everything
	  if uav == len(vehicles)-1:
	    break

	  #To make this more efficient we could remove vehicles when that vehicles thread finishes, but we get indexing issues into the vehicles list
	  for x in range(uav + 1, len(vehicles)):
	    dist = utilities.get_distance_meters(vehicles[uav].location.global_relative_frame, vehicles[x].location.global_relative_frame,None)
	  
	    #launch a monitor thread to see when two drones can resume their routes at regular altitude, or if we "crash" down the vehicles
	    #crash here
	    if dist <= 4:
		if uav not in crashedUAVs:
		  utilities.printLog("**CRASH**time-{}: UAV-{} and UAV-{} have crashed!".format(time.time(), uav, x), log_name)
		  key1 = "UAV-{}.log".format(uav)
	  	  print "UAV-{} has finished".format(uav)
		  t1 = threading.Thread(target=utilities.crash, args=(threads[key1], x,))
		  t1.start()
		  crashedUAVs.append(uav)
	  	  del threads[key1]
		if x not in crashedUAVs:
		  key2 = "UAV-{}.log".format(x)
	  	  print "UAV-{} has finished".format(x)
		  t2 = threading.Thread(target=utilities.crash, args=(threads[key2], uav,))
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
		  utilities.printLog("time-{}: UAV-{} and UAV-{} are about to crash!".format(time.time(),uav, x), log_name)
		  monitorThreads[key] = threading.Thread(target=utilities.monitor, args=(threads[key1], threads[key2], vehicles[uav], vehicles[x],))
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

 
