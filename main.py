#Justin Garrard
import utils
from dronekit import VehicleMode, LocationGlobalRelative


connection_string = None
sitl = None
fname = "FlightLog.log"
utils.initLog(fname)

if len(sys.argv) > 1:
  connection_string = sys.argv[1]

configs = utils.load_json("./test.json")

waypoints = []
for wps in configs['waypoints']:
  waypoints.append(LocationGlobalRelative(wps[0], wps[1], wps[2]))


if not connection_string:
  sitl, connection_string = utils.connect_SITL(fname)
  vehicle = utils.connect_Vehicle(connection_string, fname)
else:
  vehicle = dronekit.connect(connection_string, baud=57600, wait_ready=False)


utils.arm_and_takeoff(vehicle, 20)
utils.printLog("We took off", fname)

home = vehicle.location.global_relative_frame
waypoints.append(home)

for wp in waypoints:
  utils.fly_to(1, vehicle, wp, fname)
  print "Finished waypoint {}".format(wp)

utils.printLog("Done flying", fname)

vehicle.mode =  VehicleMode("LAND")
utils.printLog("Landing", fname)

vehicle.close()
if sitl is not None:
  sitl.stop()



