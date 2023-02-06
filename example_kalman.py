from plutopy import plutoDrone
from pluto_aruco1 import *
from time import perf_counter_ns as nowtime

drone = plutoDrone()

drone.start()

aruco = plutoArUco(drone, targetID = 85)
# This should launch a camera feed.

# Now, place the drone physically to a desired point, and set it as origin
aruco.setOrigin()
print(aruco.origin)

# Now that origin is set, we can set TARGET for drone
zvar = 0
zaccelvar = 0
# So, for altitude hold, setting target:
target = [
    aruco.origin.X,
    aruco.origin.Y,
    30 # cm, Roughly 3.5 feet (3.5*30 = 105)
]
# Z (cm) should be height from ground (specifically, origin set initially)
aruco.setTarget(*target)
aruco.kal = True
aruco.configureKalman(zvar,zaccelvar,1.0)
# Once initial target is set, we can take-off the drone, and start the PID
drone.control.take_off()
aruco.start() # Starts the PID script

# Now, wait until user presses 'Enter' key, then stop all code, and close the socket.
_ = input()
drone.control.land()
aruco.stop()




