from plutoArUco import *
from plutopy import plutoDrone

# Creating a plutoDrone instance, which allows basic controls of drone, 
# through MSP Communication
drone = plutoDrone()

# Set drone to throttle mode, as we will be using manual PID through ArUco Detection
drone.control.throttleMode()

# Starting MSP Communication with drone
drone.start()

# Creating an ArUco Control instance, that will perform PID on drone
aruco = plutoArUco(drone, targetID = 5)
# This should launch a camera feed.

# Now, place the drone physically to a desired point, and set it as origin
aruco.setOrigin()
print(aruco.origin)

# Now that origin is set, we can set TARGET for drone

# So, for altitude hold, setting target:
target = [
    0,
    0,
    105 # cm, Roughly 3.5 feet (3.5*30 = 105)
]
# Z (cm) should be height from ground (specifically, origin set initially)
aruco.setTarget(*target)

# Once initial target is set, we can take-off the drone, and start the PID
drone.control.take_off()
aruco.start() # Starts the PID script

# PRESS 'Q' or 'q' to stop camera feed, and disconnect the drone.
_ = input()
drone.control.land()
aruco.stop()