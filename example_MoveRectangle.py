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
aruco = plutoArUco(drone, droneID = 5)
# This should launch a camera feed.

# Now, place the drone physically to a desired point, and set it as origin
aruco.setOrigin()
print(aruco.origin)

# Now that origin is set, we can set TARGET for drone

# So, for altitude hold, setting target:
target = [
    aruco.origin.X,
    aruco.origin.Y,
    90 # cm, Roughly 3.5 feet (3.5*30 = 105)
]
# Z (cm) should be height from ground (specifically, origin set initially)
aruco.setTarget(*target)

# Once initial target is set, we can take-off the drone, and start the PID
drone.control.take_off()
aruco.start() # Starts the PID script

'''
Now, to move in a shape of rectangle (2m by 1m) (x by y),
Let's say we are in center of rectangle, initially.
We need to move to one of the corners of rectangle.
The coordinates of corner become,
(-1m, -0.5m) -> Upper Left -> (-50, -25)
(1m, -0.5m) -> Upper Right -> (50, -25)
(1m, 0.5m) -> Bottom Right -> (50, 25)
(-1m, 0.5m) -> Bottom Left -> (-50, 25)
then back to upper left.
'''
altitude = 90 # cm, Roughly 3.5 feet (3.5*30 = 105)
top_l = (-50, -25, altitude)
top_r = (50, -25, altitude)
bot_r = (50, 25, altitude)
bot_l = (-50, 25, altitude)
path = [top_l, top_r, bot_r, bot_l, top_l]
for pt in path:
    sleep(6)
    aruco.setTarget(*pt)

# Path will be center -> top_l -> top_r -> bot_r -> bot_l -> top_l

# Now, wait until user presses 'Enter' key, then stop all code, and close the socket.
_ = input()
drone.control.land()
aruco.stop()