from plutoArUco import *
from plutopy import plutoDrone

drone = plutoDrone() 

aruco = plutoArUco(drone, targetID = 85)

#aruco.debug = True

aruco.setOrigin()
print(aruco.origin)

target = [
    aruco.origin.X,
    aruco.origin.Y,
    20 # cm, Roughly 3.5 feet (3.5*30 = 105)
]
aruco.setTarget(*target)

aruco.start() # Starts the PID script
# Press q to close camera feed

# Now, wait until user presses 'Enter' key, then stop all code, and close the socket.
_ = input()
aruco.stop()