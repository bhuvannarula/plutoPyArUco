from pluto_aruco import *
from plutopy import plutoDrone

drone = plutoDrone() 

aruco = plutoArUco(drone, arucoID=85)

aruco.debug = True

aruco.setOrigin()
print(aruco.origin)

#aruco.start() # Starts the PID script

# Press q to close camera feed

# Now, wait until user presses 'Enter' key, then stop all code, and close the socket.
_ = input()
aruco.stop()