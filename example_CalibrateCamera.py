from plutoArUco import *
#from swarmArUco import *
from plutopy import plutoDrone

'''drone = plutoDrone() 

aruco = plutoArUco(drone, droneID = 85)

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
aruco.stop()'''

drone = plutoDrone()

#camera = cameraFeed()

aruco = plutoArUco(drone, droneID = 85)

aruco.setOrigin()
print(aruco.origin)

target = [
    aruco.origin.X,
    aruco.origin.Y,
    90 # cm, Roughly 3.5 feet (3.5*30 = 105)
]
aruco.setTarget(*target)

aruco.start() # Starts the PID script
# Press q to close camera feed

# Now, wait until user presses 'Enter' key, then stop all code, and close the socket.
_ = input()
aruco.stop()