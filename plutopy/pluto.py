from time import sleep
from .protocol import *
from .commands import *
from .plutoinfo import *
import threading

class plutoDrone():
    def __init__(self, IP_ADDRESS : str = '192.168.4.1', PORT : int = 23, CAMERA_IP_ADDRESS : str = None, CAMERA_PORT :int = 9060) -> None:
        '''
        A Pluto Drone Instance

        Provides methods to send commands to drone,
        and access sensor data from drone.

        IP_ADDRESS : str (default '192.168.4.1')
        PORT : int (default 23)
        CAMERA_IP_ADDRESS : str (default None)
        CAMERA_PORT : int (default 9060)
        '''
        self.IP_ADDRESS = IP_ADDRESS
        self.PORT = PORT

        # False -> Threads Stop Running
        self._threadsRunning = True
        self._threads =  []

        # Step 1 : Initialize the State Instances of Drone
        self.activeState = plutoState()
        self.rc = self.activeState
        #self.activeStateAP = plutoState()
        self.state = plutoState()

        # Step 2 : Initialize the Read Buffer for Drone
        self.buffer = plutoBuffer()

        # Step 3 : Initialize the socket for Drone
        self.sock = plutoSock(IP_ADDRESS, PORT, self.buffer, self.state)

        # Step 4 : Initialize MSP Instance
        self.MSP = plutoMSP(self.sock)

        # Attaching command controls to class
        self.control = plutoControl(self.activeState, self.MSP)

        # Attaching info commands to class
        self.info = plutoInfo(self.state)
    
    def writeThread(self):
        '''
        Thread that continuously sends RC Data to Drone
        '''
        #requests = [MSP_RC, MSP_ATTITUDE, MSP_RAW_IMU, MSP_ALTITUDE, MSP_ANALOG]
        requests = [MSP_ALTITUDE, MSP_RAW_IMU]

        self.MSP.sendRequestMSP_ACC_TRIM()

        while (self._threadsRunning):
            state = self.activeState.array()
            '''
            if (self.activeState.isAutoPilotOn and state[7]):
                state[0] += self.activeStateAP.rcRoll - 1500
                state[1] += self.activeStateAP.rcPitch - 1500
                state[2] += self.activeStateAP.rcThrottle - 1500
                state[3] += self.activeStateAP.rcYaw - 1500
            '''
            self.MSP.sendRequestMSP_SET_RAW_RC(state)
            self.MSP.sendRequestMSP_GET_DEBUG(requests)

            if (self.activeState.commandType != NONE_COMMAND):
                self.MSP.sendRequestMSP_SET_COMMAND(self.activeState.commandType)
                self.activeState.commandType = NONE_COMMAND
            '''
            elif (self.activeStateAP.commandType != NONE_COMMAND and self.activeState.isAutoPilotOn and (state[7] == 1500)):
                self.MSP.sendRequestMSP_SET_COMMAND(self.activeStateAP.commandType)
                self.activeStateAP.commandType = NONE_COMMAND
            '''
            
            sleep(0.022)

    def readThread(self):
        '''
        Thread that continuously reads response from Drone
        '''
        while (self._threadsRunning):
            try:
                self.sock.readResponseMSP()
            except Exception as e:
                print(e)
                pass

    def reconnect(self):
        '''
        Reconnecting the socket to Drone
        '''
        self.sock.connect()

    def disconnect(self):
        '''
        Disarm the Drone, and close the connection
        '''
        self.control.kill()
        sleep(0.1)
        if (self._threadsRunning):
            self._threadsRunning = False
            for _i in self._threads:
                _i.join()
        if self._threads:
            self.sock.disconnect()

    def start(self):
        '''
        Establish Communication, and start required threads
        '''
        # Connecting Socket to Drone
        self.sock.connect()

        self._threadsRunning = True
        writeThread = threading.Thread(target=self.writeThread)
        writeThread.start()
        readThread = threading.Thread(target=self.readThread)
        readThread.start()
        self._threads = [writeThread, readThread]

