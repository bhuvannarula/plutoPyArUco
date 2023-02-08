NONE_COMMAND = 0
TAKE_OFF = 1
LAND = 2

class plutoState():
    def __init__(self) -> None:
        '''
        Pluto State Instance
        '''
        self.roll : int = 0
        self.pitch : int = 0
        self.yaw: int = 0
        self.battery : int = 0
        self.rssi : int = 0
        self.accX : int = 0
        self.accY : int = 0
        self.accZ : int = 0
        self.gyroX : float = 0.0
        self.gyroY : float = 0.0
        self.gyroZ : float = 0.0
        self.magX : float = 0.0
        self.magY : float = 0.0
        self.magZ : float = 0.0
        self.alt : float = 0.0

        self.FC_versionMajor : int = 0
        self.FC_versionMinor : int = 0
        self.FC_versionPatchLevel : int = 0

        self.trim_roll : int = 0
        self.trim_pitch : int = 0

        self.rcThrottle : float = 1500
        self.rcRoll : float = 1500
        self.rcPitch : float = 1500
        self.rcYaw : float = 1500
        self.rcAUX1 : float = 1000
        self.rcAUX2 : float = 1000
        self.rcAUX3 : float = 1000
        self.rcAUX4 : float = 1000

        self.commandType = NONE_COMMAND

        self.trim_roll = 0
        self.trim_pitch = 0
        self.isAutoPilotOn = 0

        #self.droneIndex = droneIndex
    
    def array(self) -> "list[int]":
        '''
        Returns the drone state as Array
        '''
        t_arr = [
            self.rcRoll,
            self.rcPitch,
            self.rcThrottle,
            self.rcYaw,
            self.rcAUX1,
            self.rcAUX2,
            self.rcAUX3,
            self.rcAUX4
            ]
        return t_arr