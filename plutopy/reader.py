from .common import *
from .plutostate import *

class plutoGET:
    def __init__(self, buffer :  plutoBuffer, responseState : plutoState) -> None:
        '''
        Reads Data from Buffer, Evaluates the command, and stores the response.
        '''
        self.state = responseState
        self.buffer = buffer

    def read8(self) -> list:
        t_ret = (self.buffer.get() & 0xff)
        return t_ret

    def read16(self) -> list:
        t_add = 0
        for i in range(2):
            t_add += ((self.buffer.get() & 0xff) << 8*i)
        return t_add

    def read32(self) -> list:
        t_add = 0
        for i in range(4):
            t_add += ((self.buffer.get() & 0xff) << 8*i)
        return t_add

    def evaluateCommand(self, command : int) -> None:
        if (command == MSP_FC_VERSION):
            self.state.FC_versionMajor = int(self.read8())
            self.state.FC_versionMinor = int(self.read8())
            self.state.FC_versionPatchLevel = int(self.read8())
        
        elif (command == MSP_RAW_IMU):
            self.state.accX = self.read16()
            self.state.accY = self.read16()
            self.state.accZ = self.read16()

            self.state.gyroX = self.read16() # /8
            self.state.gyroY = self.read16() # /8
            self.state.gyroZ = self.read16() # /8

            self.state.magX = self.read16() # /3
            self.state.magY = self.read16() # /3
            self.state.magZ = self.read16() # /3
        
        elif (command == MSP_ATTITUDE):
            self.state.roll = int(self.read16()/10)
            self.state.pitch = int(self.read16()/10)
            self.state.yaw = int(self.read16())
        
        elif (command == MSP_ALTITUDE):
            self.state.alt = self.read32()/10

        elif (command == MSP_ANALOG):
            self.state.battery = self.read8()/10
            self.state.rssi = int(self.read16())
        
        elif (command == MSP_ACC_TRIM):
            self.state.trim_pitch = int(self.read16())
            self.state.trim_roll = int(self.read16())
        
        elif (command == MSP_RC):
            self.state.rcRoll = self.read16()
            self.state.rcPitch = self.read16()
            self.state.rcYaw = self.read16()
            self.state.rcThrottle = self.read16()
            self.state.rcAUX1 = self.read16()
            self.state.rcAUX2 = self.read16()
            self.state.rcAUX3 = self.read16()
            self.state.rcAUX4 = self.read16()
        
        else:
            pass