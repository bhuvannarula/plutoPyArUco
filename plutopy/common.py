from time import sleep

# THIS MIGHT BE BROKEN, KEEP IN MIND
commands = { 
    'NONE_COMMAND' : 0,
    'TAKE_OFF' : 1,
    'LAND' : 2 
    }

MSP_HEADER = "$M<"

# COMMANDS
MSP_FC_VERSION = 3
MSP_RAW_IMU = 102
MSP_RC = 105
MSP_ATTITUDE = 108
MSP_ALTITUDE = 109
MSP_ANALOG = 110
MSP_SET_RAW_RC = 200
MSP_ACC_CALIBRATION = 205
MSP_MAG_CALIBRATION = 206
MSP_SET_MOTOR = 214
MSP_SET_ACC_TRIM = 239
MSP_ACC_TRIM = 240
MSP_EEPROM_WRITE = 250
MSP_SET_POS = 216
MSP_SET_COMMAND = 217

# STATES
IDLE = 0
HEADER_START = 1
HEADER_M = 2
HEADER_ARROW = 3
HEADER_SIZE = 4
HEADER_CMD = 5
HEADER_ERR = 6

class plutoBuffer():
    def __init__(self) -> None:
        '''
        Buffer Instance for Pluto
        '''
        self.inputBuffer = [0]*1024
        self.bufferIndex = 0
    def get(self):
        _t = self.inputBuffer[self.bufferIndex]
        self.bufferIndex = self.bufferIndex + 1
        return _t

#buffer = plutoBuffer()

errors = {
    '1' : 'Unable to Create Socket',
    '2' : 'Unable to change Socket Blocking',
    '3' : 'Unable to Write to Socket',
    '4' : 'Unable to Connect to Socket',
    '5' : 'Invalid Socket Options',
    '6' : 'Socket not open for Writing',
    '7' : 'KeepAlive Option Failed',
    '8' : 'Socket Failed'
}

def constrain(value : int, low: int, high : int):
    if (low < value <  high):
        return value
    elif (value <= low):
        return low
    elif (value >= high):
        return high