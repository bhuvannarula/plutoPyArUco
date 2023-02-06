from .protocol import *

TRIM_MAX = 1000
TRIM_MIN = -1000

class plutoControl():
    def __init__(self, state : plutoState, MSP : plutoMSP) -> None:
        self.cmd = state
        self.MSP = MSP

    def updateCommand(self, commandType : int):
        if (self.cmd.commandType != commandType) :
            self.cmd.commandType = commandType
            self.MSP.sendRequestMSP_SET_COMMAND(self.cmd.commandType)
        
    def arm(self):
        '''
        Arm the Drone
        '''
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1000
        self.cmd.rcAUX4 = 1500
        self.cmd.isAutoPilotOn = 0

    def box_arm(self):
        self.cmd.rcRoll = 1500
        self.cmd.rcYaw = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcAUX4 = 1500
        self.cmd.isAutoPilotOn = 0

    def disarm(self):
        self.cmd.rcThrottle = 1300
        self.cmd.rcAUX4 = 1200

    def forward(self, value = 100):
        '''
        Move the drone forward

        value (default 100) : offset for pitch, range (0, 500)
        '''
        value = constrain(value, 0, 500)
        self.cmd.rcPitch = 1500 + value

    def backward(self, value = 100):
        '''
        Move the drone backward

        value (default 100) : offset for pitch, range (0, 500)
        '''
        value = constrain(value, 0, 500)
        self.cmd.rcPitch = 1500 - value

    def left(self, value = 100):
        '''
        Move the drone towards left

        value (default 100) : in range (0, 500)
        '''
        value = constrain(value, 0, 500)
        self.cmd.rcRoll = 1500 - value

    def right(self, value = 100):
        '''
        Move the drone towards right

        value (default 100) : in range (0, 500)
        '''
        value = constrain(value, 0, 500)
        self.cmd.rcRoll = 1500 + value

    def left_yaw(self):
        '''
        Rotate the drone towards left
        '''
        self.cmd.rcYaw = 1200

    def right_yaw(self):
        '''
        Rotate the drone towards right
        '''
        self.cmd.rcYaw = 1800

    def reset(self):
        '''
        Reset the drone state (drone hovers)
        '''
        self.cmd.rcRoll = 1500
        self.cmd.rcThrottle = 1500
        self.cmd.rcPitch = 1500
        self.cmd.rcYaw = 1500
        self.cmd.commandType = 0

    def increase_height(self, value = 300):
        '''
        Increase the drone height

        value (default 300) : in range (0, 500)
        '''
        value = constrain(value, 0, 500)
        self.cmd.rcThrottle = 1500 + value

    def decrease_height(self, value = 100):
        '''
        Decrease the drone height

        value (default 300) : in range (0, 500)
        '''
        value = constrain(value, 0, 500)
        self.cmd.rcThrottle = 1500 - value

    def take_off(self):
        '''
        Take-Off Sequence for drone

        Drone will hover close to ground.
        '''
        self.reset()
        self.disarm()
        sleep(0.5)
        self.box_arm()
        self.updateCommand(1)
        sleep(0.5)
        self.updateCommand(0)
        self.cmd.rcThrottle = 1550

    def land(self):
        '''
        Landing Sequence for drone
        '''
        self.reset()
        self.updateCommand(2)
        sleep(0.5)
        self.arm()
        #self.cmd.commandType = 2

    def trimRollPitch(self, trim_roll, trim_pitch):
        '''
        Setting custom trim roll & pitch

        trim_roll : int, in range (-1000, 1000)
        trim_pitch : int, in range (-1000, 1000)
        '''
        t_trim_roll = self.cmd.trim_roll + trim_roll
        t_trim_pitch = self.cmd.trim_pitch + trim_pitch

        t_trim_roll = constrain(t_trim_roll, TRIM_MIN, TRIM_MAX)
        t_trim_pitch = constrain(t_trim_pitch, TRIM_MIN, TRIM_MAX)

        self.cmd.trim_roll = t_trim_roll
        self.cmd.trim_pitch = t_trim_pitch
        self.MSP.sendRequestMSP_SET_ACC_TRIM(t_trim_roll, t_trim_pitch)
        self.MSP.sendRequestMSP_EEPROM_WRITE()
        
    def altholdMode(self):
        '''
        Set Drone to Altitude Hold Mode (ON by default)
        Throttle Mode will turn OFF.
        '''
        self.cmd.rcAUX3 = 1500

    def throttleMode(self):
        '''
        Set Drone to Throttle Mode (OFF by default)
        Altitude Hold Mode will turn OFF.
        '''
        self.cmd.rcAUX3 = 2000

    def magholdMode(self):
        '''
        Set Drone to Mag Hold Mode (ON by default)
        HeadFree Mode will turn OFF.
        '''
        self.cmd.rcAUX1 = 1000
    
    def headfreeMode(self):
        '''
        Set Drone to HeadFree Mode (ON by default)
        Mag Hold Mode will turn OFF.
        '''
        self.cmd.rcAUX1 = 1500

    def kill(self):
        '''
        Reset the state of drone, and disarm it.
        '''
        self.reset()
        self.cmd.rcAUX4 = 1000