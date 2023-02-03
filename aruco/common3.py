from time import sleep
from math import sin, cos
from .filter import *

X, Y, Z = 0, 1, 2

# Deadband - Filter to reduce noise
def deadband(y_new, y_old, dead):
    if abs(y_new - y_old) <= dead:
        return y_old
    else:
        return y_new

class XYZ:
    def __init__(self) -> None:
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.A = 0
    def __repr__(self) -> str:
        return str((self.X, self.Y, self.Z))
    def reset(self):
        self.X = 0
        self.Y = 0
        self.Z = 0
        self.A = 0

class arucoState:
    '''
    Class to hold the raw coordinate data, as well as the filtered coordinates.

    self.X : Filtered Coordinate
    self.X_new : Raw Coordinate
    self.X_old : Previous Coordinate
    '''
    def __init__(self) -> None:
        self.X_new = [0]*3
        self.X_old = [0]*3
        self.X = [0]*3

        self.rX = lowPassFilter()
        self.rY = lowPassFilter()
        self.rZ = lowPassFilter()

        self.now = nowtime()
        self.old = self.now

        self.i_now = 0
        self.i_old = 0

        self.unit = 1e9
    
    def __repr__(self) -> str:
        return str(self.X)
    
    def update(self, coord_data : list):
        '''
        Updates the new coordinate of drone

        coord_data : Raw Coordinates
        '''
        self.X_new = list(coord_data)
        self.old = self.now
        self.now = nowtime()

        # Applying a simple deadband filter to reduce flickering in data
        self.rX.update(coord_data[X])
        self.rY.update(coord_data[Y])
        self.rZ.update(coord_data[Z])
        _tt = [self.rX.get(), self.rY.get(), self.rZ.get()]

        self.X_old = list(self.X)
        self.X = list(_tt)

def constrain(value : int, low: int, high : int):
    if (low < value <  high):
        return value
    elif (value <= low):
        return low
    elif (value >= high):
        return high

def digital(value, step, maxl):
    if (-step <= value <= step):
        return 0
    elif (step < value <= maxl):
        return maxl
    elif (-maxl <= value < -step):
        return -maxl

def radians(value):
    pi = 3.1415
    return value * 2 * pi / 360