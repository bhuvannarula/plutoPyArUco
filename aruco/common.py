from time import perf_counter_ns as nowtime
from time import sleep
from math import sin, cos
from .filter import lowPassFilter, lpfilterZ

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
        self.V = [0]*3
        self.dt = 0

        self.Z = lowPassFilter()

        self.now = nowtime()
        self.old = self.now

        self.i_now = 0
        self.i_old = 0

        self.unit = 1e9

        self.posX = lpfilterZ()
        self.posY = lpfilterZ()
        self.posZ = lpfilterZ()

        self.Vx = lpfilterZ()
        self.Vy = lpfilterZ()
        self.Vz = lpfilterZ(1.75)
    
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
        dt = self.now - self.old

        # Applying a simple deadband filter to reduce flickering in data
        _t = 0.1
        #_X = deadband(self.X_new[X], self.X_old[X], _t)
        #_Y = deadband(self.X_new[Y], self.X_old[Y], _t)
        _X = round(self.X_new[X], 2)
        _Y = round(self.X_new[Y], 2)
        self.posX.update(_X)
        self.posY.update(_Y)
        _X = round(self.posX.get())
        _Y = round(self.posY.get())
        #_Z = deadband(self.X_new[Z], self.X_old[Z], 0.05)
        _Z = round(self.X_new[Z],2)
        #self.posZ.update(_Z)
        #_Z = self.posZ.get()
        #self.Z.update(_tZ)
        #_Z = self.Z.get()
        #A_Z = self.X_new[Z]
        _tt = [_X, _Y, _Z]

        if dt:
            self.dt = dt
            for ie in range(3):
                self.V[ie] = (_tt[ie] -  self.X_old[ie]) * self.unit / dt

        self.Vx.update(self.V[X])
        self.Vy.update(self.V[Y])
        self.Vz.update(self.V[Z])

        self.V[X] = self.Vx.get()
        self.V[Y] = self.Vy.get()
        self.V[Z] = self.Vz.get()

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