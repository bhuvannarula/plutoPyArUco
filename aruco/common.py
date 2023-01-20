from time import perf_counter_ns as nowtime
from time import sleep

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
    def __repr__(self) -> str:
        return str((self.X, self.Y, self.Z))
    def reset(self):
        self.X = 0
        self.Y = 0
        self.Z = 0

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
        _t = 1
        _X = deadband(self.X_new[X], self.X_old[X], _t)
        _Y = deadband(self.X_new[Y], self.X_old[Y], _t)
        _Z = deadband(self.X_new[Z], self.X_old[Z], _t)
        _tt = [_X, _Y, _Z]

        self.X_old = list(self.X)
        self.X = list(_tt)

def constrain(value : int, low: int, high : int):
    if (low < value <  high):
        return value
    elif (value <= low):
        return low
    elif (value >= high):
        return high