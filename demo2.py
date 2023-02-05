from aruco.plutoCV import *
import threading
from plutopy import plutoDrone
from aruco.filters import deadband
from aruco.plutoPID3 import *

drone = plutoDrone('192.168.4.1')

#drone.start()

#drone.reconnect()

#drone.reconnect()

state = arucoState()

class arucoState2:
    def __init__(self) -> None:
        self.X = 0
        self.Y = 0
        self.Z = 0
    def __repr__(self) -> str:
        return str((self.X, self.Y, self.Z))

def cvThread():
    aruco = arucoGPS(state)

_thread = threading.Thread(target=cvThread)
_thread.start()

# Drone should be facing right
# Increasing X means moving forward
# If moving forward, decrease pitch_trim by 1

# Inreasing Y means moving right
# If moving right, decrease pitch_roll by 1

_t = 2

def coord_dead(state : arucoState):
    _X = deadband(state.X_new[X], state.X_old[X], _t)
    _Y = deadband(state.X_new[Y], state.X_old[Y], _t)
    _Z = deadband(state.X_new[Z], state.X_old[Z], _t)
    _tt = [_X, _Y, _Z]
    state.X_old = list(state.X)
    state.X = list(_tt)
    return _tt


def dronepid():
    sleep(10)
    fix_state = arucoState2()
    iter_n = 10
    for _i  in range(iter_n):
        _tt = coord_dead(state)
        fix_state.X += _tt[X]
        fix_state.Y += _tt[Y]
        fix_state.Z += _tt[Z]
    fix_state.X = int(fix_state.X/iter_n)
    fix_state.Y = int(fix_state.Y/iter_n)
    fix_state.Z = int(fix_state.Z/iter_n)

    trim_roll = 0
    trim_pitch = 0

    print(fix_state)
    #drone.control.take_off()

    #sleep(2)

    sleep_f = 0.05
    while True:
        sleep(sleep_f)
        _tt = coord_dead(state)
        _err = [
            fix_state.X - _tt[X],
            fix_state.Y - _tt[Y],
            fix_state.Z - _tt[Z]
        ]

        print(_err)
        trim_pitch = -int(_err[0]*1)
        trim_roll = -int(_err[1]*1)

        '''
        if _err[0] > 0.25:
            trim_pitch -= 3
        elif _err[0] < -0.25:
            trim_pitch += 3
        else:
            trim_pitch = 0

        if _err[1] > 0.25:
            trim_roll -= 3
        elif _err[1] < -0.25:
            trim_roll += 3
        else:
            trim_roll = 0
        '''

        _t_pit = maxlimit(trim_pitch, 50)
        _t_rol = maxlimit(trim_roll, 50)
        #print(_t_rol, _t_pit)
        try:
            #drone.MSP.sendRequestMSP_SET_ACC_TRIM(_t_rol, _t_pit)
            drone.activeState.rcRoll = 1500+_t_rol
            drone.activeState.rcPitch =  1500+_t_pit
            drone.activeState.rcThrottle = 1600
        except:
            pass
        trim_pitch = _t_pit
        trim_roll = _t_rol

_thread1 = threading.Thread(target=dronepid)
_thread1.start()
sleep(10)
#drone.control.land()

#drone.disconnect()