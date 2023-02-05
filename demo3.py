from aruco.plutoCV import *
import threading
from plutopy import plutoDrone
from aruco.filters import deadband
from aruco.plutoPID3 import *

class XYZ:
    def __init__(self) -> None:
        self.X = 0
        self.Y = 0
        self.Z = 0
    def __repr__(self) -> str:
        return str((self.X, self.Y, self.Z))

class plutoArUcO:
    def __init__(self, drone : plutoDrone) -> None:
        self.PIDdelay = 0.001
        self.drone = drone

        self.state = arucoState()

        self._threadsRunning = True
        self.debug = 0
        self._threads = []

    def coord_dead(self):
        _t = 1
        _X = deadband(self.state.X_new[X], self.state.X_old[X], _t)
        _Y = deadband(self.state.X_new[Y], self.state.X_old[Y], _t)
        _Z = deadband(self.state.X_new[Z], self.state.X_old[Z], _t)
        _tt = [_X, _Y, _Z]
        self.state.X_old = list(self.state.X)
        self.state.X = list(_tt)
        return _tt

    def arucoCVThread(self):
        aruco = arucoGPS(self.state)
        while self._threadsRunning:
            _err = aruco.loop()
            if _err:
                self._threadsRunning = False
                break

    def arucoPIDThread(self):
        sleep(10)
        fix_state = XYZ()
        iter_n = 10
        for _i  in range(iter_n):
            _tt = self.coord_dead()
            fix_state.X += _tt[X]
            fix_state.Y += _tt[Y]
            fix_state.Z += _tt[Z]
        fix_state.X = int(fix_state.X/iter_n)
        fix_state.Y = int(fix_state.Y/iter_n)
        fix_state.Z = int(fix_state.Z/iter_n)

        self.positionPID = positionPID()

        while self._threadsRunning:
            sleep(self.PIDdelay)
            _tt = self.coord_dead()
            _err = [
                fix_state.X - _tt[X],
                fix_state.Y - _tt[Y],
                fix_state.Z - _tt[Z]
            ]
            if self.debug: print("Pos Err:", _err)
            
            trim_pitch, trim_roll = self.positionPID.output(_err, self.state)
            trim_pitch = constrain(trim_pitch, -80, 80)
            trim_roll = constrain(trim_roll, -80, 80)
            if self.debug: print("Trim Val:", trim_roll, trim_pitch)

            self.drone.activeState.rcRoll = 1500 + int(trim_roll)
            self.drone.activeState.rcPitch = 1500 + int(trim_pitch)

    def start(self):
        procs = [self.arucoCVThread, self.arucoPIDThread]
        self._threads = []
        self._threadsRunning = True
        for proc in procs:
            _thread = threading.Thread(target=proc)
            _thread.start()
            self._threads.append(_thread)
    
    def stop(self):
        self._threadsRunning = False
        for _th in self._threads:
            _th.join()
        self.drone.control.kill()
        self.drone.disconnect()


drone = plutoDrone('192.168.4.1')
drone.activeState.rcAUX3 = 2000
#drone.reconnect()
#drone.start()
#drone.reconnect()


pluto = plutoArUcO(drone)
pluto.debug = True
pluto.start()

sleep(13)
drone.control.take_off()
drone.activeState.rcThrottle = 1550
sleep(1)
#drone.control.kill()
#pluto.drone.disconnect()