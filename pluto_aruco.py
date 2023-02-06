from aruco.plutoCV import *
from aruco.plutoPID import *
from plutopy import plutoDrone
import csv

import threading

class plutoArUco:
    def __init__(self, drone : plutoDrone, targetID : int = 43) -> None:
        self.PIDdelay = 0.001
        self.drone = drone

        self.state = arucoState()
        self.target = XYZ()
        self.origin = XYZ()
        self.droneAngle = lowPassFilter()

        self.trims = []
        self._err = []

        self._threadsRunning = True
        self.debug = 0
        self._threads = []

        self.procs = [self.arucoPIDThread]

        self.file = open(r'C:\Users\Bhuvan Narula\Downloads\dronepiddump.csv', 'w', newline='\n')
        self.csv = csv.writer(self.file)

        self.aruco = arucoGPS(self.state, targetID, self.droneAngle)

        sleep(1)

        _proc = self.arucoCVThread
        _thread = threading.Thread(target=_proc)
        _thread.start()
        self._threads.append(_thread)

        sleep(1)

        self.positionPID = positionPID()

    def arucoCVThread(self):
        while self._threadsRunning:
            _err = self.aruco.loop(self.target)
            if _err:
                self._threadsRunning = False
                self.stop_rest()
                break
            #if self.target.Z:
            #    self.arucoPID()

    def setOrigin(self, iter_n : int = 100):
        '''
        Reading first 'iter_n' values & averaging to find origin, ground
        '''
        origin = XYZ()
        _tZ = lowPassFilter()
        sleep(1)
        for _i  in range(iter_n):
            sleep(self.PIDdelay)
            _tt = self.state.X
            origin.X += _tt[X]
            origin.Y += _tt[Y]
            origin.Z += _tt[Z]
            origin.A += self.droneAngle.get()
        self.origin.X = int(origin.X/iter_n)
        self.origin.Y = int(origin.Y/iter_n)
        self.origin.Z = round(origin.Z/iter_n, 2)
        #self.origin.Z = _tZ.get()
        self.origin.A = round(origin.A/iter_n, 2)
        print(self.origin.A)

    def setTarget(self, X, Y, Z):
        if (self.origin.Z == 0):
            # origin is unset
            print('Origin is not set. Target not updated.')
        else:
            self.target.X = X
            self.target.Y = Y
            self.target.Z = Z

    def arucoPIDThread(self):
        self.positionPID = positionPID()

        rolll = lowPassFilter()
        pitcc = lowPassFilter()

        while self._threadsRunning:
            sleep(self.PIDdelay)
            #sleep(0.1)
            angle = radians(self.origin.A - 90)
            cosA = cos(angle)
            sinA = sin(angle)
            _tt = self.state.X
            _eX = round(self.target.X - _tt[X], 1)
            _eY = round(self.target.Y - _tt[Y], 1)
            _eX = _eX * cosA + _eY * sinA
            _eY = _eY * cosA - _eX * sinA
            _err = [
                _eX,
                _eY,
                self.target.Z - (self.origin.Z - _tt[Z]),
                angle
            ]
            if self.debug: print("Pos Err:", _err)
            
            pitch, roll, throttle = self.positionPID.output(_err, self.state)
            pitch = constrain(pitch, -400, 400)
            roll = constrain(roll, -500, 500)
            throttle = constrain(throttle, -300, 300)
            #pitch = pitch * cosA + roll * sinA
            #roll = roll * cosA - pitch * sinA
            dronealt = self.drone.state.alt
            self.csv.writerow([*_err[:3], dronealt, pitch, roll, throttle])
            #print(_err[2], throttle)
            pitcc.update(pitch)
            rolll.update(roll)
            #pitch = pitcc.get()-180
            #roll = rolll.get()-180
            #print(throttle)
            if self.debug: print("Trim Val:", roll, pitch, throttle)
            self.trims = [pitch, roll, throttle]
            self._err = _err

            #self.drone.MSP.sendRequestMSP_SET_ACC_TRIM(-int(roll), -int(pitch/2.5))

            self.drone.activeState.rcRoll = 1500 + int(roll)
            self.drone.activeState.rcPitch = 1500 + int(pitch)
            self.drone.activeState.rcThrottle = 1500 + int(throttle)

    def arucoPID(self):
        angle = radians(self.origin.A - 90)
        cosA = cos(angle)
        sinA = sin(angle)
        _tt = self.state.X
        _eX = self.target.X - _tt[X]
        _eY = self.target.Y - _tt[Y]
        _eX = _eX * cosA + _eY * sinA
        _eY = _eY * cosA - _eX * sinA
        _err = [
            _eX,
            _eY,
            self.target.Z - (self.origin.Z - _tt[Z]),
            angle
        ]
        if self.debug: print("Pos Err:", _err)
        
        pitch, roll, throttle = self.positionPID.output(_err, self.state)
        pitch = constrain(pitch, -400, 400)
        roll = constrain(roll, -500, 500)
        throttle = constrain(throttle, -300, 300)
        #pitch = pitch * cosA + roll * sinA
        #roll = roll * cosA - pitch * sinA
        dronealt = self.drone.state.alt
        self.csv.writerow([*_err[:3], dronealt, pitch, roll, throttle])
        print(_err[2], throttle)
        #pitch = pitcc.get()-180
        #roll = rolll.get()-180
        #print(throttle)
        if self.debug: print("Trim Val:", roll, pitch, throttle)
        self.trims = [pitch, roll, throttle]
        self._err = _err

        #self.drone.MSP.sendRequestMSP_SET_ACC_TRIM(-int(roll), -int(pitch/2.5))

        self.drone.activeState.rcRoll = 1500 + int(roll)
        self.drone.activeState.rcPitch = 1500 + int(pitch)
        self.drone.activeState.rcThrottle = 1500 + int(throttle)
        
    def start(self):
        if (self.origin.Z == 0):
            print("Warning: Origin not set!")
        #self._threads = []
        self._threadsRunning = True
        for proc in self.procs:
            _thread = threading.Thread(target=proc)
            _thread.start()
            self._threads.append(_thread)
    
    def stop_rest(self):
        self._threadsRunning = False
        for _th in self._threads[1:]:
            _th.join()
        self.file.flush()
        self.file.close()
        self.drone.control.kill()
        if self.drone._threadsRunning:
            self.drone.disconnect()

    def stop(self):
        self._threadsRunning = False
        for _th in self._threads:
            _th.join()
        self.drone.control.kill()
        self.drone.disconnect()


if __name__ == '__main__':
    drone = plutoDrone('192.168.4.1')
    drone.activeState.rcAUX3 = 2000
    drone.reconnect()
    drone.start()
    drone.reconnect()


    pluto = plutoArUco(drone)
    #pluto.debug = True
    pluto.start()

    sleep(13)
    drone.control.take_off()
    drone.activeState.rcThrottle = 1600
    sleep(1)
    drone.activeState.commandType = 0
    #drone.control.kill()
    #pluto.drone.disconnect()