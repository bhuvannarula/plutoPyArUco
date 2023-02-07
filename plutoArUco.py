from aruco.plutoCV import *
from aruco.plutoPID import *
from plutopy import plutoDrone
from aruco.filter import lpfilterZ
from aruco.kalman import KalmanFilter

import threading

class plutoArUco:
    def __init__(self, drone : plutoDrone, targetID : int) -> None:
        self.PIDdelay = 0.001
        self.drone = drone
        self.k = KalmanFilter()
        self.state = arucoState()
        self.target = XYZ()
        self.origin = XYZ()
        self.droneAngle = lowPassFilter()
        self.Zfil = lpfilterZ()
        self._err = []

        self._threadsRunning = True
        self.debug = 0
        self._threads = []

        self.procs = [self.arucoPIDThread]

        self.file = open(r'logs/dumpPID.csv', 'w', newline='\n')
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
            _err = self.aruco.loop()
            if _err:
                self._threadsRunning = False
                self.stop_rest()
                break
            if self.origin.Z:
                self.arucoPIDThread()

    def setOrigin(self, iter_n : int = 50):
        '''
        Reading first 'iter_n' values & averaging to find origin, ground
        '''
        origin = XYZ()
        for _i  in range(iter_n):
            sleep(self.PIDdelay)
            _tt = self.state.X
            origin.X += _tt[X]
            origin.Y += _tt[Y]
            origin.Z += _tt[Z]
            origin.A += self.droneAngle.get()
        self.origin.X = round(origin.X/iter_n, 2)
        self.origin.Y = round(origin.Y/iter_n, 2)
        self.origin.Z = int(origin.Z/iter_n)
        self.origin.A = round(origin.A/iter_n, 2)
        print("Angle of Drone:", self.origin.A)

        # Configuring Kalman Filter
        self.k.Configure(self.origin.Z - self.state.X[Z],0,(1-self.drone.state.accZ))

    def setTarget(self, X, Y, Z):
        if (self.origin.Z == 0):
            # origin is unset
            print('Origin is not set. Target not updated.')
        else:
            self.target.X = X
            self.target.Y = Y
            self.target.Z = Z

    def arucoPIDThread(self):
        sleep(self.PIDdelay)
        _tt = self.state.X
        self.k.Update(self.origin.Z - self.state.X[Z],self.drone.state.accZ,self.state.dt/self.state.unit)
        angle = radians(self.origin.A - 90)
        angle = radians(self.droneAngle.get() - 90)
        cosA = cos(angle)
        sinA = sin(angle)
        _eX = self.target.X - _tt[X]
        _eY = self.target.Y - _tt[Y]
        _eX = _eX * cosA + _eY * sinA
        _eY = _eY * cosA - _eX * sinA
        self.Zfil.update(self.k.z_)
        _err = [
            _eX,
            _eY,
            self.target.Z - (self.Zfil.get()),
            angle
        ]

        if self.debug: print("Pos Err:", _err)
        
        pitch, roll, throttle, = self.positionPID.output(_err, self.state)
        pitch = constrain(pitch, -400, 400)
        roll = constrain(roll, -500, 500)
        throttle = constrain(throttle, -300, 300)

        zp = self.positionPID.lastP[Z]
        zi = self.positionPID.lastI[Z]
        zd = self.positionPID.lastD[Z]
        self.csv.writerow([*_err[:3], pitch, roll, throttle, zp, zi, zd])

        if self.debug: print("Trim Val:", roll, pitch, throttle)
        
        self._err = _err

        self.drone.activeState.rcRoll = 1500 + int(roll)
        self.drone.activeState.rcPitch = 1500 + int(pitch)
        self.drone.activeState.rcThrottle = 1500 + int(throttle)
        
    def start(self):
        if (self.origin.Z == 0):
            print("Warning: Origin not set!")
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

    def stop(self):
        self.drone.control.kill()
        if self.drone._threadsRunning:
            self.drone.disconnect()