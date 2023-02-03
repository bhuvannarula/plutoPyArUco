from .common import *

class positionPID:
    def __init__(self) -> None:
        self.position_err = [0]*3

        self.now_time = nowtime()
        self.old_time = self.now_time
        self.unit = 1e9

        self.pPOS = [0]*3

        pp = 1
        ii = 0
        dd = 0

        self.pPOS[X] = 1.2
        self.pPOS[Y] = 1.2 # 1.6
        self.pPOS[Z] = 0

        self.pVEL = [0]*3
        self.iVEL = [0]*3
        self.dVEL = [0]*3

        self.pVEL[X] = pp # 2.4
        self.iVEL[X] = ii
        self.dVEL[X] = dd # 2

        self.pVEL[Y] = pp
        self.iVEL[Y] = ii
        self.dVEL[Y] = dd

        self.pVEL[Z] = 0#3
        self.iVEL[Z] = 0#.4
        self.dVEL[Z] = 0

        self.iTerm = [0]*3
        self.last_vel = [0]*3
        self.last_result = [0]*3

    def output(self, pos_err : list, state : arucoState):
        self.position_err[X] = pos_err[X]
        self.position_err[Y] = pos_err[Y]
        self.position_err[Z] = pos_err[Z]
        angle = pos_err[3]
        cosA = cos(angle)
        sinA = sin(angle)

        # X, Y, Z -> P Controller
        setVel_X = constrain(self.pPOS[X] * pos_err[X], -500, 500) # Vel Max : 200px/s
        setVel_Y = constrain(self.pPOS[Y] * pos_err[Y], -500, 500) # Vel Max : 200px/s
        setVel_Z = constrain(self.pPOS[Z] * pos_err[Z], -100, 100) # Vel Max : 100cm/s

        # Calculating Velocity
        dt = (state.now - state.old)
        if not dt:
            return self.last_result
        _newX = state.X
        _oldX = state.X_old
        #vel_X = self.unit*(_newX[X] - _oldX[X])/dt
        #vel_Y = self.unit*(_newX[Y] - _oldX[Y])/dt
        #vel_Z = self.unit*(_newX[Z] - _oldX[Z])/dt
        vel_X = state.V[X]
        vel_Y = state.V[Y]
        vel_X = vel_X * cosA + vel_Y * sinA
        vel_Y = vel_Y * cosA - vel_X * sinA
        vel_Z = state.V[Z]

        # Velocity -> PID Controller
        vel_err = [0]*3
        vel_err[X] = setVel_X - vel_X
        vel_err[Y] = setVel_Y - vel_Y
        vel_err[Z] = setVel_Z - vel_Z

        # Calculating the P-Term
        result_X = constrain(self.pVEL[X] * vel_err[X], -500, 500)
        result_Y = constrain(self.pVEL[Y] * vel_err[Y], -500, 500)
        result_Z = constrain(self.pVEL[Z] * vel_err[Z], -500, 500)
        #print("P", result_Z)

        # Calculating the I-Term
        self.iTerm[X] += (self.iVEL[X] * vel_err[X]) * dt / self.unit
        self.iTerm[Y] += (self.iVEL[Y] * vel_err[Y]) * dt / self.unit
        self.iTerm[Z] += (self.iVEL[Z] * vel_err[Z]) * dt / self.unit

        self.iTerm[X] = constrain(self.iTerm[X], -50, 50)
        self.iTerm[Y] = constrain(self.iTerm[Y], -50, 50)
        self.iTerm[Z] = constrain(self.iTerm[Z], -500, 500)
        #print("I", self.iTerm)

        result_X += self.iTerm[X]
        result_Y += self.iTerm[Y]
        result_Z += self.iTerm[Z]

        # Calculating the D-Term
        result_X += constrain(self.dVEL[X] * (self.last_vel[X] - vel_X) * self.unit / dt, -100, 100)
        result_Y += constrain(self.dVEL[Y] * (self.last_vel[Y] - vel_Y) * self.unit / dt, -100, 100)
        result_Z += constrain(self.dVEL[Z] * (self.last_vel[Z] - vel_Z) * self.unit / dt, -100, 100)
        #print("D", result_Z)

        self.last_vel = [vel_X, vel_Y, vel_Z]
        self.last_result = [result_X, result_Y, 150]

        return (result_X, result_Y, 150)

class altitudePID:
    def __init__(self) -> None:
        pass