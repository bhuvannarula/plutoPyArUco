from .common import *

class positionPID:
    def __init__(self) -> None:
        self.position_err = [0]*3

        self.now_time = nowtime()
        self.old_time = self.now_time
        self.unit = 1e9

        self.pPOS = [0]*3
        self.iPOS = [0]*3
        self.dPOS = [0]*3

        self.pPOS[X] = 1.25 # 8 # 0.2 (2 0/5) (4,1) (6,0.3) (2, 0.625)
        self.iPOS[X] = 0#.00001
        self.dPOS[X] = 0.625#9#50#10 # 0.5

        self.pPOS[Y] = 1.25
        self.iPOS[Y] = 0.005
        self.dPOS[Y] = 0.625#9#50#40

        self.pPOS[Z] = 2#2#0.1#2
        self.iPOS[Z] = 1.6#3#0.3#.015#.2
        self.dPOS[Z] = 1.25#1.45#1#.1 #(2, 1.6, 1.25)

        self.pVEL = 4
        self.iVEL = 0.8
        self.dVEL = 0.2

        self.iTerm = [0]*3
        self.lastZ = 0
        self.last_vel = [0]*3
        self.last_result = [0]*3

        self.wo = 70

    def output(self, pos_err : list, state : arucoState):
        self.position_err[X] = pos_err[X]
        self.position_err[Y] = pos_err[Y]
        self.position_err[Z] = pos_err[Z]
        angle = pos_err[3]
        cosA = cos(angle)
        sinA = sin(angle)

        # X, Y, Z -> PID Controller

        # Calculating Velocity
        dt = state.dt
        if not dt:
            return self.last_result
        vel_X = state.V[X]
        vel_Y = state.V[Y]
        vel_X = vel_X * cosA + vel_Y * sinA
        vel_Y = vel_Y * cosA - vel_X * sinA
        vel_Z = -(pos_err[Z] - self.lastZ)*state.unit/dt
        self.lastZ = pos_err[Z]

        # Calculating the P-Term
        result_X = constrain(self.pPOS[X] * pos_err[X], -100, 100)
        result_Y = constrain(self.pPOS[Y] * pos_err[Y], -100, 100)
        result_Z = constrain(self.pPOS[Z] * pos_err[Z], -200, 200)
        p_x = constrain(self.pPOS[X] * pos_err[X], -500, 500)
        p_y = constrain(self.pPOS[Y] * pos_err[Y], -500, 500)
        p_z = constrain(self.pPOS[Z] * pos_err[Z], -500, 500)

        # Calculating the I-Term
        self.iTerm[X] += (self.iPOS[X] * pos_err[X]) * dt / self.unit
        self.iTerm[Y] += (self.iPOS[Y] * pos_err[Y]) * dt / self.unit
        self.iTerm[Z] += (self.iPOS[Z] * pos_err[Z]) * dt / self.unit
        
        self.iTerm[X] = constrain(self.iTerm[X], -50, 50)
        self.iTerm[Y] = constrain(self.iTerm[Y], -500, 500)
        self.iTerm[Z] = constrain(self.iTerm[Z], -300, 300)

        result_X += self.iTerm[X]
        result_Y += self.iTerm[Y]
        result_Z += self.iTerm[Z]
        i_x = self.iTerm[X]
        i_y = self.iTerm[Y]
        i_z = self.iTerm[Z]

        # Calculating the D-Term
        result_X += constrain(self.dPOS[X] * (vel_X) * (-1), -1000, 1000)
        result_Y += constrain(self.dPOS[Y] * (vel_Y) * (-1), -1000, 1000)
        result_Z += constrain(self.dPOS[Z] * (vel_Z) * (-1), -300, 300)
        d_x= constrain(self.dPOS[X] * (vel_X) * (-1), -500, 500)
        d_y= constrain(self.dPOS[Y] * (vel_Y) * (-1), -500, 500)
        d_z= constrain(self.dPOS[Z] * (vel_Z) * (-1), -300, 300)

        self.lastP = [p_x,p_y,p_z]
        self.lastI = [i_x,i_y,i_z]
        self.lastD = [d_x,d_y,d_z]

        result_Z += self.wo

        self.last_vel = [vel_X, vel_Y, vel_Z]

        self.last_result = [result_X, result_Y, result_Z]

        return (result_X, result_Y, result_Z)