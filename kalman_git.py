from time import perf_counter_ns as nowtime

class KalmanFilter:
    def __init__(self):
        pass

    def Configure(self, zInitial, vInitial, aBiasInitial):
        zVariance = 0.3245 # ArUco Variance
        zAccelVariance = 0.1622 # Accelerometer Variance
        zAccelBiasVariance = 1.0

        self.zAccelVariance_ = zAccelVariance
        self.zAccelBiasVariance_ = zAccelBiasVariance
        self.zVariance_ = zVariance

        self.z_ = zInitial
        self.v_ = vInitial
        self.aBias_ = aBiasInitial
        self.Pzz_ = 1.0
        self.Pzv_ = 0.0
        self.Pza_ = 0.0

        self.Pvz_ = 0.0
        self.Pvv_ = 1.0
        self.Pva_ = 0.0

        self.Paz_ = 0.0
        self.Pav_ = 0.0
        self.Paa_ = 100000.0

    def Update(self, z, a, dt):
        accel = a - self.aBias_
        self.v_ += accel * dt
        self.z_ += self.v_ * dt

        self.zAccelVariance_ = abs(accel) / 50.0
        self.zAccelVariance_ = min(self.zAccelVariance_, 50.0)
        self.zAccelVariance_ = max(self.zAccelVariance_, 1.0)

        dt2div2 = dt * dt / 2.0
        dt3div2 = dt2div2 * dt
        dt4div4 = dt2div2 * dt2div2

        t00 = self.Pzz_ + dt * self.Pvz_ - dt2div2 * self.Paz_
        t01 = self.Pzv_ + dt * self.Pvv_ - dt2div2 * self.Pav_
        t02 = self.Pza_ + dt * self.Pva_ - dt2div2 * self.Paa_

        t10 = self.Pvz_ - dt * self.Paz_
        t11 = self.Pvv_ - dt * self.Pav_
        t12 = self.Pva_ - dt * self.Paa_

        t20 = self.Paz_
        t21 = self.Pav_
        t22 = self.Paa_

        self.Pzz_ = t00 + dt * t01 - dt2div2 * t02
        self.Pzv_ = t01 - dt * t02
        self.Pza_ = t02

        self.Pvz_ = t10 + dt * t11 - dt2div2 * t12
        self.Pvv_ = t11 - dt * t12
        self.Pva_ = t12

        self.Paz_ = t20 + dt * t21 - dt2div2 * t22
        self.Pav_ = t21 - dt * t22
        self.Paa_ = t22

        self.Pzz_ += dt4div4*self.zAccelVariance_
        self.Pzv_ += dt3div2*self.zAccelVariance_

        self.Pvz_ += dt3div2*self.zAccelVariance_
        self.Pvv_ += dt*dt*self.zAccelVariance_

        self.Paa_ += self.zAccelBiasVariance_

        # Error
        innov = z - self.z_
        sInv = 1.0 / (self.Pzz_ + self.zVariance_)  

        # Kalman gains
        kz = self.Pzz_ * sInv
        kv = self.Pvz_ * sInv
        ka = self.Paz_ * sInv

        # Update state 
        self.z_ += kz * innov
        self.v_ += kv * innov
        self.aBias_ += ka * innov

        # Update state covariance matrix
        self.Paz_ -= ka * self.Pzz_
        self.Pav_ -= ka * self.Pzv_
        self.Paa_ -= ka * self.Pza_

        self.Pvz_ -= kv * self.Pzz_
        self.Pvv_ -= kv * self.Pzv_
        self.Pva_ -= kv * self.Pza_

        self.Pzz_ -= kz * self.Pzz_
        self.Pzv_ -= kz * self.Pzv_
        self.Pza_ -= kz * self.Pza_