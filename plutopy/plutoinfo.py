from .plutostate import *

class plutoInfo:
    def __init__(self, response : plutoState) -> None:
        self._state = response

    def acc(self) -> tuple:
        _t_arr = (self._state.accX, self._state.accY, self._state.accZ)
        return _t_arr
    
    def gyro(self):
        _t_arr = (self._state.gyroX, self._state.gyroY, self._state.gyroZ)
        return _t_arr
    
    def mag(self):
        _t_arr =  (self._state.magX, self._state.magY, self._state.magZ)
        return _t_arr
    
    def all9(self):
        _t_alt = self._state.alt
        _t_dict = {
            'acc' : self.acc(),
            'gyro' : self.gyro(),
            'mag' : self.mag(),
            'alt' : _t_alt
        }
        return _t_dict
