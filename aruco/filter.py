from time import perf_counter_ns as nowtime
from scipy.signal import butter, lfilter
from math import pi

class lowPassFilter:
    def __init__(self) -> None:
        self.filtered = 0
        self.filteredold = 0

        self.new = 0
        self.newt = nowtime()
        
        self.old = 0
        self.oldt = nowtime()

        self.coeff = [0.854, 0.0728, 0.0728]

    def get(self):
        return self.filtered
    
    def update(self, newvalue):
        self.filteredold = self.filtered
        self.old = self.new
        self.new = newvalue

        self.oldt = self.newt
        self.newt = nowtime()

        self.filtered = self.filtered*self.coeff[0] + self.new*self.coeff[1] + self.old*self.coeff[2]
    
    def derivative(self):
        delta = self.filtered - self.filteredold
        dT = self.newt - self.oldt
        return (delta/dT)

class lpfilterZ:
    def __init__(self) -> None:
        order = 2
        fs = 30
        nyq = 0.5 * fs
        lowcut = 2
        low = lowcut/nyq
        b, a = butter(order, low, btype='low')
        self.b = b
        self.a = a
        self.record = [0]*250

        self.newt = nowtime()
        self.oldt = nowtime()

        self.fvalue = 0
        self.fvalueold = 0
    
    def update(self, newvalue):
        self.record.append(newvalue)
        self.record.pop(0)

        self.fvalueold = self.fvalue

        filtered = lfilter(self.b, self.a, self.record)
        self.fvalue = filtered[-1]

        self.oldt = self.newt
        self.newt = nowtime()

    def get(self):
        return self.fvalue
    
    def derivative(self):
        delta = self.fvalue - self.fvalueold
        dT = self.newt - self.oldt
        return (delta/dT)