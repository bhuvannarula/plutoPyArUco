from time import perf_counter_ns as nowtime

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