import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from time import sleep
from pluto_aruco import *

X,Y,Z=0,1,2
class XYZ:
    def __init__(self) -> None:
        self.X = 0
        self.Y = 0
        self.Z = 0
    def __repr__(self) -> str:
        return str((self.X, self.Y, self.Z))

class graph:
    def __init__(self, aruco : plutoArUco) -> None:
        self.graphdelay = 1
        self.state = aruco
        self.debug = 0
        LEN = 20
        self.err = [0]*3
        self.err[X] = [0]*LEN
        self.err[Y] = [0]*LEN
        self.err[Z] = [0]*LEN
        self.P = [0]*3
        self.I = [0]*3
        self.D = [0]*3
        self.P[X]= [0]*LEN
        self.P[Y]= [0]*LEN
        self.P[Z]= [0]*LEN
        self.I[X]= [0]*LEN
        self.I[Y]= [0]*LEN
        self.I[Z]= [0]*LEN
        self.D[X]= [0]*LEN
        self.D[Y]= [0]*LEN
        self.D[Z]= [0]*LEN
        self.lastresult = [0]*3
        self.lastresult[X]= [0]*LEN
        self.lastresult[Y]= [0]*LEN
        self.lastresult[Z]= [0]*LEN
        self.st = [self.err,self.P,self.I,self.D,self.lastresult]
        self.label = ['error','P','I','D','result']
    def animate(self):
        self.fig = plt.figure(figsize=(12,6),facecolor='#DEDEDE')
        self.ax = [0]*3
        self.ax[X] = plt.subplot(131)
        self.ax[Y] = plt.subplot(132)
        self.ax[Z] = plt.subplot(133) 
        self.ax[X].set_facecolor('#DEDEDE')
        self.ax[Y].set_facecolor('#DEDEDE')
        self.ax[Z].set_facecolor('#DEDEDE')
        # animate
        self.ani = FuncAnimation(self.fig, self.arucoGraph, interval=100)
        plt.show()

    def arucoGraph(self, k):
        #self.fix_state = XYZ()
        #self._tt = {X:random.random()*100,Y:random.random()*100,Z:random.random()*100}
        self._tt = [
            self.state.positionPID.position_err,
            self.state.positionPID.lastP,
            self.state.positionPID.lastI,
            self.state.positionPID.lastD,
            self.state.positionPID.last_result
        ]
        '''
        self._err = [
            self._tt[X],
            self._tt[Y],
            self._tt[Z]
        ]
        '''
        for i in range(3):
            self.ax[i].cla()
            for j in (0,1,3,4):
                self.st[j][i].pop(0)
                self.st[j][i].append(self._tt[j][i])
                self.ax[i].plot(self.st[j][i],label=self.label[j])
                self.ax[i].scatter(len(self.st[j][i])-1, self.st[j][i][-1])
                self.ax[i].text(len(self.st[j][i])-1, self.st[j][i][-1]+2, "{}".format(self.st[j][i][-1]))
            self.ax[i].set_ylim(300,-300)
            self.ax[i].legend()

if __name__ == "__main__":
    drone = plutoDrone()

    pluto = plutoArUco(drone)
    pluto.debug = True
    pluto.start()

    sleep(13)
    visual = graph(pluto)
    visual.animate()