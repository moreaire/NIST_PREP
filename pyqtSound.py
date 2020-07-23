from datetime import datetime
import serial
import serial.tools.list_ports
import platform
import pyqtgraph as pg
import numpy as np
from numpy.fft import fft
from scipy import signal
from pyqtgraph.Qt import QtGui, QtCore
import sys
import threading

BUFFER_SIZE = 2048 #buffer size
samplef = 19e6 / 32 / 6
sampling_f = 98e3 #kHz
sample = np.arange(0,BUFFER_SIZE,1)
t = np.linspace(0,BUFFER_SIZE/samplef,BUFFER_SIZE)
freq = np.arange(BUFFER_SIZE//2 + 1) * samplef/BUFFER_SIZE
f = 30000
sq_wave = signal.square(2 * np.pi * t * f)

def correlation(s1,s2):
    Tcorr = np.correlate(s1,s2,"full")
    return(Tcorr)

s = serial.Serial('/dev/tty.usbmodem0004401673841')

#Setting up plotting
app = QtGui.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Square Waves")
pg.setConfigOptions(antialias=True)


#Plotting
p1 = win.addPlot(title="Raw Sound Data")
lineL = p1.plot(pen="r")
lineR = p1.plot(pen="g")

win.nextRow()
p2 = win.addPlot(title="Frequency")
freqL = p2.plot(pen="b")
freqR = p2.plot(pen="y")
p2.setLogMode(False, True)

win.nextRow()
p3 = win.addPlot(title="Correlation of Square Wave with Output")
corrL = p3.plot(pen="m")
corrR = p3.plot(pen="w")

#ptr = 0
#For updating plots
def update():
    global t, lineL, lineR, ptr, p1, freqL, freqR, corrL, corrR, freq
    lineL.setData(t,left)
    lineR.setData(t,right)

    freqL.setData(freq,ftl)
    freqR.setData(freq,ftr)

    n = np.linspace(0, BUFFER_SIZE/sampling_f,len(cL)) #reset n
    corrL.setData(n[round(len(cL)/2):],cL[round(len(cL)/2):]) #[round(len(cL)/2):]
    corrR.setData(n[round(len(cL)/2):],cR[round(len(cL)/2):])
    #print(left)
    #print(right)
    #if ptr == 0:
        #p1.enableAutoRange('xy', False)  ## stop auto-scaling after the first data set is plotted
    #ptr += 1
    #print(ptr)

class Helper(QtCore.QObject):
        update = QtCore.pyqtSignal()
helper = Helper()
helper.update.connect(update)


def play():
    global left, right, ftl, ftr, cL, cR
    larr_raw=[]
    rarr_raw=[]
    left = np.zeros(BUFFER_SIZE)
    right = np.zeros(BUFFER_SIZE)
    ftl = np.zeros(int(BUFFER_SIZE/2)+1)
    ftr = np.zeros(int(BUFFER_SIZE/2)+1)
    cL = []
    cR = []

    while True:
        s.write(b'r')
        read = s.readline()

        try:
            read = read.decode().strip()
        except (UnicodeDecodeError, AttributeError):
            print("Error detected")
            pass
        if read == "left":
            larr_raw = s.readline().decode().strip().split(" ")
        elif read == "right":
            #print("Right array:\n")
            rarr_raw = s.readline().decode().strip().split(" ")

        elif read == "done":
            left  = np.array(list(map(int, larr_raw)))
            right = np.array(list(map(int, rarr_raw)))

            comparison = left == np.zeros(len(left))

            #To filter out initial zero arrays
            if comparison.all() == False:
                ftl = np.abs(np.fft.rfft(left))
                ftr = np.abs(np.fft.rfft(right))

                cL = correlation(np.real(left),sq_wave)
                cR = correlation(np.real(left),sq_wave)

                helper.update.emit()

thread_play = threading.Thread(target=play)
thread_play.start()

if __name__ == '__main__':
    if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):
        pg.QtGui.QApplication.exec_()
