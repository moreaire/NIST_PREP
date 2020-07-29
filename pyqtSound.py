#peak height vs detuned frequency
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
SOUND_LENGTH = BUFFER_SIZE / 16
samplef = 19e6 / 32 / 6 #for device
t = np.arange(0,BUFFER_SIZE/samplef,1/samplef)
#t = np.linspace(0,BUFFER_SIZE/samplef,BUFFER_SIZE)
freq = np.arange(BUFFER_SIZE/2 + 1) * samplef/BUFFER_SIZE
#f = 10000 #32e3
#f = 24739.583333333332
f = 32986.11111111111
#t_sq = np.linspace(0,SOUND_LENGTH/samplef,samplef)
t_sq = np.arange(0,SOUND_LENGTH/samplef,1/samplef)
sq_wave = signal.square(2 * np.pi * t_sq * f) #9499
#print(sq_wave)
# zero = np.zeros(BUFFER_SIZE - SQ_LEN)
# sqarr = np.concatenate([sq_wave,zero])

def correlation(s1,s2):
    Tcorr = np.correlate(s1,s2,"full")
    Tcorr[:len(t)] += t #Changed: t to t_sq
    return(Tcorr[:len(s1)])

s = serial.Serial('/dev/tty.usbmodem0004401681131')

#Setting up plotting
app = QtGui.QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="Square Waves")
pg.setConfigOptions(antialias=True)


#Plotting
p1 = win.addPlot(title="Raw Sound Data")
lineL = p1.plot(pen="r")
#lineR = p1.plot(pen="g")

win.nextRow()
p2 = win.addPlot(title="Frequency")
freqL = p2.plot(pen="b")
#freqR = p2.plot(pen="y")
p2.setLogMode(False, True)

win.nextRow()
p3 = win.addPlot(title="Correlation of Square Wave with Output")
corrL = p3.plot(pen="m")
#corrR = p3.plot(pen="w")

win.nextRow()
p4 = win.addPlot(title="Correlation calculated on device")
lcorrD = p4.plot()
#rcorrD = p4.plot()

#For updating plots
def update():
    global t, lineL, lineR, ptr, p1, freqL, freqR, corrL, corrR, freq, lcorrD, rcorrD
    lineL.setData(t,np.abs(left))
    #lineR.setData(t,np.abs(right))

    freqs = np.fft.rfftfreq(ftl.size, 1/samplef)
    idx = np.argsort(freqs)
    freqL.setData(freqs[idx],ftl[idx])
    #freqR.setData(freqs[idx],ftr[idx])

    n = np.linspace(0, BUFFER_SIZE/samplef,len(lcorr)) #reset n

    corrL.setData(t,cL[:len(t)]) #changed from t to t_sq
    #corrR.setData(t,cR[:len(t)])
    # corrL.setData(n, np.abs(cL[:len(left)]))
    # corrR.setData(n, cR[:len(left)])
    lcorrD.setData(n,lcorr)
    #rcorrD.setData(n,rcorr)

class Helper(QtCore.QObject):
        update = QtCore.pyqtSignal()
helper = Helper()
helper.update.connect(update)

#CALCULATING DISTANCE#
def distance(timecorrelation):
    #n = np.linspace(0,BUFFER_SIZE/samplef,len(timecorrelation))
    speed = 343
    t_est = t[np.argmax(timecorrelation)]
    #print("Estimated time:",t_est,"seconds")
    dist_est = t_est * speed
    print("Estimated distance:",dist_est,"meters")

def play():
    global left, right, ftl, ftr, cL, cR, lcorr, rcorr
    larr_raw=[]
    rarr_raw=[]
    lcorr_raw=[]
    rcorr_raw=[]
    left = np.zeros(BUFFER_SIZE)
    right = np.zeros(BUFFER_SIZE)
    ftl = np.zeros(int(BUFFER_SIZE/2)+1)
    ftr = np.zeros(int(BUFFER_SIZE/2)+1)
    cL = []
    cR = []
    lcorr = []
    rcorr = []

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
        elif read == "lcorrelation":
            lcorr_raw = s.readline().decode().strip().split(" ")
        elif read == "rcorrelation":
            rcorr_raw = s.readline().decode().strip().split(" ")

        elif read == "done":
            left  = np.array(list(map(int, larr_raw)))
            #right = np.array(list(map(int, rarr_raw)))

            comparison = left == np.zeros(len(left))

            #To filter out initial zero arrays
            if comparison.all() == False:
                ftl = np.abs(np.fft.rfft(left))
                #ftr = np.abs(np.fft.rfft(right))

                cL = correlation(np.real(left),sq_wave)
                #cR = correlation(np.real(right),sq_wave)

                if len(lcorr_raw) != 0:
                    lcorr  = np.array(list(map(int, lcorr_raw)))
                    #rcorr  = np.array(list(map(int, rcorr_raw)))
                #print(corr_raw)

                helper.update.emit()
                #distance(cL)
                #distance(cR)

            #helper.update.emit()

thread_play = threading.Thread(target=play)
thread_play.start()

if __name__ == '__main__':
    if sys.flags.interactive != 1 or not hasattr(QtCore, 'PYQT_VERSION'):
        pg.QtGui.QApplication.exec_()
