from datetime import datetime
import serial
import serial.tools.list_ports
import platform
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import numpy as np
from numpy.fft import fft
from scipy import signal

plt.rcParams.update({'figure.max_open_warning': 0})

plt.style.use('dark_background')
sampling_f = 98e3 #kHz
T_c = 0.04 #chirp duration, s
t = np.arange(0,T_c,1/sampling_f) #time, between 0 and T_c with steps of whatever 100khz (sample at 100kHz) is
f_s = 20e3 #starting frequency, Hz
f_h = 22e3 #ending frequency, Hz
phi = 0 #initial phase
B = f_h - f_s #bandwidth
mu = B / T_c #chirp rate
BUFFER_SIZE = 2048 #buffer size
samplef = 19e6 / 32 / 6

s = serial.Serial('/dev/tty.usbmodem0004401673841')

left = np.zeros(BUFFER_SIZE)
right = np.zeros(BUFFER_SIZE)
ftl = np.zeros(int(BUFFER_SIZE/2)+1)
ftr = np.zeros(int(BUFFER_SIZE/2)+1)
sample = np.arange(0,BUFFER_SIZE,1)
t_time = np.linspace(0,BUFFER_SIZE/samplef,BUFFER_SIZE)
#frequency = np.linspace(0,sampling_f/2,int(BUFFER_SIZE/2))
frequency = np.arange(BUFFER_SIZE/2 + 1) * samplef/BUFFER_SIZE
f = 25000

sq_wave = signal.square(2 * np.pi * t_time * f)


plt.ion()
fig,(ax1,ax2,ax3) = plt.subplots(1,3)
lineL1, lineR1 = ax1.plot(t_time, left, 'r-', t_time, right, 'b-',alpha=0.6)
lineFL1, lineFR1 = ax2.semilogy(frequency, ftl, 'r-', frequency, ftr, 'b-',alpha=0.6)
#lineL1 = ax1.plot(t_time, left, 'r-')
#lineR1 = ax2.plot(t_time, right, 'b-')

def correlation(s1,s2):
    #FS_n1 = fft(s1)
    #TiS_n1 = np.fft.ifft(FS_n1)
    Tcorr = np.correlate(s1,s2,"full")

    n = np.linspace(0, BUFFER_SIZE/sampling_f,len(Tcorr))

    fig, ax = plt.subplots()
    ax3.plot(n,Tcorr)
    ax3.set_title("Correlation output chirp with input chirp")
    ax3.set_xlabel("Time (s)")
    #plt.show()
    #plt.xlim(0,0.01)
    return(Tcorr,n)

ax1.set_xlabel("Time (mS)")
ax1.set_ylabel("Amplitude")
ax1.set_ylim(-2500, 2500)
ax1.set_title("PDM Microphone Data")
ax1.set_xlim(0.004,0.006)
ax2.set_xlabel("Frequency (Hz)")
ax2.set_ylabel("Amplitude")
ax2.set_ylim(10e0,10e8)
#plt.show()



fig.canvas.draw()
sound = True
        #print("Sent character to device. Recording:")

# time.sleep(0.10)
r = False
larr_raw=[]
rarr_raw=[]

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
        #print(len(left))

        if len(left) > 0:
            ftl = np.abs(np.fft.rfft(left))
            ftr = np.abs(np.fft.rfft(right))

            lineFL1.set_ydata(ftl)
            lineFR1.set_ydata(ftr)

            #print(len(left),len(right),len(t_time))
            lineL1.set_ydata(left)
            lineR1.set_ydata(right)

            correlation(np.real(left),sq_wave)
            plt.pause(0.0001)

        sound = False
    #plt.pause(0.01)
