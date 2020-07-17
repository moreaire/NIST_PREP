import serial
import matplotlib.pyplot as plt
import numpy as np

def plotsound(x,y,a):
        plt.plot(x,y)
        plt.plot(x,a)
        plt.show()


s = serial.Serial('/dev/tty.usbmodem0004401714771')

left = np.zeros(2048)
right = np.zeros(2048)
sample = np.arange(0,2048,1)
time = np.linspace(0,19e6 / 42 / 6,2048)

sound = True
s.write(b'r')

print("Starting readlines....")
while True:

    while sound:
        s.write(b'r')

        read = s.readline()
        #print(read)
        #count = count + 1
        try:
            read = read.decode().strip()
        except (UnicodeDecodeError, AttributeError):
            print("Error detected")
            pass
            print(read)
            #new_read = s.readline().strip()
        if read == "left":
                #l == True
            print("Left array:\n")
            for i in range(0,len(left)):
                if read != "right":
                    left[i] = s.readline().decode().strip()

        elif read == "right":
            print("Right array:\n")
            for i in range(0,2047):
                right[i] = s.readline().decode().strip()
                #count += 1
        elif read == "done":
            sound = False
            plotsound(time,left,right)
    sound = True
            #plotsound(sample,right)
                    #else:
                        #s.readline().decode().strip()
                            #read = s.readline().strip()


    # if l == True:
    #     if read == "Recording..." or read == "made it this far":
    #         read = s.readline().strip()
    #         count = count + 1
    #     left = np.append(left, int(read))
    #     if read == "right":
    #         print("Right array:\n")
    #         read = s.readline().strip()
    #         count = count + 1
    #         l = False
    #         r = True
    #
    # if r == True:
    #     if read == "Recording..." or read == "made it this far":
    #         read = s.readline().strip()
    #         count = count + 1
    #     right = np.append(right, int(read))

#plotsound(sample,left,sample,right)
#plotsound(sample,right)
    #print(left)

    #print("left", left)
    #print("right", right)

    # if count >= 2048:
    #     sample_l = np.linspace(0,len(left)-1)
    #     sample_r = np.linspace(0,len(right)-1)
    #
    #     plotsound(left,sample_l)
    #     plotsound(right,sample_r)

exit()

#send character and have it record from then
#use command RETARGET_readchar
#make x-axis time 19Mhz/42/6
#divide into smaller groups to figure out
# How do we do the reverse? Piezo speaker, pulse-width modulation, convert speaker to sound
# Orthogonal chirp signal processing, Python or matlab code to impliment
# Work out bluetooth and sound in PRS system
# Figure out which subroutines we may want to use in bluetooth program
