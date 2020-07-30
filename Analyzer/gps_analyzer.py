#!/usr/bin/env python
import rospy
from gps_target.msg import SurveyGPS
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from enum import Enum 
from math import sqrt
import math
import time
from numpy import mean

class Data(Enum):
    POS = 1
    ALT = 2
    SAT = 3
    STAT = 4
    SOG = 5
    COG = 6
    HACC = 7
    VACC = 8 
    VDOP = 9
    HDOP = 10
    TDOP = 11


x = [0]
y = [0]
num = [0]
z = [0]
H = [0]
V = [0]
T = [0]
ha = [0]
va = [0]
S = [0]
C = [0]
freq = [0]
avg_freq = 0
vel = [0]
stat = [0]
xmin = -100.0
xmax = 100.0
ymin = -100.0
ymax = 100.0
fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)
initTime = time.time()
lastTime = 100000
currTime = 0
seq = 0

def showData(id):
    global initTime
    global seq
    color = 'red'
    xmid = (xmax+xmin)/2
    ymid = (ymax+ymin)/2
    xdist = xmax-xmin
    ydist = ymax-ymin

    # if id == Data.POS:
    ax1.plot(x[1:], y[1:])
    ax1.plot(x[-1:], y[-1:], 'ro')

    ax1.text(xmid+0.05*xdist, ymid+0.55*ydist, 'Altitude: {0:.10}'.format(str(z[-1:])[1:-1]))
    elaps = time.time() - initTime 
    
    if(elaps < 1.5 and elaps > 1.2):
        freq.append(seq/elaps)
        seq = 0
        initTime = time.time()

    avg_freq = freq[-1]

     
    ax1.text(xmid+0.05*xdist, ymid+0.55*ydist-5, 'Freq: {0:.5}'.format(str(avg_freq)))
    
    
    ax1.text(xmid-0.25*xdist, ymid+.55*ydist, 'Satellites: {0}'.format(str(num[-1:])[1:-1]))
    
    if stat[-1:] == ['NF']:
        color = 'red'
    elif (stat[-1:] == ['G2'] or stat[-1:] == ['G3']):
        color = 'yellow'
    elif (stat[-1:] == ['D2'] or stat[-1:] == ['D3']):
        color = 'green'

    ax1.text(xmid-0.5*xdist, ymid+.55*ydist, 'Status:')
    ax1.text(xmid-0.35*xdist, ymid+.55*ydist, '{0}'.format(str(stat[-1:])[2:-2]), color = color)
    
    ax1.text(xmid - 0.55*xdist, ymid-.625*ydist, 'Speed: {0:.6}'.format(str(S[-1:])[1:-1]), fontsize=12)
    
    plot_point([float(str(x[-1:])[1:-1]), float(str(y[-1:])[1:-1])], float(str(C[-1:])[1:-1]), sqrt((abs(xmax)+abs(xmin))**2.0+(abs(ymax)+abs(ymin))**2.0)/20)
    
    ax1.text(xmid - xdist/4, ymid-0.625*ydist, 'Acc: [{0:.5}, {1:.5}]'.format(str(ha[-1:])[1:-1], str(va[-1:])[1:-1]), fontsize=12)

    ax1.text(xmid + .1*xdist, ymid-0.625*ydist, 'DOP: [{0:.5}, {1:.5}, {2:.5}]'.format(str(H[-1:])[1:-1], str(V[-1:])[1:-1], str(T[-1:])[1:-1]), fontsize=12)



def callback(data):
    x.append(data.x)
    y.append(data.y)
    z.append(data.z)
    stat.append(data.navStat)
    num.append(data.numSat)
    H.append(data.HDOP)
    V.append(data.VDOP)
    T.append(data.TDOP)
    ha.append(data.hAcc)
    va.append(data.vAcc)
    S.append(data.SOG)
    C.append(data.COG)
    vel.append(data.vel)
    global lastTime 
    global currTime 
    global initTime
    global seq
    lastTime = currTime
    currTime = data.Header.stamp.secs + data.Header.stamp.nsecs * 10**(-9)
    #freq.append(1/(currTime-lastTime))
    #print currTime-lastTime
    if seq == 0:
        initTime = time.time()
    seq += 1
    #print seq


    # t = [a - a[0] for a in t]
    
def plot_point(point, angle, length):
     # unpack the first point
    x, y = point
    
     # find the end point
    endy = length * math.sin(math.radians(angle))
    endx = length * math.cos(math.radians(angle))
    

     # plot the points
    ax1.plot([x, x+endx], [y, y+endy])
    ax1.text(x+endx, y+endy, '{0:.6}'.format(angle))

def listener():


    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/survey_message", SurveyGPS, callback)

    style.use('fivethirtyeight')

    # spin() simply keeps python from exiting until this node is stopped

    while(not rospy.is_shutdown()):
        ax1.clear()
        ax1.axis([xmin, xmax, ymin, ymax])
        if(len(x) > 0):
            showData(0)
        plt.show(False)
        plt.pause(0.001)
    

if __name__ == '__main__':
    listener()