import rosbag
import sys
from gps_target.msg import SurveyGPS
import matplotlib.pyplot as plt 

x1 = [0]
y1 = [0]
z1 = []
n1 = []
H1 = []
V1 = []
T1 = []
ha1 = []
va1 = []
S1 = []
C1 = []
vel1 = []
ts1 = []
filename1 = sys.argv[1]
stamp1 = []

x2 = [0]
y2 = [0]
z2 = []
n2 = []
H2 = []
V2 = []
T2 = []
ha2 = []
va2 = []
S2 = []
C2 = []
vel2 = []
ts2 = []
filename2 = sys.argv[2]
stamp2 = []


bag1 = rosbag.Bag(filename1, 'r')
bag2 = rosbag.Bag(filename2, 'r')

for topic, msg, t in bag1.read_messages(topics=['/survey_message']):
    if((abs(msg.x - x1[-1]) < 100 and abs(msg.y - y1[-1]) < 4)):
        x1.append(msg.x)
        y1.append(msg.y)
        z1.append(msg.z)
        n1.append(msg.numSat)
        H1.append(msg.HDOP)
        V1.append(msg.VDOP)
        T1.append(msg.TDOP)
        ha1.append(msg.hAcc)
        va1.append(msg.vAcc)
        S1.append(msg.SOG)
        C1.append(msg.COG)
        vel1.append(msg.vel)
        ts1.append(t)
        stamp1.append(msg.Header.stamp.secs + msg.Header.stamp.nsecs*10**(-9))

for topic, msg, t in bag2.read_messages(topics=['/survey_message']):
    if((abs(msg.x - x2[-1]) < 100 and abs(msg.y - y2[-1]) < 4)):
        x2.append(msg.x)
        y2.append(msg.y)
        z2.append(msg.z)
        n2.append(msg.numSat)
        H2.append(msg.HDOP)
        V2.append(msg.VDOP)
        T2.append(msg.TDOP)
        ha2.append(msg.hAcc)
        va2.append(msg.vAcc)
        S2.append(msg.SOG)
        C2.append(msg.COG)
        vel2.append(msg.vel)
        ts2.append(t)
        stamp2.append(msg.Header.stamp.secs + msg.Header.stamp.nsecs*10**(-9))

ts1 = [a.to_sec() for a in ts1]
ts1 = [a - ts1[0] for a in ts1]

ts2 = [a.to_sec() for a in ts2]
ts2 = [a - ts2[0] for a in ts2]
f1 = []
f2 = []

for j in range(0,len(stamp1)):
    if j < 20:
        f1.append(21/((stamp1[20]-stamp1[0])))
    else:
        f1.append(21/(stamp1[j]-stamp1[j-20]))
        
for j in range(0,len(stamp2)):
    if j < 20:
        f2.append(21/((stamp2[20]-stamp2[0])))
    else:
        f2.append(21/(stamp2[j]-stamp2[j-20]))

fig1 = plt.figure()

plt.plot(ts1, f1)
plt.plot(ts2, f2)

fig1 = plt.figure()

plt.plot(x1[1:], y1[1:])
plt.plot(x2[1:], y2[1:])

# naming the x axis 
plt.xlabel('x - axis') 
# naming the y axis 
plt.ylabel('y - axis') 
  
# giving a title to my graph 
plt.title('Trajectory') 
  
# function to show the plot 


fig2 = plt.figure()

plt.plot(ts1, x1[1:])
plt.plot(ts2, x2[1:])

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('x - axis') 
  
# giving a title to my graph 
plt.title('X evolution') 


# function to show the plot 
 
fig3 = plt.figure()
plt.plot(ts1, y1[1:])
plt.plot(ts2, y2[1:])

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('y - axis') 
  
# giving a title to my graph 
plt.title('y evolution') 

fig4 = plt.figure()
plt.plot(ts1, n1)
plt.plot(ts2, n2)

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('satelites') 
  
# giving a title to my graph 
plt.title('satelites evolution') 

fig5 = plt.figure()
plt.plot(ts1, H1)
plt.plot(ts2, H2)

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('HDOP') 
  
# giving a title to my graph 
plt.title('HDOP evolution') 

fig6 = plt.figure()
plt.plot(ts1, V1)
plt.plot(ts2, V2)

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('VDOP') 
  
# giving a title to my graph 
plt.title('VDOP evolution') 

fig7 = plt.figure()
plt.plot(ts1, T1)
plt.plot(ts2, T2)

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('TDOP') 
  
# giving a title to my graph 
plt.title('TDOP evolution') 

fig4 = plt.figure()
plt.plot(ts1, ha1)
plt.plot(ts2, ha2)


# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('hACC') 
  
# giving a title to my graph 
plt.title('hAcc evolution') 

fig8 = plt.figure()
plt.plot(ts1, va1)
plt.plot(ts2, va2)

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('vAcc') 
  
# giving a title to my graph 
plt.title('vAcc evolution') 

fig10 = plt.figure()
plt.plot(ts1, C1)
plt.plot(ts2, C2)

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('COG') 
  
# giving a title to my graph 
plt.title('COG evolution') 

fig11 = plt.figure()
plt.plot(ts1, S1)
plt.plot(ts2, S2)

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('SOG') 
  
# giving a title to my graph 
plt.title('SOG evolution') 

fig12 = plt.figure()
plt.plot(ts1, vel1)
plt.plot(ts2, vel2)

# naming the x axis 
plt.xlabel('time') 
# naming the y axis 
plt.ylabel('vel') 
  
# giving a title to my graph 
plt.title('Velocity evolution') 

# function to show the plot 
plt.show() 


bag.close()

